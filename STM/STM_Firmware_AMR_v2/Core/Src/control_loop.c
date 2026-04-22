#include "control_loop.h"
#include "app_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void ControlLoop_Init(ControlLoop *cl)
{
  Ramp_Init(&cl->ramp_l, 0.0f, DUTY_RAMP_RATE_PER_SEC);
  Ramp_Init(&cl->ramp_r, 0.0f, DUTY_RAMP_RATE_PER_SEC);
  Ramp_Init(&cl->ramp_v, 0.0f, V_CMD_RAMP_RATE_MPS);
  Ramp_Init(&cl->ramp_w, 0.0f, W_CMD_RAMP_RATE_RAD);
  PID_Init(&cl->pid_l, SPEED_PID_KP_L, SPEED_PID_KI_L, SPEED_PID_KD_L, SPEED_PID_DEADBAND_RPM,
           SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, SPEED_PID_I_MIN, SPEED_PID_I_MAX);
  PID_Init(&cl->pid_r, SPEED_PID_KP_R, SPEED_PID_KI_R, SPEED_PID_KD_R, SPEED_PID_DEADBAND_RPM,
           SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, SPEED_PID_I_MIN, SPEED_PID_I_MAX);
  cl->rpm_target_l = 0.0f;
  cl->rpm_target_r = 0.0f;
  cl->cmd_v_mps = 0.0f;
  cl->cmd_w_rps = 0.0f;
  cl->v_l_mps = 0.0f;
  cl->v_r_mps = 0.0f;
}

void ControlLoop_Update(ControlLoop *cl,
                        float rpm_l, float rpm_r,
                        bool enabled,
                        float v_cmd_mps,
                        float w_cmd_rps,
                        float dt_s,
                        uint32_t now_ms,
                        float *duty_cmd_l,
                        float *duty_cmd_r,
                        float *rpm_target_l_out,
                        float *rpm_target_r_out)
{
  const float rpm_scale = 30.0f / ((float)M_PI * WHEEL_RADIUS_M); // v [m/s] -> RPM
  const float half_track = 0.5f * TRACK_WIDTH_M;

  if (!enabled) {
    cl->rpm_target_l = 0.0f;
    cl->rpm_target_r = 0.0f;
    cl->cmd_v_mps = 0.0f;
    cl->cmd_w_rps = 0.0f;
    cl->v_l_mps = 0.0f;
    cl->v_r_mps = 0.0f;
    Ramp_SetTarget(&cl->ramp_l, 0.0f);
    Ramp_SetTarget(&cl->ramp_r, 0.0f);
    Ramp_SetTarget(&cl->ramp_v, 0.0f);
    Ramp_SetTarget(&cl->ramp_w, 0.0f);
  } else {
    cl->cmd_v_mps = v_cmd_mps;
    cl->cmd_w_rps = w_cmd_rps;
#if RAMPING_ENABLE
    Ramp_SetTarget(&cl->ramp_v, cl->cmd_v_mps);
    Ramp_SetTarget(&cl->ramp_w, cl->cmd_w_rps);

    // Slew v, w to targets to keep turns coordinated and smooth
    float v_cmd = Ramp_Update(&cl->ramp_v, dt_s);
    float w_cmd = Ramp_Update(&cl->ramp_w, dt_s);
#else
    float v_cmd = cl->cmd_v_mps;
    float w_cmd = cl->cmd_w_rps;
#endif

    // Differential-drive kinematics to wheel linear velocities
    float v_l = v_cmd - (w_cmd * half_track);
    float v_r = v_cmd + (w_cmd * half_track);
    cl->v_l_mps = v_l;
    cl->v_r_mps = v_r;

    // Convert to RPM targets
    cl->rpm_target_l = v_l * rpm_scale;
    cl->rpm_target_r = v_r * rpm_scale;

    // PID speed control -> duty targets
    float pid_out_l = PID_Update(&cl->pid_l, cl->rpm_target_l, rpm_l, dt_s);
    float pid_out_r = PID_Update(&cl->pid_r, cl->rpm_target_r, rpm_r, dt_s);
#if SPEED_FF_ENABLE
    float ff_l = SPEED_FF_K_DUTY_PER_RPM * cl->rpm_target_l;
    float ff_r = SPEED_FF_K_DUTY_PER_RPM * cl->rpm_target_r;
    float tgt_abs_l = fabsf(cl->rpm_target_l);
    float tgt_abs_r = fabsf(cl->rpm_target_r);
    if (tgt_abs_l > SPEED_FF_STATIC_RPM) {
      float static_ff_scale_l = 1.0f;
      float static_ff_span = SPEED_FF_STATIC_RPM_FULL - SPEED_FF_STATIC_RPM;
      if (static_ff_span > 0.0f && tgt_abs_l < SPEED_FF_STATIC_RPM_FULL) {
        static_ff_scale_l = (tgt_abs_l - SPEED_FF_STATIC_RPM) / static_ff_span;
      }
      ff_l += (cl->rpm_target_l >= 0.0f ? 1.0f : -1.0f) *
              (SPEED_FF_STATIC_DUTY * static_ff_scale_l);
    }
    if (tgt_abs_r > SPEED_FF_STATIC_RPM) {
      float static_ff_scale_r = 1.0f;
      float static_ff_span = SPEED_FF_STATIC_RPM_FULL - SPEED_FF_STATIC_RPM;
      if (static_ff_span > 0.0f && tgt_abs_r < SPEED_FF_STATIC_RPM_FULL) {
        static_ff_scale_r = (tgt_abs_r - SPEED_FF_STATIC_RPM) / static_ff_span;
      }
      ff_r += (cl->rpm_target_r >= 0.0f ? 1.0f : -1.0f) *
              (SPEED_FF_STATIC_DUTY * static_ff_scale_r);
    }
    pid_out_l += ff_l;
    pid_out_r += ff_r;
#endif

#if RAMPING_ENABLE
    // Ramp duties toward PID outputs for smooth actuation
    Ramp_SetTarget(&cl->ramp_l, pid_out_l);
    Ramp_SetTarget(&cl->ramp_r, pid_out_r);
#else
    cl->ramp_l.value = pid_out_l;
    cl->ramp_r.value = pid_out_r;
#endif
  }

#if RAMPING_ENABLE
  *duty_cmd_l = Ramp_Update(&cl->ramp_l, dt_s);
  *duty_cmd_r = Ramp_Update(&cl->ramp_r, dt_s);
#else
  *duty_cmd_l = cl->ramp_l.value;
  *duty_cmd_r = cl->ramp_r.value;
#endif
  // Saturate both duties together to preserve curvature when limiting
  float max_abs = fabsf(*duty_cmd_l);
  float abs_r = fabsf(*duty_cmd_r);
  if (abs_r > max_abs) max_abs = abs_r;
  if (max_abs > MOTOR_DUTY_MAX) {
    float scale = MOTOR_DUTY_MAX / max_abs;
    *duty_cmd_l *= scale;
    *duty_cmd_r *= scale;
  }
  if (rpm_target_l_out) {
    *rpm_target_l_out = cl->rpm_target_l;
  }
  if (rpm_target_r_out) {
    *rpm_target_r_out = cl->rpm_target_r;
  }
}
