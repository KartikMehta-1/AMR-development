#include "control_loop.h"
#include "app_config.h"

void ControlLoop_Init(ControlLoop *cl)
{
  Ramp_Init(&cl->ramp_l, 0.0f, DUTY_RAMP_RATE_PER_SEC);
  Ramp_Init(&cl->ramp_r, 0.0f, DUTY_RAMP_RATE_PER_SEC);
  PID_Init(&cl->pid_l, SPEED_PID_KP_L, SPEED_PID_KI_L, SPEED_PID_KD_L, SPEED_PID_DEADBAND_RPM,
           SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, SPEED_PID_I_MIN, SPEED_PID_I_MAX);
  PID_Init(&cl->pid_r, SPEED_PID_KP_R, SPEED_PID_KI_R, SPEED_PID_KD_R, SPEED_PID_DEADBAND_RPM,
           SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX, SPEED_PID_I_MIN, SPEED_PID_I_MAX);
  cl->rpm_target = SPEED_TEST_RPM_LOW;
  cl->last_toggle_ms = 0U;
  cl->toggle_high = 0;
}

void ControlLoop_Update(ControlLoop *cl,
                        float rpm_l, float rpm_r,
                        bool enabled,
                        float dt_s,
                        uint32_t now_ms,
                        float *duty_cmd_l,
                        float *duty_cmd_r,
                        float *rpm_target_out)
{
  if (!enabled) {
    cl->rpm_target = 0.0f;
    Ramp_SetTarget(&cl->ramp_l, 0.0f);
    Ramp_SetTarget(&cl->ramp_r, 0.0f);
  } else {
    // Toggle RPM setpoint every 5 seconds between low/high (test mode)
    if ((now_ms - cl->last_toggle_ms) >= SPEED_TEST_TOGGLE_MS) {
      cl->toggle_high = !cl->toggle_high;
      cl->rpm_target = cl->toggle_high ? SPEED_TEST_RPM_HIGH : SPEED_TEST_RPM_LOW;
      cl->last_toggle_ms = now_ms;
    }

    // PID speed control -> duty targets
    float pid_out_l = PID_Update(&cl->pid_l, cl->rpm_target, rpm_l, dt_s);
    float pid_out_r = PID_Update(&cl->pid_r, cl->rpm_target, rpm_r, dt_s);

    // Ramp duties toward PID outputs for smooth actuation
    Ramp_SetTarget(&cl->ramp_l, pid_out_l);
    Ramp_SetTarget(&cl->ramp_r, pid_out_r);
  }

  *duty_cmd_l = Ramp_Update(&cl->ramp_l, dt_s);
  *duty_cmd_r = Ramp_Update(&cl->ramp_r, dt_s);
  if (rpm_target_out) {
    *rpm_target_out = cl->rpm_target;
  }
}
