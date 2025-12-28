#include "pid.h"
#include <math.h>

// Initialize PID gains and limits.
void PID_Init(PID *pid, float kp, float ki, float kd,
              float deadband,
              float out_min, float out_max,
              float i_min, float i_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->last_p = 0.0f;
  pid->last_d = 0.0f;
  pid->last_out = 0.0f;
  pid->last_error = 0.0f;
  pid->integrator = 0.0f;
  pid->prev_meas = 0.0f;
  pid->init = 0;
  pid->deadband = deadband;
  pid->out_min = out_min;
  pid->out_max = out_max;
  pid->i_min = i_min;
  pid->i_max = i_max;
}

// Reset integrator and previous measurement.
void PID_Reset(PID *pid, float meas)
{
  pid->last_p = 0.0f;
  pid->last_d = 0.0f;
  pid->last_out = 0.0f;
  pid->last_error = 0.0f;
  pid->integrator = 0.0f;
  pid->prev_meas = meas;
  pid->init = 1;
}

// Update PID; returns saturated output. Derivative on measurement to reduce noise coupling.
float PID_Update(PID *pid, float setpoint, float meas, float dt_sec)
{
  if (dt_sec <= 0.0f) {
    return 0.0f;
  }

  if (!pid->init) {
    pid->prev_meas = meas;
    pid->init = 1;
  }

  float error = setpoint - meas;
  pid->last_error = error;
  // Apply deadband to error for integration (and proportional if desired)
  float err_for_pid = error;
  if (fabsf(error) < pid->deadband) {
    err_for_pid = 0.0f;
  }

  pid->integrator += pid->ki * err_for_pid * dt_sec;

  // Clamp integrator
  if (pid->integrator > pid->i_max) pid->integrator = pid->i_max;
  if (pid->integrator < pid->i_min) pid->integrator = pid->i_min;

  float deriv = -(meas - pid->prev_meas) / dt_sec;  // derivative on measurement
  pid->prev_meas = meas;

  float p_term = pid->kp * err_for_pid;
  float i_term = pid->integrator;
  float d_term = pid->kd * deriv;
  float out = p_term + i_term + d_term;

  if (out > pid->out_max) out = pid->out_max;
  if (out < pid->out_min) out = pid->out_min;
  pid->last_p = p_term;
  pid->last_d = d_term;
  pid->last_out = out;
  return out;
}
