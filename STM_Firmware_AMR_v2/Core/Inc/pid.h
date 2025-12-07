// Simple PID controller for speed/duty control
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prev_meas;
  uint8_t init;
  float deadband;
  float out_min;
  float out_max;
  float i_min;
  float i_max;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd,
              float deadband,
              float out_min, float out_max,
              float i_min, float i_max);

void PID_Reset(PID *pid, float meas);

float PID_Update(PID *pid, float setpoint, float meas, float dt_sec);

#endif // PID_H
