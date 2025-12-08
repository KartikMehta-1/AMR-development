// Speed PI + ramp helper with test setpoint toggle
#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "ramp.h"
#include "pid.h"

typedef struct {
  Ramp ramp_l;
  Ramp ramp_r;
  PID  pid_l;
  PID  pid_r;
  float rpm_target;
  uint32_t last_toggle_ms;
  uint8_t toggle_high;
} ControlLoop;

void ControlLoop_Init(ControlLoop *cl);
void ControlLoop_Update(ControlLoop *cl,
                        float rpm_l, float rpm_r,
                        bool enabled,
                        float dt_s,
                        uint32_t now_ms,
                        float *duty_cmd_l,
                        float *duty_cmd_r,
                        float *rpm_target_out);

#endif // CONTROL_LOOP_H
