// Simple ramp/slew helper for setpoints (e.g., RPM or duty) to change smoothly over time.
#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>

typedef struct {
  float value;   // current ramped output
  float target;  // desired target value
  float rate;    // max change per second (units/sec)
} Ramp;

// Initialize ramp with a starting value and slew rate (units/sec).
static inline void Ramp_Init(Ramp *r, float initial, float rate_per_sec)
{
  r->value = initial;
  r->target = initial;
  r->rate = rate_per_sec;
}

// Update the target setpoint (ramp will slew toward it on next update calls).
static inline void Ramp_SetTarget(Ramp *r, float target)
{
  r->target = target;
}

// Optionally update the slew rate (units/sec).
static inline void Ramp_SetRate(Ramp *r, float rate_per_sec)
{
  r->rate = rate_per_sec;
}

// Advance the ramp by dt seconds. Returns the new ramped value.
static inline float Ramp_Update(Ramp *r, float dt_sec)
{
  float max_step = r->rate * dt_sec;
  float err = r->target - r->value;
  if (err > max_step) {
    r->value += max_step;
  } else if (err < -max_step) {
    r->value -= max_step;
  } else {
    r->value = r->target;
  }
  return r->value;
}

#endif // RAMP_H
