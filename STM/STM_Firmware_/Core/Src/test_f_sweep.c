#include "test_f_sweep.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/**
 * Frequency sweep test (chirp) — useful for frequency response characterization.
 * target_rpm = offset + amplitude * sin(2π * f(t) * t)
 * where f(t) increases linearly over time.
 */

extern float target_rpm;

/* Sweep parameters */
static float sweep_amp     = 600.0f;   // amplitude (RPM)
static float sweep_offset  = 800.0f;   // base RPM
static float f_start       = 0.05f;    // start frequency (Hz)
static float f_end         = 1.0f;     // end frequency (Hz)
static float sweep_time_s  = 30.0f;    // duration of sweep
static uint32_t start_ms   = 0;

static void f_sweep_init(void)
{
    start_ms = HAL_GetTick();
    target_rpm = sweep_offset;
}

static void f_sweep_update(float dt_s)
{
    (void)dt_s;

    float t = (HAL_GetTick() - start_ms) / 1000.0f;  // time in seconds
    float ratio = t / sweep_time_s;

    if (ratio > 1.0f) ratio = 1.0f;

    float freq = f_start + (f_end - f_start) * ratio;
    float angle = 2.0f * M_PI * freq * t;

    target_rpm = sweep_offset + sweep_amp * sinf(angle);
}

/* Global test descriptor */
const Test_t Test_F_Sweep = {
    .name = "Frequency Sweep (Chirp) Test",
    .init = f_sweep_init,
    .update = f_sweep_update
};
