#include "test_sine.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern float target_rpm;

/* Sine test parameters */
static float sine_amplitude = 800.0f;  // amplitude of RPM variation
static float sine_offset    = 800.0f;  // mean RPM
static float sine_freq_hz   = 0.1f;    // one full sine every 10 sec
static uint32_t start_ms    = 0;

static void sine_init(void)
{
    start_ms = HAL_GetTick();
    target_rpm = sine_offset;
}

static void sine_update(float dt_s)
{
    (void)dt_s;
    float t = (HAL_GetTick() - start_ms) / 1000.0f;  // seconds
    target_rpm = sine_offset + sine_amplitude * sinf(2.0f * M_PI * sine_freq_hz * t);
}

/* Global test descriptor */
const Test_t Test_Sine = {
    .name = "Sine Wave Test",
    .init = sine_init,
    .update = sine_update
};
