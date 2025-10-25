#include "test_step.h"
#include "main.h"  // For HAL_GetTick
#include <stdio.h>

/* External variable shared with controller */
extern float target_rpm;

/* Test parameters */
static float step_target = 1000.0f;    // Step target RPM
static uint32_t step_delay_ms = 20000; // Wait 2 seconds before applying step
static uint32_t start_time = 0;

static void step_init(void)
{
    start_time = HAL_GetTick();
    target_rpm = 0.0f;
}

static void step_update(float dt_s)
{
    (void)dt_s;
    uint32_t elapsed = HAL_GetTick() - start_time;

    if (elapsed >= step_delay_ms)
        target_rpm = step_target;
    else
        target_rpm = 0.0f;
}

/* Global test descriptor */
const Test_t Test_Step = {
    .name = "Step Input Test",
    .init = step_init,
    .update = step_update
};
