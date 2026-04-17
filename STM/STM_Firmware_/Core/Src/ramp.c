/*
 * ramp.c
 *
 *  Created on: Oct 25, 2025
 *      Author: Kartik
 */


#include "ramp.h"
#include <math.h>

void Ramp_Init(Ramp_t* r, float initial, float max_delta_per_s) {
    r->value = initial;
    r->max_delta_per_s = fabsf(max_delta_per_s);
}

float Ramp_Update(Ramp_t* r, float target, float dt_s) {
    if (dt_s <= 0.0f) return r->value;
    float max_delta = r->max_delta_per_s * dt_s;
    float delta = target - r->value;
    if (delta > max_delta) delta = max_delta;
    else if (delta < -max_delta) delta = -max_delta;
    r->value += delta;
    return r->value;
}
