/*
 * ramp.h
 *
 *  Created on: Oct 25, 2025
 *      Author: Kartik
 */

#ifndef RAMP_H
#define RAMP_H
#include <stdint.h>

typedef struct {
    float value;         // current (ramped) value
    float max_delta_per_s; // maximum change per second (units: RPM/s)
} Ramp_t;

void Ramp_Init(Ramp_t* r, float initial, float max_delta_per_s);
float Ramp_Update(Ramp_t* r, float target, float dt_s);
#endif
