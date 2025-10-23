#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t last_cnt;
    int32_t position;        // cumulative counts
    float rpm;
    uint32_t counts_per_rev; // quadrature counts per revolution
    uint32_t sample_interval_ms;
} Encoder_t;

void Encoder_Init(Encoder_t *e, TIM_HandleTypeDef *htim, uint32_t counts_per_rev, uint32_t sample_interval_ms);
void Encoder_Update(Encoder_t *e); // call periodically every sample_interval_ms
int32_t Encoder_GetPosition(Encoder_t *e);
float Encoder_GetRPM(Encoder_t *e);

#endif // ENCODER_H
