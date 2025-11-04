// Simple quadrature encoder helper for TIM encoder mode
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;   // TIM in encoder mode (already configured by MX_TIMx_Init)
    uint32_t max_count;        // 0xFFFF for 16-bit (TIM3), 0xFFFFFFFF for 32-bit (TIM2)
    uint32_t counts_per_rev;   // mechanical counts per wheel revolution (quadrature ×4)
    uint32_t last_hw_count;    // previous raw counter sample
    int32_t position;          // accumulated counts (signed)
    float rpm;                 // last computed RPM
} EncoderChannel;

void Encoder_Init(EncoderChannel *e,
                  TIM_HandleTypeDef *htim,
                  uint32_t counts_per_rev,
                  uint32_t max_count);

HAL_StatusTypeDef Encoder_Start(EncoderChannel *e);

void Encoder_Update(EncoderChannel *e, float dt_seconds);

static inline int32_t Encoder_GetPosition(const EncoderChannel *e) { return e->position; }
static inline float   Encoder_GetRPM(const EncoderChannel *e)      { return e->rpm; }
static inline uint32_t Encoder_GetRawCount(const EncoderChannel *e) { return __HAL_TIM_GET_COUNTER(e->htim); }

#endif // ENCODER_H

