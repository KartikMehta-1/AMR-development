#include "encoder.h"

void Encoder_Init(Encoder_t *e, TIM_HandleTypeDef *htim, uint32_t counts_per_rev, uint32_t sample_interval_ms)
{
    e->htim = htim;
    e->counts_per_rev = counts_per_rev;
    e->sample_interval_ms = sample_interval_ms;
    e->last_cnt = __HAL_TIM_GET_COUNTER(htim);
    e->position = 0;
    e->rpm = 0.0f;
}

void Encoder_Update(Encoder_t *e)
{
    uint32_t cur = __HAL_TIM_GET_COUNTER(e->htim);
    if (e->htim->Init.Period <= 0xFFFFu) {
        uint16_t cur16 = (uint16_t)cur;
        int16_t d16 = (int16_t)(cur16 - (uint16_t)e->last_cnt);
        e->position += (int32_t)d16;
        e->last_cnt = cur16;
        float dt = (float)e->sample_interval_ms / 1000.0f;
        e->rpm = ((float)d16 / (float)e->counts_per_rev) * (60.0f / dt);
    } else {
        int32_t d32 = (int32_t)((int32_t)cur - (int32_t)e->last_cnt);
        e->position += d32;
        e->last_cnt = cur;
        float dt = (float)e->sample_interval_ms / 1000.0f;
        e->rpm = ((float)d32 / (float)e->counts_per_rev) * (60.0f / dt);
    }
}

int32_t Encoder_GetPosition(Encoder_t *e) { return e->position; }
float Encoder_GetRPM(Encoder_t *e) { return e->rpm; }
