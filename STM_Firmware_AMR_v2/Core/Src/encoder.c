#include "encoder.h"

static inline int32_t encoder_delta(uint32_t now, uint32_t last, uint32_t max_count)
{
    uint64_t mod = (uint64_t)max_count + 1ULL;
    int64_t diff = (int64_t)( (int64_t)now - (int64_t)last );
    // Wrap to minimal signed distance
    if (diff > (int64_t)(mod/2)) diff -= (int64_t)mod;
    else if (diff < -(int64_t)(mod/2)) diff += (int64_t)mod;
    if (diff > INT32_MAX) diff = INT32_MAX;
    if (diff < INT32_MIN) diff = INT32_MIN;
    return (int32_t)diff;
}

void Encoder_Init(EncoderChannel *e,
                  TIM_HandleTypeDef *htim,
                  uint32_t counts_per_rev,
                  uint32_t max_count)
{
    e->htim = htim;
    e->counts_per_rev = counts_per_rev;
    e->max_count = max_count;
    e->last_hw_count = 0U;
    e->position = 0;
    e->rpm = 0.0f;
}

HAL_StatusTypeDef Encoder_Start(EncoderChannel *e)
{
    HAL_StatusTypeDef st = HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);
    e->last_hw_count = __HAL_TIM_GET_COUNTER(e->htim);
    return st;
}

void Encoder_Update(EncoderChannel *e, float dt_seconds)
{
    if (dt_seconds <= 0.0f) return;
    uint32_t now = __HAL_TIM_GET_COUNTER(e->htim);
    int32_t dcnt = encoder_delta(now, e->last_hw_count, e->max_count);
    e->last_hw_count = now;
    e->position += dcnt;
    float revs = (e->counts_per_rev > 0U) ? ((float)dcnt / (float)e->counts_per_rev) : 0.0f;
    e->rpm = revs / dt_seconds * 60.0f;
}

