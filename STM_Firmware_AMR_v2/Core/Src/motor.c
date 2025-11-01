#include "motor.h"

static inline void set_compare(MotorChannel *m, uint32_t ccr)
{
    switch (m->channel) {
        case TIM_CHANNEL_1: __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_1, ccr); break;
        case TIM_CHANNEL_2: __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_2, ccr); break;
        case TIM_CHANNEL_3: __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_3, ccr); break;
        case TIM_CHANNEL_4: __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_4, ccr); break;
        default: break;
    }
}

void Motor_Init(MotorChannel *m,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                GPIO_TypeDef *dir_port,
                uint16_t dir_pin,
                uint32_t arr)
{
    m->htim = htim;
    m->channel = channel;
    m->dir_port = dir_port;
    m->dir_pin = dir_pin;
    m->arr = arr;
    // Default: stop output
    Motor_SetDuty(m, 0.0f);
}

void Motor_SetDirection(MotorChannel *m, uint8_t forward)
{
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_SetDuty(MotorChannel *m, float duty_01)
{
    if (duty_01 < 0.0f) duty_01 = 0.0f;
    if (duty_01 > 1.0f) duty_01 = 1.0f;
    uint32_t ccr = (uint32_t)((m->arr + 1) * duty_01);
    if (ccr > m->arr) ccr = m->arr;
    set_compare(m, ccr);
}

void Motor_Start(MotorChannel *m)
{
    HAL_TIM_PWM_Start(m->htim, m->channel);
}

void Motor_Stop(MotorChannel *m)
{
    HAL_TIM_PWM_Stop(m->htim, m->channel);
    Motor_SetDuty(m, 0.0f);
}

