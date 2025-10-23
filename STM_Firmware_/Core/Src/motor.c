#include "motor.h"

void Motor_Init(Motor_t *m)
{
    // start PWM output for the timer/channel
    if (m && m->htim) {
        HAL_TIM_PWM_Start(m->htim, m->channel);
        Motor_Stop(m);
    }
}

void Motor_SetPWM(Motor_t *m, int32_t pwm)
{
    if (!m || !m->htim) return;

    uint32_t duty = 0;
    if (pwm >= 0) {
        HAL_GPIO_WritePin(m->dir_port_a, m->dir_pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_port_b, m->dir_pin_b, GPIO_PIN_RESET);
        duty = (uint32_t)pwm;
    } else {
        HAL_GPIO_WritePin(m->dir_port_a, m->dir_pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_port_b, m->dir_pin_b, GPIO_PIN_SET);
        duty = (uint32_t)(-pwm);
    }
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, duty);
}

void Motor_Stop(Motor_t *m)
{
    if (!m || !m->htim) return;
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
}
