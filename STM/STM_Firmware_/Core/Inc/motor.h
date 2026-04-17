#ifndef MOTOR_H
#define MOTOR_H

#include "main.h" // brings TIM_HandleTypeDef, GPIO types

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *dir_port_a;
    uint16_t dir_pin_a;
    GPIO_TypeDef *dir_port_b;
    uint16_t dir_pin_b;
} Motor_t;

void Motor_Init(Motor_t *m);
void Motor_SetPWM(Motor_t *m, int32_t pwm); // pwm >=0 forward, <0 reverse
void Motor_Stop(Motor_t *m);

#endif // MOTOR_H
