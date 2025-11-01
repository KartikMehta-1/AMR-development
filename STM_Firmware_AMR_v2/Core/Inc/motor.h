// Simple motor driver helper for Cytron MDD20A style PWM/DIR
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;           // e.g., TIM_CHANNEL_1
    GPIO_TypeDef *dir_port;     // e.g., GPIOB
    uint16_t dir_pin;           // e.g., GPIO_PIN_4
    uint32_t arr;               // auto-reload value for duty scaling
} MotorChannel;

void Motor_Init(MotorChannel *m,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                GPIO_TypeDef *dir_port,
                uint16_t dir_pin,
                uint32_t arr);

void Motor_SetDirection(MotorChannel *m, uint8_t forward);
void Motor_SetDuty(MotorChannel *m, float duty_01);
void Motor_Start(MotorChannel *m);
void Motor_Stop(MotorChannel *m);

#endif // MOTOR_H

