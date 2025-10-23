#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    pid->integral += error * dt;

    // anti-windup clamp
    if (pid->integral * pid->Ki > pid->out_max) pid->integral = pid->out_max / pid->Ki;
    if (pid->integral * pid->Ki < pid->out_min) pid->integral = pid->out_min / pid->Ki;

    float derivative = 0.0f;
    if (dt > 0.0f) derivative = (error - pid->prev_error) / dt;

    float out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->prev_error = error;
    return out;
}
