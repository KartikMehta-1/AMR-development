#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->derivative = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    pid->last_error = error;  // store for telemetry

    // --- Integral term ---
    pid->integral += error * dt;

    // Anti-windup
    if (pid->integral * pid->Ki > pid->out_max)
        pid->integral = pid->out_max / pid->Ki;
    else if (pid->integral * pid->Ki < pid->out_min)
        pid->integral = pid->out_min / pid->Ki;

    // --- Derivative term ---
    if (dt > 0.0f)
        pid->derivative = (error - pid->prev_error) / dt;

    // --- PID output ---
    float out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

    // Clamp output
    if (out > pid->out_max) out = pid->out_max;
    else if (out < pid->out_min) out = pid->out_min;

    pid->prev_error = error;
    pid->last_output = out;  // store for UART telemetry

    return out;
}
