#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prev_error;
    float derivative;

    float out_min;
    float out_max;

    // For telemetry
    float last_error;
    float last_output;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#endif // PID_H
