#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output;
    float max_integral;
    float max_output;
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd,
              float max_integral, float max_output);
float PID_Update(pid_t *pid, float target, float feedback);
void PID_Clear(pid_t *pid);

#endif
