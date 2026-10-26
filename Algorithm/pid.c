#include "pid.h"

static float PID_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_Init(pid_t *pid, float kp, float ki, float kd,
              float max_integral, float max_output)
{
    if (pid == 0)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
    PID_Clear(pid);
}

float PID_Update(pid_t *pid, float target, float feedback)
{
    float error;
    float derivative;

    if (pid == 0)
    {
        return 0.0f;
    }

    error = target - feedback;
    pid->integral = PID_Clamp(pid->integral + pid->ki * error,
                              pid->max_integral);
    derivative = error - pid->last_error;
    pid->last_error = error;
    pid->output = PID_Clamp(pid->kp * error + pid->integral +
                            pid->kd * derivative,
                            pid->max_output);
    return pid->output;
}

void PID_Clear(pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}
