/**
 * pid.c — PID 算法
 */
#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd, float max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral = 0;
    p->prev_err = 0;
    p->out_max  = max;
}

float pid_compute(pid_t *p, float target, float actual)
{
    float err   = target - actual;
    float deriv = err - p->prev_err;

    p->integral += err;
    p->prev_err  = err;

    float out = p->kp * err + p->ki * p->integral + p->kd * deriv;

    // 输出限幅
    if (out >  p->out_max) { out =  p->out_max; p->integral -= err; }
    if (out < -p->out_max) { out = -p->out_max; p->integral -= err; }

    return out;
}
