/**
 * pid.c — PID + 死区 + 积分分离
 */
#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd, float max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral = 0; p->prev_err = 0; p->out_max = max;
}

float pid_compute(pid_t *p, float target, float actual)
{
    float err = target - actual;

    // 死区: 误差 < 3 pps 不做修正
    if (err < 3.0f && err > -3.0f) return 0.0f;

    float deriv = err - p->prev_err;

    // 积分分离: 大误差 (>300) 不积分, 小误差积分消静差
    if (err < 300.0f && err > -300.0f)
        p->integral += err;

    p->prev_err = err;

    float out = p->kp * err + p->ki * p->integral + p->kd * deriv;

    if (out >  p->out_max) { out = p->out_max; p->integral -= err; }
    if (out < -p->out_max) { out = -p->out_max; p->integral -= err; }

    return out;
}
