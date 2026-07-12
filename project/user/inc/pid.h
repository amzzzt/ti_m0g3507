/**
 * pid.h — 基础 PID 控制器
 */
#ifndef _pid_h_
#define _pid_h_

#include <stdint.h>

typedef struct {
    float kp, ki, kd;       // 系数
    float integral;         // 积分累加
    float prev_err;         // 上次误差
    float out_max;          // 输出限幅
} pid_t;

void    pid_init(pid_t *p, float kp, float ki, float kd, float max);
float   pid_compute(pid_t *p, float target, float actual);

#endif
