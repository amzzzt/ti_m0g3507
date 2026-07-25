/**
 * dc_ctrl.h — 直流电机速度闭环 (编码器+PID+TB6612)
 *
 *   encoder.c 读速度 → PID算误差 → motor.c 驱PWM
 *   左右独立双PID, 10ms控制周期
 */
#ifndef _dc_ctrl_h_
#define _dc_ctrl_h_
#include <stdint.h>

void dc_ctrl_init(void);
void dc_ctrl_set_left(float target_pps);    /* 设左轮目标速度 */
void dc_ctrl_set_right(float target_pps);   /* 设右轮目标速度 */
void dc_ctrl_update(void);                  /* 每10ms调: PID×2 */
void dc_ctrl_stop(void);

float dc_ctrl_left_speed(void);
float dc_ctrl_right_speed(void);
int16_t dc_ctrl_left_out(void);             /* 当前PWM输出 */
int16_t dc_ctrl_right_out(void);

#endif
