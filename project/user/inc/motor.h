/**
 * motor.h — TB6612 双电机驱动
 *
 *   左电机: AIN1=A13  AIN2=B27  PWMA=A26 (TIMG7 CH0)
 *   右电机: BIN1=A0   BIN2=A1   PWMB=A27 (TIMG7 CH1)
 *   STBY:  A29
 */
#ifndef _motor_h_
#define _motor_h_

#include <stdint.h>

void motor_init(void);
void motor_left(int16_t speed);
void motor_right(int16_t speed);
void motor_stop(void);

/* 速度闭环 (motor+encoder+pid) */
void    motor_control_init(void);
void    motor_control_set_pwm_ramp_ms(int ms);
void    motor_control_update(int16_t tgt_l, int16_t tgt_r);
int16_t motor_control_left_speed(void);
int16_t motor_control_right_speed(void);

#endif
