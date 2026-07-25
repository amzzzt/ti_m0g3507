/**
 * motor.h — TB6612 双电机驱动
 *
 *   左电机: AIN1=A13  AIN2=B8   PWMA=A26 (TIMG7 CH0)
 *   右电机: BIN1=A0   BIN2=A1   PWMB=A27 (TIMG7 CH1)
 *   STBY:  A29
 */
#ifndef _motor_h_
#define _motor_h_

#include <stdint.h>

void motor_init(void);
void motor_left(int16_t speed);     // -1000~1000
void motor_right(int16_t speed);    // -1000~1000
void motor_stop(void);

#endif
