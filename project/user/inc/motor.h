/**
 * motor.h — TB6612 直流电机驱动
 *
 *   STBY=A29  AIN1=B23  AIN2=B27
 *   PWMA=A26 (TIMG7 CH0)  PWMB=A27 (TIMG7 CH1)
 */
#ifndef _motor_h_
#define _motor_h_

#include <stdint.h>

void motor_init(void);
void motor_set(int16_t speed);     // -1000~1000
void motor_stop(void);

#endif
