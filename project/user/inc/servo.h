/**
 * servo.h — MG996R 舵机驱动 (基于 tick 节拍 + TIMG8 硬件 PWM)
 *
 * TIMG8 PWM → PB7:  40MHz /8 /50 = 100kHz, 周期 2000 = 20ms (50Hz)
 * 脉宽: 50(0.5ms/0°) ~ 250(2.5ms/180°)
 * 扫动计时: tick_get() 每 20ms 走 1°
 */

#ifndef _servo_h_
#define _servo_h_

#include <stdint.h>

void servo_init(void);
void servo_set_angle(uint8_t deg);      // 0-180°
void servo_sweep(void);                 // 主循环调用, 每20ms自动走1°

#endif
