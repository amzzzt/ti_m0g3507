/**
 * turn.h — 直角转弯控制
 *
 *   流程: 短直走(车头过弯) → 步进差速转(每次~30°) → 灰度查线停止
 *   调用前需先 motor_init()
 */
#ifndef _turn_h_
#define _turn_h_

void turn_left(void);    /* 左转90° */
void turn_right(void);   /* 右转90° (预留) */

#endif
