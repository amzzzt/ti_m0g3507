/**
 * encoder.h — AB 相编码器 + 速度滤波
 *
 *   左: A=B3(INT)  B=B16(DIR)
 *   右: A=B15(INT) B=B13(DIR)
 */
#ifndef _encoder_h_
#define _encoder_h_

#include <stdint.h>

void    encoder_init(void);
int32_t encoder_left_get(void);         // 原始累加计数值
int32_t encoder_right_get(void);
void    encoder_clear(void);

float   encoder_left_speed(void);       // 滤波后速度 (脉冲/秒)
float   encoder_right_speed(void);

#endif
