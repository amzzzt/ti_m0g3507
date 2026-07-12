/**
 * encoder.h — AB 相编码器
 */
#ifndef _encoder_h_
#define _encoder_h_

#include <stdint.h>

void    encoder_init(void);
int32_t encoder_left_get(void);
int32_t encoder_right_get(void);
void    encoder_clear(void);

void    encoder_filter_reset(void);
void    encoder_update(void);          // 每 10ms 一次
float   encoder_left_speed(void);
float   encoder_right_speed(void);

#endif
