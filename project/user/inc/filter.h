/**
 * filter.h — 泰山派dx/dy专用滤波 (低通+速度外推)
 */
#ifndef _filter_h_
#define _filter_h_

#include <stdint.h>

void filter_init(void);
void filter_update(int16_t raw_x, int16_t raw_y, uint32_t tick_ms);
void filter_reset(int16_t x, int16_t y);   /* 跳跃复位: 绕过平滑直设目标 */
int16_t filter_x(void);
int16_t filter_y(void);
uint8_t filter_active(void);     // 1=有有效目标, 0=目标丢失>8帧

#endif
