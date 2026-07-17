/**
 * filter.h — 偏差滤波 (低通+速度外推)
 */
#ifndef _filter_h_
#define _filter_h_

#include <stdint.h>

void filter_init(void);
void filter_update(int16_t raw_x, int16_t raw_y, uint32_t tick_ms);
int16_t filter_x(void);
int16_t filter_y(void);
uint8_t filter_active(void);     // 1=有有效目标, 0=目标丢失>8帧

#endif
