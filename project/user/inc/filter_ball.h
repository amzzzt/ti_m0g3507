/**
 * filter_ball.h — 球位置滤波
 *
 *   位置: 死区±2 + 低通 alpha=0.85
 *   速度: 简单低通 alpha=0.60
 */
#ifndef _filter_ball_h_
#define _filter_ball_h_

#include <stdint.h>

void  filter_ball_init(void);
void  filter_ball_feed(int16_t raw_pos, int16_t raw_vel);
float filter_ball_pos(void);
float filter_ball_vel(void);

#endif
