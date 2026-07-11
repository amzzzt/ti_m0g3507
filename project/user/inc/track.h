/**
 * track.h — 5 路灰度寻迹 + 中值滤波 + 位置
 *
 *   左→右: A24 B24 A22 A15 A17
 *   TIMA0 每 2ms 采样, 5 窗口中值滤波
 */
#ifndef _track_h_
#define _track_h_

#include <stdint.h>

void    track_init(void);
void    track_sample(void);         // TIMA0 每 2ms 调用
uint8_t track_filtered(void);       // 滤波后 5bit 值
int     track_bit(uint8_t idx);     // 第 idx 位 (0~4)
int     track_position(void);       // 加权位置 ×100, -200~+200

#endif
