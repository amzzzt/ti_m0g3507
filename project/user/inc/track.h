/**
 * track.h — 8路灰度 (4线多路复用)
 *
 * AD0=A27  AD1=A25  AD2=B25  OUT=B20
 * TIMA0 1ms中断刷新, 主循环直接读
 */
#ifndef _track_h_
#define _track_h_
#include <stdint.h>

void track_init(void);
void track_read_all(void);           /* 中断里调: 读8路 */
int  track_deviation(void);          /* 加权偏差 -400~+400 */
int  track_value(uint8_t ch);        /* 读单路值 0=LOW=黑 */

#endif
