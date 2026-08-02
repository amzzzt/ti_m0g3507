/**
 * track.h — 8路灰度 (4线多路复用, 沿用原5路引脚)
 *
 * AD0=A24  AD1=A25  AD2=A22  OUT=A15
 */
#ifndef _track_h_
#define _track_h_
#include <stdint.h>

void track_init(void);
void track_reset(void);              /* 重置偏差状态 */
void track_read_all(void);           /* 读8路, 约400us */
int  track_deviation(void);          /* 加权偏差 -400~+400 */
int  track_value(uint8_t ch);        /* 读单路值 1=黑 0=白 (2帧滤波) */
int  track_value_raw(uint8_t ch);    /* 读单路值 1=黑 0=白 (不过滤) */

extern int dbg_sum_n, dbg_raw;       /* 调试 */

#endif
