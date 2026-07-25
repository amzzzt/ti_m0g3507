/**
 * track.h — 8路灰度 (4线多路复用, 沿用原5路引脚)
 *
 * AD0=A24  AD1=B24  AD2=A22  OUT=A15
 */
#ifndef _track_h_
#define _track_h_
#include <stdint.h>

void track_init(void);
void track_read_all(void);           /* 读8路, 约400us */
int  track_deviation(void);          /* 加权偏差 -400~+400 */
int  track_value(uint8_t ch);        /* 读单路值 0=LOW=黑 */

#endif
