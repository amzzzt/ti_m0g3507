/**
 * track.h — 5 路灰度寻迹
 */
#ifndef _track_h_
#define _track_h_

#include <stdint.h>

void    track_init(void);
void    track_sample(void);
uint8_t track_filtered(void);
int     track_bit(uint8_t idx);
int     track_deviation(void);       // 低通后偏差 -200~+200

#endif
