/**
 * track.h — 5 路灰度寻迹
 */
#ifndef _track_h_
#define _track_h_

#include <stdint.h>

void track_init(void);
int  track_deviation(void);       // 偏差 -200~+200

#endif
