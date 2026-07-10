#ifndef _radar_h_
#define _radar_h_
#include <stdint.h>

#define RADAR_CX    80      // 圆心 X
#define RADAR_CY    105     // 圆心 Y
#define RADAR_R     72      // 半径 (80±72=8~152)
#define RADAR_SCALE 1.5f    // 1.5 px/cm → 最大 40cm

void radar_draw_base(void);
void radar_draw_point(uint8_t deg, float dist_cm);
void radar_draw_scanline(uint8_t deg);   // 移动高亮扫描线

#endif
