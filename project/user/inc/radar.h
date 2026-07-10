#ifndef _radar_h_
#define _radar_h_
#include <stdint.h>

#define RADAR_CX    80      // 圆心 X
#define RADAR_CY    105     // 圆心 Y
#define RADAR_R     72      // 半径 (80±72=8~152)
#define RADAR_SCALE 1.5f    // 1.5 px/cm → 最大 40cm

void radar_draw_base(void);
void radar_draw_point(uint8_t deg, float dist_cm);
void radar_draw_scanline(uint8_t deg);
void radar_scanline_reset(void);

void radar_add_dot(uint8_t deg, float cm);   // 记录测距点
void radar_clear_dots(void);                  // 清空 (扫完 180°)
void radar_draw_dots(void);                   // 绘制所有点

#endif
