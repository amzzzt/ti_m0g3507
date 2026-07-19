/**
 * filter.c — 偏差滤波: 低通平滑 + 零帧速度外推
 */
#include "filter.h"

#define ALPHA      0.5f         // 低通系数 (0~1, 越大响应越快)
#define V_DECAY    0.85f        // 零帧时速度衰减
#define ZERO_MAX   8            // 连续零帧上限, 超出发认为目标丢失

static float    g_fx, g_fy;     // 滤波后值
static float    g_vx, g_vy;     // 速度
static int16_t  g_lx, g_ly;     // 上一帧滤波值
static uint8_t  g_zero;         // 连续零帧计数
static uint32_t g_last_tick;

void filter_reset(int16_t x, int16_t y)
{
    g_fx = (float)x; g_fy = (float)y;
    g_vx = 0; g_vy = 0;
    g_lx = x; g_ly = y;
    g_zero = 0;
}

void filter_init(void)
{
    g_fx = 0; g_fy = 0;
    g_vx = 0; g_vy = 0;
    g_lx = 0; g_ly = 0;
    g_zero = 0;
    g_last_tick = 0;
}

void filter_update(int16_t rx, int16_t ry, uint32_t tick)
{
    float dt = (float)(tick - g_last_tick) * 0.001f;
    if (dt <= 0) dt = 0.01f;
    g_last_tick = tick;

    if (rx != 0 || ry != 0) {
        // 有效帧: 低通滤波
        g_fx = g_fx * (1.0f - ALPHA) + (float)rx * ALPHA;
        g_fy = g_fy * (1.0f - ALPHA) + (float)ry * ALPHA;
        g_vx = (g_fx - (float)g_lx) / dt;
        g_vy = (g_fy - (float)g_ly) / dt;
        g_lx = (int16_t)g_fx;
        g_ly = (int16_t)g_fy;
        g_zero = 0;
    } else if (g_zero < ZERO_MAX) {
        // 零帧: 速度外推
        g_fx += g_vx * dt;
        g_fy += g_vy * dt;
        g_lx = (int16_t)g_fx;
        g_ly = (int16_t)g_fy;
        g_zero++;
        g_vx *= V_DECAY;
        g_vy *= V_DECAY;
    } else {
        // 彻底丢失
        g_fx = 0; g_fy = 0;
        g_vx = 0; g_vy = 0;
        g_lx = 0; g_ly = 0;
    }
}

int16_t filter_x(void)     { return (int16_t)g_fx; }
int16_t filter_y(void)     { return (int16_t)g_fy; }
uint8_t filter_active(void) { return g_zero < ZERO_MAX; }
