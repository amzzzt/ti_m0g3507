/**
 * filter_ball.c — 球位置滤波
 */
#include "filter_ball.h"

#define ALPHA_POS 0.85f
#define ALPHA_VEL 0.60f

static float fx, fv;

void filter_ball_init(void)
{
    fx = 0;
    fv = 0;
}

void filter_ball_feed(int16_t raw_pos, int16_t raw_vel)
{
    /* 位置范围保护 */
    if (raw_pos > 250 || raw_pos < -250) return;

    /* 速度垃圾拦截 */
    if (raw_vel > 20 || raw_vel < -20) raw_vel = 0;

    /* 位置: 死区±2 + 低通 */
    int16_t in = (raw_pos < 2 && raw_pos > -2) ? 0 : raw_pos;
    fx = ALPHA_POS * (float)in + (1.0f - ALPHA_POS) * fx;

    /* 速度: 简单低通, 归零时快速衰减 */
    if (raw_vel == 0)
        fv *= 0.3f;
    else
        fv = ALPHA_VEL * (float)raw_vel + (1.0f - ALPHA_VEL) * fv;
}

float filter_ball_pos(void) { return fx; }
float filter_ball_vel(void) { return fv; }
