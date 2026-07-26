/**
 * turn.c — 直角转弯控制
 *
 *   流程: 短直走(车头过弯) → 步进差速转(每次~30°) → 灰度查线停止
 *   线检测: 连续2灯从边缘往内触发
 */
#include "zf_driver_gpio.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "turn.h"

/* ---------- 可调参数 ---------- */
#define FORWARD_MS      250     /* 短直走时长 ms */
#define FORWARD_SPEED   500     /* 直走速度 */

#define TURN_SPEED      3000    /* 差速转弯速度 */
#define STEP_MS         350     /* 每步时长 ~30° */
#define MAX_STEPS       4       /* 最多步数 (120°) */
#define PAUSE_MS        50      /* 步间停顿 */
/* ------------------------------ */

/* 连续2灯从边缘往内: (0,1)/(1,2) 或 (6,7)/(5,6) */
static int line_seen(int seen[8])
{
    if ((seen[0] && seen[1]) || (seen[1] && seen[2])) return 1;
    if ((seen[6] && seen[7]) || (seen[5] && seen[6])) return 1;
    return 0;
}

/* 扫描灰度, 累积 seen[] */
static void scan_seen(int seen[8])
{
    for (int i = 0; i < 8; i++)
        if (!track_value(i)) seen[i] = 1;   /* 0=黑线 */
}

/* 步进差速左转, 灰度查线 */
static void step_turn_left(int seen[8])
{
    for (int step = 0; step < MAX_STEPS; step++) {
        uint32_t start = tick_get();
        int found = 0;

        /* 转中查线 */
        while (tick_get() - start < STEP_MS) {
            motor_left(-TURN_SPEED);
            motor_right(TURN_SPEED);
            scan_seen(seen);
            if (line_seen(seen)) { found = 1; break; }
        }

        motor_stop();

        if (found) return;

        /* 停顿, 也查线 */
        uint32_t tw = tick_get();
        while (tick_get() - tw < PAUSE_MS)
            scan_seen(seen);

        if (line_seen(seen)) return;

        /* 未找到, 清空记录, 下一轮 */
        for (int i = 0; i < 8; i++) seen[i] = 0;
    }
}

/**
 * turn_left — 左转90°
 *
 *   1. 短直走 FORWARD_MS (车头过弯点)
 *   2. 步进差速左转, 灰度查线, 找到线停
 */
void turn_left(void)
{
    int seen[8] = {0};

    /* 1. 短直走 */
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < FORWARD_MS) {
        motor_left(FORWARD_SPEED);
        motor_right(FORWARD_SPEED);
    }
    motor_stop();

    /* 2. 步进差速左转 */
    step_turn_left(seen);
    motor_stop();
}

/**
 * turn_right — 右转90° (预留)
 */
void turn_right(void)
{
    /* TODO */
}
