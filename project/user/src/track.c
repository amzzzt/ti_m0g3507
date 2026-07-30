/**
 * track.c — 8路灰度 (4线多路复用, 沿用原5路引脚)
 *
 * AD0=A24  AD1=A25  AD2=A22  OUT=A15
 * 主循环调 track_read_all, 约400us
 */
#include "zf_driver_gpio.h"
#include "tick.h"
#include "track.h"

#define AD0  A24
#define AD1  A25
#define AD2  A22
#define OUT  A15

static volatile uint8_t g_val[8];
static uint8_t hist[8][2];   /* [0]=本帧, [1]=上帧 */
int dbg_sum_n, dbg_raw;      /* 调试: 最后计算的 sum_n 和 raw */

static void _select(uint8_t ch) {
    ((ch>>0)&1) ? gpio_high(AD0) : gpio_low(AD0);
    ((ch>>1)&1) ? gpio_high(AD1) : gpio_low(AD1);
    ((ch>>2)&1) ? gpio_high(AD2) : gpio_low(AD2);
}

void track_init(void) {
    gpio_init(AD0, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(OUT, GPI, 0, GPI_PULL_UP);
    for (int i = 0; i < 8; i++) {
        g_val[i] = 0;
        hist[i][0] = hist[i][1] = 0;
    }
}

void track_read_all(void) {
    uint8_t raw[8];
    for (uint8_t i = 0; i < 8; i++) {
        _select(i);
        __NOP(); __NOP();  /* 74HC4051 切换 ~20ns, 2 NOP≈25ns 足够 */
        raw[i] = !gpio_get_level(OUT);
    }
    /* 2帧确认: 连续2次一致才更新, 抗噪 */
    for (uint8_t i = 0; i < 8; i++) {
        if (raw[i] == hist[i][0])
            g_val[i] = raw[i];
        hist[i][1] = hist[i][0];
        hist[i][0] = raw[i];
    }
}

int track_value(uint8_t ch) {
    if (ch > 7) return 0;
    return (int)g_val[ch];   /* 1=黑 0=白 (2帧确认) */
}

int track_value_raw(uint8_t ch) {
    if (ch > 7) return 1;
    return (int)hist[ch][0];   /* 不经过2帧滤波, 0=黑 1=白 */
}

int track_deviation(void) {
    static int last_raw = 0;
    static uint32_t lost_since = 0;
    static float dev_f = 0;

    /* 少数有效: 0=线, 1=背景 */
    int sum_p = 0, sum_n = 0;
    for (int i = 0; i < 8; i++) {
        if (g_val[i] == 1) sum_n++;
    }
    int target = (sum_n > 4) ? 0 : 1;   /* 少数色是线 */

    sum_p = 0; sum_n = 0;
    for (int i = 0; i < 8; i++) {
        if (g_val[i] == target) { sum_p += i; sum_n++; }
    }

    int raw;
    if (sum_n > 0 && sum_n < 8) {
        float center = (float)sum_p / (float)sum_n;
        raw = (int)((center - 3.5f) * 95.0f);
        if (raw >  340) raw =  340;
        if (raw < -340) raw = -340;
        last_raw = raw;
        lost_since = tick_get();
    } else {
        /* 丢线: 保持旧值, >300ms归零 */
        if (tick_get() - lost_since > 300) { raw = 0; last_raw = 0; }
        else raw = last_raw;
    }

    dbg_sum_n = sum_n;
    dbg_raw   = raw;

    /* 低通滤波 (更丝滑) */
    dev_f = 0.30f * (float)raw + 0.70f * dev_f;
    return (int)dev_f;
}
