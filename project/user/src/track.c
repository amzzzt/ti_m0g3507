/**
 * track.c — 5 路灰度寻迹: 直接读取 + 加权平均 + 丢线保持
 *
 *   左→右: A24 B24 A22 A15 A17
 *   权重:  -2  -1   0   1   2
 *   LOW=黑线, 全白时保持上次偏差
 */
#include "zf_driver_gpio.h"
#include "track.h"

static const gpio_pin_enum pins[5] = {A24, B24, A22, A15, A17};
static const int w[5] = {-2, -1, 0, 1, 2};
static int    last_raw = 0;
static float  dev_f    = 0;
#define ALPHA  0.3f

void track_init(void)
{
    for (int i = 0; i < 5; i++)
        gpio_init(pins[i], GPI, 0, GPI_PULL_UP);
}

int track_deviation(void)
{
    int sum_w = 0, sum_n = 0;

    for (int i = 0; i < 5; i++) {
        if (!gpio_get_level(pins[i])) {   // LOW = 黑线
            sum_w += w[i];
            sum_n++;
        }
    }

    int raw;
    if (sum_n > 0) {
        raw = sum_w * 100 / sum_n;
        last_raw = raw;
    } else {
        raw = last_raw;   // 全白: 保持上次
    }

    // 低通: 滤除路面颠簸毛刺, α=0.3 约 30ms 响应
    dev_f = ALPHA * (float)raw + (1.0f - ALPHA) * dev_f;
    return (int)dev_f;
}
