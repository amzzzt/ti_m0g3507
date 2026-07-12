/**
 * track.c — 5 路灰度寻迹 + 中值滤波 + 偏差低通
 *
 *   左→右: A24 B24 A22 A15 A17
 *   TIMA0 每 2ms 采样, 5 窗口中值滤波, 偏差低通
 */
#include "zf_driver_gpio.h"
#include "track.h"

static const gpio_pin_enum pins[5] = {A24, B24, A22, A15, A17};

#define BUF_SIZE  5
static uint8_t buf[BUF_SIZE], buf_idx, buf_cnt, filtered;

void track_init(void)
{
    for (int i = 0; i < 5; i++)
        gpio_init(pins[i], GPI, 0, GPI_PULL_UP);
}

// TIMA0 每 2ms 调用
void track_sample(void)
{
    uint8_t raw = 0;
    for (int i = 0; i < 5; i++)
        if (!gpio_get_level(pins[i]))
            raw |= (1 << (4 - i));

    buf[buf_idx] = raw;
    buf_idx = (buf_idx + 1) % BUF_SIZE;
    if (buf_cnt < BUF_SIZE) buf_cnt++;

    if (buf_cnt < BUF_SIZE) { filtered = raw; return; }

    uint8_t r = 0;
    for (int bit = 0; bit < 5; bit++) {
        int cnt = 0;
        for (int i = 0; i < BUF_SIZE; i++)
            if (buf[i] & (1 << bit)) cnt++;
        if (cnt >= 3) r |= (1 << bit);
    }
    filtered = r;
}

uint8_t track_filtered(void) { return filtered; }

int track_bit(uint8_t idx)
{
    if (idx > 4) return 0;
    return (filtered >> (4 - idx)) & 1;
}

// 加权位置: -200~+200
static int track_raw_position(void)
{
    int sum_w = 0, sum_n = 0;
    int w[5] = {-2, -1, 0, 1, 2};
    for (int i = 0; i < 5; i++) {
        if (track_bit(i)) { sum_w += w[i]; sum_n++; }
    }
    if (sum_n == 0) return 0;
    return sum_w * 100 / sum_n;
}

static float dev_f = 0;
#define DEV_ALPHA  0.1f

int track_deviation(void)
{
    int raw = track_raw_position();
    dev_f = DEV_ALPHA * (float)raw + (1.0f - DEV_ALPHA) * dev_f;
    return (int)dev_f;
}
