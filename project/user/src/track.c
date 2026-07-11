/**
 * track.c — 5 路灰度寻迹 + 中值滤波 + 位置计算
 *
 *   左→右: A24 B24 A22 A15 A17
 *   TIMA0 每2ms 采样一次, 5 次中值滤波
 */
#include "zf_driver_gpio.h"
#include "track.h"

static const gpio_pin_enum pins[5] = {A24, B24, A22, A15, A17};

#define BUF_SIZE  5       // 滤波窗口

static uint8_t buf[BUF_SIZE];   // 最近 5 次原始值
static uint8_t buf_idx;
static uint8_t buf_cnt;
static uint8_t filtered;        // 滤波后稳定值 (5bit)

void track_init(void)
{
    for (int i = 0; i < 5; i++)
        gpio_init(pins[i], GPI, 0, GPI_PULL_UP);
}

// ---- TIMA0 每 2ms 调用一次 ----
void track_sample(void)
{
    // 读 5 路原始值: LOW=黑线→bit置1
    uint8_t raw = 0;
    for (int i = 0; i < 5; i++)
        if (!gpio_get_level(pins[i]))
            raw |= (1 << (4 - i));

    buf[buf_idx] = raw;
    buf_idx = (buf_idx + 1) % BUF_SIZE;
    if (buf_cnt < BUF_SIZE) buf_cnt++;

    // 中值滤波: 统计每个 bit 在窗口中出现次数, 过半则置 1
    if (buf_cnt < BUF_SIZE) { filtered = raw; return; }

    uint8_t result = 0;
    for (int bit = 0; bit < 5; bit++) {
        int cnt = 0;
        for (int i = 0; i < BUF_SIZE; i++)
            if (buf[i] & (1 << bit)) cnt++;
        if (cnt >= 3) result |= (1 << bit);   // 5 次中 ≥3 次 → 有效
    }
    filtered = result;
}

// ---- 读取 ----
uint8_t track_filtered(void) { return filtered; }

int track_bit(uint8_t idx)
{
    if (idx > 4) return 0;
    return (filtered >> (4 - idx)) & 1;
}

// 加权位置: -200 ~ +200 (放大 100 倍, 避免小数)
// 权值: A24=-2 B24=-1 A22=0 A15=+1 A17=+2
int track_position(void)
{
    int sum_w = 0, sum_n = 0;
    int w[5] = {-2, -1, 0, 1, 2};
    for (int i = 0; i < 5; i++) {
        if (track_bit(i)) {
            sum_w += w[i];
            sum_n++;
        }
    }
    if (sum_n == 0) return 0;
    return sum_w * 100 / sum_n;   // 放大 100 倍
}
