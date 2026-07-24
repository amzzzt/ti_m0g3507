/**
 * track.c — 8路灰度 (4线多路复用)
 *
 * AD0=A27  AD1=A25  AD2=B25  OUT=B20
 * 每1ms中断全读, 主循环取数组
 */
#include "zf_driver_gpio.h"
#include "track.h"

#define AD0  A27
#define AD1  A25
#define AD2  B25
#define OUT  B20

static volatile uint8_t g_val[8];    /* 中断写, 主循环读 */
static const int w[8] = {-7,-5,-3,-1,1,3,5,7};  /* 8路权重 */

static void _select(uint8_t ch) {
    gpio_set(AD0, (ch>>0)&1);
    gpio_set(AD1, (ch>>1)&1);
    gpio_set(AD2, (ch>>2)&1);
}

void track_init(void) {
    gpio_init(AD0, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AD2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(OUT, GPI, 0, GPI_PULL_UP);
    for (int i = 0; i < 8; i++) g_val[i] = 1;
}

void track_read_all(void) {
    for (uint8_t i = 0; i < 8; i++) {
        _select(i);
        system_delay_us(50);
        g_val[i] = gpio_get_level(OUT);  /* 0=黑 1=白 */
    }
}

int track_value(uint8_t ch) {
    if (ch > 7) return 1;
    return (int)g_val[ch];
}

int track_deviation(void) {
    int sum_w = 0, sum_n = 0;
    static int last_raw = 0, lost_cnt = 0;
    static float dev_f = 0;

    for (int i = 0; i < 8; i++) {
        if (g_val[i] == 0) {  /* LOW=黑线 */
            sum_w += w[i];
            sum_n++;
        }
    }

    int raw;
    if (sum_n > 0) {
        raw = sum_w * 60 / sum_n;  /* 归一化到~±400 */
        last_raw = raw;
        lost_cnt = 0;
    } else {
        if (lost_cnt < 20) lost_cnt++;
        int sign = (last_raw >= 0) ? 1 : -1;
        int mag  = (last_raw >= 0) ? last_raw : -last_raw;
        int boost = sign * (mag + lost_cnt * 15);
        if (boost >  400) boost =  400;
        if (boost < -400) boost = -400;
        raw = boost;
    }

    int cur = (int)dev_f;
    if (raw - cur >  80) raw = cur + 80;
    if (raw - cur < -80) raw = cur - 80;

    float alpha = 0.3f;
    int mag = (raw >= 0) ? raw : -raw;
    if ((mag < 50 && cur > 50) || (mag < 50 && cur < -50)) alpha = 0.6f;
    dev_f = alpha * (float)raw + (1.0f - alpha) * dev_f;
    return (int)dev_f;
}
