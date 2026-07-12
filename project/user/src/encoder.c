/**
 * encoder.c — AB 相编码器 + 固定窗口 + 滑动平均
 *
 *   EXTI 计数 → 每 10ms 算速度 → 4 次滑动平均 → 输出
 */
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "encoder.h"

#define LEFT_A   B3
#define LEFT_B   B16
#define RIGHT_A  B15
#define RIGHT_B  B13

static volatile int32_t l_cnt, r_cnt;

static void l_isr(uint32_t e, void *p) {
    (void)e;(void)p;
    if (gpio_get_level(LEFT_B)) l_cnt++; else l_cnt--;
}
static void r_isr(uint32_t e, void *p) {
    (void)e;(void)p;
    if (gpio_get_level(RIGHT_B)) r_cnt++; else r_cnt--;
}

void encoder_init(void)
{
    gpio_init(LEFT_B, GPI,0,GPI_PULL_UP); gpio_init(RIGHT_B, GPI,0,GPI_PULL_UP);
    exti_init(LEFT_A, EXTI_TRIGGER_RISING, l_isr, NULL);
    exti_init(RIGHT_A, EXTI_TRIGGER_RISING, r_isr, NULL);
}
int32_t encoder_left_get(void)  { return l_cnt; }
int32_t encoder_right_get(void) { return r_cnt; }
void    encoder_clear(void)     { l_cnt = r_cnt = 0; }

// ---- 速度: 固定 10ms 窗口 + 4 帧滑动平均 ----
#define MA_LEN  8

typedef struct {
    int32_t last_cnt;
    int32_t buf[MA_LEN];
    uint8_t idx, full;
    float   val;
} spd_t;
static spd_t sl, sr;

void encoder_filter_reset(void)
{
    sl.last_cnt = l_cnt; sl.idx = 0; sl.full = 0; sl.val = 0;
    sr.last_cnt = r_cnt; sr.idx = 0; sr.full = 0; sr.val = 0;
}

// 每 10ms 调一次 (TIMA0 ISR)
static void _update(spd_t *s, int32_t cur, int sign)
{
    int32_t d = (cur - s->last_cnt) * sign;
    s->last_cnt = cur;

    // 异常值过滤: 10ms 内不可能 >100 脉冲
    if (d > 100 || d < -100) return;

    s->buf[s->idx] = d;
    s->idx = (s->idx + 1) % MA_LEN;
    if (!s->full && s->idx == 0) s->full = 1;

    if (s->full) {
        int32_t sum = 0;
        for (int i = 0; i < MA_LEN; i++) sum += s->buf[i];
        s->val = (float)sum / (float)MA_LEN * 100.0f;
    } else {
        s->val = (float)d * 100.0f;
    }
}

void encoder_update(void)   // 每 10ms 调用
{
    _update(&sl, l_cnt, -1);
    _update(&sr, r_cnt,  1);
}

float encoder_left_speed(void)  { return sl.val; }
float encoder_right_speed(void) { return sr.val; }
