/**
 * encoder.c — AB 相编码器: EXTI 计数 + 低通滤波
 *
 *   左: A=B3  B=B16    右: A=B15  B=B13
 *   A 相上升沿中断, 读 B 相判方向
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

// ---- 速度: 低通滤波 ----
#define ALPHA  0.15f

typedef struct {
    int32_t last_cnt;
    float   val;
} spd_t;
static spd_t sl, sr;

void encoder_filter_reset(void)
{
    sl.last_cnt = l_cnt; sl.val = 0;
    sr.last_cnt = r_cnt; sr.val = 0;
}

// 每 10ms 调一次
static void _update(spd_t *s, int32_t cur, int sign)
{
    int32_t d = (cur - s->last_cnt) * sign;
    s->last_cnt = cur;

    // 异常值: 10ms 内不可能 >100 脉冲
    if (d > 100 || d < -100) return;

    float raw = (float)d * 100.0f;            // → pps

    // 变化率限制: 每次最多跳 ±30 pps, 钳住偶发大毛刺
    float diff = raw - s->val;
    if (diff >  30.0f) raw = s->val + 30.0f;
    if (diff < -30.0f) raw = s->val - 30.0f;

    s->val = ALPHA * raw + (1.0f - ALPHA) * s->val;
}

void encoder_update(void)
{
    _update(&sl, l_cnt, -1);
    _update(&sr, r_cnt,  1);
}

float encoder_left_speed(void)  { return sl.val; }
float encoder_right_speed(void) { return sr.val; }
