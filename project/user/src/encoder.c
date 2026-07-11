/**
 * encoder.c — AB 相编码器 + 简单低通
 *
 *   左: A=B3(上升沿) B=B16(方向)  右: A=B15(上升沿) B=B13(方向)
 */
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "tick.h"
#include "encoder.h"

#define LEFT_A   B3
#define LEFT_B   B16
#define RIGHT_A  B15
#define RIGHT_B  B13

static volatile int32_t left_cnt  = 0;
static volatile int32_t right_cnt = 0;

static void left_isr(uint32_t event, void *ptr)
{
    (void)event; (void)ptr;
    if (gpio_get_level(LEFT_B)) left_cnt++;
    else                        left_cnt--;
}
static void right_isr(uint32_t event, void *ptr)
{
    (void)event; (void)ptr;
    if (gpio_get_level(RIGHT_B)) right_cnt++;
    else                         right_cnt--;
}

void encoder_init(void)
{
    gpio_init(LEFT_B,  GPI, 0, GPI_PULL_UP);
    gpio_init(RIGHT_B, GPI, 0, GPI_PULL_UP);
    exti_init(LEFT_A,  EXTI_TRIGGER_RISING, left_isr,  NULL);
    exti_init(RIGHT_A, EXTI_TRIGGER_RISING, right_isr, NULL);
}

int32_t encoder_left_get(void)  { return left_cnt; }
int32_t encoder_right_get(void) { return right_cnt; }
void    encoder_clear(void)     { left_cnt = 0; right_cnt = 0; }

// ===================== 速度 (100ms 窗口 + 低通) =====================
#define E_ALPHA  0.7f

typedef struct {
    int32_t  last_cnt;
    uint32_t last_ms;
    float    val;          // 滤波后速度
} e_filt_t;

static e_filt_t el, er;

static float update_speed(e_filt_t *f, int32_t cur, int32_t sign)
{
    uint32_t now = tick_get();
    uint32_t dt  = now - f->last_ms;
    if (dt < 20) return f->val;     // 不满 20ms 不动

    int32_t diff = cur - f->last_cnt;
    f->last_cnt = cur;
    f->last_ms  = now;

    float raw = (float)(diff * sign) * 1000.0f / (float)dt;

    if (f->val == 0.0f && raw != 0.0f)
        f->val = raw;
    else
        f->val = 0.3f * raw + 0.7f * f->val;   // α=0.3 更强平滑

    return f->val;
}

float encoder_left_speed(void)
{
    return update_speed(&el, encoder_left_get(), -1);   // 左反向
}

float encoder_right_speed(void)
{
    return update_speed(&er, encoder_right_get(), 1);
}
