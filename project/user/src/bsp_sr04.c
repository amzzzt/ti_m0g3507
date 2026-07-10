/**
 * bsp_sr04.c — HC-SR04, 完整滤波系统
 * 中值滤波 → 尖峰剔除 → 一阶低通平滑
 */
#include "bsp_sr04.h"
#include "zf_common_headfile.h"
#include "zf_driver_exti.h"

#define TRIG_PIN    A9
#define ECHO_PIN    A8
#define WIN_SIZE    5              // 滑动窗口, 越小越灵敏

static void delay_us(uint32_t us) { delay_cycles(us * 80); }

volatile float sr04_measure = 0;
static volatile uint32_t ms_count;
static volatile uint8_t  done;
static volatile uint8_t  new_data;

// ---- 滤波状态 ----
static float  window[WIN_SIZE];
static uint8_t w_idx;
static uint8_t w_cnt;
static float  smoothed;

// 迟滞: 进范围 5~50, 出范围 3~55 (放宽边界防抖动)
static uint8_t out_cnt;     // 连续超限次数

// 中值滤波: 排序 → 去头尾 1 个 → 中间平均
static float median_filter(float *buf, uint8_t n)
{
    float sorted[WIN_SIZE];
    uint8_t i, j;
    for (i = 0; i < n; i++) sorted[i] = buf[i];
    for (i = 0; i < n - 1; i++)
        for (j = i + 1; j < n; j++)
            if (sorted[i] > sorted[j]) {
                float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
            }
    // 只去 1 个最小 + 1 个最大
    if (n < 3) return sorted[n / 2];
    float sum = 0;
    for (i = 1; i < n - 1; i++) sum += sorted[i];
    return sum / (float)(n - 2);
}

// 主滤波: 中值 → 跳变检测 → 轻量低通
static float filter_process(float raw)
{
    window[w_idx] = raw;
    w_idx = (w_idx + 1) % WIN_SIZE;
    if (w_cnt < WIN_SIZE) w_cnt++;

    if (w_cnt < 2) { smoothed = raw; return raw; }

    float med = median_filter(window, w_cnt);

    // 跳变检测: 变化 > 30% 直接跳, 不用平滑
    if (smoothed > 0.5f) {
        float diff = (med > smoothed) ? (med - smoothed) : (smoothed - med);
        if (diff / smoothed > 0.3f) {
            smoothed = med;
            return med;
        }
    }

    // 轻量低通 α=0.6
    smoothed = 0.6f * med + 0.4f * smoothed;
    return smoothed;
}

// ---- EXTI 回调 ----
static void echo_isr(uint32 trigger, void *ptr)
{
    (void)trigger; (void)ptr;
    if (gpio_get_level(ECHO_PIN)) {
        DL_TimerG_stopCounter(TIMG0);
        DL_TimerG_setTimerCount(TIMG0, 0);
        ms_count = 0;
        DL_TimerG_startCounter(TIMG0);
        done = 0;
    } else {
        DL_TimerG_stopCounter(TIMG0);
        uint32_t us = ms_count * 1000 + DL_TimerG_getTimerCount(TIMG0) / 40;
        float raw = (float)us / 58.0f;

        // 迟滞: 有效 5~50, 灰区 48~55, 死区需连超 5 次
        if (raw >= 5.0f && raw <= 50.0f) {
            out_cnt = 0;
            sr04_measure = filter_process(raw);
        } else if (raw < 3.0f || raw > 55.0f) {
            if (++out_cnt >= 5) {
                sr04_measure = 0;
                out_cnt = 5;
            }
        }
        new_data = 1;
        done = 1;
    }
}

// ---- TIMG0 溢出回调 ----
static void timer_isr(uint32 e, void *p) { (void)e; (void)p; ms_count++; }

void sr04_init(void)
{
    gpio_init(TRIG_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    exti_init(ECHO_PIN, EXTI_TRIGGER_BOTH, echo_isr, NULL);
    pit_us_init(PIT_TIM_G0, 1000, timer_isr, NULL);

    // 滤波状态初始化
    w_cnt = 0; w_idx = 0; smoothed = 0;
}

void sr04_trigger(void)
{
    done = 0;
    gpio_low(TRIG_PIN);
    delay_us(10);
    gpio_high(TRIG_PIN);
    delay_us(15);
    gpio_low(TRIG_PIN);
}

uint8_t sr04_ready(void)   { return done; }
float   sr04_read(void)    { return sr04_measure; }
uint8_t sr04_new_data(void) { uint8_t t = new_data; new_data = 0; return t; }
