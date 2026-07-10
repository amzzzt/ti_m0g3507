/**
 * bsp_sr04.c — HC-SR04, 参考LCKFB方案
 * TIMG0: 40MHz/40=1MHz, 1tick=1us
 * ECHO: exti 双边沿 + 读引脚电平判断升降
 */
#include "bsp_sr04.h"
#include "zf_common_headfile.h"
#include "zf_driver_exti.h"

#define TRIG_PIN    A9
#define ECHO_PIN    A8

static void delay_us(uint32_t us) { delay_cycles(us * 80); }

volatile float sr04_measure = 0;
static volatile uint32_t ms_count;
static volatile uint8_t  done;

static void echo_isr(uint32 trigger, void *ptr)
{
    (void)trigger; (void)ptr;
    if (gpio_get_level(ECHO_PIN)) {   // 高电平 = 上升沿
        DL_TimerG_stopCounter(TIMG0);
        DL_TimerG_setTimerCount(TIMG0, 0);
        ms_count = 0;
        DL_TimerG_startCounter(TIMG0);
        done = 0;
    } else {                           // 低电平 = 下降沿
        DL_TimerG_stopCounter(TIMG0);
        uint32_t us = ms_count * 1000 + DL_TimerG_getTimerCount(TIMG0) / 40;
        if (us > 120 && us < 25000) {   // 2cm~430cm 有效
            sr04_measure = (float)us / 58.0f;
            done = 1;
        }
    }
}

// TIMG0 1ms 回调 (逐飞 pit 框架)
static void timer_isr(uint32 e, void *p) { (void)e; (void)p; ms_count++; }

void sr04_init(void)
{
    gpio_init(TRIG_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    exti_init(ECHO_PIN, EXTI_TRIGGER_BOTH, echo_isr, NULL);
    pit_us_init(PIT_TIM_G0, 1000, timer_isr, NULL);    // 40MHz, 40000tick/ms
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

uint8_t sr04_ready(void) { return done; }
float   sr04_read(void)  { return sr04_measure; }
