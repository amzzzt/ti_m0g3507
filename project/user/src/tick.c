/**
 * tick.c — 系统 1ms 节拍 (基于逐飞库 pit_ms_init + TIMA0)
 */

#include "zf_driver_pit.h"
#include "tick.h"
#include "bsp_sr04.h"

static volatile uint32_t g_tick_ms;
static uint32_t next_trig = 100;
volatile uint8_t sr04_send_flag = 0;   // 主循环检查此标志发送

static void tick_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    g_tick_ms++;

    if (g_tick_ms >= next_trig) {
        next_trig = g_tick_ms + 100;
        sr04_trigger();
        sr04_send_flag = 1;
    }

    if (sr04_ready()) {
        sr04_send_flag = 1;
    }
}

// ---------- 初始化 ----------
void tick_init(void)
{
    g_tick_ms = 0;
    pit_ms_init(PIT_TIM_A0, 1, tick_callback, NULL);
}

// ---------- 获取当前 ms ----------
uint32_t tick_get(void)
{
    return g_tick_ms;
}
