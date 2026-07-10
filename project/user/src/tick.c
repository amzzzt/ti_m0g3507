/**
 * tick.c — 系统 1ms 节拍 (基于逐飞库 pit_ms_init + TIMA0)
 */

#include "zf_driver_pit.h"
#include "tick.h"
#include "zf_device_key.h"

static volatile uint32_t g_tick_ms;

// ---------- TIMA0 中断回调 (isr.c 自动分发) ----------
static void tick_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    g_tick_ms++;
    key_scanner();
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
