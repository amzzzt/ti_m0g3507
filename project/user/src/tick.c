/**
 * tick.c — 系统 1ms 节拍
 */
#include "zf_driver_pit.h"
#include "tick.h"
#include "zf_device_key.h"
#include "servo.h"
#include "track.h"

static volatile uint32_t g_tick_ms;

static void tick_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    g_tick_ms++;
    key_scanner();
    servo_sweep();

    // 每 2ms 采样灰度
    static uint8_t div = 0;
    if (++div >= 2) { div = 0; track_sample(); }
}

void tick_init(void) { g_tick_ms = 0; pit_ms_init(PIT_TIM_A0, 1, tick_callback, NULL); }
uint32_t tick_get(void) { return g_tick_ms; }
