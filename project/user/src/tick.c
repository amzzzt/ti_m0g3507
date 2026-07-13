/**
 * tick.c — 系统 1ms 节拍
 */
#include "zf_driver_pit.h"
#include "tick.h"
#include "zf_device_key.h"
#include "servo.h"

static volatile uint32_t g_tick_ms;
static volatile uint8_t  g_ctrl_flag = 0;   // 10ms 控制标志

static void tick_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    g_tick_ms++;
    key_scanner();
    servo_sweep();

    static uint8_t d10 = 0;
    if (++d10 >= 10) { d10 = 0; g_ctrl_flag = 1; }
}

uint8_t tick_ctrl_ready(void) { return g_ctrl_flag; }
void    tick_ctrl_clear(void) { g_ctrl_flag = 0; }

void tick_init(void) { g_tick_ms = 0; pit_ms_init(PIT_TIM_A0, 1, tick_callback, NULL); }
uint32_t tick_get(void) { return g_tick_ms; }
