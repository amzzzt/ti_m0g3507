/**
 * tick.c — 系统 1ms 节拍
 *
 *   SysTick: 1ms ISR → sys_tick_ms++
 *   TIMA0:   1ms ISR → key_scanner + track_read_all + stepper_tick
 */
#include "zf_driver_pit.h"
#include "tick.h"
#include "zf_device_key.h"

static volatile uint32_t sys_tick_ms;
static volatile uint8_t  g_ctrl_flag = 0;

void SysTick_Handler(void) { sys_tick_ms++; }

static void tick_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    key_scanner();

    static uint8_t d10 = 0;
    if (++d10 >= 10) { d10 = 0; g_ctrl_flag = 1; }
}

uint8_t tick_ctrl_ready(void) { return g_ctrl_flag; }
void    tick_ctrl_clear(void) { g_ctrl_flag = 0; }

void tick_init(void)  { sys_tick_ms = 0; pit_ms_init(PIT_TIM_A0, 1, tick_callback, NULL); }
void tick_start(void) { sys_tick_ms = 0; SysTick->LOAD = 79999; SysTick->VAL = 0; SysTick->CTRL = 7; }
uint32_t tick_get(void) { return sys_tick_ms; }
