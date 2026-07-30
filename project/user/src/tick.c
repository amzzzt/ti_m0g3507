/**
 * tick.c — 系统 1ms 节拍
 *
 *   SysTick: 1ms ISR → sys_tick_ms++ (计时, 在所有 init 之后启动)
 *   TIMA0:   1ms ISR → key_scanner + track_read_all (仅扫描)
 */
#include "zf_driver_pit.h"
#include "tick.h"
#include "zf_device_key.h"
#include "track.h"
#include "mode_line.h"

static volatile uint32_t sys_tick_ms;
static volatile uint8_t  g_ctrl_flag = 0;   // 10ms 控制标志

/* SysTick 1ms 中断: 只计时 */
void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* TIMA0 1ms 中断: 只扫描, 不管计时 */
static void tick_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
    key_scanner();
    track_read_all();           /* 8路灰度 1ms刷新 */
    mode_line_stop_isr();       /* 用原始值快速检测停车线 */

    static uint8_t d10 = 0;
    if (++d10 >= 10) { d10 = 0; g_ctrl_flag = 1; }
}

uint8_t tick_ctrl_ready(void) { return g_ctrl_flag; }
void    tick_ctrl_clear(void) { g_ctrl_flag = 0; }

/* 初始化: 只启动 TIMA0 扫描, 不启动 SysTick */
void tick_init(void)
{
    sys_tick_ms = 0;
    pit_ms_init(PIT_TIM_A0, 1, tick_callback, NULL);
}

/* 启动 SysTick 1ms 计时 (在所有设备 init 之后调用, 避免 system_delay_ms 破坏) */
void tick_start(void)
{
    sys_tick_ms = 0;
    SysTick->LOAD = 80000000 / 1000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

uint32_t tick_get(void) { return sys_tick_ms; }
