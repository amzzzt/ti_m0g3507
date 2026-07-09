/**
 * tick.c — 系统 1ms 节拍, 基于 TIMA0 (COUNT)
 *
 * MIS 轮询 + LED 诊断: 验证停顿期间 tick 是否仍在走.
 */
#include "tick.h"

static volatile uint32_t g_tick_ms;

void tick_init(void)
{
}

void tick_poll(void)
{
    if (DL_TimerA_getEnabledInterruptStatus(COUNT_INST,
            DL_TIMERA_INTERRUPT_ZERO_EVENT)) {

        DL_TimerA_clearInterruptStatus(COUNT_INST,
            DL_TIMERA_INTERRUPT_ZERO_EVENT);

        g_tick_ms++;
    }
}

uint32_t tick_get(void)
{
    return g_tick_ms;
}
