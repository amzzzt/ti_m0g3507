/**
 * tick.h — 系统 1ms 节拍 (基于逐飞库 PIT, TIMA0)
 *
 * 使用 pit_ms_init(PIT_TIM_A0, 1, ...) 注册回调
 * 中断由 isr.c:TIMA0_IRQHandler → pit_callback_list[0] 自动分发
 * 各模块通过 tick_get() 获取统一时间基准
 */

#ifndef _tick_h_
#define _tick_h_

#include <stdint.h>

void     tick_init(void);
uint32_t tick_get(void);        // 返回 ms 计数器

#endif
