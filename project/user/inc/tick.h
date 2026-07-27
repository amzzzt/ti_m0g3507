#ifndef _tick_h_
#define _tick_h_
#include <stdint.h>
void     tick_init(void);        // 启动 TIMA0 扫描
void     tick_start(void);       // 启动 SysTick (在所有 init 之后调用)
uint32_t tick_get(void);
uint8_t  tick_ctrl_ready(void);    // 10ms 控制标志
void     tick_ctrl_clear(void);
#endif
