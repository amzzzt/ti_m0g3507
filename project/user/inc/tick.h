#ifndef _tick_h_
#define _tick_h_
#include <stdint.h>
void     tick_init(void);
uint32_t tick_get(void);
uint8_t  tick_ctrl_ready(void);    // 10ms 控制标志
void     tick_ctrl_clear(void);
#endif
