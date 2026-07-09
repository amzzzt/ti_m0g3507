#ifndef _tick_h_
#define _tick_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>

void     tick_init(void);
void     tick_poll(void);   /* 主循环高速调用, 轮询 MIS */
uint32_t tick_get(void);

#endif
