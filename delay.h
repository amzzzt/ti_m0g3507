#ifndef _delay_h_
#define _delay_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>

void system_delay_ms(uint32_t time);
void system_delay_us(uint32_t time);
void system_delay_init(void);

#endif
