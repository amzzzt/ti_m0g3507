#ifndef _servo_h_
#define _servo_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>

void servo_init(void);
void servo_set_angle(uint8_t deg);
void servo_sweep(void);     // 主循环调, 读硬件计数器计时

#endif
