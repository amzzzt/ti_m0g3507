/**
 * balance.h — 单轴平衡控制 (IMU pitch → PID → 步进电机)
 *
 *   平台倾斜: STEP1 (A12/B23/A16, TMC2209)
 *   反馈:      IMU660RC pitch 角
 *   目标:      保持 pitch=0° (水平), 小球居中
 */
#ifndef _balance_h_
#define _balance_h_

#include <stdint.h>

void    balance_init(void);
void    balance_update(void);
void    balance_enable(void);
void    balance_disable(void);
uint8_t balance_is_active(void);

#endif
