/**
 * control.h — 速度闭环控制
 */
#ifndef _control_h_
#define _control_h_

#include <stdint.h>

void    control_init(void);
void    control_update(void);                 // 每 10ms 调一次
void    control_set_target(int16_t l, int16_t r);
int16_t control_target_left(void);
int16_t control_target_right(void);

#endif
