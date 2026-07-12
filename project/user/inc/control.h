/**
 * control.h — 巡线 + 速度闭环控制
 */
#ifndef _control_h_
#define _control_h_

void control_init(void);
void control_update(void);
void control_set_speed(int16_t base);   // 设定基础速度

#endif
