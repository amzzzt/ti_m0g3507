/**
 * mode_line.h — 模式1: 巡线 + 启停线检测
 */
#ifndef _mode_line_h_
#define _mode_line_h_

void mode_line_init(void);
void mode_line_update(void);
void mode_line_stop_isr(void);   /* TIMA0 ISR 每1ms调用 */
int  mode_line_is_stopped(void); /* 停车完成返回1 */
void mode_line_set_speed(int s);       /* 设置巡线速度 */
void mode_line_set_stop_frames(int n); /* 设置停车检测帧数 */
void mode_line_set_stop_delay(int ms); /* 检测后继续跑多久(ms) */
void mode_line_set_ramp_ms(int ms);    /* 起跑/停车缓加(减)速时长(ms), 默认800 */

#endif
