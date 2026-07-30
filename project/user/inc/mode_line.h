/**
 * mode_line.h — 模式1: 巡线 + 启停线检测
 */
#ifndef _mode_line_h_
#define _mode_line_h_

void mode_line_init(void);
void mode_line_update(void);
void mode_line_stop_isr(void);   /* TIMA0 ISR 每1ms调用 */

#endif
