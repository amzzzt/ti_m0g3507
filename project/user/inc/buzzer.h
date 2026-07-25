/**
 * buzzer.h — 蜂鸣器 (B27, 有源高有效)
 */
#ifndef _buzzer_h_
#define _buzzer_h_
#include <stdint.h>

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint16_t ms);

#endif
