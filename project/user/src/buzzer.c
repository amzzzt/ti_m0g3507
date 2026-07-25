/**
 * buzzer.c — 蜂鸣器 B27
 */
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "buzzer.h"

#define BUZZER_PIN  B27

void buzzer_init(void) {
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);  /* 上电拉低, 不叫 */
}

void buzzer_on(void)  { gpio_high(BUZZER_PIN); }
void buzzer_off(void) { gpio_low(BUZZER_PIN);  }

void buzzer_beep(uint16_t ms) {
    buzzer_on();
    system_delay_ms(ms);
    buzzer_off();
}
