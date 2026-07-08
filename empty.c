#include "ti_msp_dl_config.h"
#include "delay.h"
#include "key.h"

int main(void)
{
    int led_on = 1;

    SYSCFG_DL_init();
    key_init();
    __enable_irq();

    DL_GPIO_setPins(LED_PORT, LED_LED_0_PIN | LED_LED_1_PIN);

    while (1) {
        key_scanner();
        key_state_enum s = key_get_state(KEY_1);

        if (s == KEY_SHORT_PRESS) {
            led_on = 0;
            DL_GPIO_clearPins(LED_PORT, LED_LED_0_PIN | LED_LED_1_PIN);
        } else if (s == KEY_LONG_PRESS) {
            led_on = 1;
            DL_GPIO_setPins(LED_PORT, LED_LED_0_PIN | LED_LED_1_PIN);
        }

        if (led_on) {
            system_delay_ms(1);
        }
    }
}
