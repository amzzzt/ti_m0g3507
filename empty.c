#include "ti_msp_dl_config.h"
#include "tick.h"
#include "servo.h"

int main(void)
{
    SYSCFG_DL_init();
    tick_init();
    servo_init();

    while (1) {
        tick_poll();
        servo_sweep();

        /*
         * LED_0 以 tick 为时钟 1Hz 闪烁.
         * 舵机停顿时看 LED: 还闪 → tick 正常, 舵机问题
         *                    不闪 → tick 停了, 这是根因
         */
        if ((tick_get() / 500) % 2) {
            DL_GPIO_setPins(LED_PORT, LED_LED_0_PIN);
        } else {
            DL_GPIO_clearPins(LED_PORT, LED_LED_0_PIN);
        }
    }
}
