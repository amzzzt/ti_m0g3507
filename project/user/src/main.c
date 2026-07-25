/**
 * main.c — 直流电机速度闭环测试
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "motor.h"

#define TARGET  400

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    motor_control_init();

    while (1) {
        motor_control_update(TARGET, TARGET);

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 20) {
            pt = now;
            char buf[48];
            sprintf(buf, "%d,%d,%d\r\n",
                    TARGET,
                    motor_control_left_speed(),
                    motor_control_right_speed());
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
