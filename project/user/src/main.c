/**
 * main.c — 双电机开环测试 (参考a382898)
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "motor.h"
#include "encoder.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    motor_init();
    encoder_init();

    motor_left(-3000);
    motor_right(-3000);

    while (1) {
        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;
            char buf[40];
            sprintf(buf, "L:%.0f R:%.0f\r\n",
                    encoder_left_speed(), encoder_right_speed());
            wireless_uart_send_string(buf);
        }
    }
}
