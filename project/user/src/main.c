/**
 * main.c — 直流电机速度闭环测试
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "dc_ctrl.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    gpio_init(A12, GPO, 1, GPO_PUSH_PULL);
    gpio_init(A8,  GPO, 1, GPO_PUSH_PULL);
    wireless_uart_init();
    tick_init();

    dc_ctrl_init();
    dc_ctrl_set_left(2000);   /* 目标2000脉冲/秒 */
    dc_ctrl_set_right(2000);

    uint32_t next_print = 0;

    while (1) {
        dc_ctrl_update();  /* 自带10ms计时, 直接调 */

        uint32_t now = tick_get();
        if ((int32_t)(now - next_print) >= 0) {
            next_print = now + 50;
            char buf[64];
            sprintf(buf, "L:%.0f R:%.0f\r\n",
                    dc_ctrl_left_speed(), dc_ctrl_right_speed());
            wireless_uart_send_string(buf);
        }
    }
}
