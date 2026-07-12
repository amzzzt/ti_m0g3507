/**
 * main.c — 速度闭环 + 最简打印
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "encoder.h"
#include "control.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    control_init();
    control_set_target(1000, 1000);

    while (1)
    {
        if (tick_ctrl_ready()) {
            tick_ctrl_clear();
            control_update();
        }

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 100) {
            pt = now;
            int sl = (int)encoder_left_speed();
            int sr = (int)encoder_right_speed();
            char buf[30];
            sprintf(buf, "%d,%d,%d\r\n", 1000, sl, sr);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
