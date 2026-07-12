/**
 * main.c — 速度闭环测试
 *
 *   TIMA0(10ms): encoder_update → PID → motor
 *   主循环: 50ms打印 + 2s切换目标
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "encoder.h"
#include "control.h"

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();

    control_init();
    control_set_target(1000, 1000);     // 固定目标 1000

    static uint32_t prt_tk = 0;
    char buf[60];

    while (1)
    {
        if (tick_ctrl_ready()) {
            tick_ctrl_clear();
            control_update();
        }

        uint32_t now = tick_get();

        if (now - prt_tk >= 100) {
            prt_tk = now;
            sprintf(buf, "%4d,%4d,%4d\r\n",
                1000,
                (int)encoder_left_speed(), (int)encoder_right_speed());
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
