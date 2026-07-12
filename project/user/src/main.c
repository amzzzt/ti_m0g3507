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
    control_set_target(0, 0);

    static uint32_t prt_tk = 0, chg_tk = 0;
    static uint8_t  step   = 0;
    uint32_t now;
    char buf[60];

    while (1)
    {
        // ISR 每 10ms 设标志, 主循环执行控制
        if (tick_ctrl_ready()) {
            tick_ctrl_clear();
            control_update();
        }

        now = tick_get();

        if (now - prt_tk >= 100) {
            prt_tk = now;
            sprintf(buf, "%4d,%4d,%4d,%4d\r\n",
                control_target_left(),  control_target_right(),
                (int)encoder_left_speed(), (int)encoder_right_speed());
            // 等待无线模块空闲, 超时跳过
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }

        if (now - chg_tk >= 2000) {
            chg_tk = now;
            step = !step;
            control_set_target(step ? 400 : 0, step ? 400 : 0);
        }
    }
}
