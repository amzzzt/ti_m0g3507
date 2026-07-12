/**
 * main.c — 巡线控制
 *
 *   TIMA0(10ms标志) → control_update → 200ms打印
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "encoder.h"
#include "control.h"
#include "track.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();

    control_init();
    control_set_speed(250);

    while (1)
    {
        if (tick_ctrl_ready()) {
            tick_ctrl_clear();
            control_update();
        }

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 200) {
            pt = now;
            int d = track_deviation();
            int sl = (int)encoder_left_speed();
            int sr = (int)encoder_right_speed();
            char buf[30];
            sprintf(buf, "%d,%d,%d\r\n", d, sl, sr);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
