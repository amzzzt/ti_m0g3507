/**
 * main.c — 灰度循迹 + 电机速度闭环
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"

#define BASE_SPEED  400

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();

    while (1) {
        int dev = track_deviation();
        int16_t tgt_l = (int16_t)(BASE_SPEED + dev);
        int16_t tgt_r = (int16_t)(BASE_SPEED - dev);
        motor_control_update(tgt_l, tgt_r);

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;
            char buf[80];
            sprintf(buf, "%d,%d,%d,%d\r\n",
                tgt_l, tgt_r,
                motor_control_left_speed(), motor_control_right_speed());
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
