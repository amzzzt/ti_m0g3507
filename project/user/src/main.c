/**
 * main.c — 按键→等2秒→巡线 + yaw显示
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "course.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();
    imu_init();
    tft180_init();
    tft180_clear();
    tick_start();

    tft180_show_string(0, 0, "Press KEY1");
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);

    tft180_show_string(0, 0, "Wait 2s...");
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < 2000);

    course_init();

    while (1) {
        course_update();

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;
            int tl, tr;
            course_targets(&tl, &tr);
            float yaw = imu_yaw();
            int sl = motor_control_left_speed();
            int sr = motor_control_right_speed();
            char buf[80];
            sprintf(buf, "C%d %d,%d,%d,%d Y:%.1f\r\n", course_state(), tl, tr, sl, sr, yaw);
            wireless_uart_send_string(buf);
            char dis[16];
            sprintf(dis, "Y:%.1f", yaw);
            tft180_show_string(0, 0, dis);
        }
    }
}
