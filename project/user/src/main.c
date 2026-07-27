/**
 * main.c — 灰度循迹 + IMU yaw + TFT180 显示
 *
 * IMU: SPI0 (B18 SCK, B17 MOSI, B19 MISO, A2 CS, B24 INT2)
 * TFT180: SPI1 (B9 SCK, B8 MOSI, B10 RES, B11 DC, B14 CS, B26 BL)
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"

#define BASE_SPEED  400

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);            /* 按键初始化, 必须在 tick_init 前 */
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();

    imu_init();
    tft180_init();
    tft180_clear();

    /* 所有 init 完成, 启动 SysTick 计时 */
    tick_start();

    /* 等待 KEY_1 (A30) 按下 */
    tft180_show_string(0, 0, "Press KEY1");
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);

    /* 等 2 秒 */
    tft180_show_string(0, 0, "Wait 2s...");
    {
        uint32_t t0 = tick_get();
        while (tick_get() - t0 < 2000);
    }
    tft180_show_string(0, 0, "GO       ");

    while (1) {
        int dev = track_deviation();
        int16_t tgt_l = (int16_t)(BASE_SPEED + dev);
        int16_t tgt_r = (int16_t)(BASE_SPEED - dev);
        motor_control_update(tgt_l, tgt_r);

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;

            float yaw   = imu_yaw();
            int   spd_l = motor_control_left_speed();
            int   spd_r = motor_control_right_speed();

            /* 串口 */
            char buf[80];
            sprintf(buf, "Y:%.1f | %d,%d,%d,%d\r\n",
                yaw, tgt_l, tgt_r, spd_l, spd_r);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);

            /* 屏幕 */
            tft180_show_string(0, 0, "            ");
            sprintf(buf, "Y:%.1f", yaw);
            tft180_show_string(0, 0, buf);
        }
    }
}
