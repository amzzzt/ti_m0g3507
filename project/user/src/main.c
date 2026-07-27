/**
 * main.c — 按键触发: 直走1s → 左转85° → 直走1s → 停
 *
 * IMU: SPI0 (B18 SCK, B17 MOSI, B19 MISO, A2 CS, B24 INT2)
 * TFT180: SPI1 (B9 SCK, B8 MOSI, B10 RES, B11 DC, B14 CS, B26 BL)
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "motor.h"
#include "imu.h"

#define SPEED  3000   /* PWM 占空比 (0~8000, 8000=100%) */

typedef enum {
    S_WAIT_KEY,
    S_WAIT_2S,
    S_STRAIGHT1,
    S_TURN,
    S_STRAIGHT2,
    S_DONE
} state_t;

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    wireless_uart_init();
    motor_init();

    imu_init();
    tft180_init();
    tft180_clear();
    tick_start();

    state_t    state = S_WAIT_KEY;
    uint32_t   t0    = 0;
    float      yaw_start;

    while (1) {
        float   yaw = imu_yaw();
        uint32_t now = tick_get();

        /* ---- 状态机 ---- */
        switch (state) {
        case S_WAIT_KEY:
            tft180_show_string(0, 0, "Press KEY1");
            if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_1);
                state = S_WAIT_2S;
                t0 = now;
                tft180_show_string(0, 0, "Wait 2s...");
            }
            break;

        case S_WAIT_2S:
            if (now - t0 >= 2000) {
                state = S_STRAIGHT1;
                t0 = now;
                motor_left(SPEED);
                motor_right(SPEED);
                tft180_show_string(0, 0, "GO FW     ");
            }
            break;

        case S_STRAIGHT1:
            if (now - t0 >= 1000) {
                state = S_TURN;
                yaw_start = yaw;
                motor_left(-SPEED);
                motor_right(SPEED);
                tft180_show_string(0, 0, "TURN L    ");
            }
            break;

        case S_TURN: {
            /* 前 100ms 消抖, 不判断角度 */
            if (now - t0 < 100) break;
            /* 左转角度 (处理 0/360 回绕 + 边界抖动) */
            float d = yaw_start - yaw;
            if (d < 0) d += 360.0f;
            if (d >= 85.0f && d <= 180.0f) {
                state = S_STRAIGHT2;
                t0 = now;
                motor_left(SPEED);
                motor_right(SPEED);
                tft180_show_string(0, 0, "GO FW     ");
            }
            break;
        }

        case S_STRAIGHT2:
            if (now - t0 >= 1000) {
                state = S_DONE;
                motor_stop();
                tft180_show_string(0, 0, "DONE      ");
            }
            break;

        case S_DONE:
            break;
        }

        /* ---- 串口打印 50ms ---- */
        static uint32_t pt = 0;
        if (now - pt >= 50) {
            pt = now;
            char buf[80];
            sprintf(buf, "S:%d Y:%.1f T:%u\r\n", state, yaw, (unsigned int)now);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
