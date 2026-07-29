/**
 * main.c — 球追踪: 低通滤波 alpha=0.6 + 100ms打印
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "protocol.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    gpio_init(A8,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(A12, GPO, 1, GPO_PUSH_PULL);

    key_init(1);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();
    imu_init();
    protocol_init(115200);
    tick_start();

    float   filt = 0;
    int16_t prev = 0;

    while (1)
    {
        uint32_t now = tick_get();

        /* 收帧滤波 */
        offset_t o = protocol_get();
        if (o.updated && o.found && o.dy == 0) {
            int16_t raw = o.dx;

            /* 野值: 跳变>400 丢弃 (全量程-199~199, 不会超) */
            int16_t jump = (raw > prev) ? (raw - prev) : (prev - raw);
            if (jump <= 400) {
                prev = raw;
                /* 死区 ±3 + 低通 alpha=0.6 */
                int16_t in = (raw < 3 && raw > -3) ? 0 : raw;
                filt = 0.6f * (float)in + 0.4f * filt;
            }
        }

        /* 100ms 定时打印 */
        static uint32_t pt = 0;
        if (now - pt >= 100) {
            pt = now;
            char s[16];
            sprintf(s, "%d\r\n", (int)filt);
            wireless_uart_send_string(s);
        }
    }
}
