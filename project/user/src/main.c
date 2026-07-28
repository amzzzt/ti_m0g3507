/**
 * main.c — 从 course 版本改: 只加 protocol, init 全部保留
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
    key_init(1);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();
    imu_init();
    protocol_init(115200);
    tft180_init();
    tft180_clear();
    tick_start();

    tft180_show_string(0, 0, "Proto test");

    while (1)
    {
        offset_t o = protocol_get();
        if (o.updated) {
            char buf[32];
            sprintf(buf, "%d %d\r\n", o.dx, o.dy);
            wireless_uart_send_string(buf);
        }
    }
}
