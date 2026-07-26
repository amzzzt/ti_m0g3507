/**
 * main.c — IMU660RC 陀螺仪测试
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "imu.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();

    if (imu_init()) {
        while (1) { uart_write_string(UART_1, "IMU FAIL\r\n"); system_delay_ms(500); }
    }

    while (1) {
        imu_update();

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;
            char buf[40];
            sprintf(buf, "%.1f\r\n", imu_yaw());
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
