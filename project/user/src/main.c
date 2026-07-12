#include "zf_common_headfile.h"
#include "imu.h"

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();

    while (imu_init()) {
        if (!gpio_get_level(B2))
            wireless_uart_send_string("imu error\r\n");
        system_delay_ms(500);
    }

    while (true)
    {
        imu_update();

        char buf[80];
        sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n",
                imu_acc_x(), imu_acc_y(), imu_acc_z(),
                imu_gyro_x(), imu_gyro_y(), imu_gyro_z());
        if (!gpio_get_level(B2))
            wireless_uart_send_string(buf);

        system_delay_ms(50);
    }
}
