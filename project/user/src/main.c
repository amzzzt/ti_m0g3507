/**
 * main.c — IMU660RC 例程方式: INT2 回调自动采集, 主循环只读
 *   引脚: SCK=B23 MOSI=B22 MISO=B21 CS=B19 INT2=B24
 */
#include "zf_common_headfile.h"
#include "zf_device_imu660rc.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();

    /* 初始化和重试, 照官方例程 */
    while (1) {
        if (imu660rc_init(IMU660RC_QUARTERNION_120HZ)) {
            /* 失败: 短暂延时后重试 */
            system_delay_ms(200);
        } else {
            break;  /* 成功 */
        }
    }

    /* 主循环只读全局变量, 不调任何 imu 采集函数 */
    /* 数据由 INT2 中断 → GROUP1_IRQHandler → imu660rc_callback 自动采集 */
    while (1) {
        system_delay_ms(100);
        char buf[80];
        sprintf(buf, "Y=%.1f P=%.1f R=%.1f\r\n",
            imu660rc_yaw, imu660rc_pitch, imu660rc_roll);
        if (!gpio_get_level(B2)) wireless_uart_send_string(buf);
    }
}
