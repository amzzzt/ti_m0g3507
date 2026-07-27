/**
 * main.c — IMU660RC + 转弯: 先全部初始化, 最后设优先级
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "turn.h"
#include "zf_device_imu660rc.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    track_init();
    tick_init();
    wireless_uart_init();
    motor_init();
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);

    /* 所有 init 完成后, 再设优先级: 只有 GROUP1(INT2+编码器) 最高, 其余全部降级 */
    interrupt_set_priority(GPIOA_INT_IRQn,   0);
    interrupt_set_priority(TIMA0_INT_IRQn,   1);
    interrupt_set_priority(UART1_INT_IRQn,   1);   /* 无线串口 */
    interrupt_set_priority(TIMG7_INT_IRQn,   1);   /* 电机 PWM */

    /* 等 2 秒 DMP 校准 */
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < 2000) {}

    if (!gpio_get_level(B2))
        wireless_uart_send_string("GO\r\n");

    char buf[80];
    sprintf(buf, "Y=%.1f\r\n", imu660rc_yaw);
    if (!gpio_get_level(B2)) wireless_uart_send_string(buf);

    while (1) {
        track_read_all();
        uint32_t t1 = tick_get();
        while (tick_get() - t1 < 100) {}

        sprintf(buf, "Y=%.1f\r\n", imu660rc_yaw);
        if (!gpio_get_level(B2)) wireless_uart_send_string(buf);
    }
}
