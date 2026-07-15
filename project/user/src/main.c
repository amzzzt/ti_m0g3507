/**
 * main.c — IMU660RC 测试
 *   SPI0: SCK=B18 MOSI=B17 MISO=B19 CS=A2 INT2=B24
 *   无线 UART1: B5/B6/B2
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "zf_device_imu660rc.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();
    imu660rc_init(IMU660RC_QUARTERNION_60HZ);

    while (1) {
        char buf[50];
        system_delay_ms(20);
        float r = imu660rc_roll, p = imu660rc_pitch, y = imu660rc_yaw;
        if(r != r || r > 1e4f || r < -1e4f) r = 0;
        if(p != p || p > 1e4f || p < -1e4f) p = 0;
        if(y != y || y > 1e4f || y < -1e4f) y = 0;
        int ir = (int)(r);
        int ip = (int)(p);
        int iy = (int)(y);
        if(ir > 1000 || ir < -1000) ir = 0;
        if(ip > 1000 || ip < -1000) ip = 0;
        if(iy > 1000 || iy < -1000) iy = 0;
        sprintf(buf, "%d,%d,%d\r\n", ir, ip, iy);
        wireless_uart_send_string(buf);
    }
}
