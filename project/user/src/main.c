#include "zf_common_headfile.h"
#include "tick.h"
#include "bsp_sr04.h"
#include <stdio.h>

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    sr04_init();

    gpio_init(B22, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_high(B22);   // 上电亮灯 = 新程序已烧入

    while (true)
    {
        if (sr04_send_flag) {
            sr04_send_flag = 0;
            float v = sr04_read();
            char buf[32];
            sprintf(buf, "%.1f\r\n", (double)v);
            wireless_uart_send_string(buf);
        }
    }
}
