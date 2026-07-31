/**
 * main.c — 小球序列测试
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "servo.h"
#include "protocol.h"
#include "ball_seq.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    servo_init();
    wireless_uart_init();
    protocol_init(115200);
    tft180_init();
    tft180_clear();
    tick_start();

    ball_seq_init();

    while (1)
    {
        ball_seq_update();
    }
}
