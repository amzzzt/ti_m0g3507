/**
 * main.c — 直角转弯测试: 等2秒 → 左转
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "turn.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    track_init();
    tick_init();
    wireless_uart_init();
    motor_init();

    /* 上电等 2 秒 */
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < 2000) {}

    if (!gpio_get_level(B2))
        wireless_uart_send_string("GO\r\n");

    turn_left();

    if (!gpio_get_level(B2))
        wireless_uart_send_string("DONE\r\n");

    while (1) {}
}
