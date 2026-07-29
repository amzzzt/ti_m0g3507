/**
 * main.c — 模式调度
 *
 *   共享初始化 + 调用当前模式
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "mode_line.h"

int main(void)
{
    /* === 共享初始化 === */
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    track_init();               /* TIMA0 ISR 需要, 必须在 tick_init 前 */
    tick_init();
    wireless_uart_init();
    motor_control_init();
    imu_init();
    tft180_init();
    tft180_clear();
    tick_start();               /* 最后启动 SysTick */

    /* === 模式1: 巡线 === */
    mode_line_init();

    while (1) {
        mode_line_update();
    }
}
