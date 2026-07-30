/**
 * main.c — 模式调度 (75e8936 初始化, 不动)
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "mode_line.h"

int main(void)
{
    /* === 75e8936 原版初始化 (不动, IMU 正常工作的根基) === */
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();
    imu_init();
    for (volatile uint32_t d = 0; d < 4000000; d++);  /* ~50ms DMP 预热 */
    tft180_init();
    tft180_clear();
    tick_start();

    /* === 模式1: 巡线 === */
    mode_line_init();

    while (1) {
        mode_line_update();
    }
}
