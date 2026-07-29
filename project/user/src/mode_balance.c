/**
 * mode_balance.c — 模式3: 单轴稳姿 (下电机 STEP1)
 *
 *   IMU pitch → PID → stepper STEP1 补偿
 *   目标: 保持平台水平, 抵消车身摆动
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "imu.h"
#include "stepper.h"
#include "balance.h"
#include "mode_balance.h"

static uint8_t active;

void mode_balance_init(void)
{
    balance_init();
    active = 0;
    tft180_show_string(0, 0, "Mode3:Bal ");
    tft180_show_string(0, 1, "Press KEY1");
}

void mode_balance_update(void)
{
    uint32_t now = tick_get();

    /* KEY1 切换启停 */
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        key_clear_state(KEY_1);
        active = !active;
        if (active) {
            balance_enable();
            tft180_show_string(0, 0, "BAL ON  ");
        } else {
            balance_disable();
            tft180_show_string(0, 0, "BAL OFF ");
        }
    }

    /* 稳姿更新 */
    balance_update();

    /* TFT 显示俯仰角 */
    {
        float pitch = imu_pitch();
        char dis[16];
        sprintf(dis, "P:%.1f %s", pitch, active ? "ON " : "OFF");
        tft180_show_string(0, 1, dis);
    }

    /* === 100ms 调试打印 === */
    static uint32_t pt = 0;
    if (now - pt >= 100) {
        pt = now;
        float pitch = imu_pitch();
        char buf[48];
        sprintf(buf, "BAL P:%.1f ACT:%d\r\n", pitch, active);
        wireless_uart_send_string(buf);
    }
}
