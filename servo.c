/**
 * servo.c — MG996R 舵机驱动, TIMG8 PWM CCP1 → PB7
 *
 * ======================== 工作原理 ========================
 *
 * 1. PWM 硬件自动运行 (syscfg 配置):
 *    TIMG8 计数器以 100kHz 从 0 计数到 1999, 每 2000 次 = 20ms = 一个 PWM 周期(50Hz)
 *    CCP1 比较值决定脉宽: 50(0.5ms) ~ 250(2.5ms)
 *
 * 2. 扫动计时 (servo_sweep):
 *    读 TIMG8 自己的计数器, cnt < last_cnt → 绕回 → 20ms 过去了
 *    一个定时器既输出 PWM 又计时, 无额外外设
 */
#include "servo.h"

#define SERVO_MIN  50     /* 0.5ms × 100kHz */
#define SERVO_MAX  250    /* 2.5ms × 100kHz */

static int      angle    = 0;
static int      dir      = 1;
static uint16_t last_cnt = 0;

void servo_set_angle(uint8_t deg)
{
    if (deg > 180) deg = 180;

    uint32_t val = SERVO_MIN
                 + ((uint32_t)deg * (SERVO_MAX - SERVO_MIN)) / 180;

    DL_TimerG_setCaptureCompareValue(SERVO_INST, val, GPIO_SERVO_C1_IDX);
}

void servo_sweep(void)
{
    uint16_t cnt = (uint16_t)DL_TimerG_getTimerCount(SERVO_INST);

    /*
     * 计数器: 0→1→...→1999→0→1→...
     * cnt < last_cnt → 绕回 → 一个 PWM 周期 (20ms) 结束 → 走 1°
     */
    if (cnt < last_cnt) {
        angle += dir;

        if (angle >= 180) { angle = 180; dir = -1; }
        if (angle <= 0)   { angle = 0;   dir =  1; }

        servo_set_angle(angle);
    }
    last_cnt = cnt;
}

void servo_init(void)
{
    angle    = 90;
    dir      = 1;
    servo_set_angle(90);
    last_cnt = (uint16_t)DL_TimerG_getTimerCount(SERVO_INST);
}
