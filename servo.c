/**
 * servo.c — MG996R 舵机驱动, TIMG8 PWM CCP1 → PB7
 *
 * ======================== 工作原理 ========================
 *
 * 1. PWM 硬件自动运行 (syscfg 配置):
 *    TIMG8 产生 PWM 信号, CCP1 比较值决定脉宽
 *    比较值 SERVO_MIN(0°) ~ SERVO_MAX(180°) 对应 0.5ms~2.5ms
 *    写入 CCP1 比较寄存器 → 下一个 PWM 周期生效 → 舵机转动
 *
 * 2. 角度控制 (servo_set_angle):
 *    输入 0~180°, 线性映射到 SERVO_MIN ~ SERVO_MAX (计数比较值)
 *
 * 3. 扫动计时 (servo_sweep):
 *    使用独立 TIMA0 1ms 节拍 (tick_get), 每 20ms 走 1°
 *    TIMG8 只管 PWM 输出, 不读计数器, 不绕回检测
 *    纯中断驱动, 无阻塞、无忙等
 *
 * 4. MG996R 调参:
 *    先把角度设 0°, 调大 SERVO_MIN 直到舵机刚停下不动
 *    再把角度设 180°, 调小 SERVO_MAX 直到舵机刚停下不动
 *    这两个值就是你的舵机实际可用范围
 */
#include "servo.h"
#include "tick.h"

/* ---------- 舵机参数 ---------- */
#define SERVO_MIN  50     /* 0.5ms × 100kHz */
#define SERVO_MAX  250    /* 2.5ms × 100kHz */

/* ---------- 扫动状态 ---------- */
static int angle = 0;
static int dir   = 1;

/* ===========================================================
 * servo_set_angle — 设置舵机角度
 * =========================================================== */
void servo_set_angle(uint8_t deg)
{
    if (deg > 180) deg = 180;

    uint32_t val = SERVO_MIN
                 + ((uint32_t)deg * (SERVO_MAX - SERVO_MIN)) / 180;

    DL_TimerG_setCaptureCompareValue(SERVO_INST, val, GPIO_SERVO_C1_IDX);
}

/* ===========================================================
 * servo_sweep — 主循环调用, 1ms tick 计时, 每 20ms 走 1°
 * =========================================================== */
void servo_sweep(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = tick_get();

    if (now - last_tick >= 20) {
        last_tick = now;

        angle += dir;

        if (angle >= 180) { angle = 180; dir = -1; }
        if (angle <= 0)   { angle = 0;   dir =  1; }

        servo_set_angle(angle);
    }
}

/* ===========================================================
 * servo_init — 上电归中
 * =========================================================== */
void servo_init(void)
{
    angle = 90;
    dir   = 1;
    servo_set_angle(90);
}
