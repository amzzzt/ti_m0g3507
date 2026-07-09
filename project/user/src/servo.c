/**
 * servo.c — MG996R 舵机驱动 (逐飞库 PWM + tick 节拍)
 *
 * TIMG8 CH1 → PB7: 50Hz PWM, pwm_set_duty 控制脉宽
 * 扫动计时: tick_get() 每 20ms 走 1°
 */

#include "zf_driver_pwm.h"
#include "tick.h"
#include "servo.h"

// 50Hz 时 duty 范围: 0.5ms=2.5%=250, 2.5ms=12.5%=1250
#define SERVO_MIN_DUTY      250     // 0°
#define SERVO_MAX_DUTY      1250    // 180°

static int angle = 0;
static int dir   = 1;

// ============================== 初始化 ==============================

void servo_init(void)
{
    // TIMG8 CH1 → PB7, 50Hz, 初始 90° (duty=750 = 1.5ms)
    pwm_init(PWM_TIM_G8_CH1_B7, 50, 750);

    angle = 90;
    dir   = 1;
}

// ============================== 角度控制 ==============================

void servo_set_angle(uint8_t deg)
{
    if (deg > 180) deg = 180;

    uint32_t duty = SERVO_MIN_DUTY
                  + ((uint32_t)deg * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)) / 180;

    pwm_set_duty(PWM_TIM_G8_CH1_B7, duty);
}

// ============================== 扫动 (tick 驱动) ==============================

void servo_sweep(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = tick_get();

    if (now - last_tick < 20) return;   // 不到 20ms, 跳过
    last_tick = now;

    angle += dir;

    if (angle >= 180) { angle = 180; dir = -1; }
    if (angle <= 0)   { angle = 0;   dir =  1; }

    servo_set_angle(angle);
}
