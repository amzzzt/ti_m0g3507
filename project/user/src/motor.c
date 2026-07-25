/**
 * motor.c — TB6612 直流电机驱动 (TIMG7, 逐飞库)
 *
 *   引脚:  STBY=A29   IN1=A13   IN2=B8
 *         PWMA=A26 (TIMG7 CH0)   PWMB=A27 (TIMG7 CH1)
 *
 *   TB6612 真值表:
 *   ┌──────┬──────┬───────┬──────────────┐
 *   │ STBY │ IN1  │ IN2   │    状态      │
 *   ├──────┼──────┼───────┼──────────────┤
 *   │  0   │  X   │   X   │ 电机不工作   │
 *   │  1   │  0   │   0   │ 停止(惯性)   │
 *   │  1   │  0   │   1   │ PWM=H反转/L制动 │
 *   │  1   │  1   │   0   │ PWM=H正转/L制动 │
 *   │  1   │  1   │   1   │ 制动(急停)   │
 *   └──────┴──────┴───────┴──────────────┘
 *
 *   PWM: 80MHz不分频, 周期8000 → 10kHz
 */
#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "motor.h"

#define MOTOR_FREQ  10000

#define STBY_PIN    A29
#define L_IN1       A13
#define L_IN2       B7     /* 原B27, 腾给蜂鸣器 */
#define R_IN1       A0
#define R_IN2       A1

void motor_init(void)
{
    gpio_init(STBY_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(L_IN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(L_IN2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(R_IN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(R_IN2, GPO, 0, GPO_PUSH_PULL);
    pwm_init(PWM_TIM_G7_CH0_A26, MOTOR_FREQ, 0);
    pwm_init(PWM_TIM_G7_CH1_A27, MOTOR_FREQ, 0);
}

void motor_left(int16_t speed)
{
    speed = -speed;
    if (speed > 8000) speed = 8000;
    if (speed < -8000) speed = -8000;
    uint32_t duty = (uint32_t)(speed >= 0 ? speed : -speed);
    if (speed > 0)      { gpio_high(L_IN1); gpio_low(L_IN2); }
    else if (speed < 0) { gpio_low(L_IN1);  gpio_high(L_IN2); }
    else                { gpio_low(L_IN1);  gpio_low(L_IN2); duty = 0; }
    pwm_set_duty(PWM_TIM_G7_CH0_A26, duty);
}

void motor_right(int16_t speed)
{
    if (speed > 8000) speed = 8000;
    if (speed < -8000) speed = -8000;
    uint32_t duty = (uint32_t)(speed >= 0 ? speed : -speed);
    if (speed > 0)      { gpio_high(R_IN1); gpio_low(R_IN2); }
    else if (speed < 0) { gpio_low(R_IN1);  gpio_high(R_IN2); }
    else                { gpio_low(R_IN1);  gpio_low(R_IN2); duty = 0; }
    pwm_set_duty(PWM_TIM_G7_CH1_A27, duty);
}

void motor_stop(void)
{
    gpio_low(L_IN1); gpio_low(L_IN2);
    gpio_low(R_IN1); gpio_low(R_IN2);
    pwm_set_duty(PWM_TIM_G7_CH0_A26, 0);
    pwm_set_duty(PWM_TIM_G7_CH1_A27, 0);
}
