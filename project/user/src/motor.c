/**
 * motor.c — TB6612 直流电机驱动 (TIMG7, 逐飞库)
 *
 *   引脚:  STBY=A29   AIN1=B23   AIN2=B27
 *         PWMA=A26 (TIMG7 CH0)   PWMB=A27 (TIMG7 CH1, 预留)
 *
 *   TB6612 真值表:
 *   ┌──────┬──────┬───────┬──────────────┐
 *   │ STBY │ AIN1 │ AIN2  │    状态      │
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

#define MOTOR_FREQ  10000   // 10kHz

#define STBY_PIN    A29
#define AIN1_PIN    B23
#define AIN2_PIN    B27

void motor_init(void)
{
    // GPIO 初始化: STBY 高使能, AIN1/AIN2 低
    gpio_init(STBY_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(AIN1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AIN2_PIN, GPO, 0, GPO_PUSH_PULL);

    // TIMG7 PWM: CH0→A26左, CH1→A27右
    pwm_init(PWM_TIM_G7_CH0_A26, MOTOR_FREQ, 0);
    pwm_init(PWM_TIM_G7_CH1_A27, MOTOR_FREQ, 0);
}

void motor_set(int16_t speed)
{
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    // 0~1000 映射到 PWM 0~10000 (PWM_DUTY_MAX)
    uint32_t duty = (uint32_t)(speed >= 0 ? speed : -speed) * 10;

    if (speed > 0) {
        gpio_high(AIN1_PIN);
        gpio_low(AIN2_PIN);
    } else if (speed < 0) {
        gpio_low(AIN1_PIN);
        gpio_high(AIN2_PIN);
    } else {
        gpio_low(AIN1_PIN);
        gpio_low(AIN2_PIN);
        duty = 0;
    }

    pwm_set_duty(PWM_TIM_G7_CH0_A26, duty);
}

void motor_stop(void)
{
    gpio_low(AIN1_PIN);
    gpio_low(AIN2_PIN);
    pwm_set_duty(PWM_TIM_G7_CH0_A26, 0);
    pwm_set_duty(PWM_TIM_G7_CH1_A27, 0);
}
