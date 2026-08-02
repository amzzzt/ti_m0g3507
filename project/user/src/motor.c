
/**
 * motor.c — TB6612 直流电机驱动 (TIMG7, 逐飞库)
 *
 *   引脚:  STBY=A29   AIN1=A13   AIN2=B27
 *         PWMA=A26 (TIMG7 CH0)   PWMB=A27 (TIMG7 CH1)
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
#include "encoder.h"
#include "pid.h"
#include "tick.h"

#define MOTOR_FREQ  10000   // 10kHz

#define STBY_PIN    A29
#define AIN1_PIN    A13
#define AIN2_PIN    B27
#define BIN1_PIN    A0
#define BIN2_PIN    A1

void motor_init(void)
{
    // STBY 高使能
    gpio_init(STBY_PIN, GPO, 1, GPO_PUSH_PULL);
    // 左电机方向: AIN1, AIN2
    gpio_init(AIN1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(AIN2_PIN, GPO, 0, GPO_PUSH_PULL);
    // 右电机方向: BIN1, BIN2
    gpio_init(BIN1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(BIN2_PIN, GPO, 0, GPO_PUSH_PULL);

    // TIMG7 PWM: CH0→A26左, CH1→A27右
    pwm_init(PWM_TIM_G7_CH0_A26, MOTOR_FREQ, 0);
    pwm_init(PWM_TIM_G7_CH1_A27, MOTOR_FREQ, 0);
}

void motor_left(int16_t speed)
{
    if (speed > 8000) speed = 8000;
    if (speed < -8000) speed = -8000;

    uint32_t duty = (uint32_t)(speed >= 0 ? speed : -speed);

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

void motor_right(int16_t speed)
{
    speed = -speed;  /* 右电机物理反装 */
    if (speed > 8000) speed = 8000;
    if (speed < -8000) speed = -8000;

    uint32_t duty = (uint32_t)(speed >= 0 ? speed : -speed);

    if (speed > 0) {
        gpio_high(BIN1_PIN);
        gpio_low(BIN2_PIN);
    } else if (speed < 0) {
        gpio_low(BIN1_PIN);
        gpio_high(BIN2_PIN);
    } else {
        gpio_low(BIN1_PIN);
        gpio_low(BIN2_PIN);
        duty = 0;
    }
    pwm_set_duty(PWM_TIM_G7_CH1_A27, duty);
}

void motor_stop(void)
{
    gpio_low(AIN1_PIN); gpio_low(AIN2_PIN);
    gpio_low(BIN1_PIN); gpio_low(BIN2_PIN);
    pwm_set_duty(PWM_TIM_G7_CH0_A26, 0);
    pwm_set_duty(PWM_TIM_G7_CH1_A27, 0);
}

// ===================== 速度闭环 =====================
#define CTRL_KP     1.0f
#define CTRL_KI     0.08f
#define CTRL_KD     3.0f
#define CTRL_MAX    8000
#define CTRL_KFF    7.0f

static pid_t ctrl_pl, ctrl_pr;
static uint32_t ctrl_last;

/* PWM 起步限幅 */
static uint32_t pwm_ramp_t0;
static int      pwm_ramp_ms;

void motor_control_set_pwm_ramp_ms(int ms)
{
    pwm_ramp_ms = ms;
    pwm_ramp_t0 = tick_get();
}

void motor_control_init(void)
{
    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();
    pid_init(&ctrl_pl, CTRL_KP, CTRL_KI, CTRL_KD, CTRL_MAX);
    pid_init(&ctrl_pr, CTRL_KP, CTRL_KI, CTRL_KD, CTRL_MAX);
    ctrl_last = tick_get();
}

void motor_control_update(int16_t tgt_l, int16_t tgt_r)
{
    uint32_t now = tick_get();
    if ((int32_t)(now - ctrl_last) < 10) return;
    ctrl_last = now;

    encoder_update();
    int sl = (int)encoder_left_speed();
    int sr = (int)encoder_right_speed();
    if (sl < 0) sl = -sl;
    if (sr < 0) sr = -sr;

    float ol = pid_compute(&ctrl_pl, (float)tgt_l, (float)sl);
    float or = pid_compute(&ctrl_pr, (float)tgt_r, (float)sr);
    int pl = (int)((float)tgt_l * CTRL_KFF + ol);
    int pr = (int)((float)tgt_r * CTRL_KFF + or);
    if (pl >  8000) pl =  8000; if (pl < -8000) pl = -8000;
    if (pr >  8000) pr =  8000; if (pr < -8000) pr = -8000;

    /* PWM 起步限幅: f¹ 线性 */
    if (pwm_ramp_ms > 0) {
        uint32_t el = tick_get() - pwm_ramp_t0;
        if (el < (uint32_t)pwm_ramp_ms) {
            float f = (float)el / (float)pwm_ramp_ms;
            int cap = (int)(8000.0f * f * f * f);
            if (pl >  cap) pl =  cap;
            if (pl < -cap) pl = -cap;
            if (pr >  cap) pr =  cap;
            if (pr < -cap) pr = -cap;
        }
    }

    motor_left(pl);
    motor_right(pr);
}

int16_t motor_control_left_speed(void)
{
    int s = (int)encoder_left_speed();
    return (int16_t)(s < 0 ? -s : s);
}

int16_t motor_control_right_speed(void)
{
    int s = (int)encoder_right_speed();
    return (int16_t)(s < 0 ? -s : s);
}
