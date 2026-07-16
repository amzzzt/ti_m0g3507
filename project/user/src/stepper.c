/**
 * stepper.c — 双步进电机驱动 (TIMA1 PWM)
 *
 *   引脚说明:
 *     RST: 高=工作, 低=停
 *     SLP: 高=锁定(手拧不动), 低=休眠(手可拧)
 *     DCY: 高=大扭矩, 低=小扭矩
 *     DIR: 高=正转, 低=反转
 *     PWM: 1脉冲=1微步=0.05625°, 6400脉冲/圈
 *
 *   电机1: A16(TIMA1 CH1)   电机2: A15(TIMA1 CH0)
 */
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_delay.h"
#include "stepper.h"

#define STEP1_PWM  PWM_TIM_A1_CH1_A16   // TIMA1 CH1
#define STEP2_PWM  PWM_TIM_A1_CH0_A15   // TIMA1 CH0

typedef struct {
    gpio_pin_enum    rst, slp, dcy, dir;
    pwm_channel_enum pwm;
    uint16_t         hz;
} stepper_t;

static stepper_t g_step[2];

void stepper_init(stepper_id_t id) {
    stepper_t *s = &g_step[id];
    if (id == STEP1) {
        s->rst = STEP1_RST_PIN; s->slp = STEP1_SLP_PIN;
        s->dcy = STEP1_DCY_PIN; s->dir = STEP1_DIR_PIN;
        s->pwm = STEP1_PWM;
    } else {
        s->rst = STEP2_RST_PIN; s->slp = STEP2_SLP_PIN;
        s->dcy = STEP2_DCY_PIN; s->dir = STEP2_DIR_PIN;
        s->pwm = STEP2_PWM;
    }
    s->hz = 1000;
    gpio_init(s->rst, GPO, 1, GPO_PUSH_PULL);
    gpio_init(s->slp, GPO, 1, GPO_PUSH_PULL);
    gpio_init(s->dcy, GPO, 1, GPO_PUSH_PULL);
    gpio_init(s->dir, GPO, 0, GPO_PUSH_PULL);
}

void stepper_enable(stepper_id_t id) {
    stepper_t *s = &g_step[id];
    gpio_high(s->rst); gpio_high(s->slp); gpio_high(s->dcy);
}

void stepper_disable(stepper_id_t id) {
    stepper_t *s = &g_step[id];
    gpio_low(s->rst); gpio_low(s->slp);
}

void stepper_set_dir(stepper_id_t id, uint8_t forward) {
    stepper_t *s = &g_step[id];
    forward ? gpio_high(s->dir) : gpio_low(s->dir);
}

void stepper_set_speed(stepper_id_t id, uint16_t hz) {
    if (hz < 1) hz = 1;
    g_step[id].hz = hz;
}

void stepper_rotate_deg(stepper_id_t id, float deg) {
    stepper_t *s = &g_step[id];
    uint32_t steps = (uint32_t)(deg / STEPPER_STEP_ANGLE + 0.5f);
    if (steps == 0) return;
    uint32_t dur_ms = (uint32_t)((float)steps * 1000.0f / (float)s->hz);
    if (dur_ms < 1) dur_ms = 1;
    pwm_init(s->pwm, s->hz, 5000);   // 50% duty (PWM_DUTY_MAX=10000)
    system_delay_ms(dur_ms);
    pwm_set_duty(s->pwm, 0);
}
