/**
 * stepper.c — 双路 TMC2209 (独立定时器)
 *   M1: EN=A12 DIR=B23 STEP=A16(TIMA1 CH1)
 *   M2: EN=A8  DIR=B4  STEP=B6 (TIMG6 CH0)
 */
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "tick.h"
#include "stepper.h"

#define PWM_M1     PWM_TIM_A1_CH1_A16
#define EN_M1      A12
#define DIR_M1     B23

#define PWM_M2     PWM_TIM_G6_CH0_B6
#define EN_M2      A8
#define DIR_M2     B4

#define DUTY       5000
#define PULSE_DEG  0.225f

typedef struct {
    pwm_channel_enum pwm;
    gpio_pin_enum en, dir;
    uint8_t running;
    uint32_t stop_tick;
    uint16_t speed;
} stepper_t;

static stepper_t g_step[2];

void stepper_init(stepper_id_t id)
{
    stepper_t *s = &g_step[id];
    if (id == STEP1) {
        s->pwm = PWM_M1; s->en = EN_M1; s->dir = DIR_M1;
    } else {
        s->pwm = PWM_M2; s->en = EN_M2; s->dir = DIR_M2;
    }
    s->running = 0;
    s->speed = 1000;
    gpio_init(s->en,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(s->dir, GPO, 0, GPO_PUSH_PULL);
}

void stepper_enable(stepper_id_t id)  { gpio_low(g_step[id].en); }
void stepper_disable(stepper_id_t id) { gpio_high(g_step[id].en); pwm_set_duty(g_step[id].pwm, 0); g_step[id].running = 0; }
void stepper_set_dir(stepper_id_t id, uint8_t fwd) { fwd ? gpio_high(g_step[id].dir) : gpio_low(g_step[id].dir); }
void stepper_set_speed(stepper_id_t id, uint16_t hz) { if (hz > 0) g_step[id].speed = hz; }
uint8_t stepper_is_running(stepper_id_t id) { return g_step[id].running; }

void stepper_run(stepper_id_t id, uint16_t hz)
{
    if (hz < 1) hz = 1;
    stepper_t *s = &g_step[id];
    s->running = 1;
    s->stop_tick = 0;
    pwm_init(s->pwm, hz, DUTY);
}

void stepper_stop(stepper_id_t id)
{
    stepper_t *s = &g_step[id];
    pwm_set_duty(s->pwm, 0);
    s->running = 0;
}

void stepper_rotate(stepper_id_t id, float deg)
{
    stepper_t *s = &g_step[id];
    if (deg < 0) { stepper_set_dir(id, 0); deg = -deg; }
    else         stepper_set_dir(id, 1);
    uint32_t steps = (uint32_t)(deg / PULSE_DEG + 0.5f);
    if (steps == 0) return;
    uint32_t dur = steps * 1000 / s->speed;
    if (dur < 1) dur = 1;
    pwm_init(s->pwm, s->speed, DUTY);
    s->stop_tick = tick_get() + dur;
    s->running = 1;
}

void stepper_tick(void)
{
    for (int i = 0; i < 2; i++) {
        stepper_t *s = &g_step[i];
        if (s->running && s->stop_tick && (int32_t)(tick_get() - s->stop_tick) >= 0) {
            pwm_set_duty(s->pwm, 0);
            s->running = 0;
        }
    }
}
