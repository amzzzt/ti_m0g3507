/**
 * dc_ctrl.c — 直流电机速度闭环 (PI×2, 自带dt)
 */
#include "dc_ctrl.h"
#include "encoder.h"
#include "motor.h"
#include "tick.h"

/* PI: Kp=2.0 Ki=0.3 积分限±4000 */
#define KP  2.0f
#define KI  0.3f
#define MAX_I  4000.0f

typedef struct {
    float target;
    float integral;
    int16_t out;
} dc_pid_t;

static dc_pid_t  g_left, g_right;
static uint32_t  g_last_ms;

void dc_ctrl_init(void) {
    motor_init();
    encoder_init();
    encoder_filter_reset();
    g_left  = (dc_pid_t){0,0,0};
    g_right = (dc_pid_t){0,0,0};
    g_last_ms = tick_get();
}

void dc_ctrl_set_left(float t)  { g_left.target = t; }
void dc_ctrl_set_right(float t) { g_right.target = t; }

static void _pi(dc_pid_t *p, float actual, void (*motor)(int16_t)) {
    float err = p->target - actual;
    p->integral += err * 0.01f;
    if (p->integral >  MAX_I) p->integral =  MAX_I;
    if (p->integral < -MAX_I) p->integral = -MAX_I;
    float out = KP * err + KI * p->integral;
    if (out >  8000) out =  8000;
    if (out < -8000) out = -8000;
    if (p->target == 0 && actual < 10 && actual > -10)
        { p->integral = 0; out = 0; }
    p->out = (int16_t)out;
    motor(p->out);
}

void dc_ctrl_update(void) {
    uint32_t now = tick_get();
    if ((int32_t)(now - g_last_ms) < 10) return;  /* 不足10ms跳过 */
    g_last_ms = now;

    encoder_update();
    _pi(&g_left,  encoder_left_speed(),  motor_left);
    _pi(&g_right, encoder_right_speed(), motor_right);
}

void dc_ctrl_stop(void) {
    motor_stop();
    g_left.integral  = 0;
    g_right.integral = 0;
}

float dc_ctrl_left_speed(void)  { return encoder_left_speed(); }
float dc_ctrl_right_speed(void) { return encoder_right_speed(); }

void dc_ctrl_stop(void) {
    motor_stop();
    g_left.integral  = 0;
    g_right.integral = 0;
}

float dc_ctrl_left_speed(void)  { return encoder_left_speed(); }
float dc_ctrl_right_speed(void) { return encoder_right_speed(); }
