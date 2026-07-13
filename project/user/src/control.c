/**
 * control.c — 巡线+速度闭环: 读偏差→算转向→编码器+PID→电机
 */
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "track.h"
#include "control.h"

#define PID_MAX 3000
#define KP      2.0f
#define STEER_MAX  400
#define BASE_DEF   250

static pid_t   pl, pr;
static int16_t base_speed = BASE_DEF;

void control_init(void)
{
    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();
    track_init();

    pid_init(&pl, 3.0f, 0.1f, 2.0f, PID_MAX);
    pid_init(&pr, 3.0f, 0.1f, 2.0f, PID_MAX);
}

void control_set_speed(int16_t base) { base_speed = base; }

void control_update(void)
{
    // 1. 读偏差
    int dev = track_deviation();

    // 2. P转向 + 限幅
    int steer = (int)(KP * (float)dev);
    if (steer >  STEER_MAX) steer =  STEER_MAX;
    if (steer < -STEER_MAX) steer = -STEER_MAX;

    int16_t tgt_l = base_speed - steer;
    int16_t tgt_r = base_speed + steer;

    // 3. 速度闭环
    encoder_update();
    float sl = encoder_left_speed();
    float sr = encoder_right_speed();

    int16_t pwm_l = (int16_t)pid_compute(&pl, (float)tgt_l, sl);
    int16_t pwm_r = (int16_t)pid_compute(&pr, (float)tgt_r, sr);
    if (pwm_l >  8000) pwm_l =  8000; if (pwm_l < -8000) pwm_l = -8000;
    if (pwm_r >  8000) pwm_r =  8000; if (pwm_r < -8000) pwm_r = -8000;

    motor_left(pwm_l);
    motor_right(pwm_r);
}
