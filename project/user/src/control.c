/**
 * control.c — 速度闭环: 编码器 → PID → 电机
 *
 *   每 10ms: encoder_update() → PID → motor_left/right
 */
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "control.h"

#define KFF  7.0f
#define PID_MAX  3000

static pid_t   pl, pr;
static int16_t tl, tr;

void control_init(void)
{
    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();

    pid_init(&pl, 3.0f, 0.1f, 2.0f, PID_MAX);
    pid_init(&pr, 3.0f, 0.1f, 2.0f, PID_MAX);
}

void control_set_target(int16_t l, int16_t r) { tl = l; tr = r; }
int16_t control_target_left(void)  { return tl; }
int16_t control_target_right(void) { return tr; }

void control_update(void)
{
    encoder_update();

    // 目标=0 → 直接停
    if (tl == 0 && tr == 0) {
        motor_stop();
        pl.integral = 0; pl.prev_err = 0;
        pr.integral = 0; pr.prev_err = 0;
        return;
    }

    float sl = encoder_left_speed();
    float sr = encoder_right_speed();

    int16_t bl = (int16_t)((float)tl * KFF);
    int16_t br = (int16_t)((float)tr * KFF);

    float ol = pid_compute(&pl, (float)tl, sl);
    float or = pid_compute(&pr, (float)tr, sr);

    int16_t pl_w = bl + (int16_t)ol;
    int16_t pr_w = br + (int16_t)or;
    if (pl_w > 8000) pl_w = 8000; if (pl_w < -8000) pl_w = -8000;
    if (pr_w > 8000) pr_w = 8000; if (pr_w < -8000) pr_w = -8000;

    motor_left(pl_w);
    motor_right(pr_w);
}
