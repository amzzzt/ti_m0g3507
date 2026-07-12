/**
 * control.c — 速度闭环: 编码器 → PID → 电机
 *
 *   每 10ms: encoder_update() → PID → motor_left/right
 */
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "control.h"

#define KFF  6.0f
#define PID_MAX  3000

static pid_t   pl, pr;
static int16_t tl, tr;

void control_init(void)
{
    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();

    pid_init(&pl, 0.5f, 0.02f, 1.0f, PID_MAX);
    pid_init(&pr, 0.5f, 0.02f, 1.0f, PID_MAX);
}

void control_set_target(int16_t l, int16_t r) { tl = l; tr = r; }
int16_t control_target_left(void)  { return tl; }
int16_t control_target_right(void) { return tr; }

void control_update(void)
{
    // 1. 编码器 10ms 更新
    encoder_update();

    // 2. 读速度
    float sl = encoder_left_speed();
    float sr = encoder_right_speed();

    // 3. 前馈
    int16_t bl = (int16_t)((float)tl * KFF);
    int16_t br = (int16_t)((float)tr * KFF);

    // 4. PID
    float ol = pid_compute(&pl, (float)tl, sl);
    float or = pid_compute(&pr, (float)tr, sr);

    // 5. 输出
    int16_t pl = bl + (int16_t)ol;
    int16_t pr = br + (int16_t)or;
    if (pl > 8000) pl = 8000; if (pl < -8000) pl = -8000;
    if (pr > 8000) pr = 8000; if (pr < -8000) pr = -8000;

    motor_left(pl);
    motor_right(pr);
}
