/**
 * main.c — 任务调度: 按键选任务→确认→2s→执行
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "servo.h"
#include "protocol.h"
#include "track.h"
#include "motor.h"
#include "mode_line.h"
#include "ball_control.h"
#include "ball_seq.h"

/* ========== 球控参数 ========== */
#define ALPHA       0.70f
#define ALPHA_V     0.95f
#define JUMP_MAX    150.0f
#define V_DECAY     0.85f
#define LOST_MAX    8
#define KP          0.04f
#define KD          2.20f
#define D_MAX       16.0f
#define SLEW_MAX    6.0f
#define MAX_ANGLE   16.0f
#define DB_INNER    10.0f
#define DB_OUTER    20.0f
#define KI          0.05f
#define I_MAX       5.0f
#define ARRIVE_DB   25.0f
#define ARRIVE_VMAX 5.0f
#define SETTLE_125  10
#define TASK_COUNT  5

static const char *task_names[TASK_COUNT] = {
    "0.Ball Ctrl",
    "1.Line+Stop",
    "2.+5cm/-5cm",
    "3.(TODO)",
    "4.(TODO)",
};

static void task0_ball_track(void);
static void task1_line_track(void);
static void task2_ball_seq(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    wireless_uart_init();
    track_init();
    motor_control_init();
    servo_init();
    protocol_init(115200);
    tft180_init();
    tft180_clear();
    for (volatile uint32_t d = 0; d < 4000000; d++);  /* DMP 预热延迟 */
    tick_start();

    uint8_t num = 0;
    char    dis[16];

    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear();
    tft180_show_string(0, 0, "== Task ==");
    tft180_show_string(0, 24, (char *)task_names[num]);

    while (1) {
        if (KEY_SHORT_PRESS == key_get_state(KEY_3)) {
            key_clear_state(KEY_3);
            if (num < TASK_COUNT - 1) { num++; tft180_show_string(0, 24, (char *)task_names[num]); }
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_2)) {
            key_clear_state(KEY_2);
            if (num > 0) { num--; tft180_show_string(0, 24, (char *)task_names[num]); }
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            switch (num) {
            case 0: task0_ball_track(); break;
            case 1: task1_line_track(); break;
            case 2: task2_ball_seq();  break;
            }
            /* 任务返回, 恢复菜单 */
            tft180_set_color(RGB565_WHITE, RGB565_BLACK);
            tft180_clear();
            tft180_show_string(0, 0, "== Task ==");
            tft180_show_string(0, 24, (char *)task_names[num]);
        }

        for (volatile uint32_t d = 0; d < 5000; d++);  /* 微延迟防过冲 */
    }
}

/* ================================================================
 * task0: 小球追中心 (原版逻辑, 进任务后等确定再跑)
 * ================================================================ */
static void task0_ball_track(void)
{
    float   dx_f = 0.0f, vx = 0.0f, last_angle = 0.0f, integral = 0.0f;
    int16_t raw_prev = 0, prev = 0;
    uint8_t lost = 0;
    int     ball_ok = 0;
    uint32_t last_tick = tick_get();
    char    buf[80];

    while (1) {
        uint32_t now = tick_get();
        float angle = last_angle;
        offset_t o = protocol_get();

        if (o.updated) {
            float dt = (float)(now - last_tick) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            last_tick = now;
            int valid = 0;

            if (o.found && o.dx > -400 && o.dx < 400) {
                int16_t jump = (o.dx > raw_prev) ? (o.dx - raw_prev) : (raw_prev - o.dx);
                raw_prev = o.dx;
                if (ball_ok == 0 || jump <= JUMP_MAX) {
                    dx_f = dx_f * (1.0f - ALPHA) + (float)o.dx * ALPHA;
                    {
                        float cam_v = (float)o.dy;
                        if (cam_v > -40 && cam_v < 40) {
                            if (ball_ok == 0) vx = cam_v;
                            else vx = vx * (1.0f - ALPHA_V) + cam_v * ALPHA_V;
                        }
                    }
                    prev = (int16_t)dx_f; lost = 0; valid = 1; ball_ok = 1;

                    float error = dx_f, ae = (error > 0 ? error : -error);
                    float av = (vx > 0 ? vx : -vx);

                    if (ae < DB_INNER) { angle = 0.0f; integral = 0.0f; }
                    else {
                        if (ae < 120.0f) {
                            integral += error * dt * KI;
                            if (integral > I_MAX) integral = I_MAX;
                            if (integral < -I_MAX) integral = -I_MAX;
                        } else integral = 0.0f;
                        float d_term = KD * vx;
                        if (d_term > D_MAX) d_term = D_MAX;
                        if (d_term < -D_MAX) d_term = -D_MAX;
                        angle = KP * error + d_term + integral;
                        if (angle > MAX_ANGLE) angle = MAX_ANGLE;
                        if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
                        if (ae < DB_OUTER) {
                            float limit;
                            if (av < 1.5f) limit = 0.5f;
                            else if (av < 3.0f) limit = 1.0f;
                            else if (av < 6.0f) limit = 2.0f;
                            else limit = 4.0f;
                            limit *= (ae - DB_INNER) / (DB_OUTER - DB_INNER);
                            if (angle > limit) angle = limit;
                            if (angle < -limit) angle = -limit;
                        }
                    }
                    float delta = angle - last_angle;
                    if (delta > SLEW_MAX) delta = SLEW_MAX;
                    if (delta < -SLEW_MAX) delta = -SLEW_MAX;
                    angle = last_angle + delta;
                    last_angle = angle;
                }
            }
            if (!valid) {
                if (lost < LOST_MAX) { dx_f += vx * dt; prev = (int16_t)dx_f; vx *= V_DECAY; lost++; }
                else { dx_f *= 0.9f; if (dx_f < 3.0f && dx_f > -3.0f) { dx_f = 0.0f; lost = 0; ball_ok = 0; } vx = 0.0f; }
            }
        }
        if (!ball_ok && o.updated) { angle *= 0.95f; if (angle < 0.5f && angle > -0.5f) angle = 0.0f; last_angle = angle; }
        servo_set_angle((uint8_t)(90.0f + angle));

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) { key_clear_state(KEY_4); servo_set_angle(90); return; }

        static uint32_t pt = 0;
        if (now - pt >= 200) { pt = now; sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n", (int)o.dx, (int)o.dy, prev, (int)vx, (int)angle, ball_ok); wireless_uart_send_string(buf); }
    }
}

/* ================================================================
 * task1: 巡线一圈+停车
 * ================================================================ */
static void task1_line_track(void)
{
    mode_line_init();
    while (1) {
        mode_line_update();
        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) { key_clear_state(KEY_4); motor_stop(); return; }
    }
}

/* ================================================================
 * task2: 小球 0→+106→-140
 * ================================================================ */
static void task2_ball_seq(void)
{
    ball_seq_init();
    while (1) {
        ball_seq_update();
        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            servo_set_angle(90);
            return;
        }
    }
}
