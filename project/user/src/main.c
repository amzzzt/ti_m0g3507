/**
 * main.c — 任务调度: 按键选任务→确认→2s→执行
 *   所有球控统一走 ball_control 模块
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
#include "mode_a2b.h"
#include "ball_seq.h"
#include "init_capture.h"
#include "ball_twopoint.h"

#define TASK_COUNT  8

static const char *task_names[TASK_COUNT] = {
    "0.Ball Ctrl",
    "1.Line+Stop",
    "2.+5cm/-5cm",
    "3.A2B+Ball",
    "4.Line+Ball",
    "5.InitCap+Line",
    "6.TwoPoint",
    "7.InitCap Ball",
};

static void task0_ball_track(void);
static void task1_line_track(void);
static void task2_ball_seq(void);
static void task3_line_ball(void);
static void task4_a2b(void);
static void task5_initcap_line(void);
static void task6_twopoint(void);
static void task7_initcap_ball(void);

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
    for (volatile uint32_t d = 0; d < 4000000; d++);
    tick_start();

    uint8_t num = 0;

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
            case 0: task0_ball_track();    break;
            case 1: task1_line_track();    break;
            case 2: task2_ball_seq();      break;
            case 3: task4_a2b();           break;
            case 4: task3_line_ball();     break;
            case 5: task5_initcap_line();  break;
            case 6: task6_twopoint();       break;
            case 7: task7_initcap_ball();   break;
            }
            tft180_set_color(RGB565_WHITE, RGB565_BLACK);
            tft180_clear();
            tft180_show_string(0, 0, "== Task ==");
            tft180_show_string(0, 24, (char *)task_names[num]);
        }
        for (volatile uint32_t d = 0; d < 5000; d++);
    }
}

/* ================================================================
 * task0: 小球追中心 — 走 ball_control 模块
 * ================================================================ */
static void task0_ball_track(void)
{
    ball_control_t b;
    uint32_t lt = tick_get();
    char buf[80];

    ball_control_init(&b);
    servo_set_angle(90);

    while (1) {
        uint32_t now = tick_get();
        float angle = ball_control_get_angle(&b);
        offset_t o = protocol_get();

        if (o.updated) {
            float dt = (float)(now - lt) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            lt = now;
            ball_control_update(&b, o.dx, o.dy, o.found, dt);
            angle = ball_control_get_angle(&b);
            if (!ball_control_is_ok(&b)) {
                angle *= 0.95f;
                if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
            }
        }

        servo_set_angle((uint8_t)(90.0f + angle));

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            servo_set_angle(90);
            return;
        }

        static uint32_t pt = 0;
        if (now - pt >= 50) {
            pt = now;
            sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n",
                    (int)o.dx, (int)o.dy, (int)b.dx_f, (int)b.vx,
                    (int)angle, ball_control_is_ok(&b));
            wireless_uart_send_string(buf);
        }
    }
}

/* ================================================================
 * task1: 巡线一圈+停车
 * ================================================================ */
static void task1_line_track(void)
{
    mode_line_init();
    mode_line_set_speed(590);
    mode_line_set_ramp_ms(0);
    mode_line_set_stop_delay(0);
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
        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) { key_clear_state(KEY_4); servo_set_angle(90); return; }
    }
}

/* ================================================================
 * task3: 巡线一圈 + 小球平衡, 停车后球控也停
 * ================================================================ */
static void task3_line_ball(void)
{
    ball_control_t b;
    uint32_t blt = tick_get();

    ball_control_init(&b);
    servo_set_angle(90);

    mode_line_init();
    mode_line_set_speed(364);
    mode_line_set_stop_frames(3);
    mode_line_set_auto_stop_ms(33000);
    { uint32_t t0 = tick_get();  /* 记录启动时刻 */

    while (1) {
        uint32_t now = tick_get();

        if (!mode_line_is_stopped()) {
            float angle = ball_control_get_angle(&b);
            offset_t o = protocol_get();

            if (o.updated) {
                float dt = (float)(now - blt) * 0.001f;
                if (dt <= 0.0f) dt = 0.01f;
                blt = now;
                ball_control_update(&b, o.dx, o.dy, o.found, dt);
                angle = ball_control_get_angle(&b);
                if (!ball_control_is_ok(&b)) {
                    angle *= 0.95f;
                    if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
                }
                /* 前3秒极小幅调整 */
                if (now - t0 < 3000) {
                    if (angle >  3.0f) angle =  3.0f;
                    if (angle < -3.0f) angle = -3.0f;
                }
            }
            servo_set_angle((uint8_t)(90.0f + angle));
        }

        mode_line_update();

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            motor_stop();
            servo_set_angle(90);
            return;
        }
    }
    }
}

/* ================================================================
 * task4: 从A到B 循迹 + 小球平衡, 7.5秒自动停
 * ================================================================ */
static void task4_a2b(void)
{
    mode_a2b_init();
    while (1) {
        mode_a2b_update();
        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            motor_stop();
            servo_set_angle(90);
            return;
        }
    }
}

/* ================================================================
 * task5: 抓初始位置 + 巡线一圈 + 小球追那个位置
 * ================================================================ */
static void task5_initcap_line(void)
{
    ball_control_t b;
    uint32_t blt = tick_get();
    char buf[40];

    ball_control_init(&b);
    servo_set_angle(90);

    float target = init_capture_run();
    ball_control_set_target(&b, target);

    sprintf(buf, "Cap: %d", (int)target);
    tft180_show_string(0, 32, buf);

    mode_line_init();
    mode_line_set_speed(364);
    mode_line_set_stop_frames(3);
    mode_line_set_auto_stop_ms(33000);
    { uint32_t t0 = tick_get();

    while (1) {
        uint32_t now = tick_get();

        if (!mode_line_is_stopped()) {
            float angle = ball_control_get_angle(&b);
            offset_t o = protocol_get();

            if (o.updated) {
                float dt = (float)(now - blt) * 0.001f;
                if (dt <= 0.0f) dt = 0.01f;
                blt = now;
                ball_control_update(&b, o.dx, o.dy, o.found, dt);
                angle = ball_control_get_angle(&b);
                if (!ball_control_is_ok(&b)) {
                    angle *= 0.95f;
                    if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
                }
                if (now - t0 < 3000) {
                    if (angle >  3.0f) angle =  3.0f;
                    if (angle < -3.0f) angle = -3.0f;
                }
            }
            servo_set_angle((uint8_t)(90.0f + angle));
        }

        mode_line_update();

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            motor_stop();
            servo_set_angle(90);
            return;
        }
    }
    }
}

/* ================================================================
 * task6: 双点序列 +118(停1s) → -113(停1s)
 * ================================================================ */
static void task6_twopoint(void)
{
    ball_twopoint_init();
    while (1) {
        ball_twopoint_update();
        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            servo_set_angle(90);
            return;
        }
    }
}

/* ================================================================
 * task7: 抓初始位置 → 纯球控追那个点
 * ================================================================ */
static void task7_initcap_ball(void)
{
    ball_control_t b;
    uint32_t lt = tick_get();
    char buf[80];

    ball_control_init(&b);
    servo_set_angle(90);

    float target = init_capture_run();
    ball_control_set_target(&b, target);

    sprintf(buf, "Tgt: %d", (int)target);
    tft180_show_string(0, 32, buf);

    while (1) {
        uint32_t now = tick_get();
        float angle = ball_control_get_angle(&b);
        offset_t o = protocol_get();

        if (o.updated) {
            float dt = (float)(now - lt) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            lt = now;
            ball_control_update(&b, o.dx, o.dy, o.found, dt);
            angle = ball_control_get_angle(&b);
            if (!ball_control_is_ok(&b)) {
                angle *= 0.95f;
                if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
            }
        }

        servo_set_angle((uint8_t)(90.0f + angle));

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            servo_set_angle(90);
            return;
        }

        static uint32_t pt = 0;
        if (now - pt >= 50) {
            pt = now;
            sprintf(buf, "%d,%d,%d,%d,%d,%d,%d\r\n",
                    (int)o.dx, (int)o.dy, (int)b.dx_f, (int)b.vx,
                    (int)angle, ball_control_is_ok(&b),
                    (int)b.target);
            wireless_uart_send_string(buf);
        }
    }
}
