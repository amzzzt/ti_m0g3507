/**
 * mode_a2b.c — 从A到B: 循迹 + 小球平衡, 7.5 秒自动停车
 *
 *   状态机 (照搬 b78e161 直角转弯的写法):
 *     S_WAIT_KEY → S_WAIT_2S → S_FOLLOW → S_DONE
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "servo.h"
#include "protocol.h"
#include "ball_control.h"
#include "mode_a2b.h"

#define BASE_SPEED  450
#define AUTO_STOP_MS 7500
#define RAMP_MS     800    /* 起跑/停车缓加速总时长 */

typedef enum {
    S_WAIT_KEY,
    S_WAIT_2S,
    S_FOLLOW,
    S_DONE
} state_t;

static state_t       st;
static uint32_t      t0;
static uint32_t      lap_start;
static uint32_t      lap_time;
static ball_control_t ball;
static uint32_t      blt;

void mode_a2b_init(void)
{
    blt = tick_get();
    ball_control_init(&ball);
    servo_set_angle(90);

    st        = S_WAIT_KEY;
    t0        = 0;
    lap_start = 0;
    lap_time  = 0;

    tft180_show_string(0, 0, "Press KEY1");
}

void mode_a2b_update(void)
{
    uint32_t now = tick_get();

    switch (st)
    {
    case S_WAIT_KEY:
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            st = S_WAIT_2S;
            t0 = now;
            tft180_show_string(0, 0, "Wait 2s...");
        }
        break;

    case S_WAIT_2S:
        if (now - t0 >= 2000) {
            st = S_FOLLOW;
            t0 = now;
            lap_start = now;
            tft180_show_string(0, 0, "FOLLOW   ");
        }
        break;

    case S_FOLLOW: {
        /* 起跑/停车缓加速, 二次曲线: 前慢后快, 减少小球晃动 */
        float ramp;
        uint32_t elapsed = now - t0;
        if (elapsed < RAMP_MS) {
            float f = (float)elapsed / (float)RAMP_MS;
            ramp = f * f;                              /* 起跑: 越往后越快 */
        } else if (elapsed > AUTO_STOP_MS - RAMP_MS) {
            float f = (float)(AUTO_STOP_MS - elapsed) / (float)RAMP_MS;
            ramp = 1.0f - (1.0f - f) * (1.0f - f);    /* 停车: 越往后降得越快 */
            if (ramp < 0.0f) ramp = 0.0f;
        } else {
            ramp = 1.0f;
        }
        int speed = (int)((float)BASE_SPEED * ramp);

        int dev = track_deviation();
        int16_t tgt_l = (int16_t)(speed + dev);
        int16_t tgt_r = (int16_t)(speed - dev);
        motor_control_update(tgt_l, tgt_r);

        float angle = ball_control_get_angle(&ball);
        offset_t o = protocol_get();
        if (o.updated) {
            float dt = (float)(now - blt) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            blt = now;
            ball_control_update(&ball, o.dx, o.dy, o.found, dt);
            angle = ball_control_get_angle(&ball);
            if (!ball_control_is_ok(&ball)) {
                angle *= 0.95f;
                if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
            }
        }
        servo_set_angle((uint8_t)(90.0f + angle));

        if (now - t0 >= AUTO_STOP_MS) {
            motor_stop();
            servo_set_angle(90);
            lap_time = tick_get() - lap_start;
            st = S_DONE;
        }
        break;
    }

    case S_DONE:
        motor_stop();
        break;
    }

    /* TFT 持续显示时间 (只在运行/停止阶段) */
    {
        static uint32_t tf = 0;
        if (now - tf >= 100 && st >= S_FOLLOW) {
            tf = now;
            uint32_t t = (st == S_DONE) ? lap_time : (tick_get() - lap_start);
            char dis[16];
            sprintf(dis, "T:%d.%02d", (int)(t/1000), (int)((t%1000)/10));
            tft180_show_string(0, 0, dis);
        }
    }

    /* 50ms 串口 */
    static uint32_t pt = 0;
    if (now - pt >= 50) {
        pt = now;
        int dev = track_deviation();
        int sl = (int)motor_control_left_speed();
        int sr = (int)motor_control_right_speed();
        uint32_t t = (st == S_DONE) ? lap_time : (tick_get() - lap_start);
        char buf[80];
        sprintf(buf, "S:%d D:%d L:%d R:%d T:%d.%02d\r\n",
                (int)st, dev, sl, sr,
                (int)(t/1000), (int)((t%1000)/10));
        wireless_uart_send_string(buf);
    }
}
