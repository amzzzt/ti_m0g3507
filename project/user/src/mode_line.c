/**
 * mode_line.c — 模式1: 巡线 + 启停线检测
 *
 *   IDLE:  等 KEY1
 *   FOLLOW: track_deviation → motor_control_update(400±dev)
 *           启停线: 最右侧三灯(5/6/7)全黑 + 持续50ms → STOP
 *   STOP:  电机停, TFT显示圈时
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "mode_line.h"

typedef enum { S_IDLE, S_FOLLOW, S_STOP } state_t;

static state_t  st;
static uint32_t follow_t0;
static uint32_t line_t0;
static uint32_t lap_start;
static uint32_t lap_time;       /* 冻结的圈时 */

static int is_stop_line(void) {
    return track_value(5) == 0
        && track_value(6) == 0
        && track_value(7) == 0;
}

void mode_line_init(void)
{
    st = S_IDLE;
    follow_t0 = 0;
    line_t0   = 0;
    lap_start = 0;
    lap_time  = 0;
    tft180_show_string(0, 0, "Mode1:Line");
    tft180_show_string(0, 1, "Press KEY1");
}

void mode_line_update(void)
{
    uint32_t now = tick_get();

    switch (st) {

    case S_IDLE:
        motor_stop();
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            st = S_FOLLOW;
            follow_t0 = now;
            lap_start = now;
            tft180_show_string(0, 0, "FOLLOW  ");
            tft180_show_string(0, 1, "        ");
        }
        break;

    case S_FOLLOW: {
        int dev = track_deviation();
        int tl = 600 + dev;
        int tr = 600 - dev;
        if (tl >  8000) tl =  8000; if (tl < -8000) tl = -8000;
        if (tr >  8000) tr =  8000; if (tr < -8000) tr = -8000;
        motor_control_update((int16_t)tl, (int16_t)tr);

        /* 实时显示时间 */
        {
            uint32_t t = now - lap_start;
            char dis[16];
            sprintf(dis, "%d.%02ds", (int)(t/1000), (int)((t%1000)/10));
            tft180_show_string(0, 1, dis);
        }

        /* 前500ms不检测 + 持续50ms消抖 */
        if (now - follow_t0 > 500) {
            if (is_stop_line()) {
                if (line_t0 == 0) line_t0 = now;
                if (now - line_t0 > 50) {
                    motor_stop();
                    st = S_STOP;
                    lap_time = now - lap_start;     /* 冻结 */
                    {
                        char dis[16];
                        sprintf(dis, "%d.%02ds", (int)(lap_time/1000), (int)((lap_time%1000)/10));
                        tft180_show_string(0, 0, "STOP    ");
                        tft180_show_string(0, 1, dis);
                    }
                }
            } else {
                line_t0 = 0;
            }
        }
        break;
    }

    case S_STOP:
        motor_stop();
        break;
    }

    /* === 50ms 调试打印 === */
    static uint32_t pt = 0;
    if (now - pt >= 50) {
        pt = now;
        int dev = track_deviation();
        int sl = (int)motor_control_left_speed();
        int sr = (int)motor_control_right_speed();
        uint32_t elapsed = (st == S_STOP) ? lap_time : (now - lap_start);
        char buf[64];
        sprintf(buf, "S:%d D:%d L:%d R:%d T:%d.%02d\r\n",
                (int)st, dev, sl, sr,
                (int)(elapsed/1000), (int)((elapsed%1000)/10));
        wireless_uart_send_string(buf);
    }
}
