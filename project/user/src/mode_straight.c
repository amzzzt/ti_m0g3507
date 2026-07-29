/**
 * mode_straight.c — 模式2: 直线 → 遇右转弯停
 *
 *   IDLE: 等 KEY1
 *   GO:   巡线直走, 检测右转弯(5/6/7见黑) → STOP
 *   STOP: 电机停, 冻结圈时
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "mode_straight.h"

typedef enum { S_IDLE, S_GO, S_COAST, S_STOP } state_t;

static state_t  st;
static uint32_t go_t0;
static uint32_t turn_t0;
static uint32_t coast_t0;
static uint32_t lap_start;
static uint32_t lap_time;

/* 右转弯: 右侧三灯见黑 */
static int is_right_turn(void) {
    return track_value(5) == 0
        || track_value(6) == 0
        || track_value(7) == 0;
}

void mode_straight_init(void)
{
    st = S_IDLE;
    go_t0 = 0;
    turn_t0 = 0;
    lap_start = 0;
    lap_time  = 0;
    tft180_show_string(0, 0, "Mode2:Strt");
    tft180_show_string(0, 1, "Press KEY1");
}

void mode_straight_update(void)
{
    uint32_t now = tick_get();

    switch (st) {

    case S_IDLE:
        motor_stop();
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            st = S_GO;
            go_t0 = now;
            lap_start = now;
            tft180_show_string(0, 0, "GO      ");
            tft180_show_string(0, 1, "        ");
        }
        break;

    case S_GO: {
        int dev = track_deviation();
        int tl = 420 + dev;
        int tr = 420 - dev;
        if (tl >  8000) tl =  8000; if (tl < -8000) tl = -8000;
        if (tr >  8000) tr =  8000; if (tr < -8000) tr = -8000;
        motor_control_update((int16_t)tl, (int16_t)tr);

        /* 实时时间 */
        {
            uint32_t t = now - lap_start;
            char dis[16];
            sprintf(dis, "%d.%02ds", (int)(t/1000), (int)((t%1000)/10));
            tft180_show_string(0, 1, dis);
        }

        /* 前500ms跳过(避开启停线) + 20ms消抖 */
        if (now - go_t0 > 500) {
            if (is_right_turn()) {
                if (turn_t0 == 0) turn_t0 = now;
                if (now - turn_t0 > 20) {
                    st = S_COAST;
                    coast_t0 = now;
                    tft180_show_string(0, 0, "COAST   ");
                }
            } else {
                turn_t0 = 0;
            }
        }
        break;
    }

    case S_COAST: {
        /* 转弯后继续巡线 0.3 秒, 车身完全进弯 */
        int dev = track_deviation();
        int tl = 420 + dev;
        int tr = 420 - dev;
        if (tl >  8000) tl =  8000; if (tl < -8000) tl = -8000;
        if (tr >  8000) tr =  8000; if (tr < -8000) tr = -8000;
        motor_control_update((int16_t)tl, (int16_t)tr);

        {
            uint32_t t = now - lap_start;
            char dis[16];
            sprintf(dis, "%d.%02ds", (int)(t/1000), (int)((t%1000)/10));
            tft180_show_string(0, 1, dis);
        }

        if (now - coast_t0 > 300) {
            motor_stop();
            st = S_STOP;
            lap_time = now - lap_start;
            {
                char dis[16];
                sprintf(dis, "%d.%02ds", (int)(lap_time/1000), (int)((lap_time%1000)/10));
                tft180_show_string(0, 0, "STOP    ");
                tft180_show_string(0, 1, dis);
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
