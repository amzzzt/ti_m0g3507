/**
 * mode_line.c — 巡线 + 灰度停车
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "mode_line.h"

static int base_speed = 590;
static int stop_frames = 3;

void mode_line_set_speed(int s)       { base_speed  = s; }
void mode_line_set_stop_frames(int n) { stop_frames = n; }

static uint32_t lap_start;
static uint32_t follow_t0;
static int      stopped;
static int      stop_pending;
static uint32_t stop_t0;
static uint32_t lap_time;       /* 最终圈时(ms), 停后冻结 */
static int      line_cnt;
static uint32_t lost_t0;        /* 丢线开始时刻 */

void mode_line_init(void)
{
    base_speed  = 590;
    stop_frames = 3;
    tft180_show_string(0, 0, "Press KEY1");
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);

    motor_stop();
    tft180_show_string(0, 0, "Wait 2s...");
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < 2000);

    lap_start = tick_get();
    follow_t0 = lap_start;
    stopped      = 0;
    stop_pending = 0;
    line_cnt     = 0;
    lost_t0      = 0;
}

/* TIMA0 ISR 每1ms调用: 用原始值快速检测停车线 */
void mode_line_stop_isr(void)
{
    if (stopped || stop_pending) return;

    uint32_t elapsed = tick_get() - lap_start;
    if (elapsed < 15000) return;

    /* 通道2~7, 原始值(不过滤), 0=黑 */
    int black = 0;
    for (int i = 2; i < 8; i++)
        if (track_value_raw(i) == 0) black++;

    if (black >= 4) {
        line_cnt++;
        if (line_cnt >= stop_frames) {
            stop_pending = 1;           /* 检测到线, 再跑0.3s */
            stop_t0 = tick_get();
        }
    } else {
        line_cnt = 0;
    }
}

void mode_line_update(void)
{
    uint32_t now = tick_get();

    int dev = track_deviation();
    int16_t tgt_l = (int16_t)(base_speed + dev);
    int16_t tgt_r = (int16_t)(base_speed - dev);

    if (!stopped) {
        /* 丢线检测: 8 路全白持续 500ms → 停车 */
        int seen = 0;
        for (int i = 0; i < 8; i++)
            if (track_value_raw(i) == 0) { seen = 1; break; }
        if (!seen) {
            if (lost_t0 == 0) lost_t0 = now;
            if (now - lost_t0 >= 500) {
                motor_stop();
                stopped  = 1;
                lap_time = now - lap_start;
            }
        } else {
            lost_t0 = 0;
        }

        if (!stopped) {
            motor_control_update(tgt_l, tgt_r);
        }

        /* 检测到停车线后继续跑0.3秒再停 */
        if (stop_pending && now - stop_t0 >= 300) {
            motor_stop();
            stopped  = 1;
            lap_time = now - lap_start;     /* 冻结圈时 */
        }
    }

    /* === 50ms: 灰度停车检测 + 串口 + TFT === */
    static uint32_t pt = 0;
    if (now - pt >= 50) {
        pt = now;
        int sl = (int)motor_control_left_speed();
        int sr = (int)motor_control_right_speed();
        uint32_t elapsed = stopped ? lap_time : (now - lap_start);

        /* ISR已处理停车检测, 这里只处理TFT显示 */
        if (stopped) {
            tft180_show_string(0, 0, "STOP    ");
        }

        /* 读取各通道值用于调试 */
        int v[8];
        for (int i = 0; i < 8; i++) v[i] = track_value(i);

        char buf[100];
        sprintf(buf, "%d%d%d%d%d%d%d%d C:%d %s T:%d.%02d\r\n",
                v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],
                line_cnt, stopped ? "STOP" : "",
                (int)(elapsed/1000), (int)((elapsed%1000)/10));
        wireless_uart_send_string(buf);

        char d1[24];
        sprintf(d1, "C:%d T:%d.%02d", line_cnt,
                (int)(elapsed/1000), (int)((elapsed%1000)/10));
        tft180_show_string(0, 1, d1);
    }
}

int mode_line_is_stopped(void)
{
    return stopped;
}
