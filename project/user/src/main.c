/**
 * main.c — 巡线 + 启停线检测
 *
 *   IDLE:  等 KEY1
 *   FOLLOW: track_deviation → motor_control_update(400±dev)
 *   启停线: 最右侧三灯(5/6/7)全黑 + 持续50ms → STOP, 显示圈时
 *   STOP:  电机停
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"

typedef enum { S_IDLE, S_FOLLOW, S_STOP } state_t;

static int start_line(void) {
    /* 最右侧三灯全黑 = 启停线 */
    return track_value(5) == 0
        && track_value(6) == 0
        && track_value(7) == 0;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);

    key_init(1);
    track_init();               /* TIMA0 ISR 需要, 必须在 tick_init 前 */
    tick_init();
    wireless_uart_init();
    motor_control_init();
    imu_init();
    tft180_init();
    tft180_clear();
    tick_start();

    state_t st = S_IDLE;
    uint32_t follow_t0 = 0;
    uint32_t line_t0   = 0;
    uint32_t lap_start = 0;
    tft180_show_string(0, 0, "Press KEY1");

    while (1)
    {
        /* === 状态机 === */
        switch (st) {

        case S_IDLE:
            motor_stop();
            if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_1);
                st = S_FOLLOW;
                follow_t0 = tick_get();
                lap_start = tick_get();     /* 开始计时 */
                tft180_show_string(0, 0, "FOLLOW  ");
            }
            break;

        case S_FOLLOW: {
            int dev = track_deviation();
            int tl = 400 + dev;
            int tr = 400 - dev;
            if (tl >  8000) tl =  8000; if (tl < -8000) tl = -8000;
            if (tr >  8000) tr =  8000; if (tr < -8000) tr = -8000;
            motor_control_update((int16_t)tl, (int16_t)tr);

            /* 启停线检测: 前500ms禁止 + 持续50ms消抖 */
            if (tick_get() - follow_t0 > 500) {
                if (start_line()) {
                    if (line_t0 == 0) line_t0 = tick_get();
                    if (tick_get() - line_t0 > 50) {
                        motor_stop();
                        st = S_STOP;
                        tft180_show_string(0, 0, "STOP    ");
                    }
                } else {
                    line_t0 = 0;
                }
            }
            break;
        }

        case S_STOP: {
            motor_stop();
            /* 显示圈时 */
            uint32_t lap_ms = tick_get() - lap_start;
            char dis[16];
            sprintf(dis, "%d.%02ds", (int)(lap_ms/1000), (int)((lap_ms%1000)/10));
            tft180_show_string(0, 1, dis);
            break;
        }
        }

        /* === 50ms 调试打印 === */
        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 50) {
            pt = now;
            int dev = track_deviation();
            int sl = (int)motor_control_left_speed();
            int sr = (int)motor_control_right_speed();
            char buf[64];
            uint32_t elapsed = tick_get() - lap_start;
            sprintf(buf, "S:%d D:%d L:%d R:%d T:%d.%02d\r\n",
                    (int)st, dev, sl, sr,
                    (int)(elapsed/1000), (int)((elapsed%1000)/10));
            wireless_uart_send_string(buf);
        }
    }
}
