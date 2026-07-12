/**
 * main.c — 灰度寻迹测试版
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "track.h"

#define BASE    250
#define KP      2.0f
#define DEAD    10         // 死区
#define STEER_MAX 400      // 转向限幅
#define KFF     7.0f
#define PID_MAX 3000

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();

    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();
    track_init();

    pid_t pl, pr;
    pid_init(&pl, 3.0f, 0.1f, 2.0f, PID_MAX);
    pid_init(&pr, 3.0f, 0.1f, 2.0f, PID_MAX);

    float dev_f = 0;    // 偏差低通值

    while (1)
    {
        // ---- 灰度采样+偏差滤波 ----
        track_sample();
        int raw = track_position();
        dev_f = 0.15f * (float)raw + 0.85f * dev_f;
        int dev = (int)dev_f;

        // P转向 + 死区 + 限幅
        int steer = (int)(KP * (float)dev);
        if (steer > -DEAD && steer < DEAD)   steer = 0;
        if (steer >  STEER_MAX) steer =  STEER_MAX;
        if (steer < -STEER_MAX) steer = -STEER_MAX;

        int tgt_l = BASE - steer;
        int tgt_r = BASE + steer;

        // ---- 速度闭环 ----
        encoder_update();
        float sl = encoder_left_speed();
        float sr = encoder_right_speed();

        int out_l = (int)pid_compute(&pl, (float)tgt_l, sl);
        int out_r = (int)pid_compute(&pr, (float)tgt_r, sr);

        int pwm_l = (int)((float)tgt_l * KFF) + out_l;
        int pwm_r = (int)((float)tgt_r * KFF) + out_r;
        if (pwm_l >  8000) pwm_l =  8000;
        if (pwm_l < -8000) pwm_l = -8000;
        if (pwm_r >  8000) pwm_r =  8000;
        if (pwm_r < -8000) pwm_r = -8000;
        motor_left(pwm_l);
        motor_right(pwm_r);

        // ---- 打印: 偏差, 左速度, 右速度 (200ms一次) ----
        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 200) {
            pt = now;
            int il = (int)sl, ir = (int)sr;
            if (il < -999) il = -999; if (il > 9999) il = 9999;
            if (ir < -999) ir = -999; if (ir > 9999) ir = 9999;
            char buf[25];
            sprintf(buf, "%d,%d,%d\r\n", dev, il, ir);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }

        system_delay_ms(10);
    }
}
