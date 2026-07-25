/**
 * main.c — 直流电机速度闭环 + 波形测试
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

#define TARGET  400
#define KP      1.0f
#define KI      0.08f
#define KD      3.0f
#define PID_MAX 8000

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    motor_init();
    encoder_init();
    encoder_clear();
    encoder_filter_reset();

    pid_t pl, pr;
    pid_init(&pl, KP, KI, KD, PID_MAX);
    pid_init(&pr, KP, KI, KD, PID_MAX);

    while (1) {
        encoder_update();

        int sl = (int)encoder_left_speed();
        int sr = (int)encoder_right_speed();
        if (sl < 0) sl = -sl;
        if (sr < 0) sr = -sr;

        /* PID 速度闭环 */
        float out_l = pid_compute(&pl, (float)TARGET, (float)sl);
        float out_r = pid_compute(&pr, (float)TARGET, (float)sr);
        int pwm_l = (int)((float)TARGET * 7.0f + out_l);
        int pwm_r = (int)((float)TARGET * 7.0f + out_r);
        if (pwm_l >  8000) pwm_l =  8000;
        if (pwm_l < -8000) pwm_l = -8000;
        if (pwm_r >  8000) pwm_r =  8000;
        if (pwm_r < -8000) pwm_r = -8000;
        motor_left(pwm_l);
        motor_right(pwm_r);

        static uint32_t pt = 0;
        uint32_t now = tick_get();
        if (now - pt >= 20) {
            pt = now;
            char buf[48];
            sprintf(buf, "%d,%d,%d\r\n", TARGET, sl, sr);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
        system_delay_ms(10);
    }
}
