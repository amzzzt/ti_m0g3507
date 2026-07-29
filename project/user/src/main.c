/**
 * main.c — 步进电机测试: STEP1 正转90° + 反转90°
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "stepper.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    track_init();
    tick_init();
    wireless_uart_init();
    stepper_init(STEP1);
    stepper_set_speed(STEP1, 200);      /* 200Hz */
    stepper_enable(STEP1);
    tft180_init();
    tft180_clear();
    tick_start();

    tft180_show_string(0, 0, "Stepper Test");

    while (1)
    {
        /* 正转90° */
        stepper_rotate(STEP1, 90.0f);

        while (stepper_is_running(STEP1)) {
            stepper_tick();
            char buf[16];
            sprintf(buf, "T:%d R:1", (int)tick_get());
            tft180_show_string(0, 0, buf);
        }
        tft180_show_string(0, 0, "DONE FWD");

        { uint32_t t0 = tick_get(); while (tick_get() - t0 < 1000); }

        /* 反转90° (负角度=反方向) */
        stepper_rotate(STEP1, -90.0f);

        while (stepper_is_running(STEP1)) stepper_tick();
        tft180_show_string(0, 0, "DONE REV");

        { uint32_t t0 = tick_get(); while (tick_get() - t0 < 1000); }
    }
}
