/**
 * main.c — 双电机时钟补偿测试
 *   M1(TIMA1): 2000→实际~1000Hz   M2(TIMG6): 1000→实际~1000Hz
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "stepper.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    stepper_init(STEP1);
    stepper_init(STEP2);
    stepper_set_speed(STEP1, 2000);
    stepper_set_speed(STEP2, 1000);
    stepper_enable(STEP1);
    stepper_enable(STEP2);
    stepper_set_dir(STEP1, 1);
    stepper_set_dir(STEP2, 1);

    while (1) {
        stepper_rotate(STEP1, 3600.0f);
        stepper_rotate(STEP2, 3600.0f);
        while (stepper_is_running(STEP1) || stepper_is_running(STEP2)) {
            stepper_tick();
        }
    }
}
