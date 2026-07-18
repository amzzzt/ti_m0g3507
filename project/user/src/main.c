/**
 * main.c — 双电机慢速来回90度
 *   M1: A16(TIMA1)  M2: B6(TIMG6)
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "stepper.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    stepper_init(STEP1);
    stepper_init(STEP2);
    stepper_set_speed(STEP1, 800);
    stepper_set_speed(STEP2, 800);
    stepper_enable(STEP1);
    stepper_enable(STEP2);

    uint8_t dir = 1;
    while (1) {
        stepper_rotate(STEP1, dir ? 90.0f : -90.0f);
        stepper_rotate(STEP2, dir ? 90.0f : -90.0f);
        while (stepper_is_running(STEP1) || stepper_is_running(STEP2)) {
            stepper_tick();
        }
        dir = !dir;
    }
}
