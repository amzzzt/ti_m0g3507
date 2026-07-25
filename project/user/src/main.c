/**
 * main.c — 直流电机正转测试
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "motor.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    motor_init();

    motor_left(3000);
    motor_right(3000);

    while (1);
}
