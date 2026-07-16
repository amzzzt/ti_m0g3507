/**
 * main.c — stepper motor 1, 533Hz target
 */
#include "zf_common_headfile.h"
#include "zf_driver_pwm.h"
#include "stepper.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();

    char buf[50];
    sprintf(buf, "%d\r\n", 123);
    wireless_uart_send_string(buf);

    // 电机1 暂不启动
    // stepper_init(STEP1);
    // stepper_enable(STEP1);
    // stepper_set_dir(STEP1, 1);

    stepper_init(STEP2);
    stepper_enable(STEP2);
    stepper_set_dir(STEP2, 1);

    sprintf(buf, "%d\r\n", 456);
    wireless_uart_send_string(buf);

    // 电机2 持续旋转 ~533Hz
    pwm_init(PWM_TIM_A1_CH0_A15, 1066, 5000);

    while (1);
}
