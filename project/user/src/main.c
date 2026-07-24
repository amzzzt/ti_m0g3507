/**
 * main.c — 精密锁定 CS_LOCK
 */
// ============ 直角转弯测试(注释保留) ============
// #include "zf_common_headfile.h"
// #include "zf_device_wireless_uart.h"
// #include "zf_device_imu660rc.h"
// #include "stepper.h"
// static float yaw_read(void) {
//     float y = imu660rc_yaw;
//     if (y != y || y > 1e4f || y < -1e4f) y = 0;
//     return y;
// }
// int main(void) {
//     clock_init(SYSTEM_CLOCK_80M);
//     wireless_uart_init();
//     stepper_init(STEP2);
//     stepper_enable(STEP2);
//     imu660rc_init(IMU660RC_QUARTERNION_60HZ);
//     system_delay_ms(500);
//     float yaw_start = yaw_read();
//     float target = yaw_start + 90.0f;
//     if (target >= 360.0f) target -= 360.0f;
//     stepper_set_dir(STEP2, 1);
//     stepper_run(STEP2, 100);
//     char buf[64];
//     sprintf(buf, "START %.1f -> %.1f\r\n", yaw_start, target);
//     wireless_uart_send_string(buf);
//     while (1) {
//         system_delay_ms(20);
//         float yaw = yaw_read();
//         float diff = yaw - yaw_start;
//         if (diff < 0) diff += 360.0f;
//         if (diff >= 88.0f) {
//             stepper_stop(STEP2);
//             sprintf(buf, "DONE %.1f\r\n", yaw);
//             wireless_uart_send_string(buf);
//             while (1) system_delay_ms(100);
//         }
//     }
// }
// ============ 直角转弯测试结束 ============

#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"
#include "stepper.h"
#include "control.h"

static void motor_run(stepper_id_t id, uint16_t hz, uint8_t dir) {
    if (hz) { stepper_set_dir(id, dir); stepper_set_speed(id, hz); stepper_run(id, hz); }
    else    { stepper_stop(id); }
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    gpio_init(A12, GPO, 1, GPO_PUSH_PULL);
    gpio_init(A8,  GPO, 1, GPO_PUSH_PULL);
    wireless_uart_init();
    protocol_init(115200);
    filter_init();
    stepper_init(STEP1);
    stepper_init(STEP2);

    control_t y_ctrl, x_ctrl;
    control_init(&y_ctrl, STEP1, 3.5f, 0.2f);
    control_init(&x_ctrl, STEP2, 3.5f, 0.2f);

    /* 开机进精准锁定 */
    stepper_enable(STEP1); y_ctrl.enabled = 1;
    stepper_enable(STEP2); x_ctrl.enabled = 1;
    control_force_state(&y_ctrl, CS_LOCK);
    control_force_state(&x_ctrl, CS_LOCK);

    static int16_t raw_dx, raw_dy;

    while (1) {
        offset_t o = protocol_get();
        if (o.updated) {
            raw_dx = o.dx; raw_dy = o.dy;
            uint8_t lost = !o.found;
            uint8_t zero = (o.dx == 0 && o.dy == 0);
            if (!lost && !zero) filter_update(o.dx, o.dy, tick_get());
        }

        /* 控制更新: 全速跑 */
        {
            static uint32_t ctrl_last = 0;
            uint32_t now = tick_get();
            float dt = (float)(now - ctrl_last) * 0.001f;
            if (dt <= 0 || dt > 1.0f) dt = 0.02f;
            ctrl_last = now;

            float fx = (float)filter_x(), fy = (float)filter_y();

            control_update(&y_ctrl, fy, -fy, dt, now);
            control_update(&x_ctrl, fx,  fx, dt, now);

            motor_run(STEP1, control_get_hz(&y_ctrl), control_get_dir(&y_ctrl));
            motor_run(STEP2, control_get_hz(&x_ctrl), control_get_dir(&x_ctrl));

            /* 100ms 调试打印 */
            {
                static uint32_t lp = 0;
                if ((int32_t)(now - lp) >= 100) {
                    lp = now;
                    char buf[128];
                    sprintf(buf, "r[%d,%d] f[%.0f,%.0f] LOCK %s h[%d,%d]\r\n",
                            raw_dx, raw_dy, fx, fy,
                            (control_is_locked(&y_ctrl) && control_is_locked(&x_ctrl))
                                ? "HOLD" : "",
                            control_get_hz(&y_ctrl), control_get_hz(&x_ctrl));
                    wireless_uart_send_string(buf);
                }
            }
        }
    }
}
