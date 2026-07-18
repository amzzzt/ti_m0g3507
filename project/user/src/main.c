/**
 * main.c — 双轴追球
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"
#include "stepper.h"
#include "control.h"

static void motor_run(stepper_id_t id, uint16_t hz, uint8_t dir) {
    if (hz) {
        stepper_set_dir(id, dir);
        stepper_set_speed(id, hz);
        stepper_run(id, hz);
    } else {
        stepper_stop(id);
    }
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    protocol_init(115200);
    filter_init();
    stepper_init(STEP1);
    stepper_init(STEP2);

    control_t y_ctrl, x_ctrl;
    control_init(&y_ctrl, STEP1, 3.5f, 0.2f);
    control_init(&x_ctrl, STEP2, 3.5f, 0.2f);

    uint32_t next_ms = 0, last_ms = 0;

    while (1) {
        /* 数据接收 */
        offset_t o = protocol_get();
        if (o.updated) {
            uint8_t lost = !o.found;
            uint8_t zero = (o.dx == 0 && o.dy == 0);
            control_feed(&y_ctrl, lost, zero);
            control_feed(&x_ctrl, lost, zero);
            if (!lost && !zero) {
                filter_update(o.dx, o.dy, tick_get());
                if (!y_ctrl.enabled && ++y_ctrl.frm_cnt > 20)
                    { stepper_enable(STEP1); y_ctrl.enabled = 1;
                      stepper_enable(STEP2); x_ctrl.enabled = 1; }
            }
        }

        /* 控制更新(20ms) */
        uint32_t now = tick_get();
        if ((int32_t)(now - next_ms) >= 0) {
            float dt = (float)(now - last_ms) * 0.001f;
            if (dt <= 0) dt = 0.02f;
            last_ms = now; next_ms = now + 20;

            float fx = (float)filter_x(), fy = (float)filter_y();
            control_update(&y_ctrl, fy, -fy, dt, now);
            control_update(&x_ctrl, fx,  fx, dt, now);

            motor_run(STEP1, control_get_hz(&y_ctrl), control_get_dir(&y_ctrl));
            motor_run(STEP2, control_get_hz(&x_ctrl), control_get_dir(&x_ctrl));

            char buf[100];
            sprintf(buf, "fy=%.0f st=%d hY=%d | fx=%.0f st=%d hX=%d\r\n",
                    fy, y_ctrl.state, control_get_hz(&y_ctrl),
                    fx, x_ctrl.state, control_get_hz(&x_ctrl));
            wireless_uart_send_string(buf);
        }
    }
}
