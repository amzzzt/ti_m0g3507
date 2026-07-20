/**
 * main.c — 梯形加减速测试 (全在主函数, 不改模块)
 *
 * 原理: control_update 输出"目标Hz", main.c 用 slew rate 限制后
 *       再送给 motor_run, 模拟梯形速度曲线.
 *       SLEW_UP=20 → 每20ms最多加速20Hz → 1000Hz/s
 *       SLEW_DN=40 → 每20ms最多减速40Hz → 2000Hz/s
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

    uint32_t next_ms = 0, last_ms = 0;
    static int16_t raw_dx, raw_dy;

    while (1) {
        offset_t o = protocol_get();
        if (o.updated) {
            raw_dx = o.dx; raw_dy = o.dy;
            uint8_t lost = !o.found;
            uint8_t zero = (o.dx == 0 && o.dy == 0);
            control_feed(&y_ctrl, lost, zero);
            control_feed(&x_ctrl, lost, zero);
            if (!lost && !zero) {
                filter_update(o.dx, o.dy, tick_get());
                if (!y_ctrl.enabled && ++y_ctrl.frm_cnt > 20) {
                    stepper_enable(STEP1); y_ctrl.enabled = 1;
                    stepper_enable(STEP2); x_ctrl.enabled = 1;
                    /* 默认进 CS_LOCK: ae×2, 8~300Hz */
                    control_force_state(&y_ctrl, CS_LOCK);
                    control_force_state(&x_ctrl, CS_LOCK);
                }
            }
        }

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

            {
                static uint32_t lp = 0;
                if ((int32_t)(now - lp) >= 100) {
                    lp = now;
                    char buf[128];
                    sprintf(buf, "r[%d,%d] f[%.0f,%.0f] st=%d %s h[%d,%d]\r\n",
                            raw_dx, raw_dy, fx, fy,
                            y_ctrl.state,
                            (control_is_locked(&y_ctrl) && control_is_locked(&x_ctrl))
                                ? "HOLD" : "",
                            control_get_hz(&y_ctrl), control_get_hz(&x_ctrl));
                    wireless_uart_send_string(buf);
                }
            }
        }
    }
}
