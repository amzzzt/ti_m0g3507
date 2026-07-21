/**
 * main.c — 五模式双轴控制
 *
 * CS_SCAN   开机X轴旋转找目标 → 找到切CS_CIRCLE
 * CS_TRACK  快速追踪: PID 50~600Hz
 * CS_LOCK   精准锁定: ae×2, 8~100Hz
 * CS_TRACE  绕框描边: ae×0.5, 4~25Hz
 * CS_CIRCLE 寻圈: ae×1.5, 8~75Hz (丢→追→停)
 */
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

    /* 开机: CS_SCAN旋转找目标 */
    stepper_enable(STEP1); y_ctrl.enabled = 1;
    stepper_enable(STEP2); x_ctrl.enabled = 1;
    control_force_state(&y_ctrl, CS_SCAN);
    control_force_state(&x_ctrl, CS_SCAN);
    uint8_t  scanning = 1;
    uint16_t scan_cnt = 0;

    uint32_t next_ms = 0, last_ms = 0, last_data = 0;
    static int16_t raw_dx, raw_dy;

    while (1) {
        offset_t o = protocol_get();
        if (o.updated) {
            last_data = tick_get();
            raw_dx = o.dx; raw_dy = o.dy;
            uint8_t lost = !o.found;
            uint8_t zero = (o.dx == 0 && o.dy == 0);
            control_feed(&y_ctrl, lost, zero);
            control_feed(&x_ctrl, lost, zero);

            /* 扫描中: 连续10帧有效→切CS_CIRCLE */
            if (scanning && !lost && !zero) {
                if (++scan_cnt > 10) {
                    scanning = 0;
                    filter_reset(o.dx, o.dy);
                    control_force_state(&y_ctrl, CS_CIRCLE);
                    control_force_state(&x_ctrl, CS_CIRCLE);
                }
            } else if (scanning && lost) {
                scan_cnt = 0;
            }

            /* 非扫描: 正常滤波(拐角>15px直跳) */
            if (!scanning && !lost && !zero) {
                int16_t fx0 = filter_x(), fy0 = filter_y();
                int32_t j = (int32_t)(o.dx-fx0)*(o.dx-fx0)+(int32_t)(o.dy-fy0)*(o.dy-fy0);
                if (j > 225) filter_reset(o.dx, o.dy);
                else         filter_update(o.dx, o.dy, tick_get());
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

            /* 收到过数据后500ms无新数据→停转 */
            if (last_data && (int32_t)(now - last_data) > 500) {
                motor_run(STEP1, 0, 0);
                motor_run(STEP2, 0, 0);
            } else {
                motor_run(STEP1, control_get_hz(&y_ctrl), control_get_dir(&y_ctrl));
                motor_run(STEP2, control_get_hz(&x_ctrl), control_get_dir(&x_ctrl));
            }

            {
                static uint32_t lp = 0;
                static const char *names[] = {"IDLE","TRACK","SEARCH","LOCK","TRACE","SCAN","CIRCLE"};
                if ((int32_t)(now - lp) >= 100) {
                    lp = now;
                    char buf[128];
                    sprintf(buf, "r[%d,%d] f[%.0f,%.0f] %s %s h[%d,%d]\r\n",
                            raw_dx, raw_dy, fx, fy,
                            scanning ? "SCAN" : names[y_ctrl.state],
                            (control_is_locked(&y_ctrl) && control_is_locked(&x_ctrl))
                                ? "HOLD" : "",
                            control_get_hz(&y_ctrl), control_get_hz(&x_ctrl));
                    wireless_uart_send_string(buf);
                }
            }
        }
    }
}
