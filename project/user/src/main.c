/**
 * main.c — 小球 PID 调参 (照搬 3a42ffe 框架)
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "servo.h"
#include "protocol.h"
#include "ball_control.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    key_init(1);
    tick_init();
    servo_init();
    wireless_uart_init();
    protocol_init(115200);
    tick_start();

    ball_control_t b;
    uint32_t lt = tick_get();
    char buf[80];

    ball_control_init(&b);
    servo_set_angle(90);

    /* 等 KEY1 后再开始 */
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);

    while (1) {
        uint32_t now = tick_get();
        float angle = ball_control_get_angle(&b);
        offset_t o = protocol_get();

        if (o.updated) {
            float dt = (float)(now - lt) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            lt = now;
            ball_control_update(&b, o.dx, o.dy, o.found, dt);
            angle = ball_control_get_angle(&b);
            if (!ball_control_is_ok(&b)) {
                angle *= 0.95f;
                if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
            }
        }

        servo_set_angle((uint8_t)(90.0f + angle));

        static uint32_t pt = 0;
        if (now - pt >= 50) {
            pt = now;
            sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n",
                    (int)o.dx, (int)o.dy, (int)b.dx_f, (int)b.vx, (int)angle,
                    ball_control_is_ok(&b));
            wireless_uart_send_string(buf);
        }
    }
}
