/**
 * ball_seq.c — 小球序列: 固定抬升 → 追+133(2s) → 追-133(3s)
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "servo.h"
#include "protocol.h"
#include "ball_control.h"
#include "ball_seq.h"

static ball_control_t b;
static uint32_t lt;
static uint8_t  st = 0;
static uint32_t phase_t0;

enum { S_TO_P133, S_TO_N133 };

void ball_seq_init(void)
{
    lt = tick_get();

    ball_control_init(&b);
    ball_control_set_target(&b, 133.0f);
    servo_set_angle(90);

    /* 等 KEY1 + 2s */
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear();
    tft180_show_string(0, 0, "Press KEY1");
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);
    tft180_show_string(0, 0, "Wait 2s...");
    { uint32_t w0 = tick_get(); while (tick_get() - w0 < 2000); }
    tft180_show_string(0, 0, "GO +133 ");

    st = S_TO_P133;
    phase_t0 = tick_get();
}

void ball_seq_update(void)
{
    uint32_t now = tick_get();
    float angle = ball_control_get_angle(&b);
    offset_t o = protocol_get();
    char buf[64];

    if (o.updated && st >= S_TO_P133) {
        /* 不对称 PID: 追正目标强力, 追负目标(近舵机)柔和 */
        if (b.target > 0) {
            b.kp = 0.082f;  b.kd = 3.3f;  b.max_angle = 10.0f;  b.d_max = 16.0f;
            b.ki = 0.10f;   b.i_max = 3.0f;
        } else {
            b.kp = 0.060f;  b.kd = 8.0f;  b.max_angle = 7.0f;   b.d_max = 20.0f;
            b.ki = 0.12f;   b.i_max = 4.0f;
        }
        float dt = (float)(now - lt) * 0.001f; if (dt <= 0.0f) dt = 0.01f;
        lt = now;
        ball_control_update(&b, o.dx, o.dy, o.found, dt);
        angle = ball_control_get_angle(&b);
        if (!ball_control_is_ok(&b)) { angle *= 0.95f; if (angle < 0.5f && angle > -0.5f) angle = 0.0f; }
    }

    switch (st) {
    case S_TO_P133:
        if (now - phase_t0 >= 2420) {
            ball_control_set_target(&b, -133.0f);
            st = S_TO_N133;
            phase_t0 = now;
        }
        break;
    case S_TO_N133:
        if (now - phase_t0 >= 3000) {
            /* hold at -133 */
        }
        break;
    }

    if (st >= S_TO_P133)
        servo_set_angle((uint8_t)(90.0f + angle));

    static uint32_t pt = 0;
    if (now - pt >= 200) {
        pt = now;
        sprintf(buf, "S:%d dx:%d vx:%d ang:%d ok:%d tgt:%d\r\n",
                (int)st, (int)b.dx_f, (int)b.vx, (int)angle,
                ball_control_is_ok(&b), (int)b.target);
        wireless_uart_send_string(buf);
        char ds[16];
        sprintf(ds, "S%d T%d  ", (int)st, (int)b.target);
        tft180_show_string(0, 1, ds);
    }
}
