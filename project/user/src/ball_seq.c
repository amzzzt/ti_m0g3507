/**
 * ball_seq.c — 小球序列: 0→+106→-140
 *
 *   从 main.c 原封不动搬过来, 只封装不改逻辑
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
static uint8_t  st = 0, settle = 0;

void ball_seq_init(void)
{
    lt = tick_get();

    ball_control_init(&b);
    servo_set_angle(90);

    /* 等 KEY1 + 2s */
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear();
    tft180_show_string(0, 0, "Press KEY1");
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);
    tft180_show_string(0, 0, "Wait 2s...");
    { uint32_t w0 = tick_get(); while (tick_get() - w0 < 2000); }

    ball_control_set_target(&b, 106.0f);
}

void ball_seq_update(void)
{
    uint32_t now = tick_get();
    float angle = ball_control_get_angle(&b);
    offset_t o = protocol_get();
    char buf[64];

    if (o.updated) {
        float dt = (float)(now - lt) * 0.001f; if (dt <= 0.0f) dt = 0.01f;
        lt = now;
        ball_control_update(&b, o.dx, o.dy, o.found, dt);
        angle = ball_control_get_angle(&b);
        if (!ball_control_is_ok(&b)) { angle *= 0.95f; if (angle < 0.5f && angle > -0.5f) angle = 0.0f; }
    }

    /* 第一个点: 连续 1 秒 (±15px, |vx|<2) */
    if (st == 0 && ball_control_is_ok(&b)) {
        float ae = (b.dx_f > 106.0f) ? (b.dx_f - 106.0f) : (106.0f - b.dx_f);
        float av = (b.vx > 0 ? b.vx : -b.vx);
        if (ae < 15.0f && av < 2.0f) {
            settle++;
            if (settle >= 50) { st = 1; settle = 0; ball_control_set_target(&b, -140.0f); }
        } else { settle = 0; }
    }

    servo_set_angle((uint8_t)(90.0f + angle));

    static uint32_t pt = 0;
    if (now - pt >= 200) { pt = now; sprintf(buf, "S:%d dx:%d vx:%d ang:%d ok:%d\r\n", (int)st, (int)b.dx_f, (int)b.vx, (int)angle, ball_control_is_ok(&b)); wireless_uart_send_string(buf); }
}
