/**
 * ball_seq.c — 小球序列: 固定抬升→+118 → 等1s → 追-113
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
static uint32_t kick_t0;

enum { S_KICK, S_PAUSE, S_WAIT, S_TO_N118 };

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

    /* 固定抬升: 左倾推球向右到+118区域, 持续300ms */
    st = S_KICK;
    kick_t0 = tick_get();
    servo_set_angle(77);   /* 90-13=77°, 左倾 */
}

void ball_seq_update(void)
{
    uint32_t now = tick_get();
    float angle = ball_control_get_angle(&b);
    offset_t o = protocol_get();
    char buf[64];

    if (o.updated && st >= S_WAIT) {
        float dt = (float)(now - lt) * 0.001f; if (dt <= 0.0f) dt = 0.01f;
        lt = now;
        ball_control_update(&b, o.dx, o.dy, o.found, dt);
        angle = ball_control_get_angle(&b);
        if (!ball_control_is_ok(&b)) { angle *= 0.95f; if (angle < 0.5f && angle > -0.5f) angle = 0.0f; }
    }

    switch (st) {
    case S_KICK:
        if (now - kick_t0 >= 300) {
            servo_set_angle(90);
            st = S_PAUSE;
            kick_t0 = now;
        }
        break;
    case S_PAUSE:
        if (now - kick_t0 >= 300) {
            ball_control_set_target(&b, -113.0f);
            st = S_WAIT;
            kick_t0 = now;
        }
        break;
    case S_WAIT:
        if (now - kick_t0 >= 1000) {
            st = S_TO_N118;
        }
        break;
    case S_TO_N118:
        break;
    }

    if (st >= S_WAIT)
        servo_set_angle((uint8_t)(90.0f + angle));

    static uint32_t pt = 0;
    if (now - pt >= 200) {
        pt = now;
        sprintf(buf, "S:%d dx:%d vx:%d ang:%d ok:%d\r\n",
                (int)st, (int)b.dx_f, (int)b.vx, (int)angle, ball_control_is_ok(&b));
        wireless_uart_send_string(buf);
    }
}
