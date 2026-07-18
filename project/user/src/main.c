/**
 * main.c — Y轴追球 状态机控制
 *
 *   IDLE    : 等待首帧, 电机停
 *   TRACK   : 球可见, PI调速追踪
 *   SEARCH  : 球丢失, 慢转搜索, 超时回IDLE
 *
 *   防抖: 连续3帧零→进入搜索, 连续3帧有效→回到追踪
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"
#include "stepper.h"

/* PI */
#define KP         3.5f
#define KI         0.25f
#define OUT_MAX    800
#define OUT_MIN   -800
#define DEADBAND   10
#define DT_MS      20

/* 状态机 */
#define DEBOUNCE   3       // 连续N帧才切换状态
#define SEARCH_HZ  150     // 搜索转速
#define SEARCH_TO  3000    // 搜索超时 ms

typedef enum { IDLE, TRACK, SEARCH } state_t;

typedef struct {
    float Kp, Ki;
    float integral;
    float prev_err;
} pid_t;

static float pid_step(pid_t *p, float err, float dt) {
    float P = p->Kp * err;
    float D = 0;
    if (dt > 0.001f) D = 0;  // D=0
    p->prev_err = err;

    float I = p->Ki * p->integral;
    float out_u = P + I + D;
    if (!((out_u <= OUT_MIN && err < 0) || (out_u >= OUT_MAX && err > 0)))
        p->integral += err * dt;
    if (p->integral >  100) p->integral =  100;
    if (p->integral < -100) p->integral = -100;
    I = p->Ki * p->integral;

    float out = P + I + D;
    if (out > OUT_MAX) out = OUT_MAX;
    if (out < OUT_MIN) out = OUT_MIN;
    return out;
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    protocol_init(115200);
    filter_init();
    stepper_init(STEP1);

    pid_t    pid      = {KP, KI, 0, 0};
    state_t  state    = IDLE;
    uint16_t cur_hz   = 0;
    uint8_t  cur_dir  = 0;
    uint8_t  last_dir = 0;   // 0=未知 1=右 2=左
    uint32_t next_ms  = 0;
    uint32_t last_ms  = 0;
    uint32_t search_t0= 0;
    uint8_t  lost_cnt = 0;
    uint8_t  found_cnt= 0;
    uint8_t  enabled  = 0;
    uint16_t frm_cnt  = 0;

    while (1) {
        /* === 收数 + 防抖计数 === */
        offset_t o = protocol_get();
        if (o.updated) {
            if (!o.found || (o.dx == 0 && o.dy == 0)) {
                lost_cnt++;
                found_cnt = 0;
            } else {
                found_cnt++;
                lost_cnt = 0;
                filter_update(o.dx, o.dy, tick_get());
                if (!enabled && ++frm_cnt > 20) {
                    stepper_enable(STEP1); enabled = 1;
                    pid.integral = 0; pid.prev_err = 0;
                }
            }
        }

        uint32_t now = tick_get();

        /* === 控制(每 DT_MS) === */
        if ((int32_t)(now - next_ms) >= 0) {
            float dt = (float)(now - last_ms) * 0.001f;
            if (dt <= 0) dt = 0.02f;
            last_ms = now;
            next_ms = now + DT_MS;

            float fy   = (float)filter_y();
            float err  = -fy;
            float out  = 0;

            /* 状态切换 */
            switch (state) {
            case IDLE:
                if (enabled && found_cnt >= DEBOUNCE) {
                    state = TRACK;
                    pid.integral = 0; pid.prev_err = 0;
                }
                break;
            case TRACK:
                if (lost_cnt >= DEBOUNCE) {
                    state = SEARCH;
                    search_t0 = now;
                    pid.integral = 0; pid.prev_err = 0;
                }
                break;
            case SEARCH:
                if (found_cnt >= DEBOUNCE) {
                    state = TRACK;
                    pid.integral = 0; pid.prev_err = 0;
                } else if ((int32_t)(now - search_t0) >= SEARCH_TO) {
                    state = IDLE;
                }
                break;
            }

            /* 状态执行 */
            switch (state) {
            case IDLE:
                if (cur_hz != 0) { stepper_stop(STEP1); cur_hz = 0; }
                break;

            case SEARCH:
                if (last_dir) {
                    uint8_t d = last_dir - 1;
                    if (cur_hz != SEARCH_HZ || cur_dir != d) {
                        stepper_set_dir(STEP1, d);
                        stepper_set_speed(STEP1, SEARCH_HZ);
                        stepper_run(STEP1, SEARCH_HZ);
                        cur_hz = SEARCH_HZ; cur_dir = d;
                    }
                }
                break;

            case TRACK:
                out = pid_step(&pid, err, dt);
                /* 记录方向供搜索 */
                if (fy > 8)      last_dir = 2;
                else if (fy < -8) last_dir = 1;

                if (err > -DEADBAND && err < DEADBAND) {
                    if (cur_hz != 0) { stepper_stop(STEP1); cur_hz = 0; }
                } else {
                    uint8_t  d = (out > 0) ? 1 : 0;
                    uint16_t h = (uint16_t)(out > 0 ? out : -out);
                    if (h < 30) h = 30;
                    if (h > OUT_MAX) h = OUT_MAX;
                    if (h != cur_hz || d != cur_dir) {
                        if (d != cur_dir) stepper_set_dir(STEP1, d);
                        stepper_set_speed(STEP1, h);
                        stepper_run(STEP1, h);
                        cur_hz = h; cur_dir = d;
                    }
                }
                break;
            }

            char buf[80];
            sprintf(buf, "fy=%.0f S=%d L=%d out=%.0f hz=%d\r\n",
                    fy, state, lost_cnt, out, cur_hz);
            wireless_uart_send_string(buf);
        }
        stepper_tick();
    }
}
