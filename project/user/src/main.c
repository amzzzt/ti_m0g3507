/**
 * main.c — 双轴追球 固定转速+PID方向决策
 *   两轴统一160Hz, PID输出定方向/启停
 *   丢球搜索: 来回各0.5s, 总1s后停, 不缠线
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"
#include "stepper.h"

#define FIXED_HZ   160
#define KP         3.0f
#define KI         0.15f
#define KD         0.0f
#define OUT_MAX    500
#define DB_STOP    4
#define DB_START   12
#define DT_MS      20
#define DEBOUNCE   15
#define SEARCH_HZ  150
#define SEARCH_MS  500

typedef enum { IDLE, TRACK, SEARCH } state_t;

typedef struct {
    float Kp, Ki, Kd, integral, prev_err;
} pid_t;

typedef struct {
    stepper_id_t motor;
    pid_t   pid;
    state_t state;
    uint8_t  cur_dir;
    uint32_t search_t0;
    uint8_t  search_phase;
    uint8_t  lost_cnt, found_cnt, enabled;
    uint16_t frm_cnt;
    uint8_t  stopped;
    uint8_t  running;
} axis_t;

static float pid_step(pid_t *p, float err, float dt) {
    float P = p->Kp * err;
    p->integral += err * dt;
    if (p->integral >  80) p->integral =  80;
    if (p->integral < -80) p->integral = -80;
    float I = p->Ki * p->integral;
    float out = P + I;
    if (out >  OUT_MAX) out =  OUT_MAX;
    if (out < -OUT_MAX) out = -OUT_MAX;

    /* 输出饱和时冻结积分 */
    if ((out >= OUT_MAX && err > 0) || (out <= -OUT_MAX && err < 0))
        p->integral -= err * dt;
    return out;
}

static void axis_calc(axis_t *a, float fv, float err, float dt, uint32_t now,
                      uint8_t *dir, uint8_t *run) {
    *dir = 0; *run = 0;

    /* 状态切换 */
    switch (a->state) {
    case IDLE:
        if (a->enabled && a->found_cnt >= DEBOUNCE) {
            a->state = TRACK; a->stopped = 0; a->pid.integral = 0;
        }
        break;
    case TRACK:
        if (a->lost_cnt >= DEBOUNCE) {
            a->state = SEARCH; a->search_t0 = now; a->search_phase = 0;
            a->pid.integral = 0; a->stopped = 0;
        }
        break;
    case SEARCH:
        if (a->found_cnt >= DEBOUNCE) {
            a->state = TRACK; a->pid.integral = 0; a->stopped = 0;
        } else if (a->search_phase < 2) {
            uint32_t el = now - a->search_t0;
            if (el >= SEARCH_MS) { a->search_phase++; a->search_t0 = now; }
        } else {
            a->state = IDLE;  // 搜索完毕, 停
        }
        break;
    }

    /* 状态执行 */
    switch (a->state) {
    case IDLE: break;
    case SEARCH:
        if (a->search_phase < 2) {
            *dir = (a->search_phase == 0) ? 0 : 1;
            *run = 1;
        }
        break;
    case TRACK: {
        float out = pid_step(&a->pid, err, dt);

        /* 滞回: 停了要>START才重启, 跑着<STOP才停 */
        float ae = (err > 0) ? err : -err;
        if (a->stopped) {
            if (ae > DB_START) a->stopped = 0;
        } else {
            if (ae < DB_STOP)  a->stopped = 1;
        }

        if (!a->stopped) {
            *dir = (out > 0) ? 1 : 0;
            *run = 1;
        }
        break;
    }
    }
}

static void motors_apply(axis_t *y, axis_t *x) {
    if (y->running) {
        stepper_set_dir(y->motor, y->cur_dir);
        stepper_set_speed(y->motor, FIXED_HZ);
        stepper_run(y->motor, FIXED_HZ);
    } else stepper_stop(y->motor);

    if (x->running) {
        stepper_set_dir(x->motor, x->cur_dir);
        stepper_set_speed(x->motor, FIXED_HZ);
        stepper_run(x->motor, FIXED_HZ);
    } else stepper_stop(x->motor);
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    protocol_init(115200);
    filter_init();
    stepper_init(STEP1);
    stepper_init(STEP2);

    axis_t y_axis = {STEP1, {KP,KI,KD,0,0}, IDLE, 0, 0,0, 0,0,0,0,0,0};
    axis_t x_axis = {STEP2, {KP,KI,KD,0,0}, IDLE, 0, 0,0, 0,0,0,0,0,0};

    uint32_t next_ms = 0, last_ms = 0;

    while (1) {
        offset_t o = protocol_get();
        if (o.updated) {
            if (!o.found) {
                y_axis.lost_cnt++; y_axis.found_cnt = 0;
                x_axis.lost_cnt++; x_axis.found_cnt = 0;
            } else if (o.dx != 0 || o.dy != 0) {
                y_axis.found_cnt++; y_axis.lost_cnt = 0;
                x_axis.found_cnt++; x_axis.lost_cnt = 0;
                filter_update(o.dx, o.dy, tick_get());
                if (!y_axis.enabled && ++y_axis.frm_cnt > 20) {
                    stepper_enable(STEP1); y_axis.enabled = 1;
                    stepper_enable(STEP2); x_axis.enabled = 1;
                }
            }
        }

        uint32_t now = tick_get();
        if ((int32_t)(now - next_ms) >= 0) {
            float dt = (float)(now - last_ms) * 0.001f;
            if (dt <= 0) dt = 0.02f;
            last_ms = now; next_ms = now + DT_MS;
            float fx = (float)filter_x(), fy = (float)filter_y();

            uint8_t yd, xd, yr, xr;
            axis_calc(&y_axis, fy, -fy, dt, now, &yd, &yr);
            axis_calc(&x_axis, fx,  fx, dt, now, &xd, &xr);
            y_axis.cur_dir = yd; y_axis.running = yr;
            x_axis.cur_dir = xd; x_axis.running = xr;
            motors_apply(&y_axis, &x_axis);

            char buf[100];
            sprintf(buf, "fy=%.0f Sy=%d rY=%d | fx=%.0f Sx=%d rX=%d\r\n",
                    fy, y_axis.state, yr, fx, x_axis.state, xr);
            wireless_uart_send_string(buf);
        }
        stepper_tick();
    }
}
