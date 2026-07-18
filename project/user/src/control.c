/**
 * control.c — 单轴追球控制实现
 */
#include "control.h"
#include "tick.h"

static float pid_step(pid_t *p, float err, float dt, uint16_t max_hz) {
    float P = p->Kp * err;
    p->integral += err * dt;
    if (p->integral >  60) p->integral =  60;
    if (p->integral < -60) p->integral = -60;
    float I = p->Ki * p->integral;
    float out = P + I;
    if (out >  (float)max_hz) { out = max_hz; p->integral -= err * dt; }
    if (out < -(float)max_hz) { out = -max_hz; p->integral -= err * dt; }
    p->prev_err = err;
    return out;
}

void control_init(control_t *c, stepper_id_t motor, float kp, float ki) {
    c->motor   = motor;
    c->pid     = (pid_t){kp, ki, 0, 0};
    c->state   = CS_IDLE;
    c->cur_hz  = 0;
    c->cur_dir = 0;
    c->stopped = 0;
    c->lost_cnt = 0; c->found_cnt = 0;
    c->enabled = 0; c->frm_cnt = 0;
    /* 默认参数 */
    c->stop_db   = 4;
    c->start_db  = 12;
    c->max_hz    = 600;
    c->min_hz    = 50;
    c->search_hz = 150;
    c->debounce  = 15;
    c->search_ms = 500;
}

void control_feed(control_t *c, uint8_t is_lost, uint8_t is_zero) {
    if (is_lost)         { c->lost_cnt++;  c->found_cnt = 0; }
    else if (!is_zero)   { c->found_cnt++; c->lost_cnt  = 0; }
    /* 零帧不计入任何计数 */
}

void control_update(control_t *c, float fv, float err, float dt, uint32_t now) {
    uint16_t hz = 0;
    uint8_t  dir = 0;

    /* 状态切换 */
    switch (c->state) {
    case CS_IDLE:
        if (c->enabled && c->found_cnt >= c->debounce)
            { c->state = CS_TRACK; c->stopped = 0; c->pid.integral = 0; }
        break;
    case CS_TRACK:
        if (c->lost_cnt >= c->debounce)
            { c->state = CS_SEARCH; c->search_t0 = now; c->search_phase = 0; c->pid.integral = 0; }
        break;
    case CS_SEARCH:
        if (c->found_cnt >= c->debounce)
            { c->state = CS_TRACK; c->pid.integral = 0; c->stopped = 0; }
        else if (c->search_phase < 2) {
            if ((int32_t)(now - c->search_t0) >= (int32_t)c->search_ms)
                { c->search_phase++; c->search_t0 = now; }
        } else c->state = CS_IDLE;
        break;
    }

    /* 状态执行 */
    switch (c->state) {
    case CS_IDLE:
        break;
    case CS_SEARCH:
        if (c->search_phase < 2)
            { dir = (c->search_phase == 0) ? 0 : 1; hz = c->search_hz; }
        break;
    case CS_TRACK: {
        float ae = (err > 0) ? err : -err;
        if (c->stopped) {
            if (ae > c->start_db) c->stopped = 0;
            else { c->pid.integral = 0; break; }
        } else {
            if (ae < c->stop_db) { c->stopped = 1; c->pid.integral = 0; break; }
        }
        float out = pid_step(&c->pid, err, dt, c->max_hz);
        dir = (out > 0) ? 1 : 0;
        hz  = (uint16_t)(out > 0 ? out : -out);
        if (hz < c->min_hz) hz = c->min_hz;
        if (hz > c->max_hz) hz = c->max_hz;
        break;
    }
    }

    c->cur_hz  = hz;
    c->cur_dir = dir;
}

uint16_t control_get_hz(control_t *c)  { return c->cur_hz; }
uint8_t  control_get_dir(control_t *c) { return c->cur_dir; }
