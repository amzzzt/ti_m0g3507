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
    /* 精密锁定: lock_db 放宽, 防止远处目标噪声触发假修正 */
    c->lock_hz   = 60;
    c->lock_db   = 3.0f;
    c->lock_hz_smooth = 0;
}

void control_force_state(control_t *c, ctrl_state_t st) {
    c->state = st;
    c->pid.integral = 0;
    c->stopped = 0;
    c->lost_cnt = 0;
    c->found_cnt = 0;
    c->lock_hz_smooth = 0;
}

uint8_t control_is_locked(control_t *c) {
    return (c->state == CS_LOCK && c->cur_hz == 0) ? 1 : 0;
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
    case CS_LOCK:
        /* LOCK 模式手动切换, 不自动退出 */
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
    case CS_LOCK: {
        float ae = (err > 0) ? err : -err;
        if (ae <= c->lock_db) {
            hz = 0; c->lock_hz_smooth = 0;
            break;
        }
        dir = (err > 0) ? 1 : 0;
        /* 连续比例: ae*2 Hz, 8~100 */
        float target = 2.0f * ae;
        if (target < 8)   target = 8;
        if (target > 100) target = 100;
        /* 死区5Hz: 只在大变化时更新, 避免 PWM 微小重设 */
        if (target > c->lock_hz_smooth + 5 || target < c->lock_hz_smooth - 5)
            c->lock_hz_smooth = target;
        hz = (uint16_t)c->lock_hz_smooth;
        if (hz < 1) hz = 1;
        break;
    }
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
