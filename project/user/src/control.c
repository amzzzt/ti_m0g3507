/**
 * control.c — 双轴控制模块
 *
 * ============================ 状态机 ============================
 * CS_IDLE    空闲, 电机不上电, 不响应
 * CS_TRACK   纯快追: PID全程, 带迟滞防抖
 * CS_LOCK    精密锁定: 比例降速, 偏差≤lock_db锁死
 * CS_SEARCH  目标丢失: 朝脱离方向追赶, 超时退IDLE
 * CS_TRACE   绕框描边: 超低速跟随
 * ================================================================
 */
#include "control.h"
#include "tick.h"

/* ======================== PID 核心 ======================== */
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

/* ======================== 初始化 ======================== */
void control_init(control_t *c, stepper_id_t motor, float kp, float ki) {
    c->motor   = motor;
    c->pid     = (pid_t){kp, ki, 0, 0};
    c->state   = CS_IDLE;
    c->cur_hz  = 0;
    c->cur_dir = 0;
    c->stopped = 0;
    c->lost_cnt = 0; c->found_cnt = 0;
    c->enabled = 0; c->frm_cnt = 0;
    c->chase_dir = 1;

    /* CS_TRACK 参数 */
    c->stop_db   = 4;
    c->start_db  = 12;
    c->max_hz    = 600;
    c->min_hz    = 50;

    /* CS_LOCK 参数 */
    c->lock_hz   = 60;
    c->lock_db   = 3.0f;
    c->lock_hz_smooth = 0;

    /* CS_SCAN 参数 */
    c->scan_hz   = 150;

    /* CS_SEARCH 参数 */
    c->search_hz = 100;
    c->debounce  = 15;
    c->search_ms = 1500;
}

/* ======================== 公开接口 ======================== */
void control_force_state(control_t *c, ctrl_state_t st) {
    c->state = st;
    c->pid.integral = 0;
    c->stopped = 0;
    c->lost_cnt = 0;
    c->found_cnt = 0;
    c->lock_hz_smooth = 0;
}

uint8_t control_is_locked(control_t *c) {
    return ((c->state == CS_LOCK || c->state == CS_TRACE) && c->cur_hz == 0) ? 1 : 0;
}

void control_feed(control_t *c, uint8_t is_lost, uint8_t is_zero) {
    if (is_lost)         { c->lost_cnt++;  c->found_cnt = 0; }
    else if (!is_zero)   { c->found_cnt++; c->lost_cnt  = 0; }
}

uint16_t control_get_hz(control_t *c)  { return c->cur_hz; }
uint8_t  control_get_dir(control_t *c) { return c->cur_dir; }

/* ======================== 主更新 ======================== */
void control_update(control_t *c, float fv, float err, float dt, uint32_t now) {
    uint16_t hz = 0;
    uint8_t  dir = 0;
    float    ae = (err > 0) ? err : -err;

    /* ---- 状态切换 ---- */
    switch (c->state) {
    case CS_IDLE:
        if (c->enabled && c->found_cnt >= c->debounce)
            { c->state = CS_TRACK; c->stopped = 0; c->pid.integral = 0; }
        break;

    case CS_SCAN:
        /* SCAN 就是扫, 不触发丢失 — 切LOCK由main.c管 */
        break;

    /* 三模式统一丢目标处理: 丢失→SEARCH追赶→超时→IDLE */
    case CS_LOCK:
    case CS_TRACK:
    case CS_TRACE:
        if (c->lost_cnt >= c->debounce) {
            c->chase_dir = c->cur_dir;
            c->state = CS_SEARCH; c->search_t0 = now;
            c->pid.integral = 0;
        }
        break;

    case CS_SEARCH:
        if (c->found_cnt >= c->debounce)
            { c->state = CS_TRACK; c->pid.integral = 0; c->stopped = 0; }
        else if ((int32_t)(now - c->search_t0) >= (int32_t)c->search_ms)
            { c->state = CS_IDLE; }
        break;
    }

    /* ---- 状态执行 ---- */
    switch (c->state) {
    case CS_IDLE:
        break;

    /* ========== CS_SCAN: 开机旋转扫描寻目标 ========== */
    case CS_SCAN:
        if (c->motor == STEP2) { dir = 1; hz = c->scan_hz; }  /* X轴(STEP2)旋转 */
        break;

    /* ========== CS_SEARCH: 目标丢失, 朝脱离方向追赶 ========== */
    case CS_SEARCH:
        dir = c->chase_dir;
        hz  = c->search_hz;
        break;

    /* ========== CS_LOCK: 精密锁定 (纯锁, 不退TRACK) ========== *
     * |err|≤3 → 锁死, hz=0
     * |err|>3 → ae*2 比例降速, 8~300Hz
     */
    case CS_LOCK: {
        float ae = (err > 0) ? err : -err;
        if (ae <= c->lock_db) {
            hz = 0; c->lock_hz_smooth = 0;
            break;
        }
        dir = (err > 0) ? 1 : 0;
        float target = 2.0f * ae;
        if (target < 8)   target = 8;
        if (target > 100) target = 100;
        if (target > c->lock_hz_smooth + 5 || target < c->lock_hz_smooth - 5)
            c->lock_hz_smooth = target;
        hz = (uint16_t)c->lock_hz_smooth;
        if (hz < 1) hz = 1;
        break;
    }

    /* ========== CS_TRACE: 绕框描边 ========== *
     * 超低速: ae*0.5 Hz, 4~25
     * 用于逐点跟随矩形边缘
     */
    case CS_TRACE:
        if (ae <= c->lock_db) {
            hz = 0; c->lock_hz_smooth = 0;
            break;
        }
        dir = (err > 0) ? 1 : 0;
        {
            float target = 0.5f * ae;
            if (target < 4)   target = 4;
            if (target > 25)  target = 25;
            if (target > c->lock_hz_smooth + 3 || target < c->lock_hz_smooth - 3)
                c->lock_hz_smooth = target;
            hz = (uint16_t)c->lock_hz_smooth;
            if (hz < 1) hz = 1;
        }
        break;

    /* ========== CS_TRACK: 纯快追, PID全程 ========== */
    case CS_TRACK:
        if (c->stopped) {
            if (ae > c->start_db) c->stopped = 0;
            else { c->pid.integral = 0; break; }
        } else {
            if (ae < c->stop_db) { c->stopped = 1; c->pid.integral = 0; break; }
        }
        {
            float out = pid_step(&c->pid, err, dt, c->max_hz);
            dir = (out > 0) ? 1 : 0;
            hz  = (uint16_t)(out > 0 ? out : -out);
            if (hz < c->min_hz) hz = c->min_hz;
            if (hz > c->max_hz) hz = c->max_hz;
        }
        break;
    }

    c->cur_hz  = hz;
    c->cur_dir = dir;
}
