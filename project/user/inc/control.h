/**
 * control.h — 单轴追球控制模块
 */
#ifndef _control_h_
#define _control_h_
#include <stdint.h>
#include "stepper.h"

typedef struct { float Kp, Ki, integral, prev_err; } pid_t;
typedef enum { CS_IDLE, CS_TRACK, CS_SEARCH, CS_LOCK, CS_TRACE } ctrl_state_t;

typedef struct {
    stepper_id_t motor;
    pid_t        pid;
    ctrl_state_t state;
    uint16_t     cur_hz;
    uint8_t      cur_dir;
    uint32_t     search_t0;
    uint8_t      search_phase;
    int8_t       chase_dir;
    uint8_t      lost_cnt, found_cnt, enabled;
    uint16_t     frm_cnt;
    uint8_t      stopped;
    float        stop_db, start_db;
    uint16_t     max_hz, min_hz, search_hz, debounce, search_ms;
    /* 精密锁定参数 */
    uint16_t     lock_hz;
    float        lock_db;
    float        lock_hz_smooth;   /* 频率平滑, 防台阶跳变 */
} control_t;

void control_init(control_t *c, stepper_id_t motor, float kp, float ki);
void control_update(control_t *c, float fv, float err, float dt, uint32_t now);
void control_feed(control_t *c, uint8_t is_lost, uint8_t is_zero);
void control_force_state(control_t *c, ctrl_state_t st);
uint16_t control_get_hz(control_t *c);
uint8_t  control_get_dir(control_t *c);
uint8_t  control_is_locked(control_t *c);
#endif
