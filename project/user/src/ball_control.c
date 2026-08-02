/**
 * ball_control.c — 小球平衡控制 (三区 PID + 滤波 + 丢球恢复)
 *
 *   从 main.c 完整迁移, 增加 setpoint 支持 (target 默认 0=居中)
 */
#include "ball_control.h"

/* ========== 默认参数 ========== */
#define DFL_ALPHA_POS   0.70f
#define DFL_ALPHA_VEL   0.95f
#define DFL_JUMP_MAX    150.0f
#define DFL_V_DECAY     0.85f
#define DFL_LOST_MAX    8

#define DFL_KP          0.065f
#define DFL_KI          0.10f
#define DFL_KD          3.30f
#define DFL_D_MAX       16.0f
#define DFL_SLEW_MAX    5.0f
#define DFL_MAX_ANGLE   9.0f
#define DFL_I_MAX       3.0f
#define DFL_DB_INNER    8.0f
#define DFL_DB_OUTER    80.0f

/* ================================================================ */

void ball_control_init(ball_control_t *b)
{
    /* 滤波参数 */
    b->alpha_pos = DFL_ALPHA_POS;
    b->alpha_vel = DFL_ALPHA_VEL;
    b->jump_max  = DFL_JUMP_MAX;
    b->v_decay   = DFL_V_DECAY;
    b->lost_max  = DFL_LOST_MAX;

    /* PID 参数 */
    b->kp        = DFL_KP;
    b->ki        = DFL_KI;
    b->kd        = DFL_KD;
    b->d_max     = DFL_D_MAX;
    b->slew_max  = DFL_SLEW_MAX;
    b->max_angle = DFL_MAX_ANGLE;
    b->i_max     = DFL_I_MAX;
    b->db_inner  = DFL_DB_INNER;
    b->db_outer  = DFL_DB_OUTER;

    /* 目标位置 */
    b->target = 0.0f;

    /* 运行状态清零 */
    b->dx_f     = 0.0f;
    b->vx       = 0.0f;
    b->integral = 0.0f;
    b->last_angle = 0.0f;
    b->raw_prev = 0;
    b->lost     = 0;
    b->ok       = 0;
}

void ball_control_set_target(ball_control_t *b, float target)
{
    b->target = target;
}

float ball_control_get_angle(ball_control_t *b)
{
    return b->last_angle;
}

int ball_control_is_ok(ball_control_t *b)
{
    return b->ok;
}

/* ================================================================ */

void ball_control_update(ball_control_t *b, int16_t dx, int16_t dy,
                         uint8_t found, float dt)
{
    float angle = b->last_angle;   /* 默认保持 */
    int   valid = 0;

    /* ---- 有效帧: 滤波 + PID ---- */
    if (found && dx > -500 && dx < 500) {
        int16_t jump = (dx > b->raw_prev) ? (dx - b->raw_prev)
                                          : (b->raw_prev - dx);
        b->raw_prev = dx;   /* 每帧更新, 避免连锁拒绝 */

        if (b->ok == 0 || jump <= b->jump_max) {
            /* 位置低通滤波 */
            b->dx_f = b->dx_f * (1.0f - b->alpha_pos)
                    + (float)dx * b->alpha_pos;

            /* 速度: 相机原始值独立轻滤 */
            {
                float cam_v = (float)dy;
                if (cam_v > -40 && cam_v < 40) {
                    if (b->ok == 0)
                        b->vx = cam_v;
                    else
                        b->vx = b->vx * (1.0f - b->alpha_vel)
                              + cam_v * b->alpha_vel;
                }
            }

            b->lost = 0;
            valid   = 1;
            b->ok   = 1;

            /* ========== 三区 PID ========== */
            float error = b->dx_f - b->target;
            float ae    = (error > 0 ? error : -error);
            float av    = (b->vx > 0 ? b->vx : -b->vx);

            if (ae < b->db_inner) {
                /* 区1: 不动区, angle=0, 积分衰减 */
                angle    = 0.0f;
                b->integral *= 0.85f;
            } else {
                /* 积分 (大误差不积) */
                if (ae < 120.0f) {
                    b->integral += error * dt * b->ki;
                    if (b->integral >  b->i_max) b->integral =  b->i_max;
                    if (b->integral < -b->i_max) b->integral = -b->i_max;
                } else {
                    b->integral = 0.0f;
                }

                /* D 项 */
                float d_term = b->kd * b->vx;
                if (d_term >  b->d_max) d_term =  b->d_max;
                if (d_term < -b->d_max) d_term = -b->d_max;

                angle = b->kp * error + d_term + b->integral;
                if (angle >  b->max_angle) angle =  b->max_angle;
                if (angle < -b->max_angle) angle = -b->max_angle;

                /* 反向力限幅: 全区间, 远处也限刹 */
                {
                    float ratio = (ae - b->db_inner) / (b->db_outer - b->db_inner);
                    if (ratio > 1.0f) ratio = 1.0f;
                    if (ratio < 0.0f) ratio = 0.0f;
                    float lim;
                    if (av < 1.5f)       lim = 5.0f;
                    else if (av < 3.0f)  lim = 7.0f;
                    else if (av < 6.0f)  lim = 9.0f;
                    else                 lim = 9.0f;
                    lim *= ratio;
                    int opposing = (error > 0 && angle < 0) || (error < 0 && angle > 0);
                    if (opposing) {
                        if (angle >  lim) angle =  lim;
                        if (angle < -lim) angle = -lim;
                    }
                }
            }

            /* SLEW 限幅 */
            float delta = angle - b->last_angle;
            if (delta >  b->slew_max) delta =  b->slew_max;
            if (delta < -b->slew_max) delta = -b->slew_max;
            angle = b->last_angle + delta;
        }
    }

    /* ---- 无效帧: 速度外推 + 衰减 ---- */
    if (!valid) {
        if (b->lost < b->lost_max) {
            b->dx_f += b->vx * dt;
            b->vx   *= b->v_decay;
            b->lost++;
        } else {
            b->dx_f *= 0.9f;
            if (b->dx_f < 3.0f && b->dx_f > -3.0f) {
                b->dx_f = 0.0f;
                b->lost  = 0;
                b->ok    = 0;
            }
            b->vx = 0.0f;
        }
    }

    /* ---- 丢球: 角度衰减回中 ---- */
    if (!b->ok) {
        angle = b->last_angle;
        angle *= 0.95f;
        if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
    }

    b->last_angle = angle;
}
