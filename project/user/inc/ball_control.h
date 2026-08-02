/**
 * ball_control.h — 小球平衡控制模块
 *
 *   三区 PID + 低通滤波 + 跳变过滤 + 丢球恢复
 *   支持任意目标位置 (setpoint), 默认 target=0 追中心
 */
#ifndef _ball_control_h_
#define _ball_control_h_

#include <stdint.h>

typedef struct {
    /* === 滤波参数 === */
    float   alpha_pos;      // 位置低通  (0~1, 越大响应越快)
    float   alpha_vel;      // 速度低通
    float   jump_max;       // 跳变阈值 (px)
    float   v_decay;        // 丢帧时速度衰减
    uint8_t lost_max;       // 连续丢帧上限

    /* === PID 参数 === */
    float   kp, ki, kd;
    float   d_max;          // D 项限幅 (°)
    float   slew_max;       // 每帧最大角度变化 (°)
    float   max_angle;      // 最大倾角 (°)
    float   i_max;          // 积分限幅 (°)
    float   db_inner;       // 不动区边界 (px)
    float   db_outer;       // 出手区边界 (px)

    /* === 目标位置 === */
    float   target;         // 目标球位置 (默认 0=居中)

    /* === 运行状态 === */
    float   dx_f;           // 滤波后位置
    float   vx;             // 滤波后速度
    float   integral;       // PID 积分项
    float   last_angle;     // 上一帧输出角度
    int16_t raw_prev;       // 上一帧原始值 (跳变检测)
    uint8_t lost;           // 连续丢帧计数
    int     ok;             // 1=已锁定目标
    uint32_t start_tick;    // 启动时刻
} ball_control_t;

void  ball_control_init(ball_control_t *b);
void  ball_control_set_target(ball_control_t *b, float target);
void  ball_control_update(ball_control_t *b, int16_t dx, int16_t dy,
                          uint8_t found, float dt);
float ball_control_get_angle(ball_control_t *b);

int   ball_control_is_ok(ball_control_t *b);

#endif
