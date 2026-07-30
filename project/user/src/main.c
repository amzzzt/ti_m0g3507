/**
 * main.c — 小球追踪完整框架 (滤波 + PD + 舵机)
 *
 *   架构: 收帧→滤波→PD→舵机, 预留巡线接口
 *   舵机 PB7, 泰山派 UART0, 串口 UART1
 */
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "servo.h"
#include "protocol.h"

/* ========== 小球滤波参数 ========== */
#define ALPHA       0.4f
#define JUMP_MAX    80.0f
#define V_DECAY     0.85f
#define LOST_MAX    8

/* ========== PD 控制参数 ========== */
/* PD 三段增益, 见下方计算 */
#define MAX_ANGLE   25.0f       /* 机械限位 ±25° */
#define DEADBAND    1.0f        /* 死区 */
#define TARGET_DX   0           /* 目标位置: 0=中心 */

int main(void)
{
    /* ============================================= */
    /* 初始化                                         */
    /* ============================================= */
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    servo_init();               /* 先舵机, 避免 PWM 抢 UART 时钟 */
    wireless_uart_init();
    protocol_init(115200);
    tick_start();

    /* ============================================= */
    /* 状态变量                                       */
    /* ============================================= */
    /* 滤波 */
    float   dx_f  = 0.0f;
    float   vx    = 0.0f;
    int16_t prev  = 0;
    uint8_t lost  = 0;
    uint32_t last_tick = 0;
    int     ball_ok = 0;
    int16_t raw_prev = 0;       /* 上一帧原始值, 用于跳变判断 */

    /* PD */
    float   prev_err  = 0.0f;
    float   last_angle = 0.0f;

    char buf[80];

    /* ============================================= */
    /* 主循环                                         */
    /* ============================================= */
    while (1) {
        uint32_t now = tick_get();

        /* ---- 1. 收帧 + 滤波 + PD (只在来帧时更新) ---- */
        float angle = last_angle;   /* 默认保持上次角度 */
        offset_t o = protocol_get();
        if (o.updated) {
            float dt = (float)(now - last_tick) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            last_tick = now;

            int valid = 0;
            if (o.found && o.dx > -300 && o.dx < 300) {
                int16_t jump = (o.dx > raw_prev) ? (o.dx - raw_prev) : (raw_prev - o.dx);
                if (ball_ok == 0 || jump <= JUMP_MAX) {
                    /* 首次有效帧或跳变不大: 接受 */
                    raw_prev = o.dx;
                    dx_f = dx_f * (1.0f - ALPHA) + (float)o.dx * ALPHA;
                    vx   = (dx_f - (float)prev) / dt;
                    prev = (int16_t)dx_f;
                    lost = 0;
                    valid = 1;
                    ball_ok = 1;

                    /* 控制策略: 两端轻推, 中间滑行 */
                    float error = dx_f - (float)TARGET_DX;
                    float deriv = error - prev_err;
                    prev_err = error;
                    float ae = (error > 0 ? error : -error);

                    if (ae > 200) {
                        /* 最远端: 轻推起步 */
                        angle = (error > 0) ? 25.0f : -25.0f;
                    } else if (ae > 100) {
                        /* 远端: 弱P控制 */
                        angle = 0.08f * error;
                    } else if (ae > 40) {
                        /* 中途: 几乎不干预, 让惯性滑 */
                        angle = 0.03f * error;
                    } else {
                        /* 中心附近: 极弱反馈, 慢慢衰减 */
                        angle = 0.01f * error + 0.02f * deriv;
                    }
                    if (angle >  MAX_ANGLE) angle =  MAX_ANGLE;
                    if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
                    if (ae < DEADBAND) angle = 0.0f;

                    last_angle = angle;
                }
            }

            if (!valid) {
                if (lost < LOST_MAX) {
                    dx_f += vx * dt;
                    prev = (int16_t)dx_f;
                    vx *= V_DECAY;
                    lost++;
                } else {
                    dx_f *= 0.9f;
                    if (dx_f < 3.0f && dx_f > -3.0f) {
                        dx_f = 0.0f; lost = 0; ball_ok = 0;
                    }
                    vx = 0.0f;
                }
            }
        }

        /* ---- 2. 舵机输出 ---- */
        if (!ball_ok) {
            angle = last_angle;     /* 丢球也不松, 保持倾角 */
        }
        servo_set_angle((uint8_t)(90.0f + angle));

        /* ---- 3. (预留) 巡线 + 小车电机 ---- */
        // int dev = track_deviation();
        // motor_control_update(BASE_SPEED + dev, BASE_SPEED - dev);

        /* ---- 4. 100ms 串口 ---- */
        static uint32_t pt = 0;
        if (now - pt >= 100) {
            pt = now;
            sprintf(buf, "%d,%d,%d\r\n", prev, (int)angle, ball_ok);
            wireless_uart_send_string(buf);
        }
    }
}
