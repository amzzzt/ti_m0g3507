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
#define KP          0.10f       /* P增益 */
#define KD          0.70f       /* D阻尼 */
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
    float   last_deriv = 0.0f;  /* 保存真实D值供串口 */
    float   last_angle = 0.0f;
    uint32_t lost_since = 0;

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
                if (raw_prev == 0 || jump <= JUMP_MAX) {
                    raw_prev = o.dx;
                    dx_f = dx_f * (1.0f - ALPHA) + (float)o.dx * ALPHA;
                    vx   = (dx_f - (float)prev) / dt;
                    prev = (int16_t)dx_f;
                    lost = 0;
                    valid = 1;
                    ball_ok = 1;

                    /* PD: 只在收到有效帧时计算 */
                    float error = dx_f - (float)TARGET_DX;
                    float deriv = error - prev_err;
                    prev_err = error;
                    last_deriv = deriv;

                    angle = KP * error + KD * deriv;
                    if (angle >  MAX_ANGLE) angle =  MAX_ANGLE;
                    if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
                    float ae = (error > 0 ? error : -error);
                    if (ae < DEADBAND) angle = 0.0f;

                    last_angle = angle;
                    lost_since = 0;
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

        /* ---- 2. 舵机输出 (每帧更新, 保持倾角) ---- */
        if (!ball_ok) {
            uint32_t lost_ms = now - lost_since;
            if (lost_since == 0) lost_since = now;
            if (lost_ms < 500)       angle = last_angle;
            else if (lost_ms < 2000) angle = last_angle * 0.8f;
            else if (lost_ms < 4000) angle = -last_angle * 0.5f;
            else                     angle = 0.0f;
        }
        servo_set_angle((uint8_t)(90.0f + angle));

        /* ---- 3. (预留) 巡线 + 小车电机 ---- */
        // int dev = track_deviation();
        // motor_control_update(BASE_SPEED + dev, BASE_SPEED - dev);

        /* ---- 4. 200ms 全数据串口 ---- */
        static uint32_t pt = 0;
        if (now - pt >= 200) {
            pt = now;
            float error = ball_ok ? (dx_f - (float)TARGET_DX) : 0.0f;
            sprintf(buf, "R:%d F:%.1f E:%.1f D:%.2f A:%d %s\r\n",
                    prev, dx_f, error, last_deriv, (int)angle,
                    ball_ok ? "" : "LOST");
            wireless_uart_send_string(buf);
        }
    }
}
