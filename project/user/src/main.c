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
#define KP          0.10f       /* P增益: dx=200→20° */
#define KD          0.20f       /* D阻尼 */
#define MAX_ANGLE   20.0f       /* 机械限位 ±20° */
#define DEADBAND    1.5f        /* 死区: 误差<1.5°不调 */
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
    int     first_frame = 1;    /* 首次收到有效帧, 直接初始化 */

    /* PD */
    float   prev_err = 0.0f;

    char buf[80];

    /* ============================================= */
    /* 主循环                                         */
    /* ============================================= */
    while (1) {
        uint32_t now = tick_get();

        /* ---- 1. 收帧 + 滤波 ---- */
        offset_t o = protocol_get();
        if (o.updated) {
            float dt = (float)(now - last_tick) * 0.001f;
            if (dt <= 0.0f) dt = 0.01f;
            last_tick = now;

            int valid = 0;
            if (o.found && o.dx > -300 && o.dx < 300) {
                float jump = (o.dx > dx_f) ? (o.dx - dx_f) : (dx_f - o.dx);
                if (first_frame || jump <= JUMP_MAX) {
                    first_frame = 0;
                    /* 有效帧: 低通 + 算速度 */
                    dx_f = dx_f * (1.0f - ALPHA) + (float)o.dx * ALPHA;
                    vx   = (dx_f - (float)prev) / dt;
                    prev = (int16_t)dx_f;
                    lost = 0;
                    valid = 1;
                    ball_ok = 1;
                }
            }

            if (!valid) {
                if (lost < LOST_MAX) {
                    /* 短暂丢球: 速度外推 */
                    dx_f += vx * dt;
                    prev = (int16_t)dx_f;
                    vx *= V_DECAY;
                    lost++;
                } else {
                    /* 长时间丢球: 缓慢归零 */
                    dx_f *= 0.9f;
                    if (dx_f < 3.0f && dx_f > -3.0f) {
                        dx_f = 0.0f;
                        lost = 0;
                        ball_ok = 0;
                    }
                    vx = 0.0f;
                }
            }
        }

        /* ---- 2. 小球 PD → 舵机 ---- */
        float angle = 0.0f;
        if (ball_ok) {
            float error = dx_f - (float)TARGET_DX;   /* 球在+X→正角度→舵机>90 */
            float deriv = error - prev_err;
            prev_err = error;

            angle = KP * error + KD * deriv;
            if (angle >  MAX_ANGLE) angle =  MAX_ANGLE;
            if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;

            float ad = (angle > 0 ? angle : -angle);
            if (ad < DEADBAND) angle = 0.0f;

            servo_set_angle((uint8_t)(90.0f + angle));
        } else {
            /* 球无效: 舵机回中 */
            servo_set_angle(90);
            prev_err = 0.0f;
        }

        /* ---- 3. (预留) 巡线 + 小车电机 ---- */
        // int dev = track_deviation();
        // motor_control_update(BASE_SPEED + dev, BASE_SPEED - dev);

        /* ---- 4. 200ms 串口 (短包, 避免堵UART) ---- */
        static uint32_t pt = 0;
        if (now - pt >= 200) {
            pt = now;
            sprintf(buf, "%d,%d\r\n", prev, (int)angle);
            wireless_uart_send_string(buf);
        }
    }
}
