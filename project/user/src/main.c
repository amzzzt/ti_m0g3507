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
#define KP          0.04f       /* P 增益: 100px → 4° */
#define KD          0.020f      /* D 增益: 500px/s → 10°刹车 */
#define SLEW_MAX    8.0f        /* 每帧最大角度变化 ±8° */
#define MAX_ANGLE   8.0f        /* 先小幅度调稳, 再逐步放大到20 */
#define DEADBAND    3.0f        /* 死区 ±3px (只在速度<50时生效) */

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

                    /* 连续 PD: 直接用滤波器速度 vx 做 D 项 */
                    float error = dx_f;
                    float ae = (error > 0 ? error : -error);
                    angle = KP * error + KD * vx;
                    if (angle >  MAX_ANGLE) angle =  MAX_ANGLE;
                    if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
                    if (ae < DEADBAND) {
                        float av = (vx > 0 ? vx : -vx);
                        if (av < 50.0f) angle = 0.0f;  /* 真停了才归零 */
                    }

                    /* 限幅: 每帧最多变 ±8°, 防甩杆但不削弱刹车 */
                    float delta = angle - last_angle;
                    if (delta >  SLEW_MAX) delta =  SLEW_MAX;
                    if (delta < -SLEW_MAX) delta = -SLEW_MAX;
                    angle = last_angle + delta;

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
        if (!ball_ok && o.updated) {
            /* 丢球: 每帧衰减 5%, ~1秒回中 */
            angle *= 0.95f;
            if (angle < 0.5f && angle > -0.5f) angle = 0.0f;
            last_angle = angle;
        }
        servo_set_angle((uint8_t)(90.0f + angle));

        /* ---- 3. (预留) 巡线 + 小车电机 ---- */
        // int dev = track_deviation();
        // motor_control_update(BASE_SPEED + dev, BASE_SPEED - dev);

        /* ---- 4. 200ms 串口 ---- */
        static uint32_t pt = 0;
        if (now - pt >= 200) {
            pt = now;
            sprintf(buf, "%d,%d,%d,%d,%d\r\n",
                    (int)o.dx, prev, (int)vx, (int)angle, ball_ok);
            wireless_uart_send_string(buf);
        }
    }
}
