/**
 * main.c — Y轴追球 PID控制
 *
 *   设定值=0(画面中心), 测量值=filter_y(), 输出=电机速度
 *   PID每20ms更新, 连续调速
 */
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"
#include "stepper.h"

/* PID 参数 */
#define KP         1.0f
#define KI         0.5f
#define KD         0.2f
#define OUT_MAX    400
#define OUT_MIN   -400
#define DEADBAND   8
#define DT_MS      20

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_err;
} pid_t;

static float pid_step(pid_t *p, float err, float dt) {
    /* P */
    float P = p->Kp * err;

    /* I — 积分(输出钳位天然抗饱和) */
    p->integral += err * dt;
    float I = p->Ki * p->integral;

    /* D — 微分抑制震荡 */
    float D = 0;
    if (dt > 0.001f) D = p->Kd * (err - p->prev_err) / dt;
    p->prev_err = err;

    float out = P + I + D;
    if (out > OUT_MAX) out = OUT_MAX;
    if (out < OUT_MIN) out = OUT_MIN;
    return out;
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    wireless_uart_init();
    protocol_init(115200);
    filter_init();
    stepper_init(STEP1);
    // 收到足够帧后才使能, 避免开机乱转
    pid_t    pid = {KP, KI, KD, 0, 0};
    uint16_t cur_hz  = 0;
    uint8_t  cur_dir = 0;
    uint32_t next_ms = 0;
    uint32_t last_ms = 0;
    uint16_t frm_cnt = 0;
    uint8_t  enabled = 0;

    while (1) {
        /* 收数 + 滤波(每帧) */
        offset_t o = protocol_get();
        if (o.updated && o.found) {
            filter_update(o.dx, o.dy, tick_get());
            if (++frm_cnt > 20) {
                if (!enabled) { stepper_enable(STEP1); enabled = 1; }
            }
        }

        uint32_t now = tick_get();

        /* PID 控制(每20ms) */
        if ((int32_t)(now - next_ms) >= 0) {
            float dt = (float)(now - last_ms) * 0.001f;
            if (dt <= 0) dt = 0.02f;
            last_ms = now;
            next_ms = now + DT_MS;

            float fy  = (float)filter_y();
            float err = -fy;
            uint8_t active = filter_active();

            float out = 0;
            /* 未使能或球丢失 → 停车 */
            if (!enabled || !active) {
                if (cur_hz != 0) { stepper_stop(STEP1); cur_hz = 0; }
                pid.integral = 0;
                pid.prev_err = 0;
            } else {
                out = pid_step(&pid, err, dt);

                if (err > -DEADBAND && err < DEADBAND) {
                    if (cur_hz != 0) { stepper_stop(STEP1); cur_hz = 0; }
                } else {
                    uint8_t  dir = (out > 0) ? 1 : 0;
                    uint16_t hz  = (uint16_t)(out > 0 ? out : -out);
                    if (hz < 30) hz = 30;
                    if (hz > OUT_MAX) hz = OUT_MAX;

                    if (hz != cur_hz || dir != cur_dir) {
                        if (dir != cur_dir) stepper_set_dir(STEP1, dir);
                        stepper_set_speed(STEP1, hz);
                        stepper_run(STEP1, hz);
                        cur_hz = hz;
                        cur_dir = dir;
                    }
                }
            }

            /* 诊断打印: raw_dy, filter_y, active, PID_out, motor_hz */
            char buf[80];
            sprintf(buf, "raw=%.0f fy=%.0f act=%d out=%.0f hz=%d\r\n",
                    (float)o.dy, fy, active, out, cur_hz);
            wireless_uart_send_string(buf);
        }
        stepper_tick();
    }
}
