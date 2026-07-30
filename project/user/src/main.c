/**
 * main.c — 小球滤波 + 丢球处理
 *
 *   found=1: 低通滤波 + 算速度
 *   found=0 ≤8帧: 速度外推
 *   found=0 >8帧: 缓慢回中
 */
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_device_wireless_uart.h"
#include "tick.h"
#include "protocol.h"

#define ALPHA       0.4f        /* 低通系数 */
#define JUMP_MAX    80.0f       /* 单帧跳变阈值 */
#define V_DECAY     0.85f       /* 丢球时速度衰减 */
#define LOST_MAX    8           /* 连续丢球上限 */

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();
    tick_init();
    protocol_init(115200);

    float   dx_f  = 0.0f;       /* 滤波后位置 */
    float   vx    = 0.0f;       /* 速度 */
    int16_t prev  = 0;          /* 上一帧滤波值(用于算速度) */
    uint8_t lost  = 0;          /* 连续丢球计数 */
    uint32_t last_tick = 0;

    char buf[80];

    while (1) {
        offset_t o = protocol_get();
        if (!o.updated) continue;

        uint32_t now = tick_get();
        float dt = (float)(now - last_tick) * 0.001f;
        if (dt <= 0.0f) dt = 0.01f;
        last_tick = now;

        int valid = 0;
        if (o.found && o.dx > -300 && o.dx < 300) {
            float jump = (o.dx > dx_f) ? (o.dx - dx_f) : (dx_f - o.dx);
            if (jump <= JUMP_MAX) {
                /* === 有效帧: 低通滤波 === */
                dx_f = dx_f * (1.0f - ALPHA) + (float)o.dx * ALPHA;
                vx   = (dx_f - (float)prev) / dt;
                prev = (int16_t)dx_f;
                lost = 0;
                valid = 1;

                sprintf(buf, "OK dx:%d f:%.1f v:%.2f l:%d\r\n",
                        o.dx, dx_f, vx, lost);
            }
        }

        if (!valid) {
            if (lost < LOST_MAX) {
                /* === 短暂丢球: 速度外推 === */
                dx_f += vx * dt;
                prev = (int16_t)dx_f;
                vx *= V_DECAY;
                lost++;

                sprintf(buf, "LOST f:%.1f v:%.2f l:%d\r\n",
                        dx_f, vx, lost);
            } else {
                /* === 长时间丢球: 缓慢回中 === */
                dx_f *= 0.9f;
                if (dx_f < 3.0f && dx_f > -3.0f) { dx_f = 0.0f; lost = 0; }
                vx = 0.0f;

                sprintf(buf, "BACK f:%.1f l:%d\r\n", dx_f, lost);
            }
        }
        wireless_uart_send_string(buf);
    }
}
