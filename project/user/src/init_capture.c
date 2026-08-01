/**
 * init_capture.c — 上电初始化: 等KEY1→2秒抓球位置→返回目标
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "protocol.h"

float init_capture_run(void)
{
    float  sum     = 0.0f;
    int    samples = 0;

    /* 等 KEY1 */
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
    key_clear_state(KEY_1);

    /* 等2秒, 同时采集球位置 */
    uint32_t t0 = tick_get();
    while (tick_get() - t0 < 2000) {
        offset_t o = protocol_get();
        if (o.updated && o.found && o.dx > -500 && o.dx < 500) {
            sum += (float)o.dx;
            samples++;
        }
    }

    return (samples > 0) ? (sum / (float)samples) : 0.0f;
}
