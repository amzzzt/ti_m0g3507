/**
 * balance.c — 单轴平衡控制
 *
 *   IMU pitch → PID(内联) → stepper STEP1 速度+方向
 *   目标: 保持 pitch=0°, 小球居中
 *
 *   PID 调参:
 *     BALANCE_KP  比例: 偏差1°→多少Hz
 *     BALANCE_KI  积分: 消除静差
 *     BALANCE_KD  微分: 阻尼防震荡
 *     BALANCE_DB  死区: |err|内停转, 避免微震
 *     BALANCE_MAX 最大步进频率 Hz
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "imu.h"
#include "stepper.h"
#include "balance.h"

/* ======================== 可调参数 ======================== */
#define BALANCE_KP      15.0f   /* 比例增益 */
#define BALANCE_KI      0.3f    /* 积分增益 (含10ms dt) */
#define BALANCE_KD      5.0f    /* 微分增益 */
#define BALANCE_DB      0.5f    /* 死区 (度) */
#define BALANCE_MAX     800     /* 最大步进频率 Hz */
#define BALANCE_MIN     30      /* 最小步进频率 Hz (太低会抖) */
#define BALANCE_DT      0.01f   /* 更新周期 10ms */
/* ========================================================== */

static float    g_integral;
static float    g_prev_err;
static uint32_t g_last;
static uint8_t  g_enabled;

void balance_init(void)
{
    stepper_init(STEP1);
    stepper_enable(STEP1);
    stepper_set_dir(STEP1, 0);

    g_integral = 0.0f;
    g_prev_err = 0.0f;
    g_last = 0;
    g_enabled = 1;
}

void balance_enable(void)  { g_enabled = 1; }
void balance_disable(void) { g_enabled = 0; stepper_stop(STEP1); }
uint8_t balance_is_active(void) { return g_enabled; }

void balance_update(void)
{
    if (!g_enabled) return;

    /* 10ms 门控 */
    uint32_t now = tick_get();
    if ((int32_t)(now - g_last) < 10) return;
    g_last = now;

    /* 读 IMU */
    imu_update();
    float pitch = imu_pitch();
    float err   = 0.0f - pitch;   /* target=0° */

    /* 死区: 锁死防微震 */
    float ae = (err > 0) ? err : -err;
    if (ae < BALANCE_DB) {
        stepper_stop(STEP1);
        g_integral *= 0.9f;       /* 缓慢泄放积分 */
        g_prev_err = err;
        return;
    }

    /* PID */
    float P = BALANCE_KP * err;

    g_integral += err;
    float max_i = (float)BALANCE_MAX / (BALANCE_KI + 0.001f);
    if (g_integral >  max_i) g_integral =  max_i;
    if (g_integral < -max_i) g_integral = -max_i;
    float I = BALANCE_KI * g_integral;

    float D = BALANCE_KD * (err - g_prev_err);
    g_prev_err = err;

    float out = P + I + D;

    /* 输出 → 步进电机 */
    uint8_t  dir = (out > 0) ? 1 : 0;
    uint16_t hz  = (uint16_t)(out > 0 ? out : -out);
    if (hz > BALANCE_MAX) hz = BALANCE_MAX;
    if (hz < BALANCE_MIN) hz = BALANCE_MIN;

    stepper_set_dir(STEP1, dir);
    stepper_run(STEP1, hz);
}
