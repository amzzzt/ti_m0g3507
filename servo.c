/**
 * servo.c — MG996R 舵机驱动, TIMG8 PWM CCP1 → PB7
 *
 * ======================== 工作原理 ========================
 *
 * 1. PWM 硬件自动运行 (syscfg 配置):
 *    TIMG8 计数器以 100kHz 从 0 计数到 1999, 每 2000 次 = 20ms = 一个 PWM 周期(50Hz)
 *    CCP1 比较值决定 PWM 脉冲宽度: 比较值=50 → 0.5ms 脉宽, 比较值=250 → 2.5ms 脉宽
 *    舵机根据脉宽转动到对应角度
 *
 * 2. 角度控制 (servo_set_angle):
 *    输入 0~180°, 线性映射到 SERVO_MIN ~ SERVO_MAX (计数比较值)
 *    公式: 比较值 = MIN + (角度 × (MAX-MIN)) / 180
 *    写入 CCP1 比较寄存器 → 下一个 PWM 周期生效 → 舵机开始转动
 *
 * 3. 扫动计时 (servo_sweep):
 *    主循环高速调用该函数, 每圈读一次 TIMG8 硬件计数器值
 *    计数器只增不减 (0→1→2→...→1999→0→1→...)
 *    "cnt < last_cnt" 说明计数器绕回了 0, 即一个 PWM 周期 (20ms) 结束了
 *    每检测到一次绕回, 角度走 1°
 *    没有中断、没有忙等、没有阻塞延时
 *
 * 4. 关于 MG996R:
 *    标准脉宽 0.5ms(0°) ~ 2.5ms(180°)
 *    对应计数值约 50~250 (100kHz 定时器)
 *    脉冲若超出此范围 (如 <0.5ms 或 >2.5ms), 舵机内部控制器可能无法响应,
 *    表现为舵机在两端卡死不动的"停顿"现象
 *
 *    调整方法:
 *      先把角度设 0°, 调大 SERVO_MIN 直到舵机刚停下不动
 *      再把角度设 180°, 调小 SERVO_MAX 直到舵机刚停下不动
 *      这两个值就是你的舵机实际可用范围
 */
#include "servo.h"

/* ---------- 舵机参数 (你的 MG996R 需要自己微调) ---------- */
#define SERVO_MIN  30      // 0°:  0.3ms
#define SERVO_MAX  260     // 180°:2.6ms → 你的 MG996R 在这能转到 180°

/* ---------- 扫动状态 ---------- */
static int      angle    = 0;     // 当前角度 0~180
static int      dir      = 1;     // 方向: +1=右转, -1=左转
static uint16_t last_cnt = 0;     // 上次读到的计数器值

/* ===========================================================
 * servo_set_angle — 设置舵机角度
 * =========================================================== */
void servo_set_angle(uint8_t deg)
{
    if (deg > 180) deg = 180;

    // 线性映射: 角度 → PWM 比较值
    // 例: 0°→50, 90°→150, 180°→250
    uint32_t val = SERVO_MIN
                 + ((uint32_t)deg * (SERVO_MAX - SERVO_MIN)) / 180;

    // 写入 CCP1 比较寄存器, 硬件在下一个 PWM 周期自动生效
    DL_TimerG_setCaptureCompareValue(SERVO_INST, val, GPIO_SERVO_C1_IDX);
}

/* ===========================================================
 * servo_sweep — 主循环调用, 检测 PWM 周期结束并更新角度
 *
 * 主循环要高速调用此函数 (越快越好, 不阻塞).
 * 计数器 100kHz → 10us 跳一个字 → 主循环必须远快于 10us/圈.
 * 80MHz CPU 轻松做到.
 * =========================================================== */
void servo_sweep(void)
{
    uint16_t cnt = DL_TimerG_getTimerCount(SERVO_INST);

    /*
     * 计数器单调递增: 0→1→...→1999→0→1→...
     * 只有当 cnt < last_cnt 时, 说明发生了一次从 1999 到 0 的"绕回"
     * 每次绕回 = 20ms 过去了 = 一个 PWM 周期结束
     *
     * 例: last_cnt=1500, 本次 cnt=800
     *     → cnt < last_cnt → 绕回发生! 角度走 1°
     */
    if (cnt < last_cnt) {
        angle += dir;                               // 按方向走 1°

        if (angle >= 175) { angle = 175; dir = -1; } // 不顶死, 留 5° 余量
        if (angle <= 5)   { angle = 5;   dir =  1; }

        servo_set_angle(angle);                     // 更新舵机位置
    }
    last_cnt = cnt;                                 // 记住本次值, 用于下次比较
}

/* ===========================================================
 * servo_init — 初始化
 * =========================================================== */
void servo_init(void)
{
    servo_set_angle(90);                            // 上电归中
    last_cnt = DL_TimerG_getTimerCount(SERVO_INST); // 记录计数器初值
}
