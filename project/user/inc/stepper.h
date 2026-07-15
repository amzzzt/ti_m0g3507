/**
 * stepper.h — 步进电机驱动 (A4988 / DRV8825 兼容)
 *
 *   微步角度: 0.05625° → 6400 脉冲/圈
 *   工作模式: 高电平大扭矩 (DCY=1)
 *
 *   引脚 (待配置):
 *     RST - 复位, 高有效
 *     SLP - 休眠, 高有效
 *     DCY - 电流衰减, 高=大扭矩
 *     DIR - 方向, 高=正转/低=反转
 *     PWM - 脉冲, 一个上升沿=一个微步
 */
#ifndef _stepper_h_
#define _stepper_h_

#include <stdint.h>

//=================================================== 引脚配置 (用户自行修改) ===================================================
#define STEPPER_RST_PIN     ( B20 )      // RST  复位, 高电平工作
#define STEPPER_SLP_PIN     ( B18 )      // SLP  休眠, 高电平工作
#define STEPPER_DCY_PIN     ( B14 )      // DCY  大扭矩模式, 高电平
#define STEPPER_DIR_PIN     ( B12 )      // DIR  方向
#define STEPPER_PWM_PIN     ( A8  )      // PWM  脉冲信号, 用 PWM 外设或 GPIO 模拟
//============================================================================================================================

// 步进电机参数
#define STEPPER_STEP_ANGLE  0.05625f     // 单微步角度 (°)
#define STEPPER_STEPS_REV   6400         // 一圈 = 6400 微步

void stepper_init         (void);                                          // 初始化 GPIO + PWM
void stepper_enable       (void);                                          // 使能 (RST=1, SLP=1, DCY=1)
void stepper_disable      (void);                                          // 禁用 (RST=0, SLP=0)
void stepper_set_dir      (uint8_t forward);                               // 设方向 1=正转 0=反转
void stepper_pulse        (void);                                          // 发一个脉冲 → 一个微步
void stepper_rotate_deg   (float deg);                                     // 旋转指定角度 (°), 阻塞
void stepper_set_speed    (uint16_t hz);                                   // 设脉冲频率 (Hz)

#endif
