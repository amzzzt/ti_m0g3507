/**
 * stepper.h — 双步进电机驱动 (TIMA1 PWM)
 *
 *   引脚说明:
 *     RST: 高=工作, 低=停
 *     SLP: 高=锁定(手拧不动), 低=休眠(手可拧)
 *     DCY: 高=大扭矩, 低=小扭矩
 *     DIR: 高=正转, 低=反转
 *     PWM: 1脉冲=1微步=0.05625°, 6400脉冲/圈
 *
 *   电机1: A16 TIMA1 CH1   电机2: A15 TIMA1 CH0
 */
#ifndef _stepper_h_
#define _stepper_h_

#include <stdint.h>

//========== Motor 1 pins ==========
#define STEP1_RST_PIN       ( B12  )
#define STEP1_SLP_PIN       ( B7   )
#define STEP1_DCY_PIN       ( A12  )
#define STEP1_DIR_PIN       ( B23  )
// PWM: A16 TIMA1 CH1

//========== Motor 2 pins ==========
#define STEP2_RST_PIN       ( A8   )
#define STEP2_SLP_PIN       ( A9   )
#define STEP2_DCY_PIN       ( A10  )
#define STEP2_DIR_PIN       ( A11  )
// PWM: A15 TIMA1 CH0

//========== Stepper params ==========
#define STEPPER_STEP_ANGLE  0.05625f
#define STEPPER_STEPS_REV   6400

typedef enum { STEP1 = 0, STEP2 = 1 } stepper_id_t;

void stepper_init         (stepper_id_t id);
void stepper_enable       (stepper_id_t id);
void stepper_disable      (stepper_id_t id);
void stepper_set_dir      (stepper_id_t id, uint8_t forward);
void stepper_pulse        (stepper_id_t id);
void stepper_rotate_deg   (stepper_id_t id, float deg);
void stepper_set_speed    (stepper_id_t id, uint16_t hz);

#endif
