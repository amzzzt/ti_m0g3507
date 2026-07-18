/**
 * stepper.h — 双路 TMC2209 (TIMA1 CH0+CH1)
 *   M1: EN=A12 DIR=B23 STEP=A16(TIMA1 CH1)
 *   M2: EN=A8  DIR=B4  STEP=A10(TIMA1 CH0)
 */
#ifndef _stepper_h_
#define _stepper_h_

#include <stdint.h>

typedef enum { STEP1 = 0, STEP2 = 1 } stepper_id_t;

void stepper_init(stepper_id_t id);
void stepper_enable(stepper_id_t id);
void stepper_disable(stepper_id_t id);
void stepper_set_dir(stepper_id_t id, uint8_t forward);
void stepper_set_speed(stepper_id_t id, uint16_t hz);
void stepper_run(stepper_id_t id, uint16_t hz);
void stepper_stop(stepper_id_t id);
void stepper_tick(void);
void stepper_rotate(stepper_id_t id, float deg);
uint8_t stepper_is_running(stepper_id_t id);

#endif
