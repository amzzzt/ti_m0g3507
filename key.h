#ifndef _key_h_
#define _key_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define KEY_RELEASE_LEVEL      1        // 上拉: 释放=HIGH, 按下=LOW
#define KEY_MAX_SHOCK_PERIOD   5        // 消抖 ms
#define KEY_LONG_PRESS_PERIOD  1000     // 长按阈值 ms

typedef enum {
    KEY_1, KEY_2, KEY_3, KEY_4,
    KEY_NUMBER,
} key_index_enum;

typedef enum {
    KEY_RELEASE,
    KEY_SHORT_PRESS,
    KEY_LONG_PRESS,
} key_state_enum;

void           key_scanner       (void);
key_state_enum key_get_state     (key_index_enum key_n);
void           key_clear_state   (key_index_enum key_n);
void           key_clear_all_state(void);
void           key_init          (void);

#endif
