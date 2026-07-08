#include "key.h"
#include "delay.h"

#define KEY_PIN_MASK  (Key_Key1_PIN | Key_Key2_PIN | Key_Key3_PIN | Key_Key4_PIN)

static const uint32_t key_pin[KEY_NUMBER] = {
    Key_Key1_PIN, Key_Key2_PIN, Key_Key3_PIN, Key_Key4_PIN,
};

static volatile int       key_pressed[KEY_NUMBER];
static volatile int       key_released[KEY_NUMBER];
static          uint32_t  press_ms[KEY_NUMBER];
static volatile key_state_enum  key_state[KEY_NUMBER];

// GPIO 中断 — 只记边沿事件, 轻量
void GROUP1_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(Key_PORT, KEY_PIN_MASK);

    uint8_t i;
    for (i = 0; i < KEY_NUMBER; i++) {
        if (flags & key_pin[i]) {
            uint32_t raw   = DL_GPIO_readPins(Key_PORT, key_pin[i]);
            uint32_t level = (raw & key_pin[i]) ? 1 : 0;

            if (level != KEY_RELEASE_LEVEL) {
                key_pressed[i]  = 1;
                key_released[i] = 0;
                press_ms[i]     = 0;
            } else {
                key_pressed[i]  = 0;
                key_released[i] = 1;
            }
        }
    }

    DL_GPIO_clearInterruptStatus(Key_PORT, flags);
}

// 主循环调 — 用 system_delay_ms(1) 做时间基准
void key_scanner(void)
{
    system_delay_ms(1);

    uint8_t i;
    for (i = 0; i < KEY_NUMBER; i++) {
        if (key_pressed[i]) {
            press_ms[i]++;
            // 长按不等时间到, 等释放再判断
        } else if (key_released[i]) {
            key_released[i] = 0;
            if (press_ms[i] >= KEY_LONG_PRESS_PERIOD) {
                key_state[i] = KEY_LONG_PRESS;
            } else if (press_ms[i] >= KEY_MAX_SHOCK_PERIOD) {
                key_state[i] = KEY_SHORT_PRESS;
            }
            press_ms[i] = 0;
        }
    }
}

key_state_enum key_get_state(key_index_enum k)
{
    key_state_enum s = key_state[k];
    key_state[k] = KEY_RELEASE;
    return s;
}

void key_clear_state(key_index_enum k)       { key_state[k] = KEY_RELEASE; }
void key_clear_all_state(void) { uint8_t i; for(i=0;i<KEY_NUMBER;i++)key_state[i]=KEY_RELEASE; }

void key_init(void)
{
    uint8_t i;
    for (i = 0; i < KEY_NUMBER; i++) {
        key_state[i]    = KEY_RELEASE;
        key_pressed[i]  = 0;
        key_released[i] = 0;
        press_ms[i]     = 0;
    }
    NVIC_EnableIRQ(Key_INT_IRQN);
}
