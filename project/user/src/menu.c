/**
 * menu.c — TFT180 多级菜单
 * KEY1(A30)=确定, KEY2(A31)=上, KEY3(B0)=下
 */

#include "menu.h"
#include "zf_common_headfile.h"
#include "tick.h"

typedef enum { PAGE_NORMAL, PAGE_MENU_MAIN, PAGE_ROTATE } page_t;

static page_t  page = PAGE_NORMAL;
static uint8_t cursor = 0;
static uint32_t last_tick = 0;

#define ITEM_COUNT 2
static const char *items[ITEM_COUNT] = { "1.Rotate", "2.Exit" };

void menu_init(void) { page = PAGE_NORMAL; cursor = 0; }
uint8_t menu_is_active(void) { return (page != PAGE_NORMAL); }

// ---- 绘制主菜单 ----
static void draw(void)
{
    uint8_t i;
    __disable_irq();
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_clear();
    tft180_show_string(0, 0, "== Menu ==");

    for (i = 0; i < ITEM_COUNT; i++) {
        if (i == cursor)
            tft180_set_color(RGB565_WHITE, RGB565_BLUE);
        else
            tft180_set_color(RGB565_BLACK, RGB565_WHITE);
        tft180_show_string(20, 40 + i * 30, (char *)items[i]);
    }
    __enable_irq();
}

// ---- 主逻辑 ----
void menu_run(void)
{
    uint32_t now = tick_get();
    if (now - last_tick < 20) return;           // 20ms 防抖, 够快且不误触

    switch (page) {

    case PAGE_NORMAL:
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            page = PAGE_MENU_MAIN;
            cursor = 0;
            draw();
            last_tick = now;
        }
        break;

    case PAGE_MENU_MAIN:
        if (KEY_SHORT_PRESS == key_get_state(KEY_2)) {
            key_clear_state(KEY_2);
            if (cursor > 0) { cursor--; draw(); }
            last_tick = now;
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_3)) {
            key_clear_state(KEY_3);
            if (cursor < ITEM_COUNT - 1) { cursor++; draw(); }
            last_tick = now;
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            if (cursor == 0) {
                page = PAGE_ROTATE;
                __disable_irq();
                tft180_set_color(RGB565_BLACK, RGB565_WHITE);
                tft180_clear();
                tft180_show_string(0, 0,  "Rotate");
                tft180_show_string(0, 30, "OK=Back");
                __enable_irq();
            } else {
                page = PAGE_NORMAL;
                __disable_irq();
                tft180_set_color(RGB565_BLACK, RGB565_WHITE);
                tft180_clear();
                tft180_show_string(0, 0, "TFT180 OK");
                __enable_irq();
            }
            last_tick = now;
        }
        break;

    case PAGE_ROTATE:
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            page = PAGE_MENU_MAIN;
            draw();
            last_tick = now;
        }
        break;
    }
}
