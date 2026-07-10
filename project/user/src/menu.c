/**
 * menu.c — TFT180 菜单 + SR04 测距
 * KEY1=确定/启动, KEY2=上, KEY3=下, KEY4=停止/返回
 */
#include "menu.h"
#include "zf_common_headfile.h"
#include "tick.h"
#include "bsp_sr04.h"
#include "radar.h"
#include <stdio.h>

typedef enum { PAGE_NORMAL, PAGE_MENU_MAIN, PAGE_ROTATE } page_t;

static page_t  page = PAGE_NORMAL;
static uint8_t cursor = 0;
static uint32_t last_tick = 0;

#define ITEM_COUNT 2
static const char *items[ITEM_COUNT] = { "1.Rotate", "2.Exit" };

void menu_init(void) { page = PAGE_NORMAL; cursor = 0; }

// ---- 绘制主菜单 ----
static void draw_menu(void)
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

static void draw_normal(void)
{
    __disable_irq();
    radar_draw_base();
    __enable_irq();
}

// ---- 主逻辑 ----
void menu_run(void)
{
    uint32_t now = tick_get();
    if (now - last_tick < 20) return;

    switch (page) {

    case PAGE_NORMAL:
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            page = PAGE_MENU_MAIN; cursor = 0; draw_menu();
            last_tick = now;
        }
        break;

    case PAGE_MENU_MAIN: {
        uint8_t redraw = 0;
        if (KEY_SHORT_PRESS == key_get_state(KEY_2)) {
            key_clear_state(KEY_2);
            if (cursor > 0) { cursor--; redraw = 1; }
            last_tick = now;
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_3)) {
            key_clear_state(KEY_3);
            if (cursor < ITEM_COUNT - 1) { cursor++; redraw = 1; }
            last_tick = now;
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
            key_clear_state(KEY_1);
            if (cursor == 0) {
                page = PAGE_ROTATE;
                __disable_irq();
                tft180_set_color(RGB565_BLACK, RGB565_WHITE);
                tft180_clear();
                tft180_show_string(0, 0,  "SR04 Measure");
                tft180_show_string(0, 24, "KEY1=Start");
                tft180_show_string(0, 42, "KEY4=Stop/Back");
                __enable_irq();
            } else {
                page = PAGE_NORMAL; draw_normal();
            }
            last_tick = now;
        }
        if (redraw) draw_menu();
    } break;

    case PAGE_ROTATE: {
        static uint8_t  active = 0;
        static uint32_t next_trig = 0;

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            active = 0;
            __disable_irq();
            tft180_show_string(0, 60, "Status: OFF");
            __enable_irq();
            page = PAGE_MENU_MAIN; draw_menu();
            last_tick = now;
            break;
        }

        if (!active) {
            if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
                key_clear_state(KEY_1);
                active = 1;
                next_trig = now;
                __disable_irq();
                tft180_show_string(0, 60, "Status: ON ");
                __enable_irq();
                last_tick = now;
            }
            break;
        }

        // 测量中 (不操作屏幕, 防止关中断丢 ECHO)
        if (now - next_trig >= 30) {
            next_trig = now + 30;
            sr04_trigger();

            char buf[20];
            sprintf(buf, "%.1f\r\n", (double)sr04_read());
            wireless_uart_send_string(buf);
        }
    } break;
    }
}
