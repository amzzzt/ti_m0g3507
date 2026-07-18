/**
 * menu.c — TFT180 菜单 + SR04 测距
 * KEY1=确定/启动, KEY2=上, KEY3=下, KEY4=停止/返回
 */
#include "menu.h"
#include "zf_common_headfile.h"
#include "tick.h"
//#include "bsp_sr04.h" // 暂禁用超声波, 腾出 A8/A9
#include "radar.h"
//#include "servo.h"  // TIMG8 腾给步进电机
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
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear();
    tft180_show_string(40, 56, "TFT180 OK");
    __enable_irq();
}

// ---- 主逻辑 ----
void menu_run(void)
{
    uint32_t now = tick_get();

    // LED: 滤波距离<30cm → 100ms快闪
    {
        static uint32_t led_tick = 0;
        static uint8_t  led_on   = 0;
        float d = 99.0f; // sr04_read()
        if (d > 0.0f && d < 20.0f) {
            if (now - led_tick >= 50) {   // 50ms半周期
                led_tick = now;
                led_on = !led_on;
                if (led_on) gpio_high(LED1_PIN);
                else        gpio_low(LED1_PIN);
            }
        } else {
            gpio_low(LED1_PIN);
            led_on = 0;
        }
    }

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
                radar_clear_dots();
                __disable_irq();
                tft180_set_color(RGB565_WHITE, RGB565_BLACK);
                tft180_clear();
                tft180_show_string(0, 0,  "SR04 Measure");
                tft180_show_string(0, 16, "KEY1=Start");
                tft180_show_string(0, 28, "KEY4=Stop/Back");
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
        static uint32_t scan_tick = 0;
        static uint8_t  prev_deg  = 0;

        if (KEY_SHORT_PRESS == key_get_state(KEY_4)) {
            key_clear_state(KEY_4);
            active = 0;
            /*servo_disable();*/
            radar_clear_dots();
            page = PAGE_MENU_MAIN; draw_menu();
            last_tick = now;
            break;
        }

        if (!active) {
            if (KEY_SHORT_PRESS == key_get_state(KEY_1)) {
                key_clear_state(KEY_1);
                active = 1;
                next_trig = now; scan_tick = now;
                /*servo_enable();*/
                prev_deg = 0; // servo_get_angle()
                radar_clear_dots();
                radar_scanline_reset();
                radar_draw_base();
                last_tick = now;
            }
            break;
        }

        // ---- 扫描线+红点+无线 每40ms ----
        if (now - scan_tick >= 40) {
            scan_tick = now;
            uint8_t cur_ang = 0; // servo_get_angle()
            radar_draw_scanline(cur_ang);
            radar_draw_dots();

            // 逐飞助手: 通道0=角度, 通道1=滤波距离
            float dist = 99.0f; // sr04_read()
            char buf[32];
            int d = (int)(dist * 10.0f + 0.5f);
            sprintf(buf, "%d,%d.%d\r\n", (int)cur_ang, d / 10, d % 10);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }

        // 检测转向 → 180°扫完, 全清屏+红点
        {
            uint8_t cur = 0; // servo_get_angle()
            int diff = (int)cur - (int)prev_deg;
            // prev≥178且角度下降 → 刚过180°; prev≤2且角度上升 → 刚过0°
            if ((diff < 0 && prev_deg >= 178) || (diff > 0 && prev_deg <= 2)) {
                radar_clear_dots();
                radar_scanline_reset();
                radar_draw_base();
            }
            prev_deg = cur;
        }

        // 测量
        if (now - next_trig >= 60) {
            next_trig = now + 60;
            // sr04_trigger();

            float dist = 99.0f; // sr04_read()
            uint8_t ang = 0; // servo_get_angle()
            if (dist >= 2.0f && dist <= 50.0f) {
                radar_add_dot(ang, dist);
            }
        }
    } break;
    }
}
