#include "zf_common_headfile.h"
#include "tick.h"
#include "menu.h"
#include "bsp_sr04.h"
#include "radar.h"
#include "servo.h"

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    key_init(1);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    servo_init();
    wireless_uart_init();
    sr04_init();
    gpio_init(LED1_PIN, GPO, 0, GPO_PUSH_PULL);
    menu_init();

    // 初始画面
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_clear();
    tft180_show_string(40, 56, "TFT180 OK");

    while (true) { menu_run(); }
}
