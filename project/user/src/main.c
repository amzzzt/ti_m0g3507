#include "zf_common_headfile.h"
#include "tick.h"
#include "menu.h"
#include "bsp_sr04.h"
#include <stdio.h>

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    key_init(1);
    tft180_init();
    wireless_uart_init();
    sr04_init();
    menu_init();

    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_clear();
    tft180_show_string(0, 0, "TFT180 OK");

    while (true)
     { 
        menu_run(); 
     }
}
