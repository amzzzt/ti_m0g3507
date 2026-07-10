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
    radar_draw_base();
    wireless_uart_init();
    sr04_init();
    menu_init();

    while (true) { menu_run(); }
}
