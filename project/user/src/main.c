#include "zf_common_headfile.h"
#include "tick.h"
#include "menu.h"
#include "motor.h"
//#include "bsp_sr04.h"
//#include "radar.h"
//#include "servo.h"

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    tick_init();
    key_init(1);
//    tft180_set_dir(TFT180_CROSSWISE);
//    tft180_init();
//    servo_init();
    wireless_uart_init();
//    sr04_init();
//    gpio_init(LED1_PIN, GPO, 0, GPO_PUSH_PULL);
//    menu_init();

    motor_init();

    while (true)
    {
        // ---- 双电机缓慢加速→减速测试 ----
        static int16_t speed = 0;
        static int8_t  dir   = 1;
        static uint32_t next  = 0;
        uint32_t now = tick_get();

        if (now - next >= 50) {
            next = now;
            speed += dir * 10;

            if (speed >= 500)  dir = -1;
            if (speed <= 0)    dir =  1;

            motor_left(speed);
            motor_right(speed);
        }
    }
}
