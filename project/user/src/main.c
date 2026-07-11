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
        // ---- 左电机缓慢加速→减速测试 ----
        static int16_t speed = 0;
        static int8_t  dir   = 1;       // 1=加速, -1=减速
        static uint32_t next  = 0;
        uint32_t now = tick_get();

        if (now - next >= 50) {         // 每50ms 加减10
            next = now;
            speed += dir * 10;

            if (speed >= 500)  dir = -1;  // 到500(50%)→减速
            if (speed <= 0)    dir =  1;  // 到0   →加速

            motor_set(speed);
        }
    }
}
