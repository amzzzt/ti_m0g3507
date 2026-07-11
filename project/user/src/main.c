#include "zf_common_headfile.h"
#include "tick.h"
#include "menu.h"
#include "motor.h"
#include "encoder.h"
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
    encoder_init();

    while (true)
    {
        // ---- 双电机缓慢加速→减速测试 ----
        static int16_t speed = 0;
        static int8_t  dir   = 1;
        static uint32_t next  = 0;
        static uint32_t enc_tick = 0;
        uint32_t now = tick_get();

        if (now - next >= 50) {
            next = now;
            speed += dir * 100;

            if (speed >= 4000)  dir = -1;
            if (speed <= 0)     dir =  1;

            motor_left(speed);
            motor_right(speed);
        }

        // 每20ms打印滤波后速度 (CH0=左, CH1=右)
        if (now - enc_tick >= 20) {
            enc_tick = now;
            char buf[30];
            int l = (int)encoder_left_speed();
            int r = (int)encoder_right_speed();
            sprintf(buf, "%d,%d\r\n", l, r);
            if (!gpio_get_level(B2))
                wireless_uart_send_string(buf);
        }
    }
}
