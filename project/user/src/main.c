#include "zf_common_headfile.h"
#include "tick.h"
#include "menu.h"
//#include "motor.h"
//#include "encoder.h"
#include "track.h"
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

//    motor_init();
//    encoder_init();
    track_init();

    while (true)
    {
        uint32_t now = tick_get();

//        // ---- 双电机测试 (已注释) ----
//        static int16_t speed = 0;
//        static int8_t  dir   = 1;
//        static uint32_t next  = 0;
//        ...
//        motor_left(speed);
//        motor_right(speed);

//        // 编码器速度打印 (已注释)
//        static uint32_t enc_tick = 0;
//        if (now - enc_tick >= 20) {
//            enc_tick = now;
//            int l = (int)encoder_left_speed();
//            int r = (int)encoder_right_speed();
//            ...
//        }

        // 每20ms打印灰度 + 偏差 (CH0~4=5路, CH5=位置)
        {
            static uint32_t trk_tick = 0;
            if (now - trk_tick >= 20) {
                trk_tick = now;
                char buf[40];
                sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n",
                    track_bit(0), track_bit(1), track_bit(2),
                    track_bit(3), track_bit(4), track_position());
                if (!gpio_get_level(B2))
                    wireless_uart_send_string(buf);
            }
        }
    }
}
