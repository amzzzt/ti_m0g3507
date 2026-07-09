/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* ...
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "tick.h"
#include "servo.h"

// 引脚: LED_0=PB22, LED_1=PB21, KEY1=A30, KEY2=A31, KEY3=B0, KEY4=B1, 舵机PWM=PB7
#define LED0_PIN    B22
#define LED1_PIN    B21

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();

    tick_init();                        // TIMA0 PIT, 1ms 节拍 (中断内自动扫舵机)
    key_init(1);                        // 按键扫描周期 1ms

    gpio_init(LED0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    servo_init();                       // TIMG8 CH1 → PB7, 50Hz, 初始 90°
    tft180_init();                      // SPI1: PB9=SCK, PB8=MOSI

    tft180_show_string(0, 0,  "MSPM0G3507");
    tft180_show_string(0, 16, "TFT180 OK");

    while (true)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_1))
        {
            key_clear_state(KEY_1);
            gpio_toggle_level(LED0_PIN);
            gpio_toggle_level(LED1_PIN);
        }
    }
}
