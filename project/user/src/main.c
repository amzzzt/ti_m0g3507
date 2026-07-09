/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* ...
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "tick.h"
#include "servo.h"
#include <stdio.h>

#define LED0_PIN    B22
#define LED1_PIN    B21

// 正弦波查找表 (128 点)
static const uint8_t sin_table[128] = {
    128,134,140,146,152,158,164,170,176,182,187,193,198,203,208,213,
    217,222,226,229,233,236,239,242,244,246,248,249,250,251,251,251,
    251,250,249,248,246,244,242,239,236,233,229,226,222,217,213,208,
    203,198,193,187,182,176,170,164,158,152,146,140,134,128,121,115,
    109,103, 97, 90, 85, 79, 73, 67, 62, 57, 52, 47, 42, 37, 33, 29,
     25, 21, 18, 15, 12, 10,  7,  5,  4,  2,  1,  1,  0,  0,  0,  1,
      1,  2,  4,  5,  7, 10, 12, 15, 18, 21, 25, 29, 33, 37, 42, 47,
     52, 57, 62, 67, 73, 79, 85, 90, 97,103,109,115,121
};

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();

    tick_init();
    key_init(1);

    gpio_init(LED0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    servo_init();
    tft180_init();
    wireless_uart_init();

    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_color(RGB565_RED, RGB565_WHITE);
    tft180_clear();
    tft180_show_string(0, 0, "TFT180 OK");

    uint32_t tick = 0;
    char vofa_buf[80];

    while (true)
    {
        tick++;
        uint8_t idx = tick & 0x7F;

        uint32_t ch0 = 2000u + (uint32_t)sin_table[idx] * 8;                 // 正弦
        uint32_t ch1 = 1000u + (uint32_t)sin_table[(idx+32)&0x7F] * 4;       // 余弦
        uint32_t ch2 = (tick * 4) % 1000;                                     // 锯齿
        uint32_t ch3 = ((tick / 32) & 1) ? 5000u : 0u;                       // 方波
        uint32_t ch4 = 3000u + ((tick * 3) % 2000);                           // 斜波

        sprintf(vofa_buf, "%lu,%lu,%lu,%lu,%lu\r\n", ch0, ch1, ch2, ch3, ch4);

        // 有线串口 (UART0 debug) 和 无线串口 各发一份
        printf("%s", vofa_buf);
        wireless_uart_send_string(vofa_buf);

        // TFT180 显示当前数据
        char tft_str[32];
        sprintf(tft_str, "ch0:%lu", ch0);
        tft180_set_color(RGB565_BLUE, RGB565_WHITE);
        tft180_show_string(0, 16, tft_str);

        gpio_toggle_level(LED0_PIN);
        system_delay_ms(5);
    }
}
