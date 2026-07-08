/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "zf_device_tft180.h"
#include "uart_print.h"
#include "wireless_uart.h"
#include <stdio.h>

// 正弦波查找表 (128 点, 0~255, 精确计算)
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

int main(void)
{
    char vofa_buf[80];
    uint32_t tick = 0;
    uint8_t idx;

    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    tft180_init();
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_color(RGB565_RED, RGB565_WHITE);
    tft180_clear();
    tft180_show_string(0, 0, "TFT180 OK!");

    wireless_uart_init();
    tft180_show_string(0, 32, "Wireless init OK");

    while (1) {
        OLED_ShowString(0,0,(u8 *)"Hello,World",16);
        OLED_Refresh();

        // VOFA+ 数据: 5 通道, 逗号分隔, \r\n 结尾
        tick++;
        idx = tick & 0x7F;                              // 0~127 循环

        // VOFA+ 5通道: 波形各不同 + 值域拉开但不离谱
        uint32_t ch0 = 2000u + (uint32_t)sin_table[idx] * 8;           // 正弦: 2000~4000
        uint32_t ch1 = 1000u + (uint32_t)sin_table[(idx+32)&0x7F] * 4; // 余弦: 1000~2000
        uint32_t ch2 = (tick * 4) % 1000;                              // 锯齿: 0~1000
        uint32_t ch3 = ((tick / 32) & 1) ? 5000u : 0u;                // 方波: 0/5000 跳变
        uint32_t ch4 = 3000u + ((tick * 3) % 2000);                    // 斜波: 3000~5000

        sprintf(vofa_buf, "%u,%u,%u,%u,%u\r\n",
                ch0, ch1, ch2, ch3, ch4);
        uart_send_string(vofa_buf);

        // TFT180 显示数据
        char tft_str[32];
        sprintf(tft_str, "ch:%u %u", ch0, tick);
        tft180_set_color(RGB565_BLUE, RGB565_WHITE);
        tft180_show_string(0, 16, tft_str);

        // 无线串口: 每帧都发 (200Hz, 跟 VOFA 同速率)
        wireless_uart_send_string(vofa_buf);
        if (wireless_uart_available()) {
            uint8_t rx = wireless_uart_read_byte();
            char rx_str[16];
            sprintf(rx_str, "RX:0x%02X", rx);
            tft180_set_color(RGB565_RED, RGB565_WHITE);
            tft180_show_string(0, 48, rx_str);
        }

        system_delay_ms(5);                             // ~200Hz 刷新
        DL_GPIO_togglePins(LED_PORT, LED_LED_0_PIN);
    }
}
