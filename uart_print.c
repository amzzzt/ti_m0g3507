#include "uart_print.h"

// 发送单字节
void uart_send_byte(uint8_t data)
{
    DL_UART_transmitDataBlocking(Print_INST, data);
}

// 发送字符串
void uart_send_string(const char *str)
{
    while (*str) {
        DL_UART_transmitDataBlocking(Print_INST, *str++);
    }
}

// 发送数组
void uart_send_buffer(const uint8_t *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(Print_INST, data[i]);
    }
}
