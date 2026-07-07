#ifndef _uart_print_h_
#define _uart_print_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>

// 发送单字节
void uart_send_byte(uint8_t data);

// 发送字符串
void uart_send_string(const char *str);

// 发送数组
void uart_send_buffer(const uint8_t *data, uint32_t len);

#endif
