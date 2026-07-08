#ifndef _wireless_uart_h_
#define _wireless_uart_h_

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

// 发送
uint32_t wireless_uart_send_byte  (uint8_t data);
uint32_t wireless_uart_send_buffer(const uint8_t *buff, uint32_t len);
uint32_t wireless_uart_send_string(const char *str);

// 接收 (轮询, 非阻塞)
bool     wireless_uart_available  (void);
uint8_t  wireless_uart_read_byte  (void);
uint32_t wireless_uart_read_buffer(uint8_t *buff, uint32_t len);

// 初始化
void wireless_uart_init(void);

#endif
