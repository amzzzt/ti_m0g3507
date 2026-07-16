/**
 * protocol.h — 泰山派 UART0 协议模块 (轮询FIFO)
 */
#ifndef _protocol_h_
#define _protocol_h_

#include <stdint.h>

typedef struct {
    int16_t dx, dy;
    uint8_t found;
    uint8_t updated;
} offset_t;

void     protocol_init(uint32_t baud);
void     protocol_poll(void);        // 读FIFO+解析, 主循环死循环调用
offset_t protocol_get(void);         // 取最新值并清updated标志

#endif
