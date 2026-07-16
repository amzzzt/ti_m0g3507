/**
 * protocol.c — 泰山派 UART0 协议 (轮询FIFO)
 *   帧: AA BB + found(1B) + dx(int16 LE) + dy(int16 LE) + csum(1B)
 */
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "protocol.h"

static int16_t g_dx, g_dy;
static uint8_t g_found;
static uint8_t g_new;

// 跳0滤波
static int16_t g_last_dx, g_last_dy;
static uint8_t  g_zero_cnt;

// 解析状态机 (静态, 跨 protocol_parse 调用保持)
static uint8_t s_state, s_buf[5], s_idx;

void protocol_init(uint32_t baud)
{
    g_dx = 0; g_dy = 0; g_found = 0; g_new = 0;
    g_last_dx = 0; g_last_dy = 0; g_zero_cnt = 0;
    s_state = 0; s_idx = 0;

    uart_init(UART_0, baud, UART0_TX_A28, UART0_RX_A31);
    DL_UART_Main_enableFIFOs(UART0);
}

void protocol_poll(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART0)) {
        uint8_t ch = DL_UART_Main_receiveData(UART0);

        switch (s_state) {
        case 0: if (ch == 0xAA) s_state = 1; break;
        case 1:
            if (ch == 0xBB) { s_state = 2; s_idx = 0; }
            else if (ch != 0xAA) s_state = 0;
            break;
        case 2:
            s_buf[s_idx++] = ch;
            if (s_idx >= 5) s_state = 3;
            break;
        case 3:
            s_state = 0;
            if ((s_buf[0]^s_buf[1]^s_buf[2]^s_buf[3]^s_buf[4]) == ch) {
                int16_t dx = (int16_t)(s_buf[1] | ((uint16_t)s_buf[2] << 8));
                int16_t dy = (int16_t)(s_buf[3] | ((uint16_t)s_buf[4] << 8));

                if (dx == 0 && dy == 0) {
                    g_zero_cnt++;
                    if (g_zero_cnt >= 3) {
                        g_dx = 0; g_dy = 0; g_found = 0;
                        g_last_dx = 0; g_last_dy = 0;
                        g_new = 1;
                    }
                } else {
                    g_zero_cnt = 0;
                    g_dx = dx; g_dy = dy; g_found = s_buf[0];
                    g_last_dx = dx; g_last_dy = dy;
                    g_new = 1;
                }
            }
            break;
        }
    }
}

offset_t protocol_get(void)
{
    offset_t o;
    uint32_t m = __get_PRIMASK(); __disable_irq();
    o.dx = g_dx; o.dy = g_dy; o.found = g_found;
    o.updated = g_new;
    __set_PRIMASK(m);
    g_new = 0;
    return o;
}
