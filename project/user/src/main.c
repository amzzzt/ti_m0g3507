/**
 * main.c — 泰山派 UART0 偏差接收
 *   A28(TX) A31(RX) 115200
 *   输出: dx dy  (无线串口)
 */
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "tick.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();
    tick_init();
    uart_init(UART_0, 115200, UART0_TX_A28, UART0_RX_A31);
    DL_UART_Main_enableFIFOs(UART0);

    char buf[64];
    sprintf(buf, "OK\r\n");
    wireless_uart_send_string(buf);

    uint8_t state = 0, pbuf[5], pidx = 0;

    while (1) {
        if (tick_ctrl_ready()) {
            tick_ctrl_clear();

            while (!DL_UART_isRXFIFOEmpty(UART0)) {
                uint8_t ch = DL_UART_Main_receiveData(UART0);

                switch (state) {
                case 0: if (ch == 0xAA) state = 1; break;
                case 1:
                    if (ch == 0xBB) { state = 2; pidx = 0; }
                    else if (ch != 0xAA) state = 0;
                    break;
                case 2:
                    pbuf[pidx++] = ch;
                    if (pidx >= 5) state = 3;
                    break;
                case 3:
                    state = 0;
                    if ((pbuf[0]^pbuf[1]^pbuf[2]^pbuf[3]^pbuf[4]) == ch) {
                        int dx = (int16_t)(pbuf[1] | ((uint16_t)pbuf[2] << 8));
                        int dy = (int16_t)(pbuf[3] | ((uint16_t)pbuf[4] << 8));
                        sprintf(buf, "%d %d\r\n", dx, dy);
                        wireless_uart_send_string(buf);
                    }
                    break;
                }
            }
        }
    }
}
