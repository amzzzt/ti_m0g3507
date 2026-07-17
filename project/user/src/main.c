/**
 * main.c — 泰山派偏差接收 (ISR驱动)
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "protocol.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();
    tick_init();
    protocol_init(115200);

	char buf[64];
    sprintf(buf, "OK\r\n");
    wireless_uart_send_string(buf);

    while (1) {
        offset_t off = protocol_get();
        if (off.updated) {
            sprintf(buf, "%d %d\r\n", off.dx, off.dy);
            wireless_uart_send_string(buf);
        }
    }
}
