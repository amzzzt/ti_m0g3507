/**
 * main.c — Y轴滤波波形
 */
#include "zf_common_headfile.h"
#include "tick.h"
#include "protocol.h"
#include "filter.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    wireless_uart_init();
    tick_init();
    protocol_init(115200);
    filter_init();

    char buf[64];

    while (1) {
        offset_t off = protocol_get();
        if (off.updated) {
            filter_update(off.dx, off.dy, tick_get());
            sprintf(buf, "%d,%d\r\n", off.dy, filter_y());
            wireless_uart_send_string(buf);
        }
    }
}
