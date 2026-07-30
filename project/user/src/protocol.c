/**
 * protocol.c — UART0 RX中断, 帧完成即刻更新
 */
#include "ti_msp_dl_config.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"
#include "protocol.h"

static int16_t g_dx, g_dy;
static uint8_t g_found;
static volatile uint8_t g_new;

static int16_t g_last_dx, g_last_dy;
static uint8_t  g_zero_cnt;

static void rx_isr(uint32_t event, void *ptr)
{
    (void)ptr;
    if (event != UART_INTERRUPT_STATE_RX) return;

    uint8_t ch;
    if (uart_query_byte(UART_0, &ch) != ZF_TRUE) return;

    static uint8_t state, buf[5], idx;

    switch (state) {
    case 0: if (ch == 0xAA) state = 1; break;
    case 1:
        if (ch == 0xBB) { state = 2; idx = 0; }
        else if (ch != 0xAA) state = 0;
        break;
    case 2:
        buf[idx++] = ch; if (idx >= 5) state = 3;
        break;
    case 3:
        state = 0;
        if ((buf[0]^buf[1]^buf[2]^buf[3]^buf[4]) == ch) {
            int16_t dx = (int16_t)(buf[1] | ((uint16_t)buf[2] << 8));
            int16_t dy = (int16_t)(buf[3] | ((uint16_t)buf[4] << 8));
            if (dx == 0 && dy == 0) {
                if (++g_zero_cnt >= 3) {
                    g_dx = 0; g_dy = 0; g_found = 0;
                    g_last_dx = 0; g_last_dy = 0; g_new = 1;
                }
            } else {
                g_zero_cnt = 0;
                g_dx = dx; g_dy = dy; g_found = buf[0];
                g_last_dx = dx; g_last_dy = dy; g_new = 1;
            }
        }
        break;
    }
}

void protocol_init(uint32_t baud)
{
    g_dx = 0; g_dy = 0; g_found = 0; g_new = 0;
    g_last_dx = 0; g_last_dy = 0; g_zero_cnt = 0;
    uart_init(UART_0, baud, UART0_TX_A28, UART0_RX_A31);
    DL_UART_Main_enableFIFOs(UART0);
    uart_set_callback(UART_0, rx_isr, NULL);
    uart_set_interrupt_config(UART_0, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void protocol_poll(void) {}  // 空, ISR 处理

offset_t protocol_get(void)
{
    offset_t o;
    uint32_t m = __get_PRIMASK(); __disable_irq();
    o.dx = g_dx; o.dy = g_dy; o.found = g_found; o.updated = g_new;
    g_new = 0;
    __set_PRIMASK(m);
    return o;
}
