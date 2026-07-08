#include "wireless_uart.h"
#include "delay.h"

// ---- RTS 引脚 (来自 syscfg) ----
#define RTS_PORT    WireLess_RTS_PORT
#define RTS_PIN     WireLess_RTS_PIN_0_PIN

// ---- 简易环形 FIFO (64 字节, 2 的幂) ----
#define FIFO_SIZE  64
#define FIFO_MASK  (FIFO_SIZE - 1)

static uint8_t  fifo_buf[FIFO_SIZE];
static volatile uint32_t fifo_head = 0;  // 写指针
static volatile uint32_t fifo_tail = 0;  // 读指针

static inline bool fifo_empty(void) {
    return fifo_head == fifo_tail;
}
static inline bool fifo_full(void) {
    return ((fifo_head + 1) & FIFO_MASK) == fifo_tail;
}
static void fifo_push(uint8_t data) {
    if (!fifo_full()) {
        fifo_buf[fifo_head] = data;
        fifo_head = (fifo_head + 1) & FIFO_MASK;
    }
}
static uint8_t fifo_pop(void) {
    uint8_t data = fifo_buf[fifo_tail];
    fifo_tail = (fifo_tail + 1) & FIFO_MASK;
    return data;
}

// ---- 超时计数 ----
#define TIMEOUT_MS  1

// ---- 底层: 等待 RTS 就绪 (LOW = 模块空闲) ----
static bool rts_wait_ready(void)
{
    uint32_t timeout = TIMEOUT_MS;
    while (timeout) {
        if (!(DL_GPIO_readPins(RTS_PORT, RTS_PIN) & RTS_PIN)) {
            return true;   // RTS LOW, 模块就绪
        }
        system_delay_ms(1);
        timeout--;
    }
    return false;          // 超时
}

//===================================================================
// 发送
//===================================================================
uint32_t wireless_uart_send_byte(uint8_t data)
{
    if (!rts_wait_ready()) return 1;
    DL_UART_transmitDataBlocking(WireLess_INST, data);
    return 0;
}

uint32_t wireless_uart_send_buffer(const uint8_t *buff, uint32_t len)
{
    if (!buff) return len;

    while (len) {
        if (!rts_wait_ready()) break;

        uint32_t chunk = (len > 30) ? 30 : len;  // 每包最多 30 字节
        for (uint32_t i = 0; i < chunk; i++) {
            DL_UART_transmitDataBlocking(WireLess_INST, buff[i]);
        }
        buff += chunk;
        len  -= chunk;
    }
    return len;
}

uint32_t wireless_uart_send_string(const char *str)
{
    if (!str) return 0;
    uint32_t len = 0;
    while (str[len]) len++;
    return wireless_uart_send_buffer((const uint8_t *)str, len);
}

//===================================================================
// 接收 (中断驱动 — UART RX → FIFO)
//===================================================================

// UART1 中断服务函数 (覆盖 SDK 弱定义)
void WireLess_INST_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(WireLess_INST);

    if (status & DL_UART_INTERRUPT_RX) {
        // 接收中断: 读到 FIFO
        uint8_t data = (uint8_t)DL_UART_receiveData(WireLess_INST);
        fifo_push(data);
        DL_UART_clearInterruptStatus(WireLess_INST, DL_UART_INTERRUPT_RX);
    }
}

// 查询是否有数据
bool wireless_uart_available(void)
{
    return !fifo_empty();
}

// 读一个字节 (非阻塞, 无数据返回 0)
uint8_t wireless_uart_read_byte(void)
{
    if (fifo_empty()) return 0;
    return fifo_pop();
}

// 读多个字节
uint32_t wireless_uart_read_buffer(uint8_t *buff, uint32_t len)
{
    if (!buff) return 0;
    uint32_t count = 0;
    while (count < len && !fifo_empty()) {
        buff[count++] = fifo_pop();
    }
    return count;
}

//===================================================================
// 初始化
//===================================================================
void wireless_uart_init(void)
{
    // UART 和 GPIO 已由 SYSCFG_DL_init() → SYSCFG_DL_WireLess_init() 初始化

    // 清空 FIFO
    fifo_head = 0;
    fifo_tail = 0;

    // 使能 UART RX 中断
    DL_UART_enableInterrupt(WireLess_INST, DL_UART_INTERRUPT_RX);

    // 使能 NVIC
    NVIC_EnableIRQ(WireLess_INST_INT_IRQN);
}
