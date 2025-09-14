#include "transport_stub.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <string.h>

static const struct device* s_uart;

bool xrce_transport_open(void)
{
    s_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(s_uart)) {
        printk("xrce: UART not ready\n");
        return false;
    }
    printk("xrce: UART ready (stub transport)\n");
    return true;
}

void xrce_transport_close(void)
{
    ARG_UNUSED(s_uart);
}

int xrce_transport_write(const uint8_t* data, size_t len)
{
#if defined(CONFIG_UART_CONSOLE)
    for (size_t i = 0; i < len; ++i) {
        printk("%c", data[i]);
    }
    return (int)len;
#else
    ARG_UNUSED(data); ARG_UNUSED(len);
    return -1;
#endif
}

int xrce_transport_read(uint8_t* data, size_t maxlen, int timeout_ms)
{
    /* 占位：目前不從 UART 讀；之後接上 uxrclient 時再實作 */
    ARG_UNUSED(data); ARG_UNUSED(maxlen); ARG_UNUSED(timeout_ms);
    k_sleep(K_MSEC(timeout_ms));
    return 0;
}
