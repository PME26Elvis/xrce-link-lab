#include "transport_zephyr_serial.h"

#ifdef USE_UXRCLIENT

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdlib.h>

typedef struct {
    const struct device* dev;
} zephyr_serial_platform;

bool uxr_init_zephyr_serial_transport(uxrSerialTransport* transport, uint32_t baudrate)
{
    ARG_UNUSED(baudrate);
    zephyr_serial_platform* plat = (zephyr_serial_platform*)malloc(sizeof(*plat));
    if (!plat) return false;

    const struct device* u = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(u)) {
        free(plat);
        return false;
    }
    plat->dev = u;
    transport->platform = plat;
    return true;
}

/* uxrclient 會連結以下符號 */
bool uxr_close_serial_transport(uxrSerialTransport* transport)
{
    if (transport && transport->platform) {
        free(transport->platform);
        transport->platform = NULL;
    }
    return true;
}

size_t uxr_write_serial_data(uxrSerialTransport* transport,
                             const uint8_t* buf, size_t len, uint8_t* err)
{
    zephyr_serial_platform* plat = (zephyr_serial_platform*)transport->platform;
    for (size_t i = 0; i < len; ++i) {
        uart_poll_out(plat->dev, buf[i]);
    }
    if (err) *err = 0;
    return len;
}

size_t uxr_read_serial_data(uxrSerialTransport* transport,
                            uint8_t* buf, size_t len, int timeout_ms, uint8_t* err)
{
    zephyr_serial_platform* plat = (zephyr_serial_platform*)transport->platform;
    size_t n = 0;
    int64_t end = k_uptime_get() + timeout_ms;
    while (n < len && k_uptime_get() < end) {
        unsigned char c;
        if (uart_poll_in(plat->dev, &c) == 0) {
            buf[n++] = c;
        } else {
            k_sleep(K_USEC(200));
        }
    }
    if (err) *err = 0;
    return n;
}

#endif /* USE_UXRCLIENT */
