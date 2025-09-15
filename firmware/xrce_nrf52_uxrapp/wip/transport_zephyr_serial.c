#include "transport_zephyr_serial.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

typedef struct
{
    const struct device* dev;
} zephyr_serial_platform;

bool uxr_init_zephyr_serial_transport(
    uxrSerialTransport* transport,
    const char* dev,
    uint32_t baudrate)
{
    (void) baudrate; // 暫不調整，Zephyr DTS 已定義
    zephyr_serial_platform* plat = (zephyr_serial_platform*) malloc(sizeof(*plat));
    if(!plat) return false;
    plat->dev = device_get_binding(dev);
    if(!plat->dev) return false;
    transport->platform = plat;
    return true;
}

bool uxr_close_serial_transport(uxrSerialTransport* transport)
{
    free(transport->platform);
    return true;
}

size_t uxr_write_serial_data(
    uxrSerialTransport* transport,
    const uint8_t* buf, size_t len, uint8_t* err)
{
    zephyr_serial_platform* plat = (zephyr_serial_platform*) transport->platform;
    for(size_t i=0; i<len; ++i)
        uart_poll_out(plat->dev, buf[i]);
    *err = 0;
    return len;
}

size_t uxr_read_serial_data(
    uxrSerialTransport* transport,
    uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    zephyr_serial_platform* plat = (zephyr_serial_platform*) transport->platform;
    size_t count = 0;
    int64_t end = k_uptime_get() + timeout;
    while(count < len && k_uptime_get() < end)
    {
        unsigned char c;
        if(uart_poll_in(plat->dev, &c) == 0)
        {
            buf[count++] = c;
        }
    }
    *err = 0;
    return count;
}
