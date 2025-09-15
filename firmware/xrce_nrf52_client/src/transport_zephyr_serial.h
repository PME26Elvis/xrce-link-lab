#pragma once
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool uxr_init_zephyr_serial_transport(
    uxrSerialTransport* transport,
    const char* dev,
    uint32_t baudrate);

#ifdef __cplusplus
}
#endif