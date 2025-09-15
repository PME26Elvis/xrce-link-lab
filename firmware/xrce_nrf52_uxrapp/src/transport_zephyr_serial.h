#pragma once
#include <uxr/client/transport/serial/serial_transport.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool uxr_init_zephyr_serial_transport(uxrSerialTransport* transport, uint32_t baudrate);

#ifdef __cplusplus
}
#endif
