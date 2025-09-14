#pragma once
#include <zephyr/kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool xrce_transport_open(void);
void xrce_transport_close(void);
int  xrce_transport_write(const uint8_t* data, size_t len);
int  xrce_transport_read(uint8_t* data, size_t maxlen, int timeout_ms);

#ifdef __cplusplus
}
#endif
