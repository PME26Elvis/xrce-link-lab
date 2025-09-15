#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_UXRCLIENT
  #include <uxr/client/transport/serial/serial_transport.h>
  bool uxr_init_zephyr_serial_transport(uxrSerialTransport* transport, uint32_t baudrate);
#else
  /* 沒有 uxrclient 時此檔不會被編，但為了安全留個空宣告避免誤含 */
  typedef struct uxrSerialTransport_ { void* platform; } uxrSerialTransport;
  static inline bool uxr_init_zephyr_serial_transport(uxrSerialTransport* t, uint32_t b){ (void)t; (void)b; return false; }
#endif
