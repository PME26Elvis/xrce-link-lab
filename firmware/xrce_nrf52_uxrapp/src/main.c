#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifdef USE_UXRCLIENT
  #include <uxr/client/client.h>
  #include <uxr/client/transport/serial/serial_transport.h>
  #include "transport_zephyr_serial.h"
  #define CLIENT_KEY  0xAABBCCDDu
#endif

void main(void)
{
#ifndef USE_UXRCLIENT
    /* 沒有 uxrclient：走最小心跳，確保 zephyr-build-uxrapp 永遠綠燈 */
    printk("xrce_nrf52_uxrapp: heartbeat-only build (no uxrclient)\n");
    int i = 0;
    while (1) {
        printk("[uxrapp %04d] heartbeat\n", i++);
        k_sleep(K_MSEC(500));
    }
#else
    /* 有 uxrclient：建立 serial transport + session */
    printk("xrce_nrf52_uxrapp: XRCE build\n");

    uxrSerialTransport transport;
    if (!uxr_init_zephyr_serial_transport(&transport, 115200)) {
        printk("transport init failed\n");
        return;
    }

    /* 注意：在 Zephyr 上，我們自備 transport 平台，不依賴 dev 路徑字串。 */
    if (!uxr_init_serial_transport(&transport, "ignored-on-zephyr", 115200)) {
        printk("uxr_init_serial_transport failed\n");
        return;
    }

    uxrSession session;
    uxr_init_session(&session, &transport.comm, CLIENT_KEY);

    if (!uxr_create_session(&session)) {
        printk("uxr_create_session failed\n");
        return;
    }

    printk("XRCE: session established\n");

    while (1) {
        uxr_run_session_time(&session, 50);
        printk("tick\n");
        k_sleep(K_MSEC(500));
    }
#endif
}
