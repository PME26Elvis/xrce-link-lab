#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <uxr/client/client.h>
#include <uxr/client/transport/serial/serial_transport.h>
#include "transport_zephyr_serial.h"

#define CLIENT_KEY  0xAABBCCDDu

void main(void)
{
    printk("xrce_nrf52_uxrapp: boot\n");

    uxrSerialTransport transport;
    if (!uxr_init_zephyr_serial_transport(&transport, 115200)) {
        printk("transport init failed\n");
        return;
    }

    /* 注意：這裡第二參數是“裝置字串”給 POSIX/Windows 平台用。
       我們 Zephyr 平台不使用它，但 API 需要一個字串，隨便填。
       實際使用的是 transport.platform 內的 Zephyr 裝置。 */
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

    /* 保持心跳，讓 Agent 能觀察到活動 */
    while (1) {
        uxr_run_session_time(&session, 50);
        printk("tick\n");
        k_sleep(K_MSEC(500));
    }
}
