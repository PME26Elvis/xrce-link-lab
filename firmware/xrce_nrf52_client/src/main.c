#include <uxr/client/client.h>
#include "transport_zephyr_serial.h"
#include <zephyr/kernel.h>
#include <string.h>

#define CLIENT_KEY     0xAAAA
#define AGENT_ADDR     "/dev/ttyS0"

void main(void)
{
    uxrSerialTransport transport;
    if(!uxr_init_zephyr_serial_transport(&transport, "UART_0", 115200))
    {
        printk("Transport init failed\n");
        return;
    }

    uxr_init_serial_transport(&transport, AGENT_ADDR, 115200);

    uxrSession session;
    uxr_init_session(&session, &transport.comm, CLIENT_KEY);

    if(!uxr_create_session(&session))
    {
        printk("Session creation failed\n");
        return;
    }

    printk("XRCE session established\n");

    while(true)
    {
        uxr_run_session_time(&session, 1000);
        printk("heartbeat\n");
        k_sleep(K_MSEC(500));
    }
}