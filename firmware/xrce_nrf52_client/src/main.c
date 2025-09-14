#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "transport_stub.h"

/* 先做可見的「假心跳」，確保在 Renode 上能跑、能輸出 */
static const char* HB = "XRCE-STUB heartbeat\n";

void main(void)
{
    printk("xrce client (stub) starting...\n");
    if (!xrce_transport_open()) {
        printk("xrce transport open failed\n");
        return;
    }

    int i = 0;
    while (1) {
        char buf[64];
        int n = snprintk(buf, sizeof(buf), "[%04d] %s", i++, HB);
        xrce_transport_write((const uint8_t*)buf, (size_t)n);
        k_sleep(K_MSEC(500));
    }
}
