#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
    printk("xrce_nrf52_uxrapp boot\n");
    int i = 0;
    while (1) {
        printk("[uxrapp %04d] heartbeat\n", i++);
        k_sleep(K_MSEC(500));
    }
}
