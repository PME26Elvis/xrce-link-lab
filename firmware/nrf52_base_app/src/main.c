#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
    printk("xrce-link-lab: nRF52840 baseline app running.\n");
    for (int i = 0; ; ++i) {
        printk("heartbeat %d\n", i);
        k_sleep(K_MSEC(500));
    }
}
