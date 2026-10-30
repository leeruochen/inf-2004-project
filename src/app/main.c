/*
 *  main.c
 *  SDK main(): the firmware's entry point.  Owner: platform (Batch B,
 *  stage B4).  SDK side: SDK headers only.
 *
 *  Sequence (ARCHITECTURE.md "Two entry points"):
 *    1. stdio_init_all(): USB CDC only (CMakeLists.txt enables stdio USB,
 *       never stdio UART: GP0/GP1 are the right encoder).
 *    2. DBG_ENABLE builds: wait up to DBG_USB_CONNECT_WAIT_MS for a terminal,
 *       so the boot log is seen, then print "pre-kernel ok" as a sign of life
 *       before the kernel exists.  These are the only allowed SDK wait and
 *       stdio calls outside the debug drain (lines marked RULE-ALLOW).
 *    3. hal_kernel_start(): never returns.  The kernel then runs usermain()
 *       (app_program/app_main.c) in its init task.
 *
 *  This file also defines hal_console_connected(), because it is the only
 *  file that includes the SDK's USB stdio header.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "config/timing.h"
#include "src/rtos/port/hal.h"

/* Poll step while waiting for the terminal. */
#define MAIN_USB_POLL_MS    (10U)

bool hal_console_connected(void)
{
    return stdio_usb_connected();
}

int main(void)
{
    (void)stdio_init_all();

#if DBG_ENABLE
    for (uint32_t waited_ms = 0U;
         (waited_ms < DBG_USB_CONNECT_WAIT_MS) && !stdio_usb_connected();
         waited_ms += MAIN_USB_POLL_MS)
    {
        sleep_ms(MAIN_USB_POLL_MS);                         /* RULE-ALLOW: pre-kernel */
    }
    printf("pre-kernel ok\n");                              /* RULE-ALLOW: pre-kernel */
#endif /* DBG_ENABLE */

    hal_kernel_start();
}
