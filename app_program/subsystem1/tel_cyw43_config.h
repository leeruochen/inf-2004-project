/*
 *  tel_cyw43_config.h
 *  Subsystem 1 -- CYW43 driver configuration (TEL_BACKEND_WIFI=ON only).
 *  Selected with CYW43_CONFIG_FILE by the top-level CMakeLists.txt.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Silences the driver's printf output, then takes the SDK port settings
 *  unchanged.  Reason: the driver runs in the Telemetry task (P9), and only
 *  the debug drain may use stdio (rule R3a).  A printf here would contend for
 *  the stdio lock with the drain (P10); while the drain holds it, P9 would
 *  spin and the drain could never run to release it.
 */

#ifndef TEL_CYW43_CONFIG_H
#define TEL_CYW43_CONFIG_H

/* The driver's default (cyw43_config.h) includes <stdio.h> together with its
 * CYW43_PRINTF; keep the include so cyw43_stats.c still compiles cleanly.
 * Its cyw43_dump_stats() is the only direct printf left, and nothing here
 * calls it. */
#include <stdio.h>

#define CYW43_PRINTF(...)       ((void)0)

#include "cyw43_configport.h"

#endif /* TEL_CYW43_CONFIG_H */
