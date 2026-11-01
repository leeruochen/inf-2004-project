/*
 *  task_tel.h
 *  Subsystem 1 -- internal interface to the Telemetry task (P9, 20 ms wake).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Kernel side only: included by task_tel.c, subsystem1.c and
 *  subsystem1_test.c.  Never included by drv_*.c (rule R1).
 */

#ifndef TASK_TEL_H
#define TASK_TEL_H

#include "subsystem1.h"
#include "tel_link.h"

/* ---------------------------------------------------------------------------
 * Subsystem 1 private tuning (not shared, so it lives here, not config/).
 * ------------------------------------------------------------------------- */

/** One connection attempt (join + DHCP + DNS + MQTT CONNACK) may take this long. TUNE */
#define TEL_CONNECT_TIMEOUT_MS      (15000U)

/** Event frames published per wake at most, so a long backlog cannot hog a wake. */
#define TEL_FLUSH_MAX_PER_WAKE      (8U)

/** Received command messages handled per wake at most. */
#define TEL_CMD_MAX_PER_WAKE        (4U)

/** Status line period ("[TEL] CONNECTED tx=..."). */
#define TEL_STATUS_LOG_PERIOD_MS    (60000U)

/**
 * Console backend only (TEL_BACKEND_WIFI=OFF): print every Nth periodic frame
 * (motion and sensor every N x 200 ms, heartbeat every N x 1 s) so the shared
 * debug console stays readable for the other buddies.  Events are always
 * printed.  0 = print events only.
 */
#define TEL_CONSOLE_PERIODIC_DIV    (5U)

/* tel_cmd_receive() result when MBF_NAV_IN stayed full (beyond DREP_FRAME_* / TEL_WIRE_*). */
#define TEL_CMD_POST_FAIL           (-8)

/** Create the Telemetry task (from subsystem1_start()).  Returns the task ID or a negative error. */
int32_t tel_task_start(void);

/**
 * Receive path for one MQTT message on car/cmd/... (also used by the
 * on-target tests to inject commands without a broker).  Validates with
 * tel_wire_cmd(); a valid command is posted to MBF_NAV_IN with
 * NAV_MBF_SEND_TMO_MS (5 ms), anything else is counted and logged, never
 * posted.  Returns DREP_FRAME_OK, a reject reason, or TEL_CMD_POST_FAIL.
 */
int32_t tel_cmd_receive(char const *p_topic, uint8_t const *p_buf, uint32_t n);

#if APP_TEST
/** Tests only: the three snapshot words (IR, range, attitude) as stored. */
void tel_test_snapshot(uint32_t *p_ir, uint32_t *p_range, uint32_t *p_att);

/** Tests only: true while a posted obstacle profile has not been framed yet. */
bool tel_test_obstacle_pending(void);

/** Tests only: frames waiting in the event outbox. */
uint32_t tel_test_outbox_count(void);
#endif

#endif /* TASK_TEL_H */
