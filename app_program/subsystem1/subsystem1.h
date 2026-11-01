/*
 *  subsystem1.h
 *  Subsystem 1 -- telemetry (WiFi, MQTT, dashboard link).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  This header is subsystem 1's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem1/subsystem1.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem1.c and subsystem1_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Telemetry   TASK_PRI_TEL (9)   TASK_STACK_TEL_BYTES
 *    Wakes every TEL_WAKE_PERIOD_MS (20 ms, delta D2), publishes motion and
 *    sensor frames every 200 ms and a heartbeat every 1 s (FR1).
 *
 *  Interfaces (TEAM.md section 5):
 *    IF-04  MBF_TELEMETRY   receives tel_evt_t from every subsystem (owner).
 *                           Post with the tel_post_*() helpers below: they
 *                           never wait (TMO_POL) and count a full queue as a
 *                           drop (NFR1).
 *    IF-01  MBF_NAV_IN      sends validated remote commands (START, STOP,
 *                           RESET) to the Vehicle Controller (S2), 5 ms timeout.
 *    IF-08  MTX_MOTION      copies motion_state_t through motion_get_state().
 *    IF-06  EVF_SAFETY      sets and clears FAULT_WIFI_LOSS (b8, single
 *                           setter); reads the other bits with tk_ref_flg.
 *    IF-12  (proposed)      latest-value sensor snapshot for the 200 ms
 *                           telemetry_sensor frame and the newest obstacle
 *                           profile: tel_set_*() / tel_post_obstacle() below.
 *                           Single aligned word stores, never a wait.
 *
 *  Backends (CMake option TEL_BACKEND_WIFI):
 *    OFF  console: every frame is printed as "[TEL] <topic> <hex>" through
 *         the dbg_ ring (periodic frames decimated, TEL_CONSOLE_PERIODIC_DIV).
 *         No radio.  tools/dashboard/tel_dashboard.py decodes a saved log.
 *    ON   CYW43 + lwIP MQTT, all network calls inside the Telemetry task
 *         (drv_wifi.c, poll mode, assumption A9).  Needs config/secrets.h.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#ifndef TEL_BACKEND_WIFI
#error "subsystem1.h: TEL_BACKEND_WIFI must be defined by the build (robocar_options)"
#endif

/* ---------------------------------------------------------------------------
 * IF-04 producers: post an event to the Telemetry task.  Callable from any
 * task (not from ISRs).  Never waits.  Returns true if the event was queued,
 * false if MBF_TELEMETRY was full (the event is dropped and counted).
 * ------------------------------------------------------------------------- */

/** Post an already built event (drep_tel_make / drep_tel_hump). */
bool tel_post(tel_evt_t evt);

/** Build and post an event of type DREP_TEL_* stamped with the current time. */
bool tel_post_event(uint32_t type, uint32_t payload, bool oor);

/** Buddy 4: a hump, height in mm (0..100) and peak pitch in 0.5 deg units. */
bool tel_post_hump(int32_t height_mm, int32_t pitch_half_deg);

/** Buddy 3: a validated barcode command (DREP_BC_CMD_LEFT .. DREP_BC_CMD_UTURN). */
bool tel_post_barcode(uint32_t bc_cmd);

/** Buddy 4 / anyone: EVF_SAFETY bits at the moment a fault was raised. */
bool tel_post_fault(uint32_t evf_safety_bits);

/* ---------------------------------------------------------------------------
 * IF-12 (proposed): latest-value snapshot.  Each setter has ONE caller task
 * and does a single aligned 32-bit store, so it is atomic on the M0+ and
 * never waits.  The Telemetry task reads the newest values every 200 ms.
 * Values are clamped to their wire field (docs/DATA_REPRESENTATION.md 8).
 * ------------------------------------------------------------------------- */

/** Buddy 3 (Line-Following): raw ADC of IR_R and IR_C, 0..4095. */
void tel_set_ir_raw(uint32_t ir_r_raw, uint32_t ir_c_raw);

/** Buddy 5 (Obstacle Scanning): forward ultrasonic range, 0..400 cm (0 = no echo). */
void tel_set_range_cm(uint32_t range_cm);

/** Buddy 4 (IMU Processing): pitch and roll in 0.5 deg units, +-90 (= +-45 deg). */
void tel_set_attitude(int32_t pitch_half_deg, int32_t roll_half_deg);

/**
 * Buddy 5 (one caller task): a new obstacle profile.  Newest wins: the
 * Telemetry task publishes it once as an event_obstacle frame.  Carries all
 * 37 bits (the 32-bit tel_evt_t payload cannot; DATA_REPRESENTATION.md 10).
 */
void tel_post_obstacle(obstacle_profile_t prof);

/* ---------------------------------------------------------------------------
 * Status (any task)
 * ------------------------------------------------------------------------- */

/** Counters since boot, for logs, tests and the 60 s status line. */
typedef struct
{
    uint32_t link_state;        /**< TEL_LINK_* (tel_link.h), 3 bits */
    uint32_t evt_rx;            /**< tel_evt_t received from MBF_TELEMETRY */
    uint32_t evt_invalid;       /**< received but rejected (drep_tel_valid) */
    uint32_t evt_post_drops;    /**< tel_post*() refused: MBF_TELEMETRY full */
    uint32_t frames_sent;       /**< frames handed to the backend */
    uint32_t publish_fail;      /**< publishes refused by the backend */
    uint32_t offline_skipped;   /**< periodic frames skipped while the link was down */
    uint32_t outbox_dropped;    /**< buffered event frames dropped (oldest) */
    uint32_t outbox_peak;       /**< highest outbox fill level */
    uint32_t cmd_accepted;      /**< commands posted to MBF_NAV_IN */
    uint32_t cmd_rejected;      /**< commands rejected by the receive check */
    uint32_t cmd_post_fail;     /**< accepted but MBF_NAV_IN stayed full for 5 ms */
    uint32_t connects;          /**< entries into CONNECTED */
    uint32_t link_failures;     /**< failed attempts / torn-down sessions */
    uint32_t wake_overruns;     /**< 20 ms releases skipped (app_period_wait) */
    uint32_t wakes;             /**< task wakes since start */
} tel_stats_t;

/** Copy of the counters (each word read once; not a consistent snapshot). */
void tel_get_stats(tel_stats_t *p_out);

/** True once the Telemetry task runs its loop. */
bool tel_is_running(void);

#endif /* SUBSYSTEM1_H */
