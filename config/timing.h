/**
 * @file timing.h
 * @brief Periods, timeouts and timing thresholds (design.md section 3.2 and 3.3).
 *
 * Pure macros: no SDK or kernel includes. Units are in every name.
 * Tunables are tagged TUNE. Unknowns carry TODO(owner-to-confirm, buddy N).
 *
 * design.md 3.2 -> macro map (D_cmd, D_detect, side margin, a_move, degree_hump
 * and g_collision are distances/accelerations and live in thresholds.h):
 *   LF sample 5 ms ............... LF_SAMPLE_PERIOD_MS
 *   Steering post 20 ms .......... LF_STEER_POST_PERIOD_MS
 *   Motion period 20 ms .......... MOTION_PERIOD_MS
 *   IMU sample 10 ms ............. IMU_SAMPLE_PERIOD_MS
 *   IMU watchdog 50 ms ........... SAFETY_IMU_WDOG_MS
 *   Telemetry 200 ms / 1 s ....... TEL_WAKE_PERIOD_MS, TEL_PUBLISH_EVERY_N_WAKES,
 *                                  TEL_HEARTBEAT_EVERY_N_WAKES
 *   Servo step 60 ms, 5 x 30 deg . OBS_SERVO_STEP_PERIOD_MS, OBS_COARSE_BEARING_COUNT,
 *                                  OBS_COARSE_STEP_DEG
 *   N hysteresis 3 (15 ms) ....... LF_HYST_SAMPLES, LF_HYST_MS
 *   T_lost 100 ms ................ LF_T_LOST_MS
 *   T_turn 500 ms ................ LF_T_TURN_MS
 *   T_uturn 1.0 s ................ LF_T_UTURN_MS
 *   T_search 2.0 s ............... LF_T_SEARCH_MS
 *   D_cmd 30 cm .................. BC_D_CMD_CM (thresholds.h)
 *   T_end 3 x widest ............. BC_T_END_WIDEST_MULT
 *   D_detect 25 cm ............... OBS_D_DETECT_CM (thresholds.h)
 *   Side margin 5 cm ............. OBS_SIDE_MARGIN_MM (thresholds.h)
 *   Bypass budget 3.0 s .......... OBS_BYPASS_BUDGET_MS
 *   T_hold 100 ms ................ MOTION_T_HOLD_MS
 *   a_move 0.05 g ................ IMU_A_MOVE_G_X1000 (thresholds.h)
 *   degree_hump 5 deg ............ IMU_HUMP_PITCH_DEG (thresholds.h)
 *   g_collision 2.5 g ............ IMU_G_COLLISION_X1000 (thresholds.h)
 *   T_still 500 ms ............... SAFETY_T_STILL_MS
 *   T_ka 5 s / backoff 1-30 s .... TEL_KEEPALIVE_MS, TEL_BACKOFF_MIN_MS, TEL_BACKOFF_MAX_MS
 */

#ifndef TIMING_H
#define TIMING_H

/* ---------------------------------------------------------------------------
 * Kernel
 * ------------------------------------------------------------------------- */

/** Kernel system tick (CNF_TIMER_PERIOD). */
#define KERNEL_TICK_MS                  (1U)

/* ---------------------------------------------------------------------------
 * Line following (Buddy 3)
 * ------------------------------------------------------------------------- */

/** IR sample period (NFR7). */
#define LF_SAMPLE_PERIOD_MS             (5U)
/** Steering update posted to the Vehicle Controller, aligned with the PID loop. */
#define LF_STEER_POST_PERIOD_MS         (20U)
/** Samples between steering posts. */
#define LF_STEER_POST_EVERY_N_SAMPLES   (LF_STEER_POST_PERIOD_MS / LF_SAMPLE_PERIOD_MS)
/** Consecutive agreeing samples before a junction or line-lost is believed. TUNE */
#define LF_HYST_SAMPLES                 (3U)
/** Hysteresis window in time (3 x 5 ms = 15 ms). */
#define LF_HYST_MS                      (LF_HYST_SAMPLES * LF_SAMPLE_PERIOD_MS)
/** Both sensors off the line this long -> LINE_LOST. TUNE */
#define LF_T_LOST_MS                    (100U)
/** Grace period for the line to reappear after a turn. TUNE */
#define LF_T_TURN_MS                    (500U)
/** Grace period for the line to reappear after a u-turn. TUNE */
#define LF_T_UTURN_MS                   (1000U)
/** Reacquisition search deadline before TRACK_LOST. TUNE */
#define LF_T_SEARCH_MS                  (2000U)

/* ---------------------------------------------------------------------------
 * Motion control (Buddy 2)
 * ------------------------------------------------------------------------- */

/** PID loop period (FR6). */
#define MOTION_PERIOD_MS                (20U)
/** Hold the last setpoint this long without a fresh command, then stop. TUNE */
#define MOTION_T_HOLD_MS                (100U)

/* ---------------------------------------------------------------------------
 * IMU and safety (Buddy 4)
 * ------------------------------------------------------------------------- */

/** IMU sample period. */
#define IMU_SAMPLE_PERIOD_MS            (10U)
/** Safety stops the car if IMU_ALIVE is not refreshed within this time (NFR13). */
#define SAFETY_IMU_WDOG_MS              (50U)
/** Encoder speed must be zero this long before a fault reset is accepted. TUNE */
#define SAFETY_T_STILL_MS               (500U)

/* ---------------------------------------------------------------------------
 * Telemetry (Buddy 1)
 * Design delta vs design.md 3.1/3.3 (200 ms periodic, 200 ms receive timeout):
 * the task wakes every 20 ms, polls MBF_TELEMETRY, publishes every 10th wake
 * and sends a heartbeat every 50th wake. Recorded in ARCHITECTURE.md (D2).
 * TODO(owner-to-confirm, buddy 1): delta D2 is proposed, not yet approved; the
 * FR1 publish/heartbeat rates hold either way.
 * ------------------------------------------------------------------------- */

/** Telemetry task wake period. */
#define TEL_WAKE_PERIOD_MS              (20U)
/** Wakes per telemetry publish (FR1, 200 ms). */
#define TEL_PUBLISH_EVERY_N_WAKES       (10U)
/** Wakes per heartbeat (FR1, 1 s). */
#define TEL_HEARTBEAT_EVERY_N_WAKES     (50U)
/** Derived publish period. */
#define TEL_PUBLISH_PERIOD_MS           (TEL_WAKE_PERIOD_MS * TEL_PUBLISH_EVERY_N_WAKES)
/** Derived heartbeat period. */
#define TEL_HEARTBEAT_PERIOD_MS         (TEL_WAKE_PERIOD_MS * TEL_HEARTBEAT_EVERY_N_WAKES)
/** MBF_TELEMETRY receive timeout: 0 = TMO_POL on each wake (design delta vs 200 ms). */
#define TEL_MBF_RECV_TMO_MS             (0U)
/** MQTT keep-alive (FR3). TUNE */
#define TEL_KEEPALIVE_MS                (5000U)
/** First reconnect backoff (FR3). TUNE */
#define TEL_BACKOFF_MIN_MS              (1000U)
/** Reconnect backoff cap (FR3). TUNE */
#define TEL_BACKOFF_MAX_MS              (30000U)

/* ---------------------------------------------------------------------------
 * Obstacle scanning and avoidance (Buddy 5)
 * ------------------------------------------------------------------------- */

/** One coarse servo step per period. */
#define OBS_SERVO_STEP_PERIOD_MS        (60U)
/** Coarse sweep bearings. */
#define OBS_COARSE_BEARING_COUNT        (5U)
/** Coarse sweep spacing. */
#define OBS_COARSE_STEP_DEG             (30U)
/** Fine sweep spacing. Not in design.md. TUNE */
#define OBS_FINE_STEP_DEG               (10U)
/** Fine sweep half-span around the detection bearing. Not in design.md. TUNE */
#define OBS_FINE_HALF_SPAN_DEG          (30U)
/** Servo settle time before a ping. TUNE
 *  TODO(owner-to-confirm, buddy 5): settle + 3 pings x 30 ms echo timeout can
 *  exceed the 60 ms step period. */
#define OBS_SERVO_SETTLE_MS             (50U)
/** SEM_ECHO wait; no echo = nothing in range (design.md 3.3). */
#define OBS_ECHO_TIMEOUT_MS             (30U)
/** Pings filtered per bearing (NFR15). TUNE */
#define OBS_PINGS_PER_BEARING           (3U)
/** Target time for a full bypass manoeuvre (NFR14). TUNE */
#define OBS_BYPASS_BUDGET_MS            (3000U)
/** Worst-case avoidance time: bypass budget + T_search (NFR14 hard limit 5.0 s). */
#define OBS_AVOID_WORST_CASE_MS         (OBS_BYPASS_BUDGET_MS + LF_T_SEARCH_MS)

/* ---------------------------------------------------------------------------
 * Barcode (Buddy 3)
 * ------------------------------------------------------------------------- */

/** Barcode task poll period while capturing. Not in design.md (ISR-woken there). TUNE */
#define BC_POLL_PERIOD_MS               (2U)
/** Abandon a capture after this long. TUNE
 *  TODO(owner-to-confirm, buddy 3): provisional value. Must fit a
 *  BC_GAP_BITS microsecond gap (checked below). */
#define BC_CAPTURE_TIMEOUT_MS           (1000U)
/** Edge-ring gap width in bits, microseconds (delta D8; docs/DATA_REPRESENTATION.md). */
#define BC_GAP_BITS                     (20U)
/** End-of-symbol gap = this multiple of the widest element time (T_end). TUNE */
#define BC_T_END_WIDEST_MULT            (3U)
/** ISR edge ring length (one Code 39 symbol is 29 elements). */
#define BC_EDGE_RING_LEN                (64U)

/* ---------------------------------------------------------------------------
 * Queues and debug
 * ------------------------------------------------------------------------- */

/** Send timeout for one-off commands into MBF_NAV_IN (design.md 3.3). */
#define NAV_MBF_SEND_TMO_MS             (5U)
/** USB builds only: wait this long for stdio_usb_connected() before the kernel starts.
 *  This is the only allowed SDK wait, and only before hal_kernel_start(). */
#define DBG_USB_CONNECT_WAIT_MS         (3000U)
/** Debug drain task period. TUNE
 *  TODO(owner-to-confirm, buddy 1): provisional value. */
#define DBG_DRAIN_PERIOD_MS             (10U)

/* ---------------------------------------------------------------------------
 * Compile-time checks
 * ------------------------------------------------------------------------- */

#if (LF_STEER_POST_PERIOD_MS % LF_SAMPLE_PERIOD_MS) != 0U
#error "timing.h: LF_STEER_POST_PERIOD_MS must be a multiple of LF_SAMPLE_PERIOD_MS"
#endif

#if TEL_PUBLISH_PERIOD_MS != 200U
#error "timing.h: telemetry publish period must be 200 ms (FR1)"
#endif

#if TEL_HEARTBEAT_PERIOD_MS != 1000U
#error "timing.h: telemetry heartbeat period must be 1000 ms (FR1)"
#endif

#if (BC_EDGE_RING_LEN & (BC_EDGE_RING_LEN - 1U)) != 0U
#error "timing.h: BC_EDGE_RING_LEN must be a power of two"
#endif

#if (BC_CAPTURE_TIMEOUT_MS * 1000UL) >= (1UL << BC_GAP_BITS)
#error "timing.h: BC_CAPTURE_TIMEOUT_MS no longer fits a 20-bit microsecond gap"
#endif

#if OBS_AVOID_WORST_CASE_MS > 5000U
#error "timing.h: bypass budget + T_search exceeds 5.0 s (NFR14)"
#endif

#if MOTION_T_HOLD_MS < MOTION_PERIOD_MS
#error "timing.h: MOTION_T_HOLD_MS must be >= MOTION_PERIOD_MS"
#endif

#if TEL_BACKOFF_MIN_MS > TEL_BACKOFF_MAX_MS
#error "timing.h: TEL_BACKOFF_MIN_MS > TEL_BACKOFF_MAX_MS"
#endif

#if (KERNEL_TICK_MS == 0U) || (LF_SAMPLE_PERIOD_MS == 0U) || (LF_STEER_POST_PERIOD_MS == 0U) \
    || (MOTION_PERIOD_MS == 0U) || (IMU_SAMPLE_PERIOD_MS == 0U) || (TEL_WAKE_PERIOD_MS == 0U) \
    || (OBS_SERVO_STEP_PERIOD_MS == 0U) || (BC_POLL_PERIOD_MS == 0U) || (DBG_DRAIN_PERIOD_MS == 0U)
#error "timing.h: a period is zero"
#endif

#endif /* TIMING_H */
