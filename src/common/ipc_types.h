/*
 *  ipc_types.h
 *  Message and shared-data types carried by the IPC objects (design.md 3.3,
 *  TEAM.md section 5).  Owner: platform (Batch B, stage B2); each type's
 *  fields belong to its interface owner (TODO markers below).
 *  Pure header: <stdint.h> only, so drivers and host tests may include it.
 *
 *  PROVISIONAL until milestone M2 (Batch B stage B7).  These are the Week 6
 *  UNPACKED baseline types (batchB-plan O2; field lists from
 *  .agents/tasks/week8-impact-report.md C.1, sizes as the "Baseline bytes"
 *  column of docs/DATA_REPRESENTATION.md).  B7 replaces them with the
 *  minimum-bit Week 8 formats (ARCHITECTURE.md D8).  Code that reads or
 *  writes message fields may need edits between M1-lite and M2.
 */

#ifndef IPC_TYPES_H
#define IPC_TYPES_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Message-buffer depths (design.md 3.3)
 * ------------------------------------------------------------------------- */

#define MBF_NAV_IN_DEPTH        (8U)    /**< IF-01, nav_input_t */
#define MBF_MOTION_CMD_DEPTH    (4U)    /**< IF-02, motion_cmd_t */
#define MBF_OBSTACLE_DEPTH      (2U)    /**< IF-03, obstacle_profile_t */
#define MBF_TELEMETRY_DEPTH     (32U)   /**< IF-04, tel_evt_t */

/* ---------------------------------------------------------------------------
 * IF-01 MBF_NAV_IN: steering, barcode, bypass and remote commands to the
 * Vehicle Controller.  TODO(owner-to-confirm, buddy 2): field codes.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t source;                 /**< LINE, BARCODE, AVOID, REMOTE */
    uint8_t type;                   /**< meaning depends on source */
    union
    {
        int16_t steering_err_q8;    /**< Line-Following steering error, Q8 */
        uint8_t nav_cmd;            /**< barcode command */
        uint8_t bypass_cmd;         /**< obstacle bypass command */
        uint8_t remote_cmd;         /**< remote START / STOP / RESET */
    } payload;
} nav_input_t;

/* ---------------------------------------------------------------------------
 * IF-02 MBF_MOTION_CMD: the one command Motion Control executes.
 * TODO(owner-to-confirm, buddy 2): field codes.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  type;                  /**< STOP, FWD, BACK, TURN_L, TURN_R, UTURN, STEER, BRAKE */
    uint16_t distance_cm;
    int16_t  angle_deg;
    uint8_t  speed_cmps;
    int16_t  steer_q8;
} motion_cmd_t;

/* ---------------------------------------------------------------------------
 * IF-03 MBF_OBSTACLE: one obstacle profile, Scanning -> Avoidance.
 * TODO(owner-to-confirm, buddy 5): field codes.
 * ------------------------------------------------------------------------- */
typedef struct
{
    int16_t  bearing_deg;
    uint16_t width_mm;
    uint16_t closest_mm;
    uint16_t clear_left_mm;
    uint16_t clear_right_mm;
    uint8_t  valid;
    uint32_t timestamp_ms;
} obstacle_profile_t;

/* ---------------------------------------------------------------------------
 * IF-04 MBF_TELEMETRY: one status or sensor event for the dashboard.
 * TODO(owner-to-confirm, buddy 1): event types and payload layout.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  type;
    uint32_t timestamp_ms;
    uint32_t payload[2];            /**< per-type payload, 8 bytes */
} tel_evt_t;

/* ---------------------------------------------------------------------------
 * IF-08 motion_state_t: shared data under MTX_MOTION (not a queue).
 * Motion Control writes; Barcode Decode and Telemetry copy and release.
 * TODO(owner-to-confirm, buddy 2): field codes.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint16_t speed_cmps_x10;
    uint32_t distance_cm_x10;
    int32_t  enc_count[2];          /**< IF-10 encoder words, unpacked (D7) */
    uint32_t tick_ms;
} motion_state_t;

/* ---------------------------------------------------------------------------
 * IF-09 barcode edge ring element (the ring itself is owned by subsystem 3;
 * BC_EDGE_RING_LEN entries, config/timing.h).
 * TODO(owner-to-confirm, buddy 3).
 * ------------------------------------------------------------------------- */
typedef uint64_t bc_edge_t;         /**< edge time, us */

/* ---------------------------------------------------------------------------
 * Event-flag bits.  PROVISIONAL bit positions (docs/DATA_REPRESENTATION.md
 * section 3, ARCHITECTURE.md D9); fixed in stage B7.
 * ------------------------------------------------------------------------- */

/* IF-06 EVF_SAFETY.  Safety waits on COLLISION | IMU_ALIVE with
 * TWF_ORW | TWF_BITCLR (never TWF_CLR).  Other readers use tk_ref_flg. */
#define EVF_SAFETY_COLLISION        (1UL << 0)
#define EVF_SAFETY_IMU_ALIVE        (1UL << 1)
#define EVF_SAFETY_FAULT_LATCHED    (1UL << 2)
#define EVF_SAFETY_FAULT_STALL      (1UL << 3)
#define EVF_SAFETY_FAULT_ENCODER    (1UL << 4)
#define EVF_SAFETY_FAULT_IMU_I2C    (1UL << 5)
#define EVF_SAFETY_FAULT_IMU_WDOG   (1UL << 6)
#define EVF_SAFETY_STATUS_NO_ECHO   (1UL << 7)
#define EVF_SAFETY_FAULT_WIFI_LOSS  (1UL << 8)
#define EVF_SAFETY_FAULT_IR_SENSOR  (1UL << 9)

/* IF-07 EVF_NAV.  Readers poll; nobody blocks on it. */
#define EVF_NAV_AVOID_ACTIVE        (1UL << 0)
#define EVF_NAV_LINE_FOUND          (1UL << 1)
#define EVF_NAV_LINE_REACQUIRED     (1UL << 2)

/* ---------------------------------------------------------------------------
 * Baseline sizes (natural ARM EABI alignment).  B6 measures them on target;
 * a change here must update docs/DATA_REPRESENTATION.md.
 * ------------------------------------------------------------------------- */
_Static_assert(sizeof(nav_input_t) == 4U, "nav_input_t baseline is 4 B");
_Static_assert(sizeof(motion_cmd_t) == 10U, "motion_cmd_t baseline is 10 B");
_Static_assert(sizeof(obstacle_profile_t) == 16U, "obstacle_profile_t baseline is 16 B");
_Static_assert(sizeof(tel_evt_t) == 16U, "tel_evt_t baseline is 16 B");
_Static_assert(sizeof(motion_state_t) == 20U, "motion_state_t baseline is 20 B");
_Static_assert(sizeof(bc_edge_t) == 8U, "edge ring entries are 8 B");

#endif /* IPC_TYPES_H */
