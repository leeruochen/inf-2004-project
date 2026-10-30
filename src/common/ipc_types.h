/*
 *  ipc_types.h
 *  Message and shared-data types carried by the IPC objects (design.md 3.3,
 *  TEAM.md section 5).  Owner: platform (Batch B, stage B2); each type's
 *  fields belong to its interface owner (TODO markers below).
 *  Pure header: <stdint.h> only, so drivers and host tests may include it.
 *
 *  PACKED minimum-bit Week 8 formats since stage B7 (ARCHITECTURE.md D8).
 *  Read and write fields only through src/common/data_repr.h.  Three field
 *  meanings still wait for their owner (TODO(owner-to-confirm) in
 *  data_repr.h): the steering sign (buddy 3), the bypass command values and
 *  the 2-bit obstacle sequence (buddy 5).
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
 * Message and shared-data types: PACKED (stage B7, ARCHITECTURE.md D8).
 * Each is one or two 32-bit words.  The field layout (bit positions, masks,
 * ranges, clamping) and every pack/unpack function are in
 * src/common/data_repr.h (prefix drep_): never set the words by hand.
 * Field table, writers and readers: docs/DATA_REPRESENTATION.md section 2.
 * The Week 6 unpacked structs were measured and tagged first
 * (baseline/week6-unpacked, docs/measurements/week6-baseline.txt).
 * ------------------------------------------------------------------------- */

/** IF-01 MBF_NAV_IN: source 2 + type 3 + payload 8 + bypass 3 + OOR 1 = 17 bits. */
typedef struct
{
    uint32_t w;
} nav_input_t;

/** IF-02 MBF_MOTION_CMD: type 3 + speed 6 + argument 9 + OOR 1 = 19 bits. */
typedef struct
{
    uint32_t w;
} motion_cmd_t;

/** IF-03 MBF_OBSTACLE: distance 9 + bearing 4 + width 7 + clear_L 7 + sequence 2
 *  + OOR 1 in lo (30 bits), clear_R 7 in hi: 37 bits, 5 B on the wire. */
typedef struct
{
    uint32_t lo;
    uint32_t hi;
} obstacle_profile_t;

/** IF-04 MBF_TELEMETRY: header = type 5 + timestamp 24 + flags 3; payload 32 by type. */
typedef struct
{
    uint32_t hdr;
    uint32_t payload;
} tel_evt_t;

/** IF-08 shared data under MTX_MOTION: speed 10 + distance 16 + OOR 1 in w; the
 *  encoder words stay unpacked int32 (IF-10, single aligned store, D7/D8). */
typedef struct
{
    uint32_t w;
    int32_t  enc_count[2];          /**< [0] left, [1] right; free-running */
    uint32_t tick_ms;               /**< kernel time of the sample */
} motion_state_t;

/* IF-09 barcode edge ring: drep_edge_ring_t in data_repr.h (20-bit gaps). */

/* ---------------------------------------------------------------------------
 * Event-flag bits.  FIXED in stage B7 (docs/DATA_REPRESENTATION.md section 3,
 * ARCHITECTURE.md D9): one setter per bit.
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

/** Safety's wait: tk_wai_flg(EVF_SAFETY, EVF_SAFETY_WAIT_MASK, TWF_ORW | TWF_BITCLR, ...). */
#define EVF_SAFETY_WAIT_MASK        (EVF_SAFETY_COLLISION | EVF_SAFETY_IMU_ALIVE)
/** Bits counted as faults (STATUS_NO_ECHO is informational, IMU_ALIVE a watchdog input). */
#define EVF_SAFETY_FAULT_MASK       (EVF_SAFETY_COLLISION | EVF_SAFETY_FAULT_LATCHED      \
                                     | EVF_SAFETY_FAULT_STALL | EVF_SAFETY_FAULT_ENCODER  \
                                     | EVF_SAFETY_FAULT_IMU_I2C | EVF_SAFETY_FAULT_IMU_WDOG \
                                     | EVF_SAFETY_FAULT_WIFI_LOSS | EVF_SAFETY_FAULT_IR_SENSOR)
/** Every defined EVF_SAFETY bit (b0-b9, 10 bits: the event_fault payload). */
#define EVF_SAFETY_ALL_MASK         (0x3FFUL)

/* IF-07 EVF_NAV.  Readers poll; nobody blocks on it. */
#define EVF_NAV_AVOID_ACTIVE        (1UL << 0)
#define EVF_NAV_LINE_FOUND          (1UL << 1)
#define EVF_NAV_LINE_REACQUIRED     (1UL << 2)

/* ---------------------------------------------------------------------------
 * Compile-time checks: packed sizes (docs/DATA_REPRESENTATION.md "Packed
 * bytes"; the baseline sizes are in docs/measurements/week6-baseline.txt) and
 * the event-flag bits.
 * ------------------------------------------------------------------------- */
_Static_assert(sizeof(nav_input_t) == 4U, "nav_input_t is one word");
_Static_assert(sizeof(motion_cmd_t) == 4U, "motion_cmd_t is one word");
_Static_assert(sizeof(obstacle_profile_t) == 8U, "obstacle_profile_t is two words");
_Static_assert(sizeof(tel_evt_t) == 8U, "tel_evt_t is header + payload");
_Static_assert(sizeof(motion_state_t) == 16U, "motion_state_t is 16 B");

#define IPC_TYPES_BIT_DISTINCT(a, b)    (((a) & (b)) == 0UL)
_Static_assert(IPC_TYPES_BIT_DISTINCT(EVF_SAFETY_COLLISION | EVF_SAFETY_IMU_ALIVE | EVF_SAFETY_FAULT_LATCHED,
                                      EVF_SAFETY_FAULT_STALL | EVF_SAFETY_FAULT_ENCODER | EVF_SAFETY_FAULT_IMU_I2C
                                      | EVF_SAFETY_FAULT_IMU_WDOG | EVF_SAFETY_STATUS_NO_ECHO
                                      | EVF_SAFETY_FAULT_WIFI_LOSS | EVF_SAFETY_FAULT_IR_SENSOR),
               "EVF_SAFETY bits overlap");
_Static_assert((EVF_SAFETY_COLLISION | EVF_SAFETY_IMU_ALIVE | EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_FAULT_STALL
                | EVF_SAFETY_FAULT_ENCODER | EVF_SAFETY_FAULT_IMU_I2C | EVF_SAFETY_FAULT_IMU_WDOG
                | EVF_SAFETY_STATUS_NO_ECHO | EVF_SAFETY_FAULT_WIFI_LOSS | EVF_SAFETY_FAULT_IR_SENSOR)
               == EVF_SAFETY_ALL_MASK, "EVF_SAFETY: ten distinct bits b0-b9");
_Static_assert(EVF_SAFETY_WAIT_MASK == 0x3UL, "Safety waits on COLLISION | IMU_ALIVE only");
_Static_assert((EVF_SAFETY_FAULT_MASK & (EVF_SAFETY_STATUS_NO_ECHO | EVF_SAFETY_IMU_ALIVE)) == 0UL,
               "NO_ECHO and IMU_ALIVE are not faults");
_Static_assert((EVF_NAV_AVOID_ACTIVE | EVF_NAV_LINE_FOUND | EVF_NAV_LINE_REACQUIRED) == 0x7UL,
               "EVF_NAV: three distinct bits");

#endif /* IPC_TYPES_H */
