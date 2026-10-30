/*
 *  data_repr.h
 *  Minimum-bit data representation (Batch B stage B7, ARCHITECTURE.md D8,
 *  D9, D10): bit layouts, ranges, pack/unpack of every inter-task message,
 *  the barcode edge ring, state-machine widths and the MQTT binary frames.
 *  Owner: platform; each field's meaning belongs to its owner buddy
 *  (docs/DATA_REPRESENTATION.md, column "Owner buddy").
 *  Pure header: <stdbool.h>, <stddef.h> and <stdint.h> only, so SDK-side drivers,
 *  kernel-side tasks and host tests all include it.  Prefix drep_.
 *
 *  Rules (docs/DATA_REPRESENTATION.md section 1):
 *    - masks and shifts only, never C bit-fields;
 *    - a pack function CLAMPS an out-of-range value and sets the message's
 *      OOR bit (or saturates, where the table says so);
 *    - signed fields are two's complement and are sign-extended on unpack;
 *    - every multi-byte wire field is little-endian.
 *
 *  Field meanings still waiting for their owner (placeholders, as proposed
 *  in docs/DATA_REPRESENTATION.md):
 *    - steering sign: steer > 0 = line to the car's RIGHT = steer right.
 *      TODO(owner-to-confirm, buddy 3)
 *    - bypass command values: stop, continue, left, right, reverse.
 *      TODO(owner-to-confirm, buddy 5)
 *    - the 2-bit obstacle sequence number (kept).  TODO(owner-to-confirm, buddy 5)
 *    - telemetry event types.  TODO(owner-to-confirm, buddy 1)
 */

#ifndef DATA_REPR_H
#define DATA_REPR_H

#include <stdbool.h>
#include <stddef.h>                 /* NULL */
#include <stdint.h>

#include "config/timing.h"          /* BC_EDGE_RING_LEN, BC_GAP_BITS (pure) */
#include "src/common/ipc_types.h"   /* the packed types (pure) */

/* ===========================================================================
 * Helpers
 * ========================================================================= */

/** Mask of the low n bits (n < 32). */
#define DREP_MASK(bits)             ((1UL << (bits)) - 1UL)

static inline uint32_t drep_get(uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & DREP_MASK(bits);
}

static inline uint32_t drep_set(uint32_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t const mask = DREP_MASK(bits) << shift;

    return (word & ~mask) | ((value << shift) & mask);
}

/** Sign-extend the low `bits` bits of `raw` (two's complement). */
static inline int32_t drep_sext(uint32_t raw, uint32_t bits)
{
    uint32_t const sign = 1UL << (bits - 1U);

    raw &= DREP_MASK(bits);
    return (int32_t)(raw ^ sign) - (int32_t)sign;
}

/** Clamp v to [lo, hi]; sets *p_oor (if not NULL) when it had to clamp. */
static inline int32_t drep_clamp(int32_t v, int32_t lo, int32_t hi, bool *p_oor)
{
    if (v < lo)
    {
        v = lo;
    }
    else if (v > hi)
    {
        v = hi;
    }
    else
    {
        return v;
    }
    if (p_oor != NULL)
    {
        *p_oor = true;
    }
    return v;
}

/* ===========================================================================
 * IF-01 nav_input_t (MBF_NAV_IN): 17 bits in one word
 *   [1:0] source  [4:2] type  [12:5] payload  [15:13] bypass  [16] OOR
 * ========================================================================= */

#define DREP_NAV_SRC_SHIFT          (0U)
#define DREP_NAV_SRC_BITS           (2U)
#define DREP_NAV_TYPE_SHIFT         (2U)
#define DREP_NAV_TYPE_BITS          (3U)
#define DREP_NAV_PAYLOAD_SHIFT      (5U)
#define DREP_NAV_PAYLOAD_BITS       (8U)
#define DREP_NAV_BYPASS_SHIFT       (13U)
#define DREP_NAV_BYPASS_BITS        (3U)
#define DREP_NAV_OOR_SHIFT          (16U)
#define DREP_NAV_TOTAL_BITS         (17U)

/* Sources (who sent it). */
#define DREP_NAV_SRC_LINE           (0U)    /**< Line-Following (buddy 3) */
#define DREP_NAV_SRC_BARCODE        (1U)    /**< Barcode Decode (buddy 3) */
#define DREP_NAV_SRC_AVOID          (2U)    /**< Obstacle Avoidance (buddy 5) */
#define DREP_NAV_SRC_REMOTE         (3U)    /**< Telemetry, remote command (buddy 1) */

/* Types, per source. */
#define DREP_NAV_LINE_STEER         (0U)    /**< payload = steer, Q7 */
#define DREP_NAV_LINE_JUNCTION      (1U)
#define DREP_NAV_LINE_TRACK_LOST    (2U)
#define DREP_NAV_LINE_TYPES         (3U)
#define DREP_NAV_BARCODE_CMD        (0U)    /**< payload = DREP_BC_CMD_* */
#define DREP_NAV_BARCODE_TYPES      (1U)
#define DREP_NAV_AVOID_SLOW         (0U)
#define DREP_NAV_AVOID_BYPASS       (1U)    /**< bypass field = DREP_BYPASS_* */
#define DREP_NAV_AVOID_RELEASE      (2U)
#define DREP_NAV_AVOID_BLOCKED      (3U)
#define DREP_NAV_AVOID_TYPES        (4U)
#define DREP_NAV_REMOTE_START       (0U)    /**< the type IS the command; payload 0 */
#define DREP_NAV_REMOTE_STOP        (1U)
#define DREP_NAV_REMOTE_RESET       (2U)
#define DREP_NAV_REMOTE_TYPES       (3U)

/* Steering, payload of LINE/STEER: Q7, -128..+127 = -1.0..+0.992 of full
 * scale.  Sign: TODO(owner-to-confirm, buddy 3), proposal: > 0 = line to the
 * car's RIGHT = steer right (sensors are CENTRE + RIGHT, ARCHITECTURE.md D11). */
#define DREP_STEER_Q7_MIN           (-128)
#define DREP_STEER_Q7_MAX           (127)

/* Barcode commands, payload of BARCODE/CMD (Code 39 A-D). */
#define DREP_BC_CMD_LEFT            (0U)    /**< A */
#define DREP_BC_CMD_RIGHT           (1U)    /**< B */
#define DREP_BC_CMD_STRAIGHT        (2U)    /**< C */
#define DREP_BC_CMD_UTURN           (3U)    /**< D */
#define DREP_BC_CMD_BITS            (2U)

/* Bypass commands, with AVOID/BYPASS.  TODO(owner-to-confirm, buddy 5). */
#define DREP_BYPASS_STOP            (0U)
#define DREP_BYPASS_CONTINUE        (1U)
#define DREP_BYPASS_LEFT            (2U)
#define DREP_BYPASS_RIGHT           (3U)
#define DREP_BYPASS_REVERSE         (4U)
#define DREP_BYPASS_COUNT           (5U)    /**< 5-7 are rejected */

static inline nav_input_t drep_nav_make(uint32_t source, uint32_t type, uint32_t payload,
                                        uint32_t bypass, bool oor)
{
    nav_input_t m = { 0U };

    m.w = drep_set(m.w, DREP_NAV_SRC_SHIFT, DREP_NAV_SRC_BITS, source);
    m.w = drep_set(m.w, DREP_NAV_TYPE_SHIFT, DREP_NAV_TYPE_BITS, type);
    m.w = drep_set(m.w, DREP_NAV_PAYLOAD_SHIFT, DREP_NAV_PAYLOAD_BITS, payload);
    m.w = drep_set(m.w, DREP_NAV_BYPASS_SHIFT, DREP_NAV_BYPASS_BITS, bypass);
    m.w = drep_set(m.w, DREP_NAV_OOR_SHIFT, 1U, oor ? 1U : 0U);
    return m;
}

/** Line-Following steering, Q7; clamped to -128..+127 with OOR. */
static inline nav_input_t drep_nav_steer(int32_t steer_q7)
{
    bool          oor = false;
    int32_t const v = drep_clamp(steer_q7, DREP_STEER_Q7_MIN, DREP_STEER_Q7_MAX, &oor);

    return drep_nav_make(DREP_NAV_SRC_LINE, DREP_NAV_LINE_STEER, (uint32_t)v, 0U, oor);
}

/** Line-Following event without payload (JUNCTION, TRACK_LOST). */
static inline nav_input_t drep_nav_line_event(uint32_t type)
{
    return drep_nav_make(DREP_NAV_SRC_LINE, type, 0U, 0U, false);
}

/** Barcode Decode command (DREP_BC_CMD_*). */
static inline nav_input_t drep_nav_barcode(uint32_t cmd)
{
    return drep_nav_make(DREP_NAV_SRC_BARCODE, DREP_NAV_BARCODE_CMD, cmd & DREP_MASK(DREP_BC_CMD_BITS),
                         0U, false);
}

/** Obstacle Avoidance message; bypass is used with DREP_NAV_AVOID_BYPASS only. */
static inline nav_input_t drep_nav_avoid(uint32_t type, uint32_t bypass)
{
    return drep_nav_make(DREP_NAV_SRC_AVOID, type,
                         0U, (type == DREP_NAV_AVOID_BYPASS) ? bypass : 0U, false);
}

/** Remote command (DREP_NAV_REMOTE_*). */
static inline nav_input_t drep_nav_remote(uint32_t type)
{
    return drep_nav_make(DREP_NAV_SRC_REMOTE, type, 0U, 0U, false);
}

static inline uint32_t drep_nav_source(nav_input_t m)
{
    return drep_get(m.w, DREP_NAV_SRC_SHIFT, DREP_NAV_SRC_BITS);
}

static inline uint32_t drep_nav_type(nav_input_t m)
{
    return drep_get(m.w, DREP_NAV_TYPE_SHIFT, DREP_NAV_TYPE_BITS);
}

/** Steering, sign-extended (LINE/STEER only). */
static inline int32_t drep_nav_steer_q7(nav_input_t m)
{
    return drep_sext(drep_get(m.w, DREP_NAV_PAYLOAD_SHIFT, DREP_NAV_PAYLOAD_BITS), DREP_NAV_PAYLOAD_BITS);
}

/** Barcode command (BARCODE/CMD only). */
static inline uint32_t drep_nav_barcode_cmd(nav_input_t m)
{
    return drep_get(m.w, DREP_NAV_PAYLOAD_SHIFT, DREP_BC_CMD_BITS);
}

static inline uint32_t drep_nav_bypass(nav_input_t m)
{
    return drep_get(m.w, DREP_NAV_BYPASS_SHIFT, DREP_NAV_BYPASS_BITS);
}

static inline bool drep_nav_oor(nav_input_t m)
{
    return drep_get(m.w, DREP_NAV_OOR_SHIFT, 1U) != 0U;
}

/** The Vehicle Controller's reject rule: unknown type for the source, a
 *  bypass value 5-7, or bits above bit 16. */
static inline bool drep_nav_valid(nav_input_t m)
{
    static uint8_t const k_types[4] =
    {
        DREP_NAV_LINE_TYPES, DREP_NAV_BARCODE_TYPES, DREP_NAV_AVOID_TYPES, DREP_NAV_REMOTE_TYPES
    };

    if ((m.w >> DREP_NAV_TOTAL_BITS) != 0U)
    {
        return false;
    }
    if (drep_nav_type(m) >= k_types[drep_nav_source(m)])
    {
        return false;
    }
    return drep_nav_bypass(m) < DREP_BYPASS_COUNT;
}

/* ===========================================================================
 * IF-02 motion_cmd_t (MBF_MOTION_CMD): 19 bits in one word
 *   [2:0] type  [8:3] speed cm/s  [17:9] argument  [18] OOR
 * ========================================================================= */

#define DREP_MC_TYPE_SHIFT          (0U)
#define DREP_MC_TYPE_BITS           (3U)
#define DREP_MC_SPEED_SHIFT         (3U)
#define DREP_MC_SPEED_BITS          (6U)
#define DREP_MC_ARG_SHIFT           (9U)
#define DREP_MC_ARG_BITS            (9U)
#define DREP_MC_OOR_SHIFT           (18U)
#define DREP_MC_TOTAL_BITS          (19U)

#define DREP_MC_STOP                (0U)    /**< arg 0 */
#define DREP_MC_FWD                 (1U)    /**< arg = distance cm, 0..511, 0 = continuous */
#define DREP_MC_BACK                (2U)    /**< arg = distance cm, 0..511, 0 = continuous */
#define DREP_MC_TURN_L              (3U)    /**< arg = angle deg, 0..180 */
#define DREP_MC_TURN_R              (4U)    /**< arg = angle deg, 0..180 */
#define DREP_MC_UTURN               (5U)    /**< arg 0 */
#define DREP_MC_STEER               (6U)    /**< arg = steer, -256..+255 (signed) */
#define DREP_MC_BRAKE               (7U)    /**< arg 0 */

#define DREP_MC_SPEED_MAX           (50)    /**< cm/s, NFR2 cap */
#define DREP_MC_DIST_MAX            (511)
#define DREP_MC_ANGLE_MAX           (180)
#define DREP_MC_STEER_MIN           (-256)
#define DREP_MC_STEER_MAX           (255)

/** Pack a motion command; speed and arg are clamped per type, with OOR. */
static inline motion_cmd_t drep_mc_pack(uint32_t type, int32_t speed_cmps, int32_t arg)
{
    motion_cmd_t m = { 0U };
    bool         oor = false;
    int32_t      speed = drep_clamp(speed_cmps, 0, DREP_MC_SPEED_MAX, &oor);

    type &= DREP_MASK(DREP_MC_TYPE_BITS);
    switch (type)
    {
        case DREP_MC_FWD:
        case DREP_MC_BACK:
            arg = drep_clamp(arg, 0, DREP_MC_DIST_MAX, &oor);
            break;
        case DREP_MC_TURN_L:
        case DREP_MC_TURN_R:
            arg = drep_clamp(arg, 0, DREP_MC_ANGLE_MAX, &oor);
            break;
        case DREP_MC_STEER:
            arg = drep_clamp(arg, DREP_MC_STEER_MIN, DREP_MC_STEER_MAX, &oor);
            break;
        default:    /* STOP, UTURN, BRAKE: no argument */
            arg = drep_clamp(arg, 0, 0, &oor);
            break;
    }
    m.w = drep_set(m.w, DREP_MC_TYPE_SHIFT, DREP_MC_TYPE_BITS, type);
    m.w = drep_set(m.w, DREP_MC_SPEED_SHIFT, DREP_MC_SPEED_BITS, (uint32_t)speed);
    m.w = drep_set(m.w, DREP_MC_ARG_SHIFT, DREP_MC_ARG_BITS, (uint32_t)arg);
    m.w = drep_set(m.w, DREP_MC_OOR_SHIFT, 1U, oor ? 1U : 0U);
    return m;
}

static inline uint32_t drep_mc_type(motion_cmd_t m)
{
    return drep_get(m.w, DREP_MC_TYPE_SHIFT, DREP_MC_TYPE_BITS);
}

static inline uint32_t drep_mc_speed(motion_cmd_t m)
{
    return drep_get(m.w, DREP_MC_SPEED_SHIFT, DREP_MC_SPEED_BITS);
}

/** The argument, sign-extended for STEER, unsigned for every other type. */
static inline int32_t drep_mc_arg(motion_cmd_t m)
{
    uint32_t const raw = drep_get(m.w, DREP_MC_ARG_SHIFT, DREP_MC_ARG_BITS);

    return (drep_mc_type(m) == DREP_MC_STEER) ? drep_sext(raw, DREP_MC_ARG_BITS) : (int32_t)raw;
}

static inline bool drep_mc_oor(motion_cmd_t m)
{
    return drep_get(m.w, DREP_MC_OOR_SHIFT, 1U) != 0U;
}

/** Motion's reject rule: per-type argument range, speed cap, no stray bits. */
static inline bool drep_mc_valid(motion_cmd_t m)
{
    int32_t const arg = drep_mc_arg(m);

    if (((m.w >> DREP_MC_TOTAL_BITS) != 0U) || (drep_mc_speed(m) > (uint32_t)DREP_MC_SPEED_MAX))
    {
        return false;
    }
    switch (drep_mc_type(m))
    {
        case DREP_MC_FWD:
        case DREP_MC_BACK:
            return true;                    /* 0..511 is every 9-bit value */
        case DREP_MC_TURN_L:
        case DREP_MC_TURN_R:
            return arg <= DREP_MC_ANGLE_MAX;
        case DREP_MC_STEER:
            return true;                    /* -256..+255 is every 9-bit value */
        default:
            return arg == 0;
    }
}

/* ===========================================================================
 * IF-03 obstacle_profile_t (MBF_OBSTACLE): 37 bits in two words
 *   lo: [8:0] distance cm  [12:9] bearing index  [19:13] width (5 mm)
 *       [26:20] clear_L (5 mm)  [28:27] sequence  [29] OOR
 *   hi: [6:0] clear_R (5 mm)
 * Wire: lo little-endian in bytes 0-3, hi in byte 4 (5 B).
 * ========================================================================= */

#define DREP_OBS_DIST_SHIFT         (0U)
#define DREP_OBS_DIST_BITS          (9U)
#define DREP_OBS_BEAR_SHIFT         (9U)
#define DREP_OBS_BEAR_BITS          (4U)
#define DREP_OBS_WIDTH_SHIFT        (13U)
#define DREP_OBS_WIDTH_BITS         (7U)
#define DREP_OBS_CLRL_SHIFT         (20U)
#define DREP_OBS_CLR_BITS           (7U)
#define DREP_OBS_SEQ_SHIFT          (27U)
#define DREP_OBS_SEQ_BITS           (2U)    /**< TODO(owner-to-confirm, buddy 5): kept */
#define DREP_OBS_OOR_SHIFT          (29U)
#define DREP_OBS_LO_BITS            (30U)
#define DREP_OBS_CLRR_SHIFT         (0U)    /**< in hi */
#define DREP_OBS_TOTAL_BITS         (37U)
#define DREP_OBS_WIRE_BYTES         (5U)

#define DREP_OBS_DIST_MAX_CM        (400)   /**< 0 = no echo (replaces "valid") */
#define DREP_OBS_BEAR_MAX           (12)    /**< bearing = 30 deg + 10 deg x index */
#define DREP_OBS_MM_PER_UNIT        (5)
#define DREP_OBS_WIDTH_MAX_MM       (400)
#define DREP_OBS_CLR_MAX_MM         (635)   /**< 127 units = "at least 635 mm, clear" */

/** Pack an obstacle profile.  distance, bearing and width clamp with OOR;
 *  the clearances SATURATE at 635 mm (no OOR); the sequence wraps (0-3).
 *  Rounding to 5 mm is conservative: the width rounds UP and the clearances
 *  round DOWN, so a profile never shows a smaller obstacle or a wider gap
 *  than measured. */
static inline obstacle_profile_t drep_obs_pack(int32_t distance_cm, int32_t bearing_idx, int32_t width_mm,
                                               int32_t clear_l_mm, int32_t clear_r_mm, uint32_t seq)
{
    obstacle_profile_t p = { 0U, 0U };
    bool               oor = false;
    int32_t const      dist = drep_clamp(distance_cm, 0, DREP_OBS_DIST_MAX_CM, &oor);
    int32_t const      bear = drep_clamp(bearing_idx, 0, DREP_OBS_BEAR_MAX, &oor);
    int32_t const      width = drep_clamp(width_mm, 0, DREP_OBS_WIDTH_MAX_MM, &oor);
    int32_t const      clr_l = drep_clamp(clear_l_mm, 0, DREP_OBS_CLR_MAX_MM, NULL);
    int32_t const      clr_r = drep_clamp(clear_r_mm, 0, DREP_OBS_CLR_MAX_MM, NULL);

    p.lo = drep_set(p.lo, DREP_OBS_DIST_SHIFT, DREP_OBS_DIST_BITS, (uint32_t)dist);
    p.lo = drep_set(p.lo, DREP_OBS_BEAR_SHIFT, DREP_OBS_BEAR_BITS, (uint32_t)bear);
    p.lo = drep_set(p.lo, DREP_OBS_WIDTH_SHIFT, DREP_OBS_WIDTH_BITS,
                    (uint32_t)((width + (DREP_OBS_MM_PER_UNIT - 1)) / DREP_OBS_MM_PER_UNIT));
    p.lo = drep_set(p.lo, DREP_OBS_CLRL_SHIFT, DREP_OBS_CLR_BITS,
                    (uint32_t)(clr_l / DREP_OBS_MM_PER_UNIT));
    p.lo = drep_set(p.lo, DREP_OBS_SEQ_SHIFT, DREP_OBS_SEQ_BITS, seq);
    p.lo = drep_set(p.lo, DREP_OBS_OOR_SHIFT, 1U, oor ? 1U : 0U);
    p.hi = drep_set(0U, DREP_OBS_CLRR_SHIFT, DREP_OBS_CLR_BITS,
                    (uint32_t)(clr_r / DREP_OBS_MM_PER_UNIT));
    return p;
}

static inline uint32_t drep_obs_distance_cm(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_DIST_SHIFT, DREP_OBS_DIST_BITS);
}

static inline bool drep_obs_no_echo(obstacle_profile_t p)
{
    return drep_obs_distance_cm(p) == 0U;
}

static inline uint32_t drep_obs_bearing_idx(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_BEAR_SHIFT, DREP_OBS_BEAR_BITS);
}

/** Bearing in degrees: 30 + 10 x index (30..150). */
static inline uint32_t drep_obs_bearing_deg(obstacle_profile_t p)
{
    return 30U + (10U * drep_obs_bearing_idx(p));
}

static inline uint32_t drep_obs_width_mm(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_WIDTH_SHIFT, DREP_OBS_WIDTH_BITS) * (uint32_t)DREP_OBS_MM_PER_UNIT;
}

static inline uint32_t drep_obs_clear_l_mm(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_CLRL_SHIFT, DREP_OBS_CLR_BITS) * (uint32_t)DREP_OBS_MM_PER_UNIT;
}

static inline uint32_t drep_obs_clear_r_mm(obstacle_profile_t p)
{
    return drep_get(p.hi, DREP_OBS_CLRR_SHIFT, DREP_OBS_CLR_BITS) * (uint32_t)DREP_OBS_MM_PER_UNIT;
}

static inline uint32_t drep_obs_seq(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_SEQ_SHIFT, DREP_OBS_SEQ_BITS);
}

static inline bool drep_obs_oor(obstacle_profile_t p)
{
    return drep_get(p.lo, DREP_OBS_OOR_SHIFT, 1U) != 0U;
}

/* ===========================================================================
 * IF-08 motion_state_t word (MTX_MOTION): 27 bits used of one word
 *   [9:0] speed, 0.1 cm/s, signed  [25:10] distance cm (wraps)  [26] OOR
 * The encoder words and the tick stay unpacked (ipc_types.h).
 * ========================================================================= */

#define DREP_MS_SPEED_SHIFT         (0U)
#define DREP_MS_SPEED_BITS          (10U)
#define DREP_MS_DIST_SHIFT          (10U)
#define DREP_MS_DIST_BITS           (16U)
#define DREP_MS_OOR_SHIFT           (26U)
#define DREP_MS_SPEED_X10_MAX       (511)   /**< +-51.1 cm/s */

/** Speed in 0.1 cm/s (clamped to +-51.1 with OOR); distance in cm (wraps at 65536). */
static inline uint32_t drep_ms_pack(int32_t speed_cmps_x10, uint32_t distance_cm)
{
    bool          oor = false;
    int32_t const v = drep_clamp(speed_cmps_x10, -DREP_MS_SPEED_X10_MAX, DREP_MS_SPEED_X10_MAX, &oor);
    uint32_t      w = 0U;

    w = drep_set(w, DREP_MS_SPEED_SHIFT, DREP_MS_SPEED_BITS, (uint32_t)v);
    w = drep_set(w, DREP_MS_DIST_SHIFT, DREP_MS_DIST_BITS, distance_cm);
    w = drep_set(w, DREP_MS_OOR_SHIFT, 1U, oor ? 1U : 0U);
    return w;
}

static inline int32_t drep_ms_speed_x10(uint32_t w)
{
    return drep_sext(drep_get(w, DREP_MS_SPEED_SHIFT, DREP_MS_SPEED_BITS), DREP_MS_SPEED_BITS);
}

static inline uint32_t drep_ms_distance_cm(uint32_t w)
{
    return drep_get(w, DREP_MS_DIST_SHIFT, DREP_MS_DIST_BITS);
}

static inline bool drep_ms_oor(uint32_t w)
{
    return drep_get(w, DREP_MS_OOR_SHIFT, 1U) != 0U;
}

/* ===========================================================================
 * Small fields carried in telemetry payloads (section 2.6)
 * ========================================================================= */

#define DREP_PITCH_BITS             (8U)    /**< int8, Q1 (0.5 deg) */
#define DREP_PITCH_Q1_MAX           (90)    /**< +-45.0 deg */
#define DREP_HUMP_BITS              (7U)
#define DREP_HUMP_MAX_MM            (100)

/** Pitch (or roll) in 0.5-degree steps, clamped to +-45 deg; sets *p_oor. */
static inline int32_t drep_pitch_q1(int32_t pitch_half_deg, bool *p_oor)
{
    return drep_clamp(pitch_half_deg, -DREP_PITCH_Q1_MAX, DREP_PITCH_Q1_MAX, p_oor);
}

/* ===========================================================================
 * IF-04 tel_evt_t (MBF_TELEMETRY): header word + one payload word
 *   hdr: [4:0] type  [28:5] timestamp ms (24 bits, wraps after 4.66 h)
 *        [31:29] flags: [29] OOR, [31:30] spare
 * Event types: TODO(owner-to-confirm, buddy 1).
 * ========================================================================= */

#define DREP_TEL_TYPE_SHIFT         (0U)
#define DREP_TEL_TYPE_BITS          (5U)
#define DREP_TEL_TS_SHIFT           (5U)
#define DREP_TEL_TS_BITS            (24U)
#define DREP_TEL_OOR_SHIFT          (29U)

#define DREP_TEL_HUMP               (1U)    /**< payload: drep_tel_hump() */
#define DREP_TEL_BARCODE            (2U)    /**< payload: DREP_BC_CMD_* */
#define DREP_TEL_FAULT              (3U)    /**< payload: EVF_SAFETY bits b0-b9 */
#define DREP_TEL_STATE              (4U)    /**< payload: system state, 4 bits */
#define DREP_TEL_TYPE_COUNT         (5U)    /**< 0 and >= count are rejected */

/* Hump payload: [6:0] height mm  [14:7] pitch Q1 (signed)  [15] OOR */
#define DREP_HUMP_PITCH_SHIFT       (7U)
#define DREP_HUMP_OOR_SHIFT         (15U)

static inline tel_evt_t drep_tel_make(uint32_t type, uint32_t timestamp_ms, uint32_t payload, bool oor)
{
    tel_evt_t e = { 0U, 0U };

    e.hdr = drep_set(e.hdr, DREP_TEL_TYPE_SHIFT, DREP_TEL_TYPE_BITS, type);
    e.hdr = drep_set(e.hdr, DREP_TEL_TS_SHIFT, DREP_TEL_TS_BITS, timestamp_ms);
    e.hdr = drep_set(e.hdr, DREP_TEL_OOR_SHIFT, 1U, oor ? 1U : 0U);
    e.payload = payload;
    return e;
}

/** Hump event: height clamps to 100 mm, pitch to +-45 deg, both with OOR. */
static inline tel_evt_t drep_tel_hump(uint32_t timestamp_ms, int32_t height_mm, int32_t pitch_half_deg)
{
    bool          oor = false;
    int32_t const h = drep_clamp(height_mm, 0, DREP_HUMP_MAX_MM, &oor);
    int32_t const p = drep_pitch_q1(pitch_half_deg, &oor);
    uint32_t      pl = 0U;

    pl = drep_set(pl, 0U, DREP_HUMP_BITS, (uint32_t)h);
    pl = drep_set(pl, DREP_HUMP_PITCH_SHIFT, DREP_PITCH_BITS, (uint32_t)p);
    pl = drep_set(pl, DREP_HUMP_OOR_SHIFT, 1U, oor ? 1U : 0U);
    return drep_tel_make(DREP_TEL_HUMP, timestamp_ms, pl, oor);
}

static inline uint32_t drep_tel_type(tel_evt_t e)
{
    return drep_get(e.hdr, DREP_TEL_TYPE_SHIFT, DREP_TEL_TYPE_BITS);
}

static inline uint32_t drep_tel_timestamp_ms(tel_evt_t e)
{
    return drep_get(e.hdr, DREP_TEL_TS_SHIFT, DREP_TEL_TS_BITS);
}

static inline bool drep_tel_oor(tel_evt_t e)
{
    return drep_get(e.hdr, DREP_TEL_OOR_SHIFT, 1U) != 0U;
}

static inline bool drep_tel_valid(tel_evt_t e)
{
    uint32_t const t = drep_tel_type(e);

    return (t != 0U) && (t < DREP_TEL_TYPE_COUNT);
}

static inline uint32_t drep_hump_height_mm(uint32_t payload)
{
    return drep_get(payload, 0U, DREP_HUMP_BITS);
}

static inline int32_t drep_hump_pitch_q1(uint32_t payload)
{
    return drep_sext(drep_get(payload, DREP_HUMP_PITCH_SHIFT, DREP_PITCH_BITS), DREP_PITCH_BITS);
}

/* ===========================================================================
 * IF-09 barcode edge ring: 64 x 20-bit gaps, 160 B of storage
 *   gap i = lo[i] | (nibble i of hi[i / 2]) << 16  (even i: low nibble)
 * Single producer (the barcode ISR: drep_edge_push), single consumer (the
 * Barcode task: drep_edge_pop).  head and tail are free-running 0-255; the
 * ISR is the only writer of lo[], hi[] and head, the task of tail, so the
 * read-modify-write of a shared hi[] byte never loses the neighbour.
 * ========================================================================= */

#define DREP_EDGE_LEN               (BC_EDGE_RING_LEN)
#define DREP_EDGE_GAP_BITS          (BC_GAP_BITS)
#define DREP_EDGE_GAP_MAX           (DREP_MASK(DREP_EDGE_GAP_BITS))  /**< saturation = start/timeout */

/* Orders the data stores before the index store (single core, no cache). */
#define DREP_COMPILER_BARRIER()     __asm__ volatile("" ::: "memory")

typedef struct
{
    uint16_t          lo[DREP_EDGE_LEN];
    uint8_t           hi[DREP_EDGE_LEN / 2U];
    volatile uint8_t  head;         /* ISR writes */
    volatile uint8_t  tail;         /* task writes */
    volatile uint8_t  overflow;     /* ISR counts dropped edges (wraps) */
} drep_edge_ring_t;

static inline void drep_edge_init(drep_edge_ring_t *p_ring)
{
    uint32_t i;

    for (i = 0U; i < DREP_EDGE_LEN; i++)
    {
        p_ring->lo[i] = 0U;
    }
    for (i = 0U; i < (DREP_EDGE_LEN / 2U); i++)
    {
        p_ring->hi[i] = 0U;
    }
    p_ring->head = 0U;
    p_ring->tail = 0U;
    p_ring->overflow = 0U;
}

/** Store a gap (us) in slot idx; saturates at 0xFFFFF. */
static inline void drep_edge_store(drep_edge_ring_t *p_ring, uint32_t idx, uint32_t gap_us)
{
    uint32_t const g = (gap_us > DREP_EDGE_GAP_MAX) ? DREP_EDGE_GAP_MAX : gap_us;
    uint32_t const i = idx % DREP_EDGE_LEN;
    uint32_t const shift = ((i & 1U) != 0U) ? 4U : 0U;
    uint32_t       b = p_ring->hi[i / 2U];

    p_ring->lo[i] = (uint16_t)(g & 0xFFFFU);
    b = drep_set(b, shift, 4U, g >> 16);
    p_ring->hi[i / 2U] = (uint8_t)b;
}

static inline uint32_t drep_edge_load(drep_edge_ring_t const *p_ring, uint32_t idx)
{
    uint32_t const i = idx % DREP_EDGE_LEN;
    uint32_t const shift = ((i & 1U) != 0U) ? 4U : 0U;

    return (uint32_t)p_ring->lo[i] | (drep_get(p_ring->hi[i / 2U], shift, 4U) << 16);
}

/** Edges waiting (0..64). */
static inline uint32_t drep_edge_count(drep_edge_ring_t const *p_ring)
{
    return (uint8_t)(p_ring->head - p_ring->tail);
}

/** ISR side.  Returns false (and counts an overflow) when the ring is full. */
static inline bool drep_edge_push(drep_edge_ring_t *p_ring, uint32_t gap_us)
{
    uint8_t const head = p_ring->head;

    if ((uint8_t)(head - p_ring->tail) >= DREP_EDGE_LEN)
    {
        p_ring->overflow = (uint8_t)(p_ring->overflow + 1U);
        return false;
    }
    drep_edge_store(p_ring, head, gap_us);
    DREP_COMPILER_BARRIER();
    p_ring->head = (uint8_t)(head + 1U);
    return true;
}

/** Task side.  Returns false when the ring is empty. */
static inline bool drep_edge_pop(drep_edge_ring_t *p_ring, uint32_t *p_gap_us)
{
    uint8_t const tail = p_ring->tail;

    if (p_ring->head == tail)
    {
        return false;
    }
    *p_gap_us = drep_edge_load(p_ring, tail);
    DREP_COMPILER_BARRIER();
    p_ring->tail = (uint8_t)(tail + 1U);
    return true;
}

/* ===========================================================================
 * State-machine widths (section 4)
 * ========================================================================= */

#define DREP_FSM_SYSTEM_STATES      (10U)
#define DREP_FSM_SYSTEM_BITS        (4U)
#define DREP_FSM_LF_STATES          (6U)
#define DREP_FSM_LF_BITS            (3U)
#define DREP_FSM_BARCODE_STATES     (5U)
#define DREP_FSM_BARCODE_BITS       (3U)
#define DREP_FSM_OBSTACLE_STATES    (9U)    /* 8 + the composite EXECUTING_BYPASS */
#define DREP_FSM_OBSTACLE_BITS      (4U)
#define DREP_FSM_IMU_STATES         (6U)
#define DREP_FSM_IMU_BITS           (3U)
#define DREP_FSM_WIFI_STATES        (5U)
#define DREP_FSM_WIFI_BITS          (3U)
#define DREP_FSM_VC_AUTH_STATES     (2U)
#define DREP_FSM_VC_AUTH_BITS       (1U)
#define DREP_FSM_AVOID_PLAN_STATES  (DREP_BYPASS_COUNT)
#define DREP_FSM_AVOID_PLAN_BITS    (DREP_NAV_BYPASS_BITS)

/* ===========================================================================
 * MQTT binary frames (section 8, D10):  ver u8 | type u8 | len u8 | payload
 * Payload bits are packed LSB first, little-endian.  A frame is accepted only
 * if ver == DREP_WIRE_VER, type is known, len is that type's length and the
 * received length is 3 + len.
 * ========================================================================= */

#define DREP_WIRE_VER               (1U)
#define DREP_FRAME_HDR_BYTES        (3U)
#define DREP_FRAME_MAX_PAYLOAD      (15U)
#define DREP_FRAME_MAX_BYTES        (DREP_FRAME_HDR_BYTES + DREP_FRAME_MAX_PAYLOAD)

#define DREP_FT_TELEMETRY_MOTION    (1U)    /**< ts 24, speed 10, distance 16, enc_l 32, enc_r 32 -> 15 B */
#define DREP_FT_TELEMETRY_SENSOR    (2U)    /**< ts 24, IR_R 12, IR_C 12, ultrasonic 9, pitch 8, roll 8 -> 10 B */
#define DREP_FT_EVENT_BARCODE       (3U)    /**< ts 24, command 2 -> 4 B */
#define DREP_FT_EVENT_HUMP          (4U)    /**< ts 24, height 7, pitch 8, OOR 1 -> 5 B */
#define DREP_FT_EVENT_OBSTACLE      (5U)    /**< ts 24, obstacle profile 37 -> 8 B */
#define DREP_FT_EVENT_FAULT         (6U)    /**< ts 24, EVF_SAFETY b0-b9 -> 5 B */
#define DREP_FT_HEARTBEAT           (7U)    /**< uptime 32, state 4, fault bits 10 -> 6 B */
#define DREP_FT_CMD_START           (8U)    /**< no payload */
#define DREP_FT_CMD_STOP            (9U)
#define DREP_FT_CMD_RESET           (10U)
#define DREP_FT_COUNT               (11U)

/** Payload length per frame type (0 = no payload); index 0 is not a type. */
static inline uint32_t drep_frame_len(uint32_t type)
{
    static uint8_t const k_len[DREP_FT_COUNT] = { 0U, 15U, 10U, 4U, 5U, 8U, 5U, 6U, 0U, 0U, 0U };

    return (type < DREP_FT_COUNT) ? k_len[type] : 0U;
}

/* Little-endian bit writer/reader, LSB first. */
static inline void drep_bits_put(uint8_t *p_buf, uint32_t *p_pos, uint32_t value, uint32_t bits)
{
    uint32_t i;

    for (i = 0U; i < bits; i++)
    {
        uint32_t const pos = *p_pos + i;
        uint8_t const  bit = (uint8_t)(1U << (pos & 7U));

        if (((value >> i) & 1U) != 0U)
        {
            p_buf[pos >> 3] = (uint8_t)(p_buf[pos >> 3] | bit);
        }
        else
        {
            p_buf[pos >> 3] = (uint8_t)(p_buf[pos >> 3] & (uint8_t)~bit);
        }
    }
    *p_pos += bits;
}

static inline uint32_t drep_bits_get(uint8_t const *p_buf, uint32_t *p_pos, uint32_t bits)
{
    uint32_t v = 0U;
    uint32_t i;

    for (i = 0U; i < bits; i++)
    {
        uint32_t const pos = *p_pos + i;

        v |= (uint32_t)((p_buf[pos >> 3] >> (pos & 7U)) & 1U) << i;
    }
    *p_pos += bits;
    return v;
}

/* Header + zeroed payload; returns the bit position where the payload starts. */
static inline uint32_t drep_frame_begin(uint8_t *p_buf, uint32_t type)
{
    uint32_t const len = drep_frame_len(type);
    uint32_t       i;

    p_buf[0] = (uint8_t)DREP_WIRE_VER;
    p_buf[1] = (uint8_t)type;
    p_buf[2] = (uint8_t)len;
    for (i = 0U; i < len; i++)
    {
        p_buf[DREP_FRAME_HDR_BYTES + i] = 0U;
    }
    return DREP_FRAME_HDR_BYTES * 8U;
}

/* Every encoder writes into p_buf (at least DREP_FRAME_MAX_BYTES) and
 * returns the frame length in bytes (3 + payload). */

static inline uint32_t drep_frame_motion(uint8_t *p_buf, uint32_t ts_ms, uint32_t motion_word,
                                         int32_t enc_l, int32_t enc_r)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_TELEMETRY_MOTION);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, drep_get(motion_word, DREP_MS_SPEED_SHIFT, DREP_MS_SPEED_BITS), 10U);
    drep_bits_put(p_buf, &pos, drep_ms_distance_cm(motion_word), 16U);
    drep_bits_put(p_buf, &pos, (uint32_t)enc_l, 32U);
    drep_bits_put(p_buf, &pos, (uint32_t)enc_r, 32U);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_TELEMETRY_MOTION);
}

static inline uint32_t drep_frame_sensor(uint8_t *p_buf, uint32_t ts_ms, uint32_t ir_r_raw, uint32_t ir_c_raw,
                                         uint32_t ultrasonic_cm, int32_t pitch_q1, int32_t roll_q1)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_TELEMETRY_SENSOR);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, ir_r_raw, 12U);
    drep_bits_put(p_buf, &pos, ir_c_raw, 12U);
    drep_bits_put(p_buf, &pos, ultrasonic_cm, 9U);
    drep_bits_put(p_buf, &pos, (uint32_t)pitch_q1, 8U);
    drep_bits_put(p_buf, &pos, (uint32_t)roll_q1, 8U);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_TELEMETRY_SENSOR);
}

static inline uint32_t drep_frame_barcode(uint8_t *p_buf, uint32_t ts_ms, uint32_t cmd)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_EVENT_BARCODE);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, cmd, DREP_BC_CMD_BITS);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_EVENT_BARCODE);
}

/** From a hump tel_evt_t payload: height 7, pitch 8, OOR 1. */
static inline uint32_t drep_frame_hump(uint8_t *p_buf, uint32_t ts_ms, uint32_t hump_payload)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_EVENT_HUMP);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, hump_payload, 16U);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_EVENT_HUMP);
}

static inline uint32_t drep_frame_obstacle(uint8_t *p_buf, uint32_t ts_ms, obstacle_profile_t prof)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_EVENT_OBSTACLE);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, prof.lo, DREP_OBS_LO_BITS);
    drep_bits_put(p_buf, &pos, prof.hi, DREP_OBS_CLR_BITS);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_EVENT_OBSTACLE);
}

static inline uint32_t drep_frame_fault(uint8_t *p_buf, uint32_t ts_ms, uint32_t evf_safety_bits)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_EVENT_FAULT);

    drep_bits_put(p_buf, &pos, ts_ms, 24U);
    drep_bits_put(p_buf, &pos, evf_safety_bits, 10U);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_EVENT_FAULT);
}

static inline uint32_t drep_frame_heartbeat(uint8_t *p_buf, uint32_t uptime_ms, uint32_t sys_state,
                                            uint32_t evf_safety_bits)
{
    uint32_t pos = drep_frame_begin(p_buf, DREP_FT_HEARTBEAT);

    drep_bits_put(p_buf, &pos, uptime_ms, 32U);
    drep_bits_put(p_buf, &pos, sys_state, DREP_FSM_SYSTEM_BITS);
    drep_bits_put(p_buf, &pos, evf_safety_bits, 10U);
    return DREP_FRAME_HDR_BYTES + drep_frame_len(DREP_FT_HEARTBEAT);
}

/** A command frame (what the dashboard sends). */
static inline uint32_t drep_frame_cmd(uint8_t *p_buf, uint32_t cmd_type)
{
    (void)drep_frame_begin(p_buf, cmd_type);
    return DREP_FRAME_HDR_BYTES;
}

/* Frame check results. */
#define DREP_FRAME_OK               (0)
#define DREP_FRAME_SHORT            (-1)    /**< fewer than 3 bytes */
#define DREP_FRAME_BAD_VERSION      (-2)
#define DREP_FRAME_UNKNOWN_TYPE     (-3)
#define DREP_FRAME_BAD_LENGTH       (-4)    /**< len field or received length wrong */
#define DREP_FRAME_NOT_COMMAND      (-5)    /**< valid frame, but not a command */

/** Header and length check of a received frame of n bytes. */
static inline int32_t drep_frame_check(uint8_t const *p_buf, uint32_t n)
{
    uint32_t type;

    if (n < DREP_FRAME_HDR_BYTES)
    {
        return DREP_FRAME_SHORT;
    }
    if (p_buf[0] != DREP_WIRE_VER)
    {
        return DREP_FRAME_BAD_VERSION;
    }
    type = p_buf[1];
    if ((type == 0U) || (type >= DREP_FT_COUNT))
    {
        return DREP_FRAME_UNKNOWN_TYPE;
    }
    if ((p_buf[2] != drep_frame_len(type)) || (n != (DREP_FRAME_HDR_BYTES + drep_frame_len(type))))
    {
        return DREP_FRAME_BAD_LENGTH;
    }
    return DREP_FRAME_OK;
}

/**
 * Telemetry's receive path: a received MQTT command frame -> the nav_input_t
 * for MBF_NAV_IN.  Returns DREP_FRAME_OK and fills *p_nav only for a valid
 * command; anything else returns the reject reason and leaves *p_nav as it
 * was, so a rejected frame can never reach the Vehicle Controller.
 */
static inline int32_t drep_cmd_to_nav(uint8_t const *p_buf, uint32_t n, nav_input_t *p_nav)
{
    int32_t const rc = drep_frame_check(p_buf, n);

    if (rc != DREP_FRAME_OK)
    {
        return rc;
    }
    switch (p_buf[1])
    {
        case DREP_FT_CMD_START:
            *p_nav = drep_nav_remote(DREP_NAV_REMOTE_START);
            return DREP_FRAME_OK;
        case DREP_FT_CMD_STOP:
            *p_nav = drep_nav_remote(DREP_NAV_REMOTE_STOP);
            return DREP_FRAME_OK;
        case DREP_FT_CMD_RESET:
            *p_nav = drep_nav_remote(DREP_NAV_REMOTE_RESET);
            return DREP_FRAME_OK;
        default:
            return DREP_FRAME_NOT_COMMAND;
    }
}

/* ===========================================================================
 * Compile-time checks: totals fit their containers, FSM state counts fit
 * their widths, fields do not overlap.
 * ========================================================================= */

/* Every layout is contiguous: each field starts where the previous ends. */
#define DREP_NEXT(shift, bits)      ((shift) + (bits))
_Static_assert((DREP_NEXT(DREP_NAV_SRC_SHIFT, DREP_NAV_SRC_BITS) == DREP_NAV_TYPE_SHIFT)
               && (DREP_NEXT(DREP_NAV_TYPE_SHIFT, DREP_NAV_TYPE_BITS) == DREP_NAV_PAYLOAD_SHIFT)
               && (DREP_NEXT(DREP_NAV_PAYLOAD_SHIFT, DREP_NAV_PAYLOAD_BITS) == DREP_NAV_BYPASS_SHIFT)
               && (DREP_NEXT(DREP_NAV_BYPASS_SHIFT, DREP_NAV_BYPASS_BITS) == DREP_NAV_OOR_SHIFT),
               "nav_input_t fields are contiguous");
_Static_assert((DREP_NEXT(DREP_MC_TYPE_SHIFT, DREP_MC_TYPE_BITS) == DREP_MC_SPEED_SHIFT)
               && (DREP_NEXT(DREP_MC_SPEED_SHIFT, DREP_MC_SPEED_BITS) == DREP_MC_ARG_SHIFT)
               && (DREP_NEXT(DREP_MC_ARG_SHIFT, DREP_MC_ARG_BITS) == DREP_MC_OOR_SHIFT),
               "motion_cmd_t fields are contiguous");
_Static_assert((DREP_NEXT(DREP_OBS_DIST_SHIFT, DREP_OBS_DIST_BITS) == DREP_OBS_BEAR_SHIFT)
               && (DREP_NEXT(DREP_OBS_BEAR_SHIFT, DREP_OBS_BEAR_BITS) == DREP_OBS_WIDTH_SHIFT)
               && (DREP_NEXT(DREP_OBS_WIDTH_SHIFT, DREP_OBS_WIDTH_BITS) == DREP_OBS_CLRL_SHIFT)
               && (DREP_NEXT(DREP_OBS_CLRL_SHIFT, DREP_OBS_CLR_BITS) == DREP_OBS_SEQ_SHIFT)
               && (DREP_NEXT(DREP_OBS_SEQ_SHIFT, DREP_OBS_SEQ_BITS) == DREP_OBS_OOR_SHIFT),
               "obstacle_profile_t fields are contiguous");
_Static_assert((DREP_NEXT(DREP_MS_SPEED_SHIFT, DREP_MS_SPEED_BITS) == DREP_MS_DIST_SHIFT)
               && (DREP_NEXT(DREP_MS_DIST_SHIFT, DREP_MS_DIST_BITS) == DREP_MS_OOR_SHIFT),
               "motion_state word fields are contiguous");
_Static_assert((DREP_NEXT(DREP_TEL_TYPE_SHIFT, DREP_TEL_TYPE_BITS) == DREP_TEL_TS_SHIFT)
               && (DREP_NEXT(DREP_TEL_TS_SHIFT, DREP_TEL_TS_BITS) == DREP_TEL_OOR_SHIFT),
               "tel_evt_t header fields are contiguous");
_Static_assert((DREP_NEXT(0U, DREP_HUMP_BITS) == DREP_HUMP_PITCH_SHIFT)
               && (DREP_NEXT(DREP_HUMP_PITCH_SHIFT, DREP_PITCH_BITS) == DREP_HUMP_OOR_SHIFT),
               "hump payload fields are contiguous");

_Static_assert(DREP_NAV_TOTAL_BITS == (DREP_NAV_OOR_SHIFT + 1U), "nav_input_t layout");
_Static_assert((DREP_NAV_SRC_BITS + DREP_NAV_TYPE_BITS + DREP_NAV_PAYLOAD_BITS + DREP_NAV_BYPASS_BITS + 1U)
               == DREP_NAV_TOTAL_BITS, "nav_input_t: 2 + 3 + 8 + 3 + 1 = 17 bits");
_Static_assert(DREP_NAV_TOTAL_BITS <= 32U, "nav_input_t fits one word");
_Static_assert((DREP_NAV_LINE_TYPES <= 8U) && (DREP_NAV_AVOID_TYPES <= 8U) && (DREP_NAV_REMOTE_TYPES <= 8U),
               "nav types fit 3 bits");
_Static_assert(DREP_BYPASS_COUNT <= (1U << DREP_NAV_BYPASS_BITS), "bypass fits 3 bits");

_Static_assert((DREP_MC_TYPE_BITS + DREP_MC_SPEED_BITS + DREP_MC_ARG_BITS + 1U) == DREP_MC_TOTAL_BITS,
               "motion_cmd_t: 3 + 6 + 9 + 1 = 19 bits");
_Static_assert(DREP_MC_TOTAL_BITS == (DREP_MC_OOR_SHIFT + 1U), "motion_cmd_t layout");
_Static_assert(DREP_MC_SPEED_MAX < (1 << DREP_MC_SPEED_BITS), "speed fits 6 bits");
_Static_assert(DREP_MC_DIST_MAX < (1 << DREP_MC_ARG_BITS), "distance fits 9 bits");

_Static_assert((DREP_OBS_DIST_BITS + DREP_OBS_BEAR_BITS + DREP_OBS_WIDTH_BITS + (2U * DREP_OBS_CLR_BITS)
                + DREP_OBS_SEQ_BITS + 1U) == DREP_OBS_TOTAL_BITS, "obstacle: 9 + 4 + 7 + 7 + 7 + 2 + 1 = 37 bits");
_Static_assert(DREP_OBS_LO_BITS == (DREP_OBS_OOR_SHIFT + 1U), "obstacle lo layout");
_Static_assert(((DREP_OBS_TOTAL_BITS + 7U) / 8U) == DREP_OBS_WIRE_BYTES, "obstacle is 5 B on the wire");
_Static_assert(DREP_OBS_DIST_MAX_CM < (1 << DREP_OBS_DIST_BITS), "distance fits 9 bits");
_Static_assert(DREP_OBS_BEAR_MAX < (1 << DREP_OBS_BEAR_BITS), "bearing fits 4 bits");
_Static_assert((DREP_OBS_WIDTH_MAX_MM / DREP_OBS_MM_PER_UNIT) < (1 << DREP_OBS_WIDTH_BITS), "width fits 7 bits");
_Static_assert((DREP_OBS_CLR_MAX_MM / DREP_OBS_MM_PER_UNIT) == ((1 << DREP_OBS_CLR_BITS) - 1),
               "clearance saturates at 127 units");

_Static_assert((DREP_MS_OOR_SHIFT + 1U) <= 32U, "motion_state word fits");
_Static_assert((DREP_TEL_OOR_SHIFT + 3U) == 32U, "tel_evt_t header is exactly one word");
_Static_assert(DREP_TEL_TYPE_COUNT <= (1U << DREP_TEL_TYPE_BITS), "tel types fit 5 bits");
_Static_assert((DREP_HUMP_OOR_SHIFT + 1U) == 16U, "hump payload is 16 bits");

_Static_assert((DREP_EDGE_LEN % 2U) == 0U, "edge ring: two nibbles per hi byte");
_Static_assert(DREP_EDGE_LEN <= 128U, "edge ring: uint8_t free-running indices");
_Static_assert(DREP_EDGE_GAP_BITS == 20U, "edge ring stores 16 + 4 bits");
_Static_assert((sizeof(((drep_edge_ring_t *)0)->lo) + sizeof(((drep_edge_ring_t *)0)->hi)) == 160U,
               "edge ring storage is 160 B");

_Static_assert(DREP_FSM_SYSTEM_STATES <= (1U << DREP_FSM_SYSTEM_BITS), "system FSM fits");
_Static_assert(DREP_FSM_LF_STATES <= (1U << DREP_FSM_LF_BITS), "LF FSM fits");
_Static_assert(DREP_FSM_BARCODE_STATES <= (1U << DREP_FSM_BARCODE_BITS), "barcode FSM fits");
_Static_assert(DREP_FSM_OBSTACLE_STATES <= (1U << DREP_FSM_OBSTACLE_BITS), "obstacle FSM fits");
_Static_assert(DREP_FSM_IMU_STATES <= (1U << DREP_FSM_IMU_BITS), "IMU FSM fits");
_Static_assert(DREP_FSM_WIFI_STATES <= (1U << DREP_FSM_WIFI_BITS), "WiFi FSM fits");
_Static_assert(DREP_FSM_VC_AUTH_STATES <= (1U << DREP_FSM_VC_AUTH_BITS), "VC authority fits");
_Static_assert(DREP_FSM_AVOID_PLAN_STATES <= (1U << DREP_FSM_AVOID_PLAN_BITS), "avoidance plan fits");

_Static_assert(DREP_FRAME_MAX_PAYLOAD == 15U, "the longest payload is telemetry_motion");

#endif /* DATA_REPR_H */
