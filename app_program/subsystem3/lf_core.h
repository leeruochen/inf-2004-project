/*
 *  lf_core.h
 *  Subsystem 3 -- line-following core (pure C): calibration, normalisation,
 *  right-edge tracking, junction, line-lost and search decisions.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_lf_core.c builds it unchanged.  The
 *  Line-Following task (LF-3) will call lf_core_step() once per 5 ms sample
 *  and act on the result; nothing here sends a message or touches the ADC.
 *
 *  Geometry (ARCHITECTURE.md D11; measured on the car, 3 Nov 2026):
 *    CENTRE and RIGHT sensors ~22 mm apart, centre to centre; black line
 *    18 mm wide; black reads HIGHER than white (LF-1, S3-U09).
 *
 *  How it follows the line ("edge following"):
 *    CENTRE rides on the line's RIGHT edge, half over black.  Its darkness
 *    d (0..1000 per mille of the calibrated white..black span) is the error:
 *      d > 500  car drifted left   -> steer right (steer > 0)
 *      d < 500  car drifted right  -> steer left  (steer < 0)
 *    Steering sign: steer > 0 = steer right (data_repr.h, task_motion.c).
 *    RIGHT is never on the line while CENTRE tracks the edge (it sits 22 mm
 *    to the right, the line is 18 mm wide; CENTRE has left the line before
 *    RIGHT can reach it), so any darkness under RIGHT means something:
 *      - CENTRE on the line too: a cross line arriving -> hold the steering;
 *        both solid black for 3 samples -> JUNCTION;
 *      - CENTRE off the line: the car overshot left across the whole line
 *        -> full right.
 *    RIGHT is also the side memory: the line was lost to the right if RIGHT
 *    saw it shortly before CENTRE lost it, otherwise to the left.
 *
 *  Capture rule: CENTRE's edge is trusted only after CENTRE has seen solid
 *  black (zone DARK).  From the line body, "darker -> steer right" walks
 *  CENTRE out over the RIGHT edge, the stable one; the left edge is an
 *  unstable balance point the car must not start on (e.g. after a u-turn).
 */

#ifndef LF_CORE_H
#define LF_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "config/timing.h"
#include "src/common/data_repr.h"
#include "app_program/subsystem3/lf_sanity.h"

/* ---------------------------------------------------------------------------
 * Tunables (LF-2 starting values; tuned on the track in LF-3).
 * ------------------------------------------------------------------------- */

/** Darkness scale: 0 = calibrated white, LF_PM_FULL = calibrated black. */
#define LF_PM_FULL              (1000U)
/** Below this darkness a sensor is LIGHT (off the line). TUNE */
#define LF_LIGHT_MAX_PM         (250U)
/** Above this darkness a sensor is DARK (over the line body). TUNE */
#define LF_DARK_MIN_PM          (750U)
/** CENTRE's target darkness: half over the line's right edge. */
#define LF_EDGE_TARGET_PM       (500U)

/** Calibration refused if black - white is smaller than this (raw counts;
 *  same contrast rule as S3-U09, about 10 % of the 12-bit scale). */
#define LF_MIN_SPAN_RAW         (400U)

/** Steering at CENTRE fully black (+) or fully white (-), Q7. TUNE */
#define LF_KP_Q7                (64)
/** Full steering, Q7 (DREP_STEER_Q7_MAX). */
#define LF_STEER_FULL_Q7        (127)
/** Gentle right turn while looking for the line after a u-turn, Q7. TUNE */
#define LF_ACQUIRE_STEER_Q7     (32)

/** RIGHT saw darkness at most this long before CENTRE lost the line ->
 *  the line was lost to the right. TUNE */
#define LF_SIDE_MEMORY_MS       (100U)
/** Search the remembered side this long, then the other side until
 *  LF_T_SEARCH_MS. TUNE */
#define LF_SEARCH_SWAP_MS       (700U)
/** LINE_LOST posts nothing for this long, so Motion's hold timeout
 *  (MOTION_T_HOLD_MS) stops the car before the search starts. */
#define LF_T_STOP_MS            (MOTION_T_HOLD_MS + MOTION_PERIOD_MS)

/* Times as sample counts (one lf_core_step() call = LF_SAMPLE_PERIOD_MS). */
#define LF_SAMPLES(ms)          ((uint32_t)(ms) / LF_SAMPLE_PERIOD_MS)

/* ---------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

/** One sensor's calibration (raw ADC counts). */
typedef struct
{
    uint16_t white;     /**< raw reading over the white track */
    uint16_t black;     /**< raw reading over the black line body */
} lf_cal_ch_t;

/** Both sensors' calibration. */
typedef struct
{
    lf_cal_ch_t c;      /**< CENTRE */
    lf_cal_ch_t r;      /**< RIGHT */
} lf_cal_t;

/** Calibration verdict. */
typedef enum
{
    LF_CAL_OK = 0,
    LF_CAL_BAD_SENSOR,      /**< a sensor failed lf_adc_check (stuck, noisy, empty) */
    LF_CAL_NO_CONTRAST      /**< black not >= white + LF_MIN_SPAN_RAW (or reversed) */
} lf_cal_status_t;

/** A sensor's zone, after LF_HYST_SAMPLES-sample hysteresis. */
typedef enum
{
    LF_ZONE_LIGHT = 0,      /**< darkness < LF_LIGHT_MAX_PM */
    LF_ZONE_EDGE,           /**< in between: partly over the line */
    LF_ZONE_DARK            /**< darkness > LF_DARK_MIN_PM */
} lf_zone_t;

/** Line-following state (design.md section 6.1 names, plus IDLE).  The
 *  values are the telemetry FSM code (DREP_FSM_LF_STATES = 6, 3 bits). */
typedef enum
{
    LF_ST_IDLE = 0,         /**< not running (or calibration refused) */
    LF_ST_ON_LINE,          /**< tracking the right edge, posting steering */
    LF_ST_JUNCTION,         /**< JUNCTION_DETECTED: straight across, or a turn */
    LF_ST_LINE_LOST,        /**< both off the line: stop, side recorded */
    LF_ST_REACQUIRING,      /**< REACQUIRING_TRACK: arc search */
    LF_ST_SUSPENDED,        /**< SUSPENDED_BY_AVOIDANCE: sample, never post */
    LF_ST_COUNT
} lf_state_t;

/** Which side of CENTRE the line went when it was lost. */
typedef enum
{
    LF_SIDE_LEFT = 0,
    LF_SIDE_RIGHT
} lf_side_t;

/** What the Vehicle Controller is doing after a junction (LF-3 supplies
 *  it); while it is not NONE the core posts no steering, because a steering
 *  post would replace the turn in progress. */
typedef enum
{
    LF_MAN_NONE = 0,
    LF_MAN_TURN,            /**< 90 degree turn left or right */
    LF_MAN_UTURN            /**< 180 degree turn */
} lf_manoeuvre_t;

/** One-shot events for the Vehicle Controller (IF-01). */
typedef enum
{
    LF_EV_NONE = 0,
    LF_EV_JUNCTION,         /**< DREP_NAV_LINE_JUNCTION */
    LF_EV_TRACK_LOST        /**< DREP_NAV_LINE_TRACK_LOST */
} lf_event_t;

/** Inputs for one sample. */
typedef struct
{
    uint16_t       raw_c;           /**< CENTRE raw ADC (0..4095) */
    uint16_t       raw_r;           /**< RIGHT raw ADC (0..4095) */
    bool           running;         /**< the car is in autonomous navigation */
    lf_manoeuvre_t manoeuvre;       /**< junction turn in progress */
    bool           avoid_active;    /**< EVF_NAV_AVOID_ACTIVE */
    bool           reacquired;      /**< EVF_NAV_LINE_REACQUIRED */
} lf_in_t;

/** Result of one sample. */
typedef struct
{
    lf_state_t state;       /**< state after this sample */
    bool       post_steer;  /**< post steer_q7 now (every LF_STEER_POST_PERIOD_MS) */
    int32_t    steer_q7;    /**< -127..+127; > 0 = steer right */
    lf_event_t event;       /**< one-shot, LF_EV_NONE most samples */
    bool       line_found;  /**< EVF_NAV_LINE_FOUND: on the line (ON_LINE, or CENTRE DARK) */
    uint16_t   d_c_pm;      /**< CENTRE darkness 0..1000 (telemetry) */
    uint16_t   d_r_pm;      /**< RIGHT darkness 0..1000 (telemetry) */
} lf_out_t;

/** One sensor's zone filter.  Private: use the functions below. */
typedef struct
{
    lf_zone_t zone;         /**< confirmed zone */
    lf_zone_t cand;         /**< candidate zone being confirmed */
    uint32_t  cand_n;       /**< consecutive samples of cand */
    bool      primed;       /**< false until the first sample */
} lf_zone_filt_t;

/** Core state.  Private: fields are read only by lf_core.c (and tests). */
typedef struct
{
    lf_cal_t       cal;
    bool           cal_ok;
    lf_state_t     state;
    lf_zone_filt_t zc;
    lf_zone_filt_t zr;
    bool           prev_running;
    bool           junction_armed;  /**< false from a junction until RIGHT is LIGHT */
    lf_side_t      side;            /**< side memory */
    uint32_t       r_quiet_n;       /**< samples since RIGHT last read non-light */
    uint32_t       lost_n;          /**< consecutive samples with both LIGHT */
    uint32_t       timer_n;         /**< samples spent in the current state/phase */
    uint32_t       post_n;          /**< steering post cadence counter */
    int32_t        steer_q7;        /**< last computed steering */
    lf_manoeuvre_t man_seen;        /**< manoeuvre seen in JUNCTION (NONE = straight) */
    bool           man_done;        /**< that manoeuvre has finished */
} lf_core_t;

/* ---------------------------------------------------------------------------
 * Functions
 * ------------------------------------------------------------------------- */

/*
 * Start-pose calibration (at boot, car on the track, CENTRE over the middle
 * of the line, RIGHT on white).  p_c_on_line / p_r_on_white are the
 * statistics of CENTRE's and RIGHT's samples in that pose (lf_sanity.h).
 * The two sensors read alike (LF-1: white 140 vs 144), so both get
 * white = RIGHT's average and black = CENTRE's average.
 * Returns LF_CAL_OK and fills *p_cal, or the reason it refused (then *p_cal
 * is left unchanged).
 */
lf_cal_status_t lf_cal_from_start_pose(lf_adc_stats_t const *p_c_on_line,
                                       lf_adc_stats_t const *p_r_on_white,
                                       lf_cal_t *p_cal);

/*
 * Darkness of one raw reading: 0 at or below white, LF_PM_FULL at or above
 * black, linear in between.  0 if the calibration has no span.
 */
uint16_t lf_norm(lf_cal_ch_t const *p_cal, uint16_t raw);

/* Zone of one darkness value, without hysteresis. */
lf_zone_t lf_zone_of(uint16_t d_pm);

/* Zone filter: the zone changes only after LF_HYST_SAMPLES consecutive
 * samples in a new zone; the first sample sets it at once. */
void lf_zone_reset(lf_zone_filt_t *p_f);
lf_zone_t lf_zone_update(lf_zone_filt_t *p_f, uint16_t d_pm);

/*
 * Edge-tracking steering from CENTRE's darkness alone:
 * LF_KP_Q7 * (d - 500) / 500, clamped to +-LF_STEER_FULL_Q7.
 */
int32_t lf_edge_steer(uint16_t d_c_pm);

/*
 * Set up the core.  p_cal NULL, or a calibration without LF_MIN_SPAN_RAW
 * on both sensors, means "refused": the core then never posts steering and
 * reports TRACK_LOST once when the run starts, so the car stops.
 */
void lf_core_init(lf_core_t *p_lf, lf_cal_t const *p_cal);

/*
 * One sample (call every LF_SAMPLE_PERIOD_MS, in every state, so the zone
 * filters always have fresh data).  Fills *p_out.
 */
void lf_core_step(lf_core_t *p_lf, lf_in_t const *p_in, lf_out_t *p_out);

#endif /* LF_CORE_H */
