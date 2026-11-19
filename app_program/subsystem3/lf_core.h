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
 *  Geometry (ARCHITECTURE.md D11; ruler on the car, 4 Nov 2026):
 *    CENTRE and RIGHT sensors 26 mm apart, centre to centre, both 10 mm
 *    above the track; black line 18 mm wide; black reads HIGHER than white
 *    (LF-1, S3-U09).
 *
 *  How it follows the line ("edge following"):
 *    CENTRE rides just inside the line's RIGHT edge, nearly all over black.
 *    Its darkness d (0..1000 per mille of the calibrated white..black span)
 *    against LF_EDGE_TARGET_PM is the error:
 *      d > target  car drifted left   -> steer right (steer > 0)
 *      d < target  car drifted right  -> steer left  (steer < 0)
 *    Steering sign: steer > 0 = steer right (data_repr.h, task_motion.c).
 *    RIGHT is never on the line while CENTRE tracks the edge (it sits 26 mm
 *    to the right, the line is 18 mm wide; CENTRE has left the line before
 *    RIGHT can reach it), so any darkness under RIGHT means something:
 *      - CENTRE on the line too: a cross line arriving -> hold the steering;
 *        both solid black for 3 samples -> JUNCTION;
 *      - CENTRE off the line: the car overshot left across the whole line
 *        -> recovery steering to the right (LF_RECOVER_Q7).
 *    RIGHT is also the side memory: whichever sensor saw the line last
 *    names the side it was lost on (update_side() in lf_core.c).
 *
 *  Junction post distance: Motion turns by pivoting about the axle
 *  midpoint, F = LF_D_PIVOT_MM behind the sensors.  So JUNCTION is posted
 *  only after the car has travelled about F past the cross line (encoder
 *  counts from lf_link.h): if the Vehicle Controller then turns, the axle
 *  is on the cross line and the sensors land on the new branch.  Until the
 *  post the core drives straight over the cross line, then tracks the line
 *  if it goes on (straight if not: a T junction), and never reports the
 *  line ending.  Simulated at 200 mm/s (4 Nov 2026): left and right turns
 *  end on the branch for a real F of 95..145 mm, the pivot centre landing
 *  -20..+30 mm from the cross line's centre (+5 at the measured 120).  In
 *  the model the turn overruns, so most turns go LINE_LOST and search
 *  before finding the branch; the real overrun is not measured yet.
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

#include "config/thresholds.h"
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
/** CENTRE's target darkness: just starting to feel the line's right edge,
 *  on the black side of it (LF-3.3).  The sensor spot is only ~2.4 mm wide
 *  and the motors answer ~110 ms after a command, so a target on the middle
 *  of the edge puts CENTRE on white for longer than LF_T_LOST_MS on every
 *  correction.  On the black side the car has the line's width to spare. */
#define LF_EDGE_TARGET_PM       (900U)

/** Mounting criteria for the two-pose calibration, from the car (19 Nov
 *  2026, sensors ~10 mm above the mat): white 116-200, black 1888-2124
 *  (CENTRE and RIGHT, line middle); with the sensors lowered, black fell to
 *  700-1040 with white unchanged, and pose A failed.  Below these a sensor is
 *  LF_SENS_WEAK: re-mount it, do not drive on it. TUNE */
#define LF_CAL_BLACK_MIN_RAW    (1500U)
#define LF_CAL_WHITE_MAX_RAW    (400U)
#define LF_CAL_SPAN_GOOD_RAW    (1200U)

/** Calibration refused if black - white is smaller than this (raw counts;
 *  same contrast rule as S3-U09, about 10 % of the 12-bit scale). */
#define LF_MIN_SPAN_RAW         (400U)

/** Steering per LF_KP_SPAN_PM of darkness away from the target, Q7.  Low on
 *  purpose (LF-3.3): the sensors sit 120 mm ahead of the axle, so a turn
 *  swings them sideways at once, and steering at 64 oscillated across the
 *  whole line on the car.  At 24 the host model settles with no oscillation;
 *  Kp 16 failed the 15 degree start when this was tuned.  TUNE */
#define LF_KP_Q7                (24)
/** Darkness span that LF_KP_Q7 applies to. */
#define LF_KP_SPAN_PM           (500)
/** Full steering, Q7 (DREP_STEER_Q7_MAX). */
#define LF_STEER_FULL_Q7        (127)
/** Steering while CENTRE is off the line and the line is not lost yet, Q7.
 *  It carries on from lf_edge_steer() at the LIGHT limit (-31), so leaving
 *  the edge is no jump.  At 127 it crossed the 18 mm line in ~100 ms. TUNE */
#define LF_RECOVER_Q7           (32)
/** Recovery steering grows by this much per sample while CENTRE stays off the
 *  line, Q7: a brief slip stays gentle, a real one gets firmer, up to
 *  LF_STEER_FULL_Q7.  At 1 it adds 20 before LF_T_LOST_MS ends it.  It starts
 *  again from LF_RECOVER_Q7 when the side changes: carrying a value built up
 *  the wrong way over to the right side overshot in the host model. TUNE */
#define LF_RECOVER_RAMP_Q7      (1)
/** Steering while searching for a lost line, Q7.  Enough to come back within
 *  LF_T_SEARCH_MS; at 127 the car reached the line too fast to stay on it. TUNE */
#define LF_SEARCH_Q7            (64)
/** Gentle right turn while looking for the line after a u-turn, Q7. TUNE */
#define LF_ACQUIRE_STEER_Q7     (32)
/** Steering held after a u-turn from CENTRE's first solid black, Q7, in
 *  place of lf_edge_steer() (which gives only +4 on solid black).  The car
 *  reaches the line still angled across it, and the sensors sit 120 mm ahead
 *  of the axle: at +4 CENTRE slid on over the line's left edge, the unstable
 *  one, and the core then steered the wrong way (S3-U14 u-turn, 5 Nov 2026,
 *  CENTRE on 19 of 88 polls).  Simulated: 12..16 all work, 18 and 32
 *  overshoot.  Left and right turns do not use it. TUNE */
#define LF_UTURN_HOLD_Q7        (14)
/** The hold ends when CENTRE has read at least this (the line's body) and
 *  then falls below LF_EDGE_TARGET_PM: it has crossed the body to the right
 *  edge, where edge tracking is stable. */
#define LF_UTURN_HOLD_FULL_PM   (950U)
/** The hold ends after this much travel at the latest. TUNE */
#define LF_UTURN_HOLD_MM        (300U)
/** The same in encoder counts. */
#define LF_UTURN_HOLD_CNT       ((LF_UTURN_HOLD_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)

/* EXPERIMENT C (branch buddy3/ab-rescue, 16 Nov 2026), NOT production: RIGHT
 * keeps the line.  On a right bend the line slides right under the car:
 * CENTRE goes white first, and RIGHT reaches the line a few mm later.  On
 * the car (S3-U17 A/B) LINE_LOST was declared on a sample where RIGHT read
 * black in 8 of 10 drives: T_lost ended before RIGHT's 3-sample zone filter
 * confirmed it, and in LINE_LOST a line under RIGHT did nothing.  C: RIGHT
 * not LIGHT by this sample's own reading for LF_RESCUE_N consecutive samples
 * stops the lost count, and in LINE_LOST returns to ON_LINE (side RIGHT, so
 * the recovery steers right from LF_RECOVER_Q7).  Two samples, not one, so a
 * single noisy reading cannot keep the car going.  A/B on the car decides. TUNE */
#define LF_RESCUE_N             (2U)

/* EXPERIMENT T80 (branch buddy3/ab-bend-150, 17 Nov 2026, NOT for merge):
 * bend memory.  C at 150 mm/s, 16 Nov: RIGHT rescued the line on a right bend
 * and the recovery ramped to +105..+127, CENTRE came back onto the body, and
 * the edge steering dropped to +4 at once: the bend slid out again.
 *   On:  C's rescue moment (CENTRE LIGHT, RIGHT not LIGHT for LF_RESCUE_N
 *        samples, ON_LINE or LINE_LOST); the floor is the steering then.
 *   Use: when CENTRE is back with d_c >= LF_EDGE_TARGET_PM, and only if the
 *        floor at that moment is >= LF_BEND_MIN_Q7 (a long rescue = a bend;
 *        a short one is a drift): steering at least the floor, the floor
 *        easing by 1 per sample down to LF_BEND_FLOOR_Q7.  Below the target
 *        (at the right edge) the edge steering is unchanged.
 *   Off: LF_BEND_SETTLE_N samples at the edge (d_c < target, not LIGHT),
 *        LF_BEND_MM of travel, or any junction, turn, avoidance or stop. */
#define LF_BEND_MIN_Q7          (80)
#define LF_BEND_FLOOR_Q7        (32)
#define LF_BEND_SETTLE_N        (20U)
#define LF_BEND_MM              (300U)
#define LF_BEND_CNT             ((LF_BEND_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)

/** Search the remembered side this long, then the other side until
 *  LF_T_SEARCH_MS. TUNE */
#define LF_SEARCH_SWAP_MS       (700U)
/** LINE_LOST posts nothing for this long, so Motion's hold timeout
 *  (MOTION_T_HOLD_MS) stops the car before the search starts. */
#define LF_T_STOP_MS            (MOTION_T_HOLD_MS + MOTION_PERIOD_MS)

/** LINE_LOST needs both sensors LIGHT for LF_T_LOST_MS (design.md T_lost)
 *  AND for this much forward travel: the distance T_lost covered at
 *  200 mm/s, the speed the recovery was tuned at (LF-3.3).  A time alone
 *  covers less line as the car slows, and ended corrections that were still
 *  coming back: launch LINE_LOST in 3 of 5 drives at 150 mm/s against 1 of 5
 *  at 200 (S3-U17, 16 Nov 2026); host model, straights from offset starts,
 *  1 of 27 runs lost at 150 mm/s and 9 of 27 at 100, none with this rule.
 *  At 200 mm/s and above the time is the longer, so nothing changes there.
 *  Deviation from design.md T_lost: ARCHITECTURE.md D13 (proposed). TUNE */
#define LF_D_LOST_MM            (20U)
/** The same in encoder counts. */
#define LF_D_LOST_CNT           ((LF_D_LOST_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
/** LINE_LOST after this long with both LIGHT whatever the travel: a stalled
 *  or very slow car (20 mm at 50 mm/s). */
#define LF_T_LOST_MAX_MS        (400U)

/** A search that follows a genuine loss while driving the line (LINE_LOST
 *  from ON_LINE, or going straight on past a junction) ends in TRACK_LOST
 *  after this much of its own forward travel, if T_search has not ended it
 *  first.  The VC drives the search's arcs at the cruise speed, so T_search
 *  alone let the car wander: host model, tight bends at 150 mm/s, the car
 *  stopped 15-36 cm from the line; with 150 mm mostly within 5 cm, and the
 *  searches that found the line (gentle bends) found it well inside 150 mm.
 *  150 mm of the search's 64 arc is about 75 degrees of turn, past a line
 *  lost to either side.  Not used after a junction turn's look, at run
 *  start, or after an avoidance hand-back: those keep T_search alone.
 *  Shortens design.md T_search: ARCHITECTURE.md D13 (proposed). TUNE */
#define LF_D_SEARCH_MAX_MM      (150U)
/** The same in encoder counts. */
#define LF_D_SEARCH_MAX_CNT     ((LF_D_SEARCH_MAX_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)

/** Sensing point of the line sensors to the wheel axle, which is Motion's
 *  pivot centre for a turn (task_motion.c TURN_L/TURN_R/UTURN drive the
 *  wheels in opposite directions).  Ruler on the car, 4 Nov 2026. */
#define LF_D_PIVOT_MM           (120U)
/** JUNCTION is posted this much before the axle reaches the cross line,
 *  for the car's travel while the turn command takes effect (Motion's
 *  next 20 ms period, then the wheels slowing). TUNE */
#define LF_D_JCT_LEAD_MM        (0U)
/** Travel from junction detection to the JUNCTION post. */
#define LF_D_JCT_POST_MM        (LF_D_PIVOT_MM - LF_D_JCT_LEAD_MM)
/** The same in encoder counts: ENC_QUAD_MULT x ENC_CPR counts per wheel
 *  turn of WHEEL_CIRC_MM (config/thresholds.h). */
#define LF_D_JCT_POST_CNT       ((LF_D_JCT_POST_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
/** Past a cross line with both sensors white and no sign of the line going
 *  on, steer left for this much travel from detection: CENTRE rode the right
 *  edge, so if the line goes on it is to the left.  Without this, sensors
 *  leaving the cross in the same sample read as a T junction and the car
 *  drove the whole post distance at steer 0 (S3-U15, 4 Nov).  Then straight,
 *  so a real T junction is left alone: in the host model 40 mm at this
 *  steering begins to cost left turns at a T. TUNE */
#define LF_D_JCT_LOOK_MM        (30U)
/** The same in encoder counts. */
#define LF_D_JCT_LOOK_CNT       ((LF_D_JCT_LOOK_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
/** A pivot counts as past halfway from here: left minus right counts at 45
 *  degrees, 2 x pi x 114 mm x 45 / 360 x 2464 / 207.  Before this the sensors
 *  are crossing the line the car came along, not the new one. */
#define LF_PIVOT_HALF_CNT       (1065U)
/** The same for a u-turn, at 135 degrees: the cross arm goes by at 90, so only
 *  the last quarter of the pivot can show the line the car came along. */
#define LF_PIVOT_UTURN_CNT      (3U * LF_PIVOT_HALF_CNT)
/** JUNCTION is posted after this long even if the odometer has not moved
 *  (an encoder fault): the post travel's time at the cruise speed plus
 *  300 ms, so it follows a cruise change (900 ms at 200 mm/s, the value
 *  LF-3 used; 1100 ms at 150; 1500 ms at 100). */
#define LF_T_JCT_MAX_MS         (((LF_D_JCT_POST_MM * 100U) / MOTION_SPEED_CRUISE_CMPS) + 300U)

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

/** One sensor's verdict from the two-pose calibration (lf_cal_from_two_poses). */
typedef enum
{
    LF_SENS_OK = 0,
    LF_SENS_DISCONNECTED,   /**< at an ADC rail, or no samples, in either pose */
    LF_SENS_NOISY,          /**< spread above ADC_NOISE_MAX_RAW in either pose */
    LF_SENS_BLIND,          /**< black not >= white + LF_MIN_SPAN_RAW: cannot tell them apart */
    LF_SENS_WEAK            /**< tells them apart, but below the mounting criteria */
} lf_sens_t;

/** The two-pose calibration: each sensor's own white and black, and verdicts. */
typedef struct
{
    lf_cal_t  cal;          /**< each channel from its own two readings */
    lf_sens_t c;            /**< CENTRE (straddle: LEFT) */
    lf_sens_t r;            /**< RIGHT */
} lf_cal2_t;

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
    uint32_t       fwd_cnt;         /**< forward travel, encoder counts, free-running (lf_link) */
    int32_t        yaw_cnt;         /**< left minus right encoder counts, wraps (lf_link) */
    bool           post_slot;       /**< the last sample before Motion reads its queue (lf_link_post_slot) */
} lf_in_t;

/** Result of one sample. */
typedef struct
{
    lf_state_t state;       /**< state after this sample */
    bool       post_steer;  /**< post steer_q7 now (in the post slot, or on state entry) */
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
    uint32_t       lost_n;          /**< consecutive samples with both LIGHT */
    uint32_t       lost_cnt;        /**< forward counts over those samples */
    uint32_t       prev_fwd;        /**< fwd_cnt at the previous sample */
    bool           lost_driving;    /**< this LINE_LOST came while driving the line */
    bool           srch_capped;     /**< this search ends at LF_D_SEARCH_MAX_CNT */
    uint32_t       srch_cnt;        /**< its forward counts so far */
    uint32_t       timer_n;         /**< samples spent in the current state/phase */
    bool           post_now;        /**< post at the next sample that may post (state entry) */
    int32_t        steer_q7;        /**< last computed steering */
    int32_t        recover_q7;      /**< recovery steering, grows while CENTRE is off the line */
    lf_side_t      recover_side;    /**< the side recover_q7 has been growing towards */
    lf_manoeuvre_t man_seen;        /**< manoeuvre seen in JUNCTION (NONE = straight) */
    bool           man_done;        /**< that manoeuvre has finished */
    bool           jct_pending;     /**< junction detected, JUNCTION not yet posted */
    bool           jct_line_ahead;  /**< the line was seen going on past the cross line */
    uint32_t       jct_cnt0;        /**< fwd_cnt when the junction was detected */
    bool           piv_on;          /**< a pivot is being watched */
    int32_t        piv_yaw0;        /**< yaw_cnt when it started */
    bool           piv_late;        /**< it got past LF_PIVOT_HALF_CNT (u-turn: LF_PIVOT_UTURN_CNT) */
    bool           piv_left;        /**< it turned left */
    bool           piv_c_seen;      /**< CENTRE saw a line after that */
    bool           piv_r_seen;      /**< RIGHT saw a line after that */
    bool           hold_on;         /**< the hold after a u-turn is steering (LF_UTURN_HOLD_Q7) */
    bool           hold_full;       /**< CENTRE has been on the line's body during it */
    uint32_t       hold_cnt0;       /**< fwd_cnt when it started */
    bool           bend_on;         /**< T80: bend memory active */
    bool           bend_regain;     /**< T80: CENTRE is back, the floor is fixed */
    int32_t        bend_q7;         /**< T80: steering floor */
    uint32_t       bend_cnt0;       /**< T80: fwd_cnt at entry */
    uint32_t       bend_edge_n;     /**< T80: samples settled at the edge */
    uint32_t       r_dark_n;        /**< C: consecutive samples RIGHT is not LIGHT (its own reading; saturates) */
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
 * Two-pose calibration, each sensor independently.  Pose A: CENTRE over the
 * line's middle, RIGHT over white; pose B: RIGHT over the line's middle,
 * CENTRE over white (straddle mounting: shift the car so the named sensor
 * is over the line).  p_c_black / p_r_white are pose A's statistics,
 * p_c_white / p_r_black pose B's.  Each channel gets its own white and black
 * and a verdict (lf_sens_t); *p_out is always filled.  Returns true only if
 * both sensors are LF_SENS_OK.
 */
bool lf_cal_from_two_poses(lf_adc_stats_t const *p_c_black, lf_adc_stats_t const *p_r_white,
                           lf_adc_stats_t const *p_c_white, lf_adc_stats_t const *p_r_black,
                           lf_cal2_t *p_out);

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
 * LF_KP_Q7 * (d - LF_EDGE_TARGET_PM) / LF_KP_SPAN_PM, clamped to
 * +-LF_STEER_FULL_Q7.
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
