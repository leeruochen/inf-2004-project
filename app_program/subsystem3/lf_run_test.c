/*
 *  lf_run_test.c
 *  Subsystem 3 -- Line-Following task test on the track (S3-U12, S3-U13,
 *  S3-U14, S3-U15, S3-U17), APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel and pure project headers only (rule R1).
 *
 *  What it shows: the real Line-Following task driving the car on the
 *  track, with the real Vehicle Controller and Motion, at the build's
 *  cruise speed (MOTION_SPEED_CRUISE_CMPS; the default bring-up speed set
 *  gives 200 mm/s).  The test task (priority 11) never reads the ADC: it
 *  reads the task's status (lf_task_get_status), Motion's state (IF-08)
 *  and the VC's state, and plays the operator's remote START / STOP /
 *  RESET through MBF_NAV_IN as Telemetry would.
 *
 *  The motors are live (on the Robo Pico, USB alone powers them).  Every
 *  drive is bounded: the test sends STOP when the wheel travel or the time
 *  reaches the drive's limit, or when the VC hands the car to obstacle
 *  avoidance; Safety's collision brake stays active.  Between drives the
 *  VC is READY (motors stopped) and the operator places the car by hand.
 *
 *  LF-3.3 (S3-U13): LF2_STRAIGHT_ONLY runs the static poses and then the
 *  straight drive LF2_STRAIGHT_RUNS times, nothing else: straight tracking
 *  is proven on the car before recovery, junctions or curves.  Every drive
 *  is recorded every 20 ms into RAM and printed after the car has stopped:
 *  the console ring holds 2 KB and drops lines when full, so 20 ms lines
 *  printed live would lose data and load the system during the drive.
 *
 *  LF-3.3 (S3-U15): LF2_JMAT_ONLY runs the static poses and then the
 *  junction drive on the standalone junction mat, LF2_JMAT_RUNS times, in
 *  place of the straight drives: the cross is detected, JUNCTION comes F
 *  later, the car goes on straight, and the test sends STOP at once.  The
 *  line ends only 140 mm past the cross, so the stop follows the event, not
 *  a distance.  Turns are not tested here: the branches are too short.
 *
 *  LF-3.4 (S3-U14): LF2_TURN_ONLY runs the static poses and then a junction
 *  turn on the track, LF2_TURN_RUNS times: the test injects the turn command
 *  a barcode would give (drep_nav_barcode) when the cross is detected, the
 *  VC pivots at the JUNCTION post, and the core finds the branch and follows
 *  it for LF2_TURN_BRANCH_MM.  No barcode is read.
 *
 *  Curves (S3-U17, configure -DS3_CURVE_TEST=ON): in place of the turns,
 *  LF2_CURVE_GROUPS groups of LF2_CURVE_RUNS drives on a curve piece (left
 *  curve, right curve, then an S-curve if there is one).  Each drive starts
 *  on the approach straight and stops by distance on the exit straight;
 *  nothing is injected.  It counts LINE_LOST entries and junction
 *  detections (a curve has no cross, so any is false) and judges the exit
 *  straight.  The steering is today's: this measures it, it changes nothing.
 *
 *  Guides: .agents/tasks/s3-lf33-test.md, s3-u14-test.md, s3-u15-test.md.  Expected values come from
 *  config/timing.h, lf_core.h and design.md 3.2, typed in: 5 ms samples
 *  (200 per second), one steering post per 20 ms Motion period,
 *  LINE_LOST -> 120 ms stop -> 2 s search -> TRACK_LOST (2120 ms),
 *  steer > 0 = steer right (data_repr.h).
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/subsystem2.h"   /* motion_get_state, vc_get_system_state, VC_SYS_* */
#include "config/thresholds.h"          /* MOTION_SPEED_CRUISE_CMPS, ENC_*, WHEEL_CIRC_MM */
#include "config/timing.h"              /* NAV_MBF_SEND_TMO_MS */
#include "lf_core.h"                    /* lf_norm, LF_DARK_MIN_PM, LF_LIGHT_MAX_PM */
#include "lf_run_test.h"
#include "task_lf.h"

#define LF2_SECOND_MS           (1000U)
#define LF2_POLL_MS             (10U)
/** A "[LF] d" line every this many polls (100 ms) while driving. */
#define LF2_LOG_EVERY_POLLS     (10U)
/** Seconds to place the car before a drive or a static measurement. */
#define LF2_PLACE_S             (10U)
/** Static measurement time per pose. */
#define LF2_STATIC_MS           (3000U)
/** Samples in 1 s at 5 ms, +-5 for the start and end of the window. */
#define LF2_RELEASES_1S_MIN     (195U)
#define LF2_RELEASES_1S_MAX     (205U)
/** Wait this long at most for the task's first sample before (a).  The boot
 *  calibration (32 samples, 160 ms) comes first and counts no sample; when
 *  only this subsystem's tests run (APP_TEST_ONLY=3) the test starts inside
 *  it and (a) read 168 = 200 - 32 (16 Nov 2026).  The limits are unchanged. */
#define LF2_BOOT_WAIT_MS        (2000U)
/** Steering post count window on the straight (ms after START). */
#define LF2_WIN_FROM_MS         (500U)
#define LF2_WIN_TO_MS           (2500U)
/** LINE_LOST entry to TRACK_LOST: 120 ms stop + 2000 ms search, +-1 sample
 *  and the kernel tick. */
#define LF2_LOST_TO_TRACK_MIN   (2110U)
#define LF2_LOST_TO_TRACK_MAX   (2135U)
/** First steering from the start pose (CENTRE on the line body): Kp 24 x
 *  (1000 - 900) / 500 = +4 (lf_core.h): any right steering passes. */
#define LF2_STEER_BODY_MIN      (1)
/** "Settled": CENTRE first feels the right edge (below 950 per mille, not
 *  on white) within this wheel travel of START.  The host model takes 132
 *  mm from the line's middle and 192 mm from 5 mm left of it. */
#define LF2_SETTLE_PM_LO        (250U)
#define LF2_SETTLE_PM_HI        (950U)
#define LF2_SETTLE_MAX_MM       (500U)
/** Stable tracking: at least this share of polls ON_LINE with CENTRE on
 *  the line (LF_LIGHT_MAX_PM or darker).  The edge is only ~2.4 mm wide, so
 *  a band inside it is not a fair measure: CENTRE sits at ~880 pm when
 *  settled, on the black side by design. */
#define LF2_BAND_MIN_PCT        (90U)
/** Pose A: CENTRE on the line's middle must read at least this, or the boot
 *  calibration is too high for the 900 pm target (see the guide). */
#define LF2_POSE_A_MIN_PM       (950U)
/** Pose A sets the calibration, so it needs a limit of its own: grey would
 *  pass a check against itself.  The same black limit as the two-pose
 *  mounting criteria (lf_core.h): 1500 here refused the straddle mount's
 *  1250-1430, which the follower handles (host model). */
#define LF2_POSE_A_MIN_RAW      (LF_CAL_BLACK_MIN_RAW)
/** Tries at pose A before the test gives up. */
#define LF2_POSE_A_TRIES        (3U)
/** LF-3.3: only the straight drive, this many times. */
#define LF2_STRAIGHT_ONLY       (1U)
#define LF2_STRAIGHT_RUNS       (3U)
/** S3-U17 curves in place of the S3-U14 turns: -DS3_CURVE_TEST=ON
 *  (sources.cmake).  0 = the S3-U14 U-turn test, as before. */
#ifndef S3_CURVE_TEST
#define S3_CURVE_TEST           (0)
#endif
/** Controlled moving drives for team tests (-DS3_DRIVE_MM=<mm>, sources.cmake;
 *  0 = off): LF2_DRIVE_RUNS drives of up to S3_DRIVE_MM of wheel travel on any
 *  line, each from the start guard, started and stopped through MBF_NAV_IN as
 *  Telemetry would (REMOTE_RESET/START/STOP), so the Vehicle Controller owns
 *  every motor command.  Obstacle avoidance may take over and hand back: the
 *  drive goes on (the other modes STOP there).  No layout is assumed: the
 *  result is how far it got and why it stopped. */
#ifndef S3_DRIVE_MM
#define S3_DRIVE_MM             (0)
#endif
#define LF2_DRIVE_ONLY          ((S3_DRIVE_MM > 0) ? 1U : 0U)
#define LF2_DRIVE_RUNS          (3U)
#define LF2_DRIVE_EXTRA_MS      (20000U)    /* bypasses, searches and pivots on top of the cruise time */
#if (S3_DRIVE_MM > 0) && S3_CURVE_TEST
#error "lf_run_test.c: S3_DRIVE_MM and S3_CURVE_TEST are exclusive (sources.cmake)"
#endif
#if (S3_DRIVE_MM > 0) && ((S3_DRIVE_MM < 300) || (S3_DRIVE_MM > 20000))
#error "lf_run_test.c: S3_DRIVE_MM is 300..20000 mm"
#endif
/** EXPERIMENT straddle build (sources.cmake -DS3_LF_STRADDLE=ON): the
 *  sensors one each side of the line; placement and the start check follow. */
#ifndef S3_LF_STRADDLE
#define S3_LF_STRADDLE          (0)
#endif
#if S3_LF_STRADDLE
#define LF2_MODE_TEXT           "STRADDLE (sensors one each side of the line, 26 mm apart)"
#define LF2_START_TEXT          "the line centred between the two sensors, both on white, square to the line"
#define LF2_POSE_A_HINT         " (straddle mount: slide the car about 13 mm to the RIGHT)"
#define LF2_POSE_B_HINT         " (straddle mount: slide the car about 13 mm to the LEFT)"
#else
#define LF2_MODE_TEXT           "EDGE (CENTRE on the line's right edge, RIGHT on white)"
#define LF2_START_TEXT          "CENTRE on black, RIGHT on white, square to the line"
#define LF2_POSE_A_HINT         ""
#define LF2_POSE_B_HINT         ""
#endif

/** Test ID of this build's checks. */
#if (S3_DRIVE_MM > 0)
#define LF2_ID                  "S3-U19"
#elif S3_CURVE_TEST
#define LF2_ID                  "S3-U17"
#else
#define LF2_ID                  "S3-U14"
#endif
/** Trace: one row per this many polls (20 ms), and how it is printed: a
 *  burst of rows, then a pause so the console drains the 2 KB ring. */
#define LF2_TRACE_EVERY_POLLS   (2U)
#if S3_CURVE_TEST || (S3_DRIVE_MM > 0)
/* 8 s: a whole curve drive.  Drive mode keeps the LAST 8 s (a ring), so a
 * failure far along an unknown track is in the dump with the approach to it. */
#define LF2_TRACE_ROWS          (400U)
#else
#define LF2_TRACE_ROWS          (256U)
#endif
#define LF2_TRACE_BURST         (8U)
#define LF2_TRACE_PAUSE_MS      (100U)
/** Recovery drives: ON_LINE within this time of START. */
#define LF2_RECOVER_MAX_MS      (1000U)
/** After a drive ends: the wheels must not move between these times. */
#define LF2_STILL_FROM_MS       (300U)
#define LF2_STILL_TO_MS         (500U)
/** Cruise speed of this build, mm/s. */
#define LF2_CRUISE_MMPS         ((uint32_t)MOTION_SPEED_CRUISE_CMPS * 10U)
/** S3-U15, the junction mat: this replaces the straight drives. */
#define LF2_JMAT_ONLY           (0U)
#define LF2_JMAT_RUNS           (3U)
/** Backstop if no JUNCTION comes.  The car's own logs put JUNCTION at up to
 *  420 mm of wheel travel from the mat's start and the line end at 437. */
#define LF2_JMAT_LIMIT_MM       (430U)
/** LINE_LOST past this travel: the line has run out, so stop rather than
 *  search on a small mat.  Before it, the test lets the core recover. */
#define LF2_JMAT_LOST_MM        (200U)
/** Where detection may fall: ruler 250-270 mm, wheel travel read 1.00 to
 *  1.15 of that, less the few mm before the cross line's centre. */
#define LF2_JMAT_DET_MIN_MM     (230U)
#define LF2_JMAT_DET_MAX_MM     (320U)
/** Detection to JUNCTION: LF_D_JCT_POST_MM, +-2 polls of a published count.
 *  The time checks that the odometer, not the time cap, ended the travel. */
/* Relative to the configured post distance (120 mm: 110..135) and the cruise
 * (120 mm at 200 mm/s = 600 ms: 450..800), so a build with another mounting
 * distance (-DLF_D_PIVOT_MM) or speed keeps the same meaning. */
#define LF2_JMAT_DELAY_MIN_MM   (LF_D_JCT_POST_MM - 10U)
#define LF2_JMAT_DELAY_MAX_MM   (LF_D_JCT_POST_MM + 15U)
#define LF2_JMAT_NOMINAL_MS     ((LF_D_JCT_POST_MM * 1000U) / LF2_CRUISE_MMPS)
#define LF2_JMAT_DELAY_MIN_MS   ((LF2_JMAT_NOMINAL_MS * 3U) / 4U)
#define LF2_JMAT_DELAY_MAX_MS   ((LF2_JMAT_NOMINAL_MS * 4U) / 3U)
/** Past the cross line: CENTRE is judged from this far after detection. */
#define LF2_JMAT_TAIL_FROM_MM   (60U)
/** JUNCTION to STOP at most, if ON_LINE does not come first. */
#define LF2_JMAT_THROUGH_MS     (40U)
/** JUNCTION to standing still at most, wheel travel. */
#define LF2_JMAT_AFTER_MAX_MM   (35U)
/** Start guard: a jmat drive starts only with CENTRE steady on black and
 *  RIGHT on white.  CENTRE must read dark enough for the core to start
 *  tracking, with a margin.  It must not read far above calibrated black:
 *  that is a car still in the hand. */
#define LF2_JMAT_BLACK_MIN_PM   (LF_DARK_MIN_PM + 50U)
#define LF2_JMAT_LIFTED_PCT     (125U)
/** Good readings needed before the warning, then from the warning to START.
 *  A bad reading in either starts the count again. */
#define LF2_JMAT_STEADY_S       (3U)
#define LF2_JMAT_HANDS_OFF_S    (3U)
#define LF2_JMAT_LIVE_MS        (500U)
/** Give up on a drive after this long: it is not started, and fails. */
#define LF2_JMAT_WAIT_S         (180U)

/** S3-U14: a junction turn, in place of the straight and mat drives.  The
 *  command is what a barcode would post: left and right passed on 5 Nov, now
 *  the U-turn. */
#if S3_CURVE_TEST || (S3_DRIVE_MM > 0)
#define LF2_TURN_ONLY           (0U)
#else
#define LF2_TURN_ONLY           (1U)
#endif
#define LF2_TURN_RUNS           (3U)
#define LF2_TURN_CMD            DREP_BC_CMD_UTURN
#define LF2_TURN_VC_STATE       VC_SYS_NAV_U_TURN
#define LF2_TURN_NAME           "U-turn:"
/** STOP this far along the branch, from the first ON_LINE after the turn. */
#define LF2_TURN_BRANCH_MM      (300U)
/** No cross by this much wheel travel: STOP.  The start is 300 mm before it. */
#define LF2_TURN_NO_CROSS_MM    (420U)
/** Whole drive limit: approach, pivot, search and branch, with room. */
#define LF2_TURN_MS             (9000U)
/** The turn as the encoders saw it: a wide sanity window, the real angle is
 *  printed and judged by eye and against NFR4 (+-5 degrees). */
#define LF2_TURN_DEG_MIN        (165U)
#define LF2_TURN_DEG_MAX        (200U)
/** The core must be tracking within T_uturn + the stop + T_search. */
#define LF2_TURN_FIND_MAX_MS    (LF_T_UTURN_MS + LF_T_STOP_MS + LF_T_SEARCH_MS)
/** The branch is judged from this far along it, once the car has settled. */
#define LF2_TURN_TAIL_FROM_MM   (100U)
/** Effective track x pi, mm: Motion's turn geometry (MOTION_EFF_TRACK_MM 114). */
#define LF2_PI_TRACK_MM         (358U)

/** S3-U17 curve piece.  SET FROM THE PIECE before the build (H0): the drive
 *  starts LF2_CURVE_START_MM before the curve and STOPs at LF2_CURVE_MM of
 *  wheel travel, which must end at least 100 mm before the end of the exit
 *  straight.  The exit straight is judged over the last LF2_CURVE_EXIT_MM
 *  before the STOP.  TUNE per piece */
#define LF2_CURVE_ONLY          ((S3_CURVE_TEST != 0) ? 1U : 0U)
/* 16 Nov 2026, A/B on one 90 deg RIGHT bend (sample mat, 300 mm tiles; one
 * curved tile, at least two straight tiles after it): start mark one tile
 * before the curved tile.  The line ends about 1.1 m from the mark; at 900 mm
 * of wheel travel the sensors are about 0.15-0.25 m short of the end (on a
 * bend the wheels travel less than the line: the sensors run 120 mm ahead of
 * the axle), and the last 200 mm before the STOP is well past the bend.  A
 * first LINE_LOST below about 650 mm of travel is at the bend.  Only the RIGHT
 * group runs: the bend turns right from the only start. */
#define LF2_CURVE_FIRST         (1U)        /* 0 LEFT, 1 RIGHT, 2 S-curve */
#define LF2_CURVE_GROUPS        (1U)
#define LF2_CURVE_RUNS          (5U)
#define LF2_CURVE_START_MM      (300U)
#define LF2_CURVE_MM            (900U)
#define LF2_CURVE_EXIT_MM       (200U)
/** |steer| at or above this counts as "hard" steering: recovery past its
 *  first 100 ms, the search, or more (lf_core.h: recovery 32 + ramp, search 64). */
#define LF2_CURVE_HARD_Q7       (60)

/** Drive limits (wheel travel).  The time limit is the travel at cruise
 *  plus 1.5 s. */
/* INTERIM S3-U13i: 200 stops the car before the track's cross line, 250 mm
 * past the start.  The full S3-U13 needs 900 on >= 1.2 m of clear line. */
#define LF2_STRAIGHT_MM         (200U)
#define LF2_RECOVER_MM          (500U)
#define LF2_JUNCTION_MM         (800U)
#define LF2_END_MM              (1200U)
#define LF2_END_MS              (7000U)
#define LF2_MS_FOR(mm)          ((((mm) * 1000U) / LF2_CRUISE_MMPS) + 1500U)
#define LF2_NEVER               (0xFFFFFFFFU)

#if (MOTION_SPEED_CRUISE_CMPS == 0U)
#error "lf_run_test.c: MOTION_SPEED_CRUISE_CMPS must not be 0"
#endif

_Static_assert((LF2_JMAT_DELAY_MIN_MM < LF_D_JCT_POST_MM) && (LF_D_JCT_POST_MM < LF2_JMAT_DELAY_MAX_MM),
               "the (j2) window brackets the junction post distance");
_Static_assert((LF2_CURVE_EXIT_MM + LF2_CURVE_START_MM) < LF2_CURVE_MM,
               "the exit window starts after the curve's start");
_Static_assert((LF2_CURVE_GROUPS >= 1U) && ((LF2_CURVE_FIRST + LF2_CURVE_GROUPS) <= 3U), "left, right, S");
_Static_assert(LF2_CURVE_MM < 0xFFFFU, "trace mm is uint16_t");
_Static_assert((LF2_MS_FOR((uint32_t)S3_DRIVE_MM) + LF2_DRIVE_EXTRA_MS) < 0xFFFFU,
               "a drive's time limit fits the trace's uint16_t t_ms");

/** Why a drive ended. */
typedef enum
{
    LF2_END_DIST = 0,       /**< wheel travel limit: the test sent STOP */
    LF2_END_TIME,           /**< time limit: the test sent STOP */
    LF2_END_TRACK_LOST,     /**< TRACK_LOST: the VC stopped the car */
    LF2_END_FAULT,          /**< Safety latched a fault: braked */
    LF2_END_AVOID,          /**< obstacle avoidance took over: the test sent STOP */
    LF2_END_VC_STOP,        /**< the VC went READY by itself */
    LF2_END_NOT_READY,      /**< the VC was not READY: no START sent */
    LF2_END_JCT,            /**< JUNCTION came: the test sent STOP */
    LF2_END_LOST,           /**< LINE_LOST on the mat: the test sent STOP */
    LF2_END_NOT_PLACED,     /**< the start guard never passed: no START sent */
    LF2_END_BRANCH,         /**< followed the branch far enough: the test sent STOP */
    LF2_END_NO_CROSS,       /**< no cross where it should be: the test sent STOP */
    LF2_END_COUNT
} lf2_end_t;

static char const * const k_end_name[LF2_END_COUNT] =
{
    "distance limit", "time limit", "TRACK_LOST", "FAULT", "avoidance",
    "VC stopped", "VC not READY", "after JUNCTION", "LINE_LOST", "not placed",
    "along the branch", "no cross"
};

/** One bounded drive. */
typedef struct
{
    char const *p_tag;      /**< short name in the log */
    char const *p_place;    /**< how to place the car */
    uint32_t    max_mm;     /**< wheel travel limit */
    uint32_t    max_ms;     /**< time limit */
    bool        jmat;       /**< junction mat: STOP on JUNCTION or LINE_LOST */
    bool        turn;       /**< junction turn: inject the command, follow the branch */
    bool        curve;      /**< S3-U17 curve: start guard, count losses and detections */
} lf2_drive_t;

/** One row of the 20 ms trace. */
typedef struct
{
    uint16_t t_ms;      /**< since START */
    uint16_t mm;        /**< mean wheel travel since START */
    uint16_t raw_c;
    uint16_t raw_r;
    int32_t  cnt_l;     /**< left wheel counts since START */
    int32_t  cnt_r;     /**< right wheel counts since START */
    int8_t   steer_q7;  /**< last steering posted */
    uint8_t  state;     /**< lf_state_t */
} lf2_trace_row_t;

/** What one drive did. */
typedef struct
{
    lf2_end_t end;
    uint32_t  t_end_ms;         /**< START to the end */
    uint32_t  mm_end;           /**< wheel travel START to the end */
    uint32_t  mm_stop;          /**< wheel travel after the end (0.5 s) */
    bool      still;            /**< no wheel counts 300..500 ms after the end */
    lf_state_t st_end;          /**< LF state at the end */
    bool      first_seen;       /**< a steering post was seen */
    int32_t   first_steer;      /**< the first steering seen after START */
    uint32_t  t_on_ms;          /**< first poll ON_LINE (LF2_NEVER: never) */
    uint32_t  settle_mm;        /**< travel at the first settle (LF2_NEVER: never) */
    uint32_t  settled_polls;    /**< polls after the settle */
    uint32_t  settled_band;     /**< ... ON_LINE with CENTRE in the band */
    uint32_t  half_polls;       /**< polls in the second half of the travel limit */
    uint32_t  half_band;        /**< ... ON_LINE with CENTRE in the band */
    int32_t   steer_abs_max;    /**< largest |steer| after the settle */
    uint32_t  steer_flips;      /**< steering sign changes after the settle */
    bool      lost;             /**< LINE_LOST entered */
    bool      found_flag;       /**< LINE_FOUND seen set while ON_LINE */
    uint32_t  r_dark_polls;     /**< ON_LINE polls with RIGHT not LIGHT */
    uint32_t  r_white_n;        /**< ON_LINE polls with RIGHT LIGHT ... */
    uint32_t  r_white_sum;
    uint16_t  r_white_min;
    uint16_t  r_white_max;
    uint16_t  r_peak_raw;       /**< highest RIGHT raw in the drive */
    bool      win_done;         /**< the post window completed */
    bool      win_on_line;      /**< LF ON_LINE through the window */
    uint32_t  win_posts;        /**< slot posts in the window */
    uint32_t  win_ms;           /**< the window's real length */
    uint32_t  speed_mmps;       /**< wheel travel per second after 0.5 s */
    uint32_t  n_jct;            /**< JUNCTION events in the drive */
    uint32_t  n_lost;           /**< TRACK_LOST events in the drive */
    uint32_t  ev_fail;          /**< failed events in the drive */
    uint32_t  dropped;          /**< dropped steering posts in the drive */
    uint32_t  lost_to_track_ms; /**< LINE_LOST entry to TRACK_LOST */
    bool      det_seen;         /**< junction detected (state JUNCTION) */
    bool      lost_before;      /**< LINE_LOST entered before detection */
    bool      lost_after;       /**< LINE_LOST entered after detection */
    uint32_t  t_lost_det;       /**< LINE_LOST entry time as seen at detection */
    uint32_t  t_det_ms;         /**< START to detection */
    uint32_t  mm_det;           /**< wheel travel START to detection */
    bool      post_seen;        /**< the JUNCTION event went out */
    uint32_t  t_post_ms;        /**< START to JUNCTION */
    uint32_t  mm_post;          /**< wheel travel START to JUNCTION */
    uint32_t  tail_polls;       /**< polls past the cross line, before JUNCTION */
    uint32_t  tail_c_on;        /**< ... with CENTRE on the line */
    uint32_t  tail_r_dark;      /**< ... with RIGHT not white */
    uint32_t  vc_end;           /**< VC state at the end */
    uint16_t  start_c_pm;       /**< CENTRE darkness at START */
    bool      cmd_sent;         /**< the turn command was injected */
    bool      turn_seen;        /**< the VC entered the turn state */
    bool      turn_done;        /**< ... and left it */
    uint32_t  t_turn_end_ms;    /**< START to the end of the turn */
    int32_t   turn_dl;          /**< left counts over the turn */
    int32_t   turn_dr;          /**< right counts over the turn */
    uint32_t  turn_deg;         /**< the turn in degrees, from the counts */
    bool      branch_on;        /**< ON_LINE after the turn */
    uint32_t  t_branch_ms;      /**< START to that */
    uint32_t  branch_mm;        /**< travel along the branch */
    uint32_t  br_polls;         /**< polls along the branch, past the settling */
    uint32_t  br_c_on;          /**< ... ON_LINE with CENTRE on the line */
    uint32_t  cv_t_lost;        /**< S3-U17: last LINE_LOST entry time seen */
    uint32_t  cv_t_jct;         /**< ... last JUNCTION entry time seen */
    uint32_t  cv_n_lost;        /**< ... LINE_LOST entries in the drive */
    uint32_t  cv_n_det;         /**< ... junction detections (JUNCTION entries) */
    uint32_t  cv_exit_polls;    /**< ... polls in the exit window */
    uint32_t  cv_exit_band;     /**< ... ON_LINE with CENTRE on the line */
    int32_t   cv_steer_max;     /**< ... largest |steer| posted in the drive */
    uint32_t  cv_hard_polls;    /**< ... polls with |steer| >= LF2_CURVE_HARD_Q7 */
    uint32_t  cv_first_lost_mm; /**< ... travel at the first LINE_LOST (LF2_NEVER: none) */
    uint32_t  n_avoid;          /**< drive mode: times obstacle avoidance took over */
    bool      in_avoid;         /**< drive mode: the VC is in OBSTACLE_AVOIDANCE now */
} lf2_run_t;

/* The test task's stack is small: the snapshots live here. */
static lf2_run_t const  s_zero_run;
static lf2_run_t        s_run;
static lf_task_status_t s_base;
static lf_task_status_t s_now;
static motion_state_t   s_m0;
static motion_state_t   s_m;
static motion_state_t   s_m_half;
static motion_state_t   s_m_end;
static motion_state_t   s_m_turn;   /* at the JUNCTION post */
static motion_state_t   s_m_branch; /* at the first ON_LINE after the turn */
/* The 20 ms trace of the current drive (up to 8 KB; APP_TEST builds only).
 * Drive mode writes it as a ring: s_trace_w is the next row to write, so
 * once full it is also the oldest row. */
static lf2_trace_row_t  s_trace[LF2_TRACE_ROWS];
static uint32_t         s_trace_n;
static uint32_t         s_trace_w;
/* Static pose A (start pose): RIGHT over white, for the noise comparison. */
static uint16_t         s_static_r_min;
static uint16_t         s_static_r_max;

static bool lf2_line_found_flag(void)
{
    T_RFLG ref;

    return (tk_ref_flg(EVF_NAV, &ref) >= (ER)E_OK)
           && ((((uint32_t)ref.flgptn) & EVF_NAV_LINE_FOUND) != 0U);
}

static void lf2_remote(uint32_t type)
{
    nav_input_t const m = drep_nav_remote(type);

    if (tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), (TMO)NAV_MBF_SEND_TMO_MS) != E_OK)
    {
        dbg_puts("[LF] test: remote command not sent (MBF_NAV_IN full)");
    }
}

static uint32_t lf2_posts(lf_task_status_t const *p_s)
{
    return p_s->posts_slot + p_s->posts_entry;
}

static uint32_t lf2_abs_delta(int32_t a, int32_t b)
{
    int32_t const d = (int32_t)((uint32_t)b - (uint32_t)a);     /* counts wrap */

    return (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
}

/* Wheel travel between two Motion states: the mean of both wheels' counts. */
static uint32_t lf2_travel_mm(motion_state_t const *p_a, motion_state_t const *p_b)
{
    uint32_t const dl = lf2_abs_delta(p_a->enc_count[0], p_b->enc_count[0]);
    uint32_t const dr = lf2_abs_delta(p_a->enc_count[1], p_b->enc_count[1]);

    return (((dl + dr) / 2U) * WHEEL_CIRC_MM) / (ENC_QUAD_MULT * ENC_CPR);
}

static bool lf2_in_band(uint16_t d)
{
    return (d >= LF_LIGHT_MAX_PM);
}

static bool lf2_pct_ok(uint32_t part, uint32_t whole)
{
    return (whole > 0U) && ((part * 100U) >= (whole * LF2_BAND_MIN_PCT));
}

/* Tell the operator where to put the car, then count down.  The VC is
 * READY meanwhile (motors stopped).  If an earlier drive latched a fault
 * (a hard catch reads as a collision), RESET it near the end of the count:
 * the VC accepts it only after SAFETY_T_STILL_MS with the wheels still. */
static void lf2_place(char const *p_tag, char const *p_place, bool drives)
{
    dbg_printf("[LF] >>> %s: %s", p_tag, p_place);
    if (drives)
    {
        dbg_puts("[LF] >>> hands off at 0: the car DRIVES (catch and lift it to stop)");
    }
    for (uint32_t left = LF2_PLACE_S; left > 0U; left--)
    {
        if ((left <= 3U) || (left == (LF2_PLACE_S / 2U)))
        {
            dbg_printf("[LF] ... %lu", (unsigned long)left);
        }
        if ((left <= 2U) && (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED))
        {
            dbg_puts("[LF] fault latched: test sends RESET (car must be still)");
            lf2_remote(DREP_NAV_REMOTE_RESET);
        }
        (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    }
}

/* Start guard: is the car where a jmat drive may start? */
static bool lf2_start_ok(uint16_t raw_c, uint16_t raw_r)
{
    uint32_t const white = s_now.cal.c.white;
    uint32_t const span = (uint32_t)s_now.cal.c.black - white;
    uint32_t const lifted = white + ((span * LF2_JMAT_LIFTED_PCT) / 100U);

#if S3_LF_STRADDLE
    /* Straddle: the line between the sensors, both on white (lifted, both
     * read far above black instead). */
    return (raw_c <= lifted) && (raw_r <= lifted)
           && (lf_norm(&s_now.cal.c, raw_c) < LF_LIGHT_MAX_PM)
           && (lf_norm(&s_now.cal.r, raw_r) < LF_LIGHT_MAX_PM);
#else
    return (raw_c <= lifted)
           && (lf_norm(&s_now.cal.c, raw_c) >= LF2_JMAT_BLACK_MIN_PM)
           && (lf_norm(&s_now.cal.r, raw_r) < LF_LIGHT_MAX_PM);
#endif
}

/* Hold a jmat drive until CENTRE sits steady on black and RIGHT on white,
 * with a warning before START.  The car must first read wrong at least once:
 * after a drive it stands on black past the cross, and without this the
 * guard would pass there and drive it off the mat.  The VC is READY
 * meanwhile (motors stopped).  False: never placed, no START. */
static bool lf2_place_guard(char const *p_tag, char const *p_place)
{
    uint32_t const steady_ticks = (LF2_JMAT_STEADY_S * LF2_SECOND_MS) / LF2_JMAT_LIVE_MS;
    uint32_t const off_ticks = (LF2_JMAT_HANDS_OFF_S * LF2_SECOND_MS) / LF2_JMAT_LIVE_MS;
    uint32_t const wait_ticks = (LF2_JMAT_WAIT_S * LF2_SECOND_MS) / LF2_JMAT_LIVE_MS;
    uint32_t good = 0U;
    bool     moved = false;
    bool     ready = false;

    dbg_printf("[LF] >>> %s: %s", p_tag, p_place);
    dbg_printf("[LF] >>> put the car against the tape, hands off: it starts when it is steady with %s", LF2_START_TEXT);
    for (uint32_t tick = 0U; (tick < wait_ticks) && !ready; tick++)
    {
        bool ok;

        lf_task_get_status(&s_now);
        ok = lf2_start_ok(s_now.raw_c, s_now.raw_r);
        moved = moved || !ok;
        if (ok && moved)
        {
            good++;
            if (good == steady_ticks)
            {
                dbg_puts("[LF] >>> HANDS OFF: the car DRIVES in 3 s (catch and lift it to stop)");
                if (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED)
                {
                    dbg_puts("[LF] fault latched: test sends RESET (car must be still)");
                    lf2_remote(DREP_NAV_REMOTE_RESET);
                }
            }
            ready = (good >= (steady_ticks + off_ticks));
        }
        else
        {
            if (good >= steady_ticks)
            {
                dbg_puts("[LF] >>> car moved: START cancelled, place it again");
            }
            good = 0U;
        }
        dbg_printf("[LF] place: CENTRE %u pm, RIGHT %u pm: %s", lf_norm(&s_now.cal.c, s_now.raw_c),
                   lf_norm(&s_now.cal.r, s_now.raw_r),
                   ok ? (moved ? "OK, keep still" : "lift the car and put it at the start")
                      : "NOT READY: " LF2_START_TEXT);
        if (!ready)
        {
            (void)tk_dly_tsk((RELTIM)LF2_JMAT_LIVE_MS);
        }
    }

    return ready;
}

/* Static measurement in one pose, motors stopped.  Returns the average
 * darkness of CENTRE and RIGHT; keeps RIGHT's raw range. */
static void lf2_static(char const *p_tag, char const *p_place,
                       uint16_t *p_dc, uint16_t *p_dr, uint16_t *p_rmin, uint16_t *p_rmax,
                       lf_adc_stats_t *p_sc, lf_adc_stats_t *p_sr)
{
    uint32_t sum_c = 0U;
    uint32_t sum_r = 0U;
    uint32_t n = 0U;
    uint16_t c_min = 0xFFFFU;
    uint16_t c_max = 0U;
    uint16_t r_min = 0xFFFFU;
    uint16_t r_max = 0U;
    uint16_t avg_c;
    uint16_t avg_r;

    if (p_place != NULL)
    {
        lf2_place(p_tag, p_place, false);
    }
    if (p_sc != NULL)
    {
        lf_adc_stats_reset(p_sc);
    }
    if (p_sr != NULL)
    {
        lf_adc_stats_reset(p_sr);
    }
    for (uint32_t t = 0U; t < LF2_STATIC_MS; t += LF2_POLL_MS)
    {
        lf_task_get_status(&s_now);
        if (p_sc != NULL)
        {
            lf_adc_stats_add(p_sc, s_now.raw_c);
        }
        if (p_sr != NULL)
        {
            lf_adc_stats_add(p_sr, s_now.raw_r);
        }
        sum_c += s_now.raw_c;
        sum_r += s_now.raw_r;
        c_min = (s_now.raw_c < c_min) ? s_now.raw_c : c_min;
        c_max = (s_now.raw_c > c_max) ? s_now.raw_c : c_max;
        r_min = (s_now.raw_r < r_min) ? s_now.raw_r : r_min;
        r_max = (s_now.raw_r > r_max) ? s_now.raw_r : r_max;
        n++;
        (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
    }
    avg_c = (uint16_t)(sum_c / n);
    avg_r = (uint16_t)(sum_r / n);
    *p_dc = lf_norm(&s_now.cal.c, avg_c);
    *p_dr = lf_norm(&s_now.cal.r, avg_r);
    *p_rmin = r_min;
    *p_rmax = r_max;
    dbg_printf("[LF] static %s: C raw %u..%u avg %u = %u pm", p_tag, c_min, c_max, avg_c, *p_dc);
    dbg_printf("[LF] static %s: R raw %u..%u avg %u = %u pm", p_tag, r_min, r_max, avg_r, *p_dr);
}

/* One trace row.  Counts since START are int32_t: a 3 m drive is about
 * 35 700 counts, past int16_t.  The other modes keep the first
 * LF2_TRACE_ROWS rows; drive mode overwrites the oldest (a ring). */
static void lf2_trace_add(uint32_t t, uint32_t mm)
{
    if ((s_trace_n < LF2_TRACE_ROWS) || (LF2_DRIVE_ONLY != 0U))
    {
        lf2_trace_row_t *const p_r = &s_trace[s_trace_w];

        p_r->t_ms = (uint16_t)t;
        p_r->mm = (uint16_t)mm;
        p_r->raw_c = s_now.raw_c;
        p_r->raw_r = s_now.raw_r;
        p_r->cnt_l = (int32_t)((uint32_t)s_m.enc_count[0] - (uint32_t)s_m0.enc_count[0]);
        p_r->cnt_r = (int32_t)((uint32_t)s_m.enc_count[1] - (uint32_t)s_m0.enc_count[1]);
        p_r->steer_q7 = (int8_t)s_now.last_steer_q7;
        p_r->state = (uint8_t)s_now.state;
        s_trace_w = (s_trace_w + 1U) % LF2_TRACE_ROWS;
        if (s_trace_n < LF2_TRACE_ROWS)
        {
            s_trace_n++;
        }
    }
}

/* Print the trace, the car stopped.  The last line gives the ring's dropped
 * line count before and after, so a lost row is visible. */
static void lf2_trace_dump(char const *p_tag)
{
    uint32_t const dropped0 = dbg_overflow_count();
    /* Oldest row first: once a ring is full, the next row to write is the oldest. */
    uint32_t const first = (s_trace_n < LF2_TRACE_ROWS) ? 0U : s_trace_w;

    dbg_printf("[LF] tr %s: t mm st c r s L R (%lu rows, 20 ms)", p_tag, (unsigned long)s_trace_n);
    for (uint32_t i = 0U; i < s_trace_n; i++)
    {
        lf2_trace_row_t const *const p_r = &s_trace[(first + i) % LF2_TRACE_ROWS];

        dbg_printf("[LF] tr %u %u %u %u %u %d %ld %ld", p_r->t_ms, p_r->mm, p_r->state, p_r->raw_c,
                   p_r->raw_r, (int)p_r->steer_q7, (long)p_r->cnt_l, (long)p_r->cnt_r);
        if (((i + 1U) % LF2_TRACE_BURST) == 0U)
        {
            (void)tk_dly_tsk((RELTIM)LF2_TRACE_PAUSE_MS);
        }
    }
    (void)tk_dly_tsk((RELTIM)LF2_TRACE_PAUSE_MS);
    dbg_printf("[LF] tr %s end: ring dropped %lu lines", p_tag,
               (unsigned long)(dbg_overflow_count() - dropped0));
}

/* Per-poll statistics of a drive. */
static void lf2_account(lf2_run_t *p, uint32_t t, uint32_t mm, uint32_t half_mm,
                        int32_t *p_last_sign)
{
    uint16_t const d_c = lf_norm(&s_now.cal.c, s_now.raw_c);
    uint16_t const d_r = lf_norm(&s_now.cal.r, s_now.raw_r);
    bool const     on = (s_now.state == LF_ST_ON_LINE);
    bool const     band = on && lf2_in_band(d_c);

    if (!p->first_seen && (lf2_posts(&s_now) != lf2_posts(&s_base)))
    {
        p->first_seen = true;
        p->first_steer = s_now.last_steer_q7;
    }
    if (on && (p->t_on_ms == LF2_NEVER))
    {
        p->t_on_ms = t;
    }
    if (on && (p->settle_mm == LF2_NEVER) && (d_c >= LF2_SETTLE_PM_LO) && (d_c <= LF2_SETTLE_PM_HI))
    {
        p->settle_mm = mm;
    }
    if (p->settle_mm != LF2_NEVER)
    {
        int32_t const s = s_now.last_steer_q7;
        int32_t const a = (s < 0) ? -s : s;
        int32_t const sign = (s > 0) ? 1 : ((s < 0) ? -1 : 0);

        p->settled_polls++;
        p->settled_band += band ? 1U : 0U;
        p->steer_abs_max = (a > p->steer_abs_max) ? a : p->steer_abs_max;
        if ((sign != 0) && (*p_last_sign != 0) && (sign != *p_last_sign))
        {
            p->steer_flips++;
        }
        *p_last_sign = (sign != 0) ? sign : *p_last_sign;
    }
    if (mm >= half_mm)
    {
        p->half_polls++;
        p->half_band += band ? 1U : 0U;
    }
    if (s_now.t_entry_ms[LF_ST_LINE_LOST] != s_base.t_entry_ms[LF_ST_LINE_LOST])
    {
        p->lost = true;
    }
    if (on && !p->found_flag && lf2_line_found_flag())
    {
        p->found_flag = true;
    }
    if (on && (d_r < LF_LIGHT_MAX_PM))
    {
        p->r_white_n++;
        p->r_white_sum += s_now.raw_r;
        p->r_white_min = (s_now.raw_r < p->r_white_min) ? s_now.raw_r : p->r_white_min;
        p->r_white_max = (s_now.raw_r > p->r_white_max) ? s_now.raw_r : p->r_white_max;
    }
    else if (on)
    {
        p->r_dark_polls++;
    }
    else
    {
        /* not tracking: RIGHT is counted only while ON_LINE */
    }
    p->r_peak_raw = (s_now.raw_r > p->r_peak_raw) ? s_now.raw_r : p->r_peak_raw;
}

/* Junction mat: note detection, the JUNCTION event and the line past the
 * cross, one poll at a time. */
static void lf2_jmat_watch(lf2_run_t *p, uint32_t t, uint32_t mm)
{
    uint32_t const t_lost = s_now.t_entry_ms[LF_ST_LINE_LOST];

    if (!p->det_seen && (s_now.state == LF_ST_JUNCTION))
    {
        p->det_seen = true;
        p->t_det_ms = t;
        p->mm_det = mm;
        p->t_lost_det = t_lost;
        p->lost_before = (t_lost != s_base.t_entry_ms[LF_ST_LINE_LOST]);
    }
    if (p->det_seen && (t_lost != p->t_lost_det))
    {
        p->lost_after = true;
    }
    if (!p->post_seen && (s_now.n_junction != s_base.n_junction))
    {
        p->post_seen = true;
        p->t_post_ms = t;
        p->mm_post = mm;
    }
    if (p->det_seen && !p->post_seen && (mm >= (p->mm_det + LF2_JMAT_TAIL_FROM_MM)))
    {
        p->tail_polls++;
        p->tail_c_on += lf2_in_band(lf_norm(&s_now.cal.c, s_now.raw_c)) ? 1U : 0U;
        p->tail_r_dark += lf2_in_band(lf_norm(&s_now.cal.r, s_now.raw_r)) ? 1U : 0U;
    }
}

/* The turn in degrees, from the wheel counts either side of it: both wheels
 * travel an arc of pi x track x angle / 360.  Counts are signed: a pivot runs
 * the wheels in opposite directions, so lf2_travel_mm() cannot be used. */
static uint32_t lf2_turn_deg(int32_t dl, int32_t dr)
{
    uint32_t const al = (dl < 0) ? (uint32_t)(-dl) : (uint32_t)dl;
    uint32_t const ar = (dr < 0) ? (uint32_t)(-dr) : (uint32_t)dr;
    uint32_t const mean = (al + ar) / 2U;

    return (mean * WHEEL_CIRC_MM * 360U) / (ENC_QUAD_MULT * ENC_CPR * LF2_PI_TRACK_MM);
}

/* Junction turn: inject the command at detection, as a barcode read before
 * the cross would have, then follow the turn and the branch. */
static void lf2_turn_watch(lf2_run_t *p, uint32_t t, uint32_t vc)
{
    if (p->det_seen && !p->cmd_sent)
    {
        nav_input_t const m = drep_nav_barcode(LF2_TURN_CMD);

        p->cmd_sent = (tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), (TMO)NAV_MBF_SEND_TMO_MS) == E_OK);
    }
    if (!p->turn_seen && (vc == LF2_TURN_VC_STATE))
    {
        p->turn_seen = true;
        s_m_turn = s_m;
    }
    if (p->turn_seen && !p->turn_done && (vc != LF2_TURN_VC_STATE))
    {
        p->turn_done = true;
        p->t_turn_end_ms = t;
        p->turn_dl = (int32_t)((uint32_t)s_m.enc_count[0] - (uint32_t)s_m_turn.enc_count[0]);
        p->turn_dr = (int32_t)((uint32_t)s_m.enc_count[1] - (uint32_t)s_m_turn.enc_count[1]);
        p->turn_deg = lf2_turn_deg(p->turn_dl, p->turn_dr);
    }
    if (p->turn_done && !p->branch_on && (s_now.state == LF_ST_ON_LINE))
    {
        p->branch_on = true;
        p->t_branch_ms = t;
        s_m_branch = s_m;
    }
    if (p->branch_on)
    {
        p->branch_mm = lf2_travel_mm(&s_m_branch, &s_m);
        if (p->branch_mm >= LF2_TURN_TAIL_FROM_MM)
        {
            p->br_polls++;
            p->br_c_on += ((s_now.state == LF_ST_ON_LINE)
                           && lf2_in_band(lf_norm(&s_now.cal.c, s_now.raw_c))) ? 1U : 0U;
        }
    }
}

/* S3-U17 curve: LINE_LOST entries and junction detections by their entry
 * times (each entry stamps t_entry_ms; two entries cannot fall in one 10 ms
 * poll: LINE_LOST and JUNCTION each last at least 3 samples), the steering
 * used, and the exit straight. */
static void lf2_curve_watch(lf2_run_t *p, uint32_t mm)
{
    int32_t const s = s_now.last_steer_q7;
    int32_t const a = (s < 0) ? -s : s;

    if (s_now.t_entry_ms[LF_ST_LINE_LOST] != p->cv_t_lost)
    {
        p->cv_t_lost = s_now.t_entry_ms[LF_ST_LINE_LOST];
        p->cv_n_lost++;
        if (p->cv_first_lost_mm == LF2_NEVER)
        {
            p->cv_first_lost_mm = mm;
        }
    }
    if (s_now.t_entry_ms[LF_ST_JUNCTION] != p->cv_t_jct)
    {
        p->cv_t_jct = s_now.t_entry_ms[LF_ST_JUNCTION];
        p->cv_n_det++;
    }
    p->cv_steer_max = (a > p->cv_steer_max) ? a : p->cv_steer_max;
    p->cv_hard_polls += (a >= LF2_CURVE_HARD_Q7) ? 1U : 0U;
    if (mm >= (LF2_CURVE_MM - LF2_CURVE_EXIT_MM))
    {
        p->cv_exit_polls++;
        p->cv_exit_band += ((s_now.state == LF_ST_ON_LINE)
                            && lf2_in_band(lf_norm(&s_now.cal.c, s_now.raw_c))) ? 1U : 0U;
    }
}

/* One bounded drive: START, watch every 10 ms, STOP at a limit. */
static void lf2_drive(lf2_drive_t const *p_d)
{
    lf2_run_t *const p = &s_run;
    uint32_t const   half_mm = p_d->max_mm / 2U;
    uint32_t         t0;
    uint32_t         t = 0U;
    uint32_t         t_half = 0U;
    uint32_t         t_win = 0U;
    uint32_t         win_base = 0U;
    bool             win_open = false;
    bool             half_seen = false;
    bool             done = false;
    int32_t          last_sign = 0;
    uint32_t         polls = 0U;
    uint32_t         mm = 0U;

    *p = s_zero_run;
    s_trace_n = 0U;
    s_trace_w = 0U;
    p->t_on_ms = LF2_NEVER;
    p->settle_mm = LF2_NEVER;
    p->r_white_min = 0xFFFFU;
    p->win_on_line = true;

    p->cv_first_lost_mm = LF2_NEVER;
    if (p_d->jmat || p_d->turn || p_d->curve)
    {
        if (!lf2_place_guard(p_d->p_tag, p_d->p_place))
        {
            p->end = LF2_END_NOT_PLACED;
            dbg_printf("[LF] %s: never placed: no START", p_d->p_tag);
            return;
        }
    }
    else
    {
        lf2_place(p_d->p_tag, p_d->p_place, true);
    }
    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        p->end = LF2_END_NOT_READY;
        dbg_printf("[LF] %s: VC state %lu, not READY: no START", p_d->p_tag,
                   (unsigned long)vc_get_system_state());
        return;
    }

    lf_task_get_status(&s_base);
    p->start_c_pm = lf_norm(&s_base.cal.c, s_base.raw_c);
    p->cv_t_lost = s_base.t_entry_ms[LF_ST_LINE_LOST];
    p->cv_t_jct = s_base.t_entry_ms[LF_ST_JUNCTION];
    (void)motion_get_state(&s_m0);
    t0 = app_period_now_ms();
    lf2_remote(DREP_NAV_REMOTE_START);

    while (!done)
    {
        uint32_t vc;

        (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
        lf_task_get_status(&s_now);
        (void)motion_get_state(&s_m);
        t = app_period_now_ms() - t0;
        mm = lf2_travel_mm(&s_m0, &s_m);
        vc = vc_get_system_state();
        polls++;

        lf2_account(p, t, mm, half_mm, &last_sign);
        if (p_d->jmat || p_d->turn)
        {
            lf2_jmat_watch(p, t, mm);
        }
        if (p_d->turn)
        {
            lf2_turn_watch(p, t, vc);
        }
        if (p_d->curve)
        {
            lf2_curve_watch(p, mm);
        }
        if ((LF2_DRIVE_ONLY != 0U) && ((vc == VC_SYS_OBSTACLE_AVOIDANCE) != p->in_avoid))
        {
            p->in_avoid = !p->in_avoid;
            p->n_avoid += p->in_avoid ? 1U : 0U;
            dbg_printf("[LF] drive: obstacle avoidance %s at %lu mm (LF state %u, line found %d)",
                       p->in_avoid ? "took over" : "handed back", (unsigned long)mm,
                       (unsigned)s_now.state, lf2_line_found_flag() ? 1 : 0);
        }
        if ((polls % LF2_TRACE_EVERY_POLLS) == 0U)
        {
            lf2_trace_add(t, mm);
        }

        if (!half_seen && (t >= LF2_WIN_FROM_MS))
        {
            half_seen = true;
            t_half = t;
            s_m_half = s_m;
        }
        if (!win_open && !p->win_done && (t >= LF2_WIN_FROM_MS))
        {
            win_open = true;
            win_base = s_now.posts_slot;
            t_win = t;
        }
        if (win_open)
        {
            p->win_on_line = p->win_on_line && (s_now.state == LF_ST_ON_LINE);
            if (t >= LF2_WIN_TO_MS)
            {
                win_open = false;
                p->win_done = true;
                p->win_posts = s_now.posts_slot - win_base;
                p->win_ms = t - t_win;
            }
        }

        if ((polls % LF2_LOG_EVERY_POLLS) == 0U)
        {
            dbg_printf("[LF] d t=%lu mm=%lu st=%u vc=%lu c=%u r=%u dc=%u dr=%u s=%ld",
                       (unsigned long)t, (unsigned long)mm, (unsigned)s_now.state,
                       (unsigned long)vc, s_now.raw_c, s_now.raw_r,
                       lf_norm(&s_now.cal.c, s_now.raw_c), lf_norm(&s_now.cal.r, s_now.raw_r),
                       (long)s_now.last_steer_q7);
        }

        done = true;
        if (s_now.n_track_lost != s_base.n_track_lost)
        {
            p->end = LF2_END_TRACK_LOST;
        }
        else if (vc == VC_SYS_SAFETY_FAULT_LATCHED)
        {
            p->end = LF2_END_FAULT;
        }
        else if ((vc == VC_SYS_OBSTACLE_AVOIDANCE) && (LF2_DRIVE_ONLY == 0U))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_AVOID;
        }
        else if (vc == VC_SYS_READY_WAITING)
        {
            p->end = LF2_END_VC_STOP;
        }
        else if (p_d->jmat && p->post_seen
                 && ((s_now.state == LF_ST_ON_LINE) || ((t - p->t_post_ms) >= LF2_JMAT_THROUGH_MS)))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_JCT;
        }
        else if (p_d->jmat && (s_now.state == LF_ST_LINE_LOST) && (mm >= LF2_JMAT_LOST_MM))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_LOST;
        }
        else if (p_d->turn && (p->branch_mm >= LF2_TURN_BRANCH_MM))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_BRANCH;
        }
        else if (p_d->turn && !p->det_seen && (mm >= LF2_TURN_NO_CROSS_MM))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_NO_CROSS;
        }
        else if (!p_d->turn && (mm >= p_d->max_mm))
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_DIST;
        }
        else if (t >= p_d->max_ms)
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_TIME;
        }
        else
        {
            done = false;
        }
        p->vc_end = vc;
    }

    s_m_end = s_m;
    p->t_end_ms = t;
    p->mm_end = mm;
    p->st_end = s_now.state;
    p->n_jct = s_now.n_junction - s_base.n_junction;
    p->n_lost = s_now.n_track_lost - s_base.n_track_lost;
    p->ev_fail = s_now.ev_send_fail - s_base.ev_send_fail;
    p->dropped = s_now.posts_dropped - s_base.posts_dropped;
    p->lost_to_track_ms = s_now.t_track_lost_ms - s_now.t_entry_ms[LF_ST_LINE_LOST];
    if (half_seen && (t > t_half))
    {
        p->speed_mmps = (lf2_travel_mm(&s_m_half, &s_m_end) * 1000U) / (t - t_half);
    }

    /* Stopping: travel after the end, and the wheels still at 300..500 ms. */
    (void)tk_dly_tsk((RELTIM)LF2_STILL_FROM_MS);
    (void)motion_get_state(&s_m_half);
    (void)tk_dly_tsk((RELTIM)(LF2_STILL_TO_MS - LF2_STILL_FROM_MS));
    (void)motion_get_state(&s_m);
    p->still = (s_m.enc_count[0] == s_m_half.enc_count[0])
               && (s_m.enc_count[1] == s_m_half.enc_count[1]);
    p->mm_stop = lf2_travel_mm(&s_m_end, &s_m);

    dbg_printf("[LF] %s: end %s at %lu ms, %lu mm; then %lu mm, still %u, vc=%lu",
               p_d->p_tag, k_end_name[p->end], (unsigned long)p->t_end_ms,
               (unsigned long)p->mm_end, (unsigned long)p->mm_stop, (unsigned)p->still,
               (unsigned long)vc_get_system_state());
    dbg_printf("[LF] %s: speed %lu mm/s (cruise %lu), first steer %ld, ON_LINE at %ld ms",
               p_d->p_tag, (unsigned long)p->speed_mmps, (unsigned long)LF2_CRUISE_MMPS,
               (long)p->first_steer, (long)((p->t_on_ms == LF2_NEVER) ? -1 : (int32_t)p->t_on_ms));
    dbg_printf("[LF] %s: settle %ld mm, band %lu/%lu after, %lu/%lu 2nd half",
               p_d->p_tag, (long)((p->settle_mm == LF2_NEVER) ? -1 : (int32_t)p->settle_mm),
               (unsigned long)p->settled_band, (unsigned long)p->settled_polls,
               (unsigned long)p->half_band, (unsigned long)p->half_polls);
    dbg_printf("[LF] %s: |steer| max %ld, flips %lu, LINE_LOST %u, LINE_FOUND %u",
               p_d->p_tag, (long)p->steer_abs_max, (unsigned long)p->steer_flips,
               (unsigned)p->lost, (unsigned)p->found_flag);
    dbg_printf("[LF] %s: jct %lu, track_lost %lu, ev fail %lu, dropped %lu",
               p_d->p_tag, (unsigned long)p->n_jct, (unsigned long)p->n_lost,
               (unsigned long)p->ev_fail, (unsigned long)p->dropped);
}

/* RIGHT's statistics of the last drive, against the static pose A. */
static void lf2_log_right(char const *p_tag)
{
    lf2_run_t const *const p = &s_run;
    uint16_t const avg = (p->r_white_n > 0U) ? (uint16_t)(p->r_white_sum / p->r_white_n) : 0U;

    dbg_printf("[LF] %s: R white moving raw %u..%u avg %u (%lu polls), static %u..%u",
               p_tag, (p->r_white_n > 0U) ? p->r_white_min : 0U, p->r_white_max, avg,
               (unsigned long)p->r_white_n, s_static_r_min, s_static_r_max);
    dbg_printf("[LF] %s: R peak raw %u = %u pm, R not white %lu polls", p_tag, p->r_peak_raw,
               lf_norm(&s_now.cal.r, p->r_peak_raw), (unsigned long)p->r_dark_polls);
}

/* What one junction mat drive did, with the numbers the ruler check needs. */
static void lf2_jmat_report(uint32_t run)
{
    lf2_run_t const *const p = &s_run;
    uint32_t const total = p->mm_end + p->mm_stop;

    dbg_printf("[LF] jmat: START with CENTRE at %u pm", p->start_c_pm);
    dbg_printf("[LF] jmat: detected %lu mm at %lu ms, JUNCTION %lu mm at %lu ms, delay %lu mm %lu ms",
               (unsigned long)p->mm_det, (unsigned long)p->t_det_ms, (unsigned long)p->mm_post,
               (unsigned long)p->t_post_ms, (unsigned long)(p->mm_post - p->mm_det),
               (unsigned long)(p->t_post_ms - p->t_det_ms));
    dbg_printf("[LF] jmat: past the cross %lu/%lu polls CENTRE on, RIGHT dark %lu, state %u, vc %lu",
               (unsigned long)p->tail_c_on, (unsigned long)p->tail_polls,
               (unsigned long)p->tail_r_dark, (unsigned)p->st_end, (unsigned long)p->vc_end);
    dbg_printf("[LF] RULER run %lu: wheel travel START to standstill %lu mm, after JUNCTION %lu mm",
               (unsigned long)run, (unsigned long)total, (unsigned long)(total - p->mm_post));
}

/* What one junction turn did. */
static void lf2_turn_report(uint32_t run)
{
    lf2_run_t const *const p = &s_run;

    dbg_printf("[LF] turn %lu: START with CENTRE at %u pm, launch LINE_LOST %u", (unsigned long)run,
               p->start_c_pm, (unsigned)p->lost_before);
    dbg_printf("[LF] turn: detected %lu mm at %lu ms, JUNCTION %lu mm at %lu ms, delay %lu mm",
               (unsigned long)p->mm_det, (unsigned long)p->t_det_ms, (unsigned long)p->mm_post,
               (unsigned long)p->t_post_ms, (unsigned long)(p->mm_post - p->mm_det));
    dbg_printf("[LF] turn: command sent %u, VC turned %u, ended %u at %lu ms",
               (unsigned)p->cmd_sent, (unsigned)p->turn_seen, (unsigned)p->turn_done,
               (unsigned long)p->t_turn_end_ms);
    dbg_printf("[LF] turn: wheels L %ld R %ld counts = %lu degrees by the encoders",
               (long)p->turn_dl, (long)p->turn_dr, (unsigned long)p->turn_deg);
    dbg_printf("[LF] turn: back ON_LINE %ld ms after the turn, LINE_LOST on the way %u",
               (long)(p->branch_on ? (int32_t)(p->t_branch_ms - p->t_turn_end_ms) : -1),
               (unsigned)p->lost_after);
    dbg_printf("[LF] turn: branch %lu mm, CENTRE on %lu/%lu polls past %lu mm",
               (unsigned long)p->branch_mm, (unsigned long)p->br_c_on, (unsigned long)p->br_polls,
               (unsigned long)LF2_TURN_TAIL_FROM_MM);
}

/* What one curve drive did. */
static void lf2_curve_report(char const *p_group, uint32_t run)
{
    lf2_run_t const *const p = &s_run;

    dbg_printf("[LF] curve %s %lu: %s at %lu mm %lu ms, start C %u pm",
               p_group, (unsigned long)run, k_end_name[p->end], (unsigned long)p->mm_end,
               (unsigned long)p->t_end_ms, p->start_c_pm);
    dbg_printf("[LF] curve: LINE_LOST %lu (first at %ld mm), TRACK_LOST %lu",
               (unsigned long)p->cv_n_lost,
               (long)((p->cv_first_lost_mm == LF2_NEVER) ? -1 : (int32_t)p->cv_first_lost_mm),
               (unsigned long)p->n_lost);
    dbg_printf("[LF] curve: junction detected %lu, JUNCTION sent %lu (a curve has none)",
               (unsigned long)p->cv_n_det, (unsigned long)p->n_jct);
    dbg_printf("[LF] curve: |steer| max %ld, polls at >= %d: %lu",
               (long)p->cv_steer_max, (int)LF2_CURVE_HARD_Q7, (unsigned long)p->cv_hard_polls);
    dbg_printf("[LF] curve: exit straight CENTRE on %lu/%lu polls (last %lu mm)",
               (unsigned long)p->cv_exit_band, (unsigned long)p->cv_exit_polls,
               (unsigned long)LF2_CURVE_EXIT_MM);
}

/* Recovery drive checks (r1..r3): ON_LINE soon, settled in the second
 * half, never lost, ended at the distance limit. */
static bool lf2_recovered(void)
{
    lf2_run_t const *const p = &s_run;

    return (p->end == LF2_END_DIST) && (p->t_on_ms <= LF2_RECOVER_MAX_MS) && !p->lost
           && (p->n_lost == 0U) && lf2_pct_ok(p->half_band, p->half_polls);
}

void lf_run_test(uint32_t *p_pass, uint32_t *p_fail)
{
    static lf2_drive_t const k_straight =
    {
        "straight", "START POSE at the start of >= 1 m of straight line",
        LF2_STRAIGHT_MM, LF2_MS_FOR(LF2_STRAIGHT_MM), false, false, false
    };
    static lf2_drive_t const k_r1 =
    {
        "r1", "car parallel to the line, CENTRE ~1 cm RIGHT of it (on white)",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM), false, false, false
    };
    static lf2_drive_t const k_r2 =
    {
        "r2", "nose ~15 deg LEFT of the line, CENTRE on the line middle",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM), false, false, false
    };
    static lf2_drive_t const k_r3 =
    {
        "r3", "nose ~15 deg RIGHT of the line, CENTRE on the line middle",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM), false, false, false
    };
    static lf2_drive_t const k_jct =
    {
        "junction", "START POSE, a cross junction 40 cm ahead, square to it",
        LF2_JUNCTION_MM, LF2_MS_FOR(LF2_JUNCTION_MM), false, false, false
    };
    static lf2_drive_t const k_jmat =
    {
        "jmat", "junction mat: CENTRE on the right half of the line, RIGHT on white, parallel",
        LF2_JMAT_LIMIT_MM, LF2_MS_FOR(LF2_JMAT_LIMIT_MM), true, false, false
    };
    static lf2_drive_t const k_turn =
    {
        "turn", "CENTRE on the right half of the line, RIGHT on white, 30 cm before the cross",
        LF2_TURN_NO_CROSS_MM, LF2_TURN_MS, false, true, false
    };
    /* The physical cue for the direction: on a U-loop the loop's other
     * straight line lies on the side the car will turn (16 Nov: a run with
     * the directions alternated could not be labelled from the log). */
    static lf2_drive_t const k_curve[3] =
    {
        {
            "LEFT curve", "the line bends LEFT ahead; CENTRE on the mark, RIGHT on white",
            LF2_CURVE_MM, LF2_MS_FOR(LF2_CURVE_MM), false, false, true
        },
        {
            "RIGHT curve", "the line bends RIGHT ahead; CENTRE on the mark, RIGHT on white",
            LF2_CURVE_MM, LF2_MS_FOR(LF2_CURVE_MM), false, false, true
        },
        {
            "S-curve", "start mark on the S-curve's straight; CENTRE on mark, RIGHT on white",
            LF2_CURVE_MM, LF2_MS_FOR(LF2_CURVE_MM), false, false, true
        }
    };
    static char const * const k_curve_group[3] =
    {
        "LEFT curve", "RIGHT curve", "S-curve"
    };
    static lf2_drive_t const k_end =
    {
        "line end", "START POSE, the line's END 30 cm ahead (temporary line OK)",
        LF2_END_MM, LF2_END_MS, false, false, false
    };
    uint32_t const straight_runs = ((LF2_JMAT_ONLY != 0U) || (LF2_TURN_ONLY != 0U) || (LF2_CURVE_ONLY != 0U)
                                    || (LF2_DRIVE_ONLY != 0U))
                                   ? 0U : LF2_STRAIGHT_RUNS;
    uint32_t test_pass_ = *p_pass;
    uint32_t test_fail_ = *p_fail;
    uint32_t releases0;
    uint32_t vc;
    uint16_t dc;
    uint16_t dr;
    uint16_t rmin;
    uint16_t rmax;
    bool     pose_a_ok;
    lf_adc_stats_t a_c;     /* pose A: CENTRE over black */
    lf_adc_stats_t a_r;     /* pose A: RIGHT over white */
    lf_adc_stats_t b_c;     /* pose B: CENTRE over white */
    lf_adc_stats_t b_r;     /* pose B: RIGHT over black */
    lf_cal2_t      cal2;

    dbg_printf("[LF] %s on the track, motors LIVE, cruise %lu mm/s", LF2_ID,
               (unsigned long)LF2_CRUISE_MMPS);
    dbg_printf("[LF] line following: %s", LF2_MODE_TEXT);
    dbg_puts("[LF] guide: .agents/tasks/s3-u14-test.md; follow the [LF] >>> prompts");

    /* (a) The task runs every 5 ms.  First let the boot calibration end. */
    lf_task_get_status(&s_now);
    for (uint32_t waited = 0U; (s_now.releases == 0U) && (waited < LF2_BOOT_WAIT_MS); waited += LF2_POLL_MS)
    {
        (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
        lf_task_get_status(&s_now);
    }
    releases0 = s_now.releases;
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    lf_task_get_status(&s_now);
    dbg_printf("[LF] 1 s: %lu samples, %lu missed so far",
               (unsigned long)(s_now.releases - releases0), (unsigned long)s_now.missed);
    TEST_CHECK(LF2_ID, ((s_now.releases - releases0) >= LF2_RELEASES_1S_MIN)
                         && ((s_now.releases - releases0) <= LF2_RELEASES_1S_MAX)
                         && (s_now.missed == 0U),
               "(a) samples every 5 ms (195-205/s), none missed");

    /* (b) Boot calibration in the start pose. */
    dbg_printf("[LF] boot cal: status %u, C avg %u, R avg %u -> white %u black %u",
               (unsigned)s_now.cal_status, s_now.cal_c_avg, s_now.cal_r_avg,
               s_now.cal.c.white, s_now.cal.c.black);
    TEST_CHECK(LF2_ID, s_now.cal_status == LF_CAL_OK,
               "(b) boot calibration accepted");

    vc = vc_get_system_state();
    if (((vc != VC_SYS_READY_WAITING) && (vc != VC_SYS_INITIALISING)) || (s_now.cal_status != LF_CAL_OK))
    {
        dbg_printf("[LF] VC state %lu (need 0 or 1; 3 = fault, see [SAFETY]), cal %u",
                   (unsigned long)vc, (unsigned)s_now.cal_status);
        TEST_CHECK(LF2_ID, false, "(c-h) not run: VC not READY or cal refused");
        *p_pass = test_pass_;
        *p_fail = test_fail_;
        return;
    }

    /* (c) Armed, VC READY: no output. */
    lf_task_test_arm(true);
    lf_task_get_status(&s_base);
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    lf_task_get_status(&s_now);
    TEST_CHECK(LF2_ID, (lf2_posts(&s_now) == lf2_posts(&s_base))
                         && (s_now.n_junction == s_base.n_junction)
                         && (s_now.n_track_lost == s_base.n_track_lost)
                         && (s_now.state == LF_ST_IDLE) && !lf2_line_found_flag(),
               "(c) VC READY: no posts, no events, IDLE, flag clear");

    /* (g) Static levels at ride height, motors stopped.  Pose A gives
     * RIGHT's resting noise over white; pose B RIGHT's black level (the
     * residual risk of the S3-U09 waiver). */
    pose_a_ok = false;
    lf_adc_stats_reset(&a_c);
    lf_adc_stats_reset(&a_r);
    for (uint32_t tries = 0U; (tries < LF2_POSE_A_TRIES) && !pose_a_ok; tries++)
    {
        lf2_place("pose A", "CENTRE on black in the middle of the line, RIGHT on white, hands off" LF2_POSE_A_HINT, false);
        dbg_puts("[LF] calibrating here, keep the car still");
        lf_task_test_recalibrate();
        lf_task_get_status(&s_now);
        lf2_static("pose A", NULL, &dc, &dr, &rmin, &rmax, &a_c, &a_r);
        s_static_r_min = rmin;
        s_static_r_max = rmax;
        pose_a_ok = (s_now.cal_status == LF_CAL_OK) && (s_now.cal_c_avg >= LF2_POSE_A_MIN_RAW)
                    && (dc >= LF2_POSE_A_MIN_PM);
        if (!pose_a_ok)
        {
            dbg_printf("[LF] >>> pose A not good: CENTRE %u, RIGHT %u.  Try again.",
                       s_now.cal_c_avg, s_now.cal_r_avg);
        }
    }
    TEST_CHECK(LF2_ID, pose_a_ok,
               "(g2) pose A: calibrated on black, CENTRE >= 950 pm, RIGHT on white");
    if (pose_a_ok)
    {
        static char const *const k_sens[] = { "OK", "DISCONNECTED", "NOISY", "BLIND", "WEAK (re-mount)" };
        bool cal2_ok;

        lf2_static("pose B", "RIGHT over the line middle, CENTRE on white, hands off" LF2_POSE_B_HINT,
                   &dc, &dr, &rmin, &rmax, &b_c, &b_r);
        TEST_CHECK(LF2_ID, dr > LF_DARK_MIN_PM,
                   "(g) static: RIGHT on black is DARK (> 750 pm)");
        /* (g3) Each sensor's own black and white from poses A and B, against
         * the mounting criteria (lf_core.h LF_CAL_*). */
        cal2_ok = lf_cal_from_two_poses(&a_c, &a_r, &b_c, &b_r, &cal2);
        dbg_printf("[LF] two-pose: CENTRE white %u black %u span %d %s; RIGHT white %u black %u span %d %s",
                   cal2.cal.c.white, cal2.cal.c.black, (int)cal2.cal.c.black - (int)cal2.cal.c.white,
                   k_sens[cal2.c], cal2.cal.r.white, cal2.cal.r.black,
                   (int)cal2.cal.r.black - (int)cal2.cal.r.white, k_sens[cal2.r]);
        dbg_printf("[LF] mounting criteria: black >= %u, white <= %u, span >= %u raw",
                   LF_CAL_BLACK_MIN_RAW, LF_CAL_WHITE_MAX_RAW, LF_CAL_SPAN_GOOD_RAW);
        TEST_CHECK(LF2_ID, cal2_ok,
                   "(g3) two-pose calibration: both sensors meet the mounting criteria");
        if (cal2_ok)
        {
            lf_task_test_set_cal(&cal2.cal);
            lf_task_get_status(&s_now);
        }
        else
        {
            dbg_puts("[LF] >>> STOP: no drives.  Fix the sensor mounting (height ~10 mm, both the same) and restart.");
            pose_a_ok = false;
        }
    }
    else
    {
        dbg_puts("[LF] >>> STOP: no drives.  Pose A never read CENTRE on black and RIGHT on white.");
    }

    /* (d) + (n) Straight line from the start pose. */
    for (uint32_t run = 1U; run <= straight_runs; run++)
    {
        dbg_printf("[LF] straight run %lu of %lu", (unsigned long)run,
                   (unsigned long)straight_runs);
        lf2_drive(&k_straight);
        lf2_log_right("straight");
        dbg_printf("[LF] straight: window %lu slot posts in %lu ms, ON_LINE %u",
                   (unsigned long)s_run.win_posts, (unsigned long)s_run.win_ms, (unsigned)s_run.win_on_line);
        lf2_trace_dump("straight");
        TEST_CHECK(LF2_ID, s_run.first_seen && (s_run.first_steer >= LF2_STEER_BODY_MIN)
                           && (s_run.settle_mm <= LF2_SETTLE_MAX_MM),
                   "(d1) straight: steers right, feels the edge in 500 mm");
        TEST_CHECK(LF2_ID, s_run.win_done && s_run.win_on_line
                           && ((s_run.win_posts * LF_STEER_POST_PERIOD_MS) + LF_STEER_POST_PERIOD_MS >= s_run.win_ms)
                           && ((s_run.win_posts * LF_STEER_POST_PERIOD_MS) <= s_run.win_ms + LF_STEER_POST_PERIOD_MS)
                           && (s_run.dropped == 0U),
                   "(d2) one post per 20 ms in the slot, none dropped");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && !s_run.lost && s_run.found_flag
                           && lf2_pct_ok(s_run.settled_band, s_run.settled_polls),
                   "(d3) stable: CENTRE on the line (>= 90%), never lost, flag");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.r_white_n > 0U)
                           && (s_run.r_dark_polls == 0U) && (s_run.n_jct == 0U),
                   "(n) motors on: RIGHT stays LIGHT, no false junction");
    }

    /* (j) The junction mat: detection, the delayed event, straight through. */
    if ((LF2_JMAT_ONLY != 0U) && pose_a_ok)
    {
        for (uint32_t run = 1U; run <= LF2_JMAT_RUNS; run++)
        {
            dbg_printf("[LF] jmat run %lu of %lu", (unsigned long)run, (unsigned long)LF2_JMAT_RUNS);
            lf2_drive(&k_jmat);
            lf2_jmat_report(run);
            lf2_trace_dump("jmat");
            TEST_CHECK(LF2_ID, s_run.det_seen && !s_run.lost_before,
                       "(j0) on the line up to the cross, never LINE_LOST");
            TEST_CHECK(LF2_ID, s_run.det_seen && (s_run.n_jct == 1U)
                               && (s_run.mm_det >= LF2_JMAT_DET_MIN_MM)
                               && (s_run.mm_det <= LF2_JMAT_DET_MAX_MM),
                       "(j1) cross detected at 230-320 mm, one JUNCTION");
            TEST_CHECK(LF2_ID, s_run.post_seen
                               && ((s_run.mm_post - s_run.mm_det) >= LF2_JMAT_DELAY_MIN_MM)
                               && ((s_run.mm_post - s_run.mm_det) <= LF2_JMAT_DELAY_MAX_MM)
                               && ((s_run.t_post_ms - s_run.t_det_ms) >= LF2_JMAT_DELAY_MIN_MS)
                               && ((s_run.t_post_ms - s_run.t_det_ms) <= LF2_JMAT_DELAY_MAX_MS),
                       "(j2) JUNCTION 110-135 mm after detection, by the odometer");
            TEST_CHECK(LF2_ID, s_run.post_seen && !s_run.lost_after && (s_run.st_end == LF_ST_ON_LINE)
                               && (s_run.tail_r_dark == 0U)
                               && lf2_pct_ok(s_run.tail_c_on, s_run.tail_polls),
                       "(j3) straight through: back ON_LINE, on the line past the cross");
            TEST_CHECK(LF2_ID, (s_run.end == LF2_END_JCT) && s_run.still
                               && (((s_run.mm_end + s_run.mm_stop) - s_run.mm_post) <= LF2_JMAT_AFTER_MAX_MM),
                       "(j4) stopped right after JUNCTION, wheels still");
        }
    }

    /* (t) Junction turns: the injected command, the pivot, the branch. */
    if ((LF2_TURN_ONLY != 0U) && pose_a_ok)
    {
        for (uint32_t run = 1U; run <= LF2_TURN_RUNS; run++)
        {
            dbg_printf("[LF] %s turn run %lu of %lu", LF2_TURN_NAME, (unsigned long)run,
                       (unsigned long)LF2_TURN_RUNS);
            lf2_drive(&k_turn);
            lf2_turn_report(run);
            lf2_trace_dump("turn");
            TEST_CHECK(LF2_ID, s_run.det_seen && s_run.post_seen && (s_run.n_jct == 1U)
                               && ((s_run.mm_post - s_run.mm_det) >= LF2_JMAT_DELAY_MIN_MM)
                               && ((s_run.mm_post - s_run.mm_det) <= LF2_JMAT_DELAY_MAX_MM),
                       "(t1) cross detected, one JUNCTION 110-135 mm later");
            TEST_CHECK(LF2_ID, s_run.cmd_sent && s_run.turn_seen && s_run.turn_done,
                       "(t2) the VC took the command and turned at the junction");
            TEST_CHECK(LF2_ID, s_run.turn_done && (s_run.turn_deg >= LF2_TURN_DEG_MIN)
                               && (s_run.turn_deg <= LF2_TURN_DEG_MAX),
                       "(t3) the turn is in its window by the encoders");
            TEST_CHECK(LF2_ID, s_run.branch_on && (s_run.n_lost == 0U)
                               && ((s_run.t_branch_ms - s_run.t_turn_end_ms) <= LF2_TURN_FIND_MAX_MS),
                       "(t4) line found after the turn, no TRACK_LOST");
            TEST_CHECK(LF2_ID, (s_run.end == LF2_END_BRANCH) && s_run.still
                               && lf2_pct_ok(s_run.br_c_on, s_run.br_polls),
                       "(t5) followed the branch 300 mm, CENTRE on the line >= 90%");
        }
    }

    /* (k) Curves: the same piece driven each way, then an S-curve. */
    if ((LF2_CURVE_ONLY != 0U) && pose_a_ok)
    {
#if defined(LF_BEND_MIN_Q7)
        dbg_printf("[LF] variant T80: C + bend memory (floor from the regain steering >= %d, easing to %d)",
                   (int)LF_BEND_MIN_Q7, (int)LF_BEND_FLOOR_Q7);
#elif defined(LF_RESCUE_N)
        dbg_printf("[LF] variant C: RIGHT keeps the line from being lost (%u samples)", (unsigned)LF_RESCUE_N);
#elif defined(LF_CURVE_START_Q7)
        dbg_printf("[LF] variant B: curve recovery start %d after steady tracking", (int)LF_CURVE_START_Q7);
#else
        dbg_printf("[LF] line following: %s", LF2_MODE_TEXT);
#endif
        for (uint32_t g = LF2_CURVE_FIRST; g < (LF2_CURVE_FIRST + LF2_CURVE_GROUPS); g++)
        {
            dbg_printf("[LF] >>> %s: drives %lu-%lu, all %lu the same way.  Do not alternate.",
                       k_curve_group[g], (unsigned long)(((g - LF2_CURVE_FIRST) * LF2_CURVE_RUNS) + 1U),
                       (unsigned long)((g - LF2_CURVE_FIRST + 1U) * LF2_CURVE_RUNS), (unsigned long)LF2_CURVE_RUNS);
            for (uint32_t run = 1U; run <= LF2_CURVE_RUNS; run++)
            {
                dbg_printf("[LF] >>> drive %lu of %lu: %s", (unsigned long)(((g - LF2_CURVE_FIRST) * LF2_CURVE_RUNS) + run),
                           (unsigned long)(LF2_CURVE_GROUPS * LF2_CURVE_RUNS), k_curve_group[g]);
                lf2_drive(&k_curve[g]);
                lf2_curve_report(k_curve_group[g], run);
                lf2_trace_dump("curve");
                /* On the line at the STOP: a drive stopped by distance while
                 * it was still searching (A/B 16 Nov) is not a pass. */
                TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.n_lost == 0U) && s_run.still
                                   && (s_run.st_end == LF_ST_ON_LINE),
                           "(k1) on the line at the exit stop, no TRACK_LOST");
                TEST_CHECK(LF2_ID, (s_run.cv_n_det == 0U) && (s_run.n_jct == 0U),
                           "(k2) no junction detected (a curve has none)");
                TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.cv_exit_polls > 0U)
                                   && lf2_pct_ok(s_run.cv_exit_band, s_run.cv_exit_polls),
                           "(k3) exit straight: CENTRE on line >= 90%");
            }
        }
    }

    /* (d) Controlled moving drives (S3_DRIVE_MM): the production line
     * following on whatever line it is put on, with obstacle avoidance free to
     * take over and hand back.  No layout is assumed. */
    if ((LF2_DRIVE_ONLY != 0U) && pose_a_ok)
    {
        static lf2_drive_t const k_drive =
        {
            "drive", "on a STRAIGHT part of the line: " LF2_START_TEXT,
            (uint32_t)S3_DRIVE_MM, LF2_MS_FOR((uint32_t)S3_DRIVE_MM) + LF2_DRIVE_EXTRA_MS, false, false, true
        };

        dbg_printf("[LF] line following: %s", LF2_MODE_TEXT);
        dbg_printf("[LF] DRIVE: %lu drives, each up to %lu mm of wheel travel or %lu ms; obstacle avoidance may "
                   "take over and hand back", (unsigned long)LF2_DRIVE_RUNS, (unsigned long)S3_DRIVE_MM,
                   (unsigned long)k_drive.max_ms);
        for (uint32_t run = 1U; run <= LF2_DRIVE_RUNS; run++)
        {
            dbg_printf("[LF] >>> drive %lu of %lu", (unsigned long)run, (unsigned long)LF2_DRIVE_RUNS);
            lf2_drive(&k_drive);
            dbg_printf("[LF] drive %lu: %s at %lu mm %lu ms; LINE_LOST %lu, TRACK_LOST %lu, JUNCTION %lu, "
                       "avoidance %lu, still %d", (unsigned long)run, k_end_name[s_run.end],
                       (unsigned long)s_run.mm_end, (unsigned long)s_run.t_end_ms, (unsigned long)s_run.cv_n_lost,
                       (unsigned long)s_run.n_lost, (unsigned long)s_run.n_jct, (unsigned long)s_run.n_avoid,
                       s_run.still ? 1 : 0);
            /* Where it failed, and commanded against measured speed. */
            dbg_printf("[LF] drive %lu: speed %lu mm/s measured (cruise %lu commanded); first LINE_LOST at %ld mm; "
                       "LINE_LOST -> TRACK_LOST %lu ms; wheels moved %lu mm after the end; largest |steer| %ld",
                       (unsigned long)run, (unsigned long)s_run.speed_mmps, (unsigned long)LF2_CRUISE_MMPS,
                       (s_run.cv_first_lost_mm == LF2_NEVER) ? -1L : (long)s_run.cv_first_lost_mm,
                       (unsigned long)((s_run.n_lost > 0U) ? s_run.lost_to_track_ms : 0U),
                       (unsigned long)s_run.mm_stop, (long)s_run.cv_steer_max);
            lf2_trace_dump("drive");
            /* What a drive on an unknown line can show: that it stopped
             * safely, and whether it got all the way on the line. */
            TEST_CHECK(LF2_ID, s_run.still && (s_run.end != LF2_END_NOT_PLACED) && (s_run.end != LF2_END_NOT_READY),
                       "(d1) drive ran and ended with the wheels still");
            TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.n_lost == 0U) && (s_run.st_end == LF_ST_ON_LINE),
                       "(d2) reached the distance limit on the line, no TRACK_LOST");
        }
    }

    if (LF2_STRAIGHT_ONLY == 0U)
    {
        /* (r) Small offsets: recovery. */
        lf2_drive(&k_r1);
        TEST_CHECK(LF2_ID, lf2_recovered() && s_run.first_seen && (s_run.first_steer < 0),
                   "(r1) right of line: steers left, ON_LINE <1 s, settles");
        lf2_drive(&k_r2);
        TEST_CHECK(LF2_ID, lf2_recovered(),
                   "(r2) nose 15 deg left: ON_LINE <1 s, settles");
        lf2_drive(&k_r3);
        TEST_CHECK(LF2_ID, lf2_recovered(),
                   "(r3) nose 15 deg right: ON_LINE <1 s, settles");

        /* (j) Cross junction. */
        lf2_drive(&k_jct);
        lf2_log_right("junction");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.n_jct == 1U) && (s_run.ev_fail == 0U)
                             && (lf_norm(&s_now.cal.r, s_run.r_peak_raw) > LF_DARK_MIN_PM)
                             && (s_run.st_end == LF_ST_ON_LINE) && (s_run.n_lost == 0U),
                   "(j) cross: one JUNCTION, RIGHT DARK, tracking after");

        /* (f) Line end: LINE_LOST, the search, TRACK_LOST, the VC stops. */
        lf2_drive(&k_end);
        dbg_printf("[LF] line end: LINE_LOST to TRACK_LOST %lu ms", (unsigned long)s_run.lost_to_track_ms);
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_TRACK_LOST) && (s_run.n_lost == 1U)
                             && (s_run.lost_to_track_ms >= LF2_LOST_TO_TRACK_MIN)
                             && (s_run.lost_to_track_ms <= LF2_LOST_TO_TRACK_MAX),
                   "(f1) line end: one TRACK_LOST 2110-2135 ms after LOST");
        lf_task_get_status(&s_now);
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_TRACK_LOST)
                             && (vc_get_system_state() == VC_SYS_READY_WAITING)
                             && (s_now.state == LF_ST_IDLE) && !lf2_line_found_flag() && s_run.still,
                   "(f2) VC READY, wheels still, LF IDLE, flag clear");
    }

    /* (h) The whole run. */
    lf_task_get_status(&s_now);
    dbg_printf("[LF] totals: %lu samples, %lu missed, %lu dropped posts, %lu failed events",
               (unsigned long)s_now.releases, (unsigned long)s_now.missed,
               (unsigned long)s_now.posts_dropped, (unsigned long)s_now.ev_send_fail);
    TEST_CHECK(LF2_ID, (s_now.missed == 0U) && (s_now.posts_dropped == 0U) && (s_now.ev_send_fail == 0U),
               "(h) run: no missed period, dropped post, failed event");

    /* Leave the car stopped and the task disarmed for the later tests. */
    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        lf2_remote(DREP_NAV_REMOTE_STOP);
    }
    lf_task_test_arm(false);
    dbg_printf("[LF] %s done: the car stays stopped", LF2_ID);

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

#endif /* APP_TEST */
