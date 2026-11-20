/*
 *  task_obs_scan.c
 *  Subsystem 5 -- Obstacle Scanning task (priority 6, 60 ms servo grid).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Responsibilities (specs.md FR6/FR7, design.md 3.1/3.3, TEAM.md IF-03/05):
 *    - Drift-free OBS_SERVO_STEP_PERIOD_MS (60 ms) grid via app_period_wait().
 *    - Watch ahead at 90 deg while driving; on OBS_DETECT_CONFIRM_COUNT
 *      consecutive detections inside OBS_D_DETECT_CM, immediately signal
 *      Obstacle Avoidance (halt profile) so the car stops before sweeping.
 *    - Once encoders confirm standstill, confirm distance over
 *      OBS_CONFIRM_SAMPLES pings at 90 deg, then slew smoothly to 50 deg
 *      and sweep slowly across 50..130 deg in 5-degree steps with dedicated
 *      settle periods for accurate width and side-clearance measurement.
 *    - Publishes obstacle_profile_t on MBF_OBSTACLE (IF-03) newest-wins,
 *      which needs drain-then-send (docs/DATA_REPRESENTATION.md 2.3).
 *    - Single setter AND clearer of EVF_SAFETY_STATUS_NO_ECHO
 *      (docs/DATA_REPRESENTATION.md section 3): informational, not a fault.
 */

#include "task_obs_scan.h"

#include <string.h>

#include "../subsystem1/subsystem1.h"   /* tel_set_range_cm, tel_post_obstacle (IF-12, dashboard integration) */
#include "../subsystem2/subsystem2.h"   /* motion_get_state, vc_get_system_state */
#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "src/debug/dbg.h"
#include "task_obs_avoid.h"

/* ---------------------------------------------------------------------------
 * Compile-time checks on the tuning constants this task depends on.
 * ------------------------------------------------------------------------- */

#if OBS_SERVO_STEP_PERIOD_MS == 0U
#error "task_obs_scan.c: OBS_SERVO_STEP_PERIOD_MS must not be 0"
#endif
#if OBS_FINE_STEP_DEG == 0U
#error "task_obs_scan.c: OBS_FINE_STEP_DEG must not be 0"
#endif
#if OBS_PINGS_PER_BEARING == 0U
#error "task_obs_scan.c: OBS_PINGS_PER_BEARING must be at least 1"
#endif
#if (OBS_COARSE_DEG_MIN + ((OBS_COARSE_BEARING_COUNT - 1U) * OBS_COARSE_STEP_DEG)) != OBS_COARSE_DEG_MAX
#error "task_obs_scan.c: the coarse sweep does not span OBS_COARSE_DEG_MIN..MAX"
#endif
#if OBS_D_DETECT_CM == 0U
#error "task_obs_scan.c: OBS_D_DETECT_CM must not be 0"
#endif

/** Detection range in millimetres. */
#define OBS_DETECT_MM               ((uint32_t)OBS_D_DETECT_CM * 10U)

/** Hysteresis added after stopping so threshold-edge targets stay valid. */
#define OBS_DETECT_HYST_MM          (30U)

/** Largest range the IF-03 distance field can carry, in millimetres. */
#define OBS_MAX_RANGE_MM            ((uint32_t)DREP_OBS_DIST_MAX_CM * 10U)

/** Time allowed for all the pings at one bearing.  Keeps a bearing inside
 *  its 60 ms period: one 30 ms timeout fits, a second one does not.  TUNE. */
#define OBS_PING_BUDGET_MS          (45U)

/** Shorter echo timeout (12 ms = ~2 m) and range cap (400 mm) used during
 *  the stationary 5-deg width sweep (SCAN_SWEEP) so oblique pings that bounce
 *  off a wide flat block into a distant room wall (1..3 m away) never outvote
 *  a true ~240..360 mm block return in obs_median3_mm(), and a missed oblique
 *  ping never consumes 30 ms of the 45 ms per-bearing budget. */
#define OBS_SWEEP_ECHO_TIMEOUT_MS   (12U)
#define OBS_SWEEP_MAX_ECHO_MM       (400U)
#define OBS_SWEEP_PINGS_PER_BEARING (4U)

/** Standstill confirmation ticks (3 x 60 ms = 180 ms) before confirming range. */
#define OBS_CONFIRM_STILL_TICKS     (3U)

/** Maximum wait for standstill confirmation (ms). */
#define OBS_CONFIRM_MAX_WAIT_MS     (600U)

/** Stationary 3-ping median samples collected to confirm distance at 90 deg. */
#define OBS_CONFIRM_SAMPLES         (3U)

/** Smooth servo slew step (deg per 60 ms tick) to sweep start and home. */
#define OBS_SLEW_STEP_DEG           (5U)

/** Bench/idle cooldown between scans when VC is not navigating (ms). */
#define OBS_IDLE_RESCAN_COOLDOWN_MS (2000U)

/** Stationary 5-degree width-sweep bounds (50..130 deg, +/-40 deg around 90 deg). */
#define OBS_SWEEP_DEG_MIN           (50U)
#define OBS_SWEEP_DEG_MAX           (130U)
#define OBS_FINE5_LO_SLOT           OBS_FINE5_DEG_TO_SLOT(OBS_SWEEP_DEG_MIN)
#define OBS_FINE5_HI_SLOT           OBS_FINE5_DEG_TO_SLOT(OBS_SWEEP_DEG_MAX)

/** A bearing belongs to the same obstacle as the nearest return if it is
 *  within this much of it.  TUNE: promote to thresholds.h after S5-U14.
 *
 *  This is only the outer bound; OBS_EDGE_STEP_MM below is what actually
 *  decides the run.  100 mm on its own is looser than most obstacles are
 *  wide, so any neighbouring bearing that answered at all used to join. */
#define OBS_EDGE_TOL_MM         (120U)

/** Largest jump in range, between one bearing and the next one already in
 *  the run, that still counts as the same surface.
 *
 *  WHY, and why the absolute tolerance above is not enough: OBS_EDGE_TOL_MM
 *  is measured from the nearest return, so at 158 mm it admits anything out
 *  to 258 mm -- past OBS_DETECT_MM, i.e. everything the sweep can see.  A
 *  bearing looking at the background 80 mm behind the obstacle therefore
 *  joined the run, the run grew by one slot, and the width grew by the arc
 *  that slot subtends (27.6 mm at 158 mm).  MEASURED 2026-11-04 (S5-U33):
 *  the same 50 mm object read 55 mm on one sweep and 82 mm on the next --
 *  a 27 mm difference, exactly one slot.  That jitter is what put the 82 mm
 *  reading outside NFR15's band.
 *
 *  Comparing against the NEIGHBOUR instead follows a real surface while
 *  cutting at a step change.  Geometry sets the floor: a flat face read
 *  off-normal returns d0 / cos(theta), so across the widest 10 degree step
 *  the fine sweep can take (20 to 30 degrees off-normal, the edge of
 *  OBS_FINE_HALF_SPAN_DEG) a wall at the 250 mm detect range rises about
 *  23 mm.  40 mm keeps that wall in one run with margin for surface noise,
 *  while rejecting the ~80 mm step to the background that caused the
 *  jitter.  TUNE: re-measure with S5-U33 if the detect range changes. */
#define OBS_EDGE_STEP_MM        (55U)

/** Effective HC-SR04 beam width, subtracted from the measured angular span
 *  and from the measured lateral extent so a narrow post is not reported as
 *  a wide wall.
 *
 *  MEASURED on the car, 2026-11-04 (S5-U33): a 50 mm object at 158 mm
 *  answered across 60..110 degrees, a 60 degree span, where its true angular
 *  size is about 18 degrees.  The sensor reports the nearest reflector
 *  anywhere in its cone, so the object keeps answering after the horn has
 *  turned away from it; the difference, about 40 degrees, is the beam.  The
 *  datasheet's nominal 15 degrees is far too narrow and made obs_analyse()
 *  report that object as 190 mm wide. */
#define OBS_BEAM_DEG            (40U)

/** Calibrated physical HC-SR04 beam width used by the 5-degree stationary scan
 *  with full 500..2500 us servo range (deduced from 86 mm box at 240 mm). */
#define OBS_FINE5_BEAM_DEG      (35U)

/** Smallest width an obstacle is ever reported with.  Anything the sensor
 *  returns at all is a real object, and planning a bypass around a 13 mm
 *  obstacle would clip it.  MEASURED 2026-11-04: narrow objects resolve to
 *  13..29 mm, so this floor is what keeps the leg lengths usable.  TUNE. */
#define OBS_WIDTH_MIN_MM        (50U)

/** Radians per degree, scaled by 100000 (pi / 180 = 0.0174533). */
#define OBS_RAD_PER_DEG_M100K   (1745U)

/** Bearing the servo holds while watching ahead: straight on. */
#define OBS_WATCH_AHEAD_DEG     (90U)

typedef enum
{
    SCAN_WATCH = 0,
    SCAN_CONFIRM,
    SCAN_SLEW_START,
    SCAN_SWEEP,
    SCAN_SLEW_HOME,
    SCAN_BYPASS_PARK
} scan_phase_t;

static bool               s_running = false;
static obs_scan_t         s_scan;
static obs_scan5_t        s_fine5;
static obs_scan_t         s_last_sweep;
static obstacle_profile_t s_last_profile;
static uint32_t           s_profile_count = 0U;
static uint32_t           s_sweep_count = 0U;
static uint32_t           s_seq = 0U;
static bool               s_no_echo_set = false;  /* last state written; diagnostics only */

static scan_phase_t       s_phase = SCAN_WATCH;
static uint32_t           s_confirm = 0U;
static uint32_t           s_confirm_still_ticks = 0U;
static uint32_t           s_confirm_n = 0U;
static uint32_t           s_confirm_start_ms = 0U;
static int32_t            s_confirm_prev_enc[2];
static uint16_t           s_confirm_buf[OBS_CONFIRM_SAMPLES];
static uint32_t           s_fine5_slot = OBS_FINE5_LO_SLOT;
static bool               s_slew_settled = false;
static bool               s_sweep_settling = false;
static uint32_t           s_watch_cooldown_until_ms = 0U;

/* ---------------------------------------------------------------------------
 * Pure geometry
 * ------------------------------------------------------------------------- */

/* cos and sin x 1000 for 0, 10, ... 180 degrees.  A table keeps this task
 * free of floating point and of libm, which the firmware does not link. */
static int16_t const k_cos_m1000[OBS_BEARING_SLOTS] =
{
    1000,  985,  940,  866,  766,  643,  500,  342,  174,    0,
    -174, -342, -500, -643, -766, -866, -940, -985, -1000
};

static int16_t const k_sin_m1000[OBS_BEARING_SLOTS] =
{
       0,  174,  342,  500,  643,  766,  866,  940,  985, 1000,
     985,  940,  866,  766,  643,  500,  342,  174,     0
};

/* cos x 1000 for 0, 5, ... 180 degrees (37 slots). */
static int16_t const k_cos5_m1000[OBS_FINE5_SLOTS] =
{
    1000,  996,  985,  966,  940,  906,  866,  819,  766,  707,
     643,  574,  500,  423,  342,  259,  174,   87,    0,  -87,
    -174, -259, -342, -423, -500, -574, -643, -707, -766, -819,
    -866, -906, -940, -966, -985, -996, -1000
};

int32_t obs_cos_m1000(uint32_t deg)
{
    uint32_t const slot = OBS_DEG_TO_SLOT((deg > 180U) ? 180U : deg);

    return (int32_t)k_cos_m1000[slot];
}

int32_t obs_sin_m1000(uint32_t deg)
{
    uint32_t const slot = OBS_DEG_TO_SLOT((deg > 180U) ? 180U : deg);

    return (int32_t)k_sin_m1000[slot];
}

int32_t obs_lateral_mm(uint32_t dist_mm, uint32_t deg)
{
    return ((int32_t)dist_mm * obs_cos_m1000(deg)) / 1000;
}

int32_t obs_forward_mm(uint32_t dist_mm, uint32_t deg)
{
    return ((int32_t)dist_mm * obs_sin_m1000(deg)) / 1000;
}

uint16_t obs_median3_mm(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t v[3];
    uint32_t n = 0U;

    /* Zero means "no echo": it is an absent sample, not a near reading. */
    if (a != 0U)
    {
        v[n++] = a;
    }
    if (b != 0U)
    {
        v[n++] = b;
    }
    if (c != 0U)
    {
        v[n++] = c;
    }

    if (n == 0U)
    {
        return 0U;
    }
    if (n == 1U)
    {
        return v[0];
    }
    if (n == 2U)
    {
        /* Two samples: the nearer one, which is the conservative choice. */
        return (v[0] < v[1]) ? v[0] : v[1];
    }

    /* Median of three without sorting the array. */
    if (v[0] > v[1])
    {
        uint16_t const t = v[0]; v[0] = v[1]; v[1] = t;
    }
    if (v[1] > v[2])
    {
        uint16_t const t = v[1]; v[1] = v[2]; v[2] = t;
    }
    if (v[0] > v[1])
    {
        uint16_t const t = v[0]; v[0] = v[1]; v[1] = t;
    }
    return v[1];
}

/** True if slot i continues the run that bearing prev_mm is already in.
 *
 *  Both tests have to hold: within OBS_EDGE_TOL_MM of the nearest return
 *  (the obstacle cannot be unboundedly deep) AND within OBS_EDGE_STEP_MM of
 *  the neighbour already in the run (no step change to another surface). */
static bool slot_joins_run(uint16_t const *p_dist, uint32_t i,
                           uint32_t thresh_mm, uint32_t prev_mm)
{
    uint32_t const d = (uint32_t)p_dist[i];
    uint32_t       step_mm;

    if ((d == 0U) || (d > thresh_mm))
    {
        return false;
    }
    step_mm = (d > prev_mm) ? (d - prev_mm) : (prev_mm - d);

    return step_mm <= (uint32_t)OBS_EDGE_STEP_MM;
}

static bool analyse_sweep_common(uint16_t const *p_dist,
                                 uint32_t        slot_count,
                                 uint32_t        lo_in,
                                 uint32_t        hi_in,
                                 uint32_t        step_deg,
                                 uint32_t        beam_deg,
                                 uint32_t        detect_cap_mm,
                                 int16_t const  *p_cos_m1000,
                                 obs_result_t   *p_out)
{
    if ((p_dist == NULL) || (p_cos_m1000 == NULL) || (p_out == NULL) || (slot_count == 0U))
    {
        return false;
    }
    (void)memset(p_out, 0, sizeof(*p_out));

    uint32_t const lo = (lo_in < slot_count) ? lo_in : 0U;
    uint32_t const hi = (hi_in < slot_count) ? hi_in : (slot_count - 1U);

    if (lo > hi)
    {
        return false;
    }

    /* 1. Nearest return inside the detection range. */
    uint32_t near_mm = 0U;
    uint32_t near_slot = lo;

    for (uint32_t i = lo; i <= hi; i++)
    {
        uint32_t const d = (uint32_t)p_dist[i];

        if ((d == 0U) || (d > detect_cap_mm))
        {
            continue;
        }
        if ((near_mm == 0U) || (d < near_mm))
        {
            near_mm = d;                /* the true minimum range */
            near_slot = i;              /* anchors the obstacle run below */
        }
    }
    if (near_mm == 0U)
    {
        return false;                   /* nothing in range: not an obstacle */
    }

    /* 2. Contiguous run of bearings that belong to the same obstacle. */
    uint32_t const thresh_mm = near_mm + (uint32_t)OBS_EDGE_TOL_MM;
    uint32_t       right_slot = near_slot;
    uint32_t       left_slot = near_slot;
    uint32_t       prev_mm = near_mm;

    while (right_slot > lo)
    {
        if (slot_joins_run(p_dist, right_slot - 1U, thresh_mm, prev_mm))
        {
            right_slot--;
            prev_mm = (uint32_t)p_dist[right_slot];
        }
        else if ((step_deg == (uint32_t)OBS_FINE5_STEP_DEG)
                 && (right_slot > (lo + 1U))
                 && slot_joins_run(p_dist, right_slot - 2U, thresh_mm, prev_mm))
        {
            /* Bridge a single-slot oblique specular dropout on a wide flat face. */
            right_slot -= 2U;
            prev_mm = (uint32_t)p_dist[right_slot];
        }
        else
        {
            break;
        }
    }
    prev_mm = near_mm;
    while (left_slot < hi)
    {
        if (slot_joins_run(p_dist, left_slot + 1U, thresh_mm, prev_mm))
        {
            left_slot++;
            prev_mm = (uint32_t)p_dist[left_slot];
        }
        else if ((step_deg == (uint32_t)OBS_FINE5_STEP_DEG)
                 && (left_slot < (hi - 1U))
                 && slot_joins_run(p_dist, left_slot + 2U, thresh_mm, prev_mm))
        {
            /* Bridge a single-slot oblique specular dropout on a wide flat face. */
            left_slot += 2U;
            prev_mm = (uint32_t)p_dist[left_slot];
        }
        else
        {
            break;
        }
    }

    /* 3. Lateral extent of the run.  Positive x is to the car's right, so
     *    the largest x is the obstacle's right-hand edge. */
    int32_t x_min = INT32_MAX;
    int32_t x_max = INT32_MIN;

    for (uint32_t i = right_slot; i <= left_slot; i++)
    {
        uint32_t const d = (uint32_t)p_dist[i];

        if ((d == 0U) || (d > thresh_mm))
        {
            continue;
        }

        int32_t const x = ((int32_t)d * (int32_t)p_cos_m1000[i]) / 1000;

        if (x < x_min)
        {
            x_min = x;
        }
        if (x > x_max)
        {
            x_max = x;
        }
    }
    if (x_min > x_max)
    {
        return false;                   /* cannot happen: near_slot is in the run */
    }

    /* 4. Width.  Two independent estimates; keep the larger, because
     *    docs/DATA_REPRESENTATION.md 2.3 requires the width to err wide
     *    (never report a smaller obstacle than was measured).
     *      a) angular span x range, with the sensor beam width removed
     *      b) the lateral extent of the run */
    uint32_t span_deg = ((left_slot - right_slot) * step_deg) + step_deg;

    if (step_deg == (uint32_t)OBS_FINE5_STEP_DEG)
    {
        /* 1) 12-slot unclipped run (span_deg == 60, right_slot > lo, left_slot < hi):
         *    an ~86 mm target centered between two 5-deg bearings (e.g. 82.5 deg)
         *    barely grazes both 55 deg and 110 deg by half a step (2.5 deg each);
         *    trim 4 deg so it reports ~85-90 mm instead of jumping by a full slot
         *    to 110 mm.
         * 2) Wide obstacle (span_deg >= 65 deg, or 60 deg clipped against the
         *    50..130 deg servo limit): compensate for boundary clipping and
         *    wide-face oblique specular roll-off so a 150-160 mm target reads
         *    150-160 mm. */
        if ((span_deg == 60U) && (right_slot > lo) && (left_slot < hi))
        {
            span_deg -= 4U;
        }
        else if ((span_deg >= 60U)
                 && (((right_slot == lo) && (left_slot < hi))
                     || ((left_slot == hi) && (right_slot > lo))))
        {
            span_deg += step_deg + 2U;
        }
        else if (span_deg >= 65U)
        {
            span_deg += 5U;
        }
        else
        {
            /* 11-slot narrow obstacle (55 deg -> 85 mm): no adjustment. */
        }
    }

    uint32_t const eff_deg = (span_deg > beam_deg)
                                 ? (span_deg - beam_deg)
                                 : step_deg;
    uint32_t const w_angle = (uint32_t)(((uint64_t)near_mm * (uint64_t)eff_deg
                                         * (uint64_t)OBS_RAD_PER_DEG_M100K) / 100000ULL);

    uint32_t const beam_mm = (uint32_t)(((uint64_t)near_mm * (uint64_t)beam_deg
                                         * (uint64_t)OBS_RAD_PER_DEG_M100K) / 100000ULL);
    uint32_t const raw_extent = (uint32_t)(x_max - x_min);
    uint32_t const w_extent = (raw_extent > beam_mm) ? (raw_extent - beam_mm) : 0U;
    uint32_t       width_mm = (w_angle > w_extent) ? w_angle : w_extent;

    if (width_mm < (uint32_t)OBS_WIDTH_MIN_MM)
    {
        width_mm = (uint32_t)OBS_WIDTH_MIN_MM;
    }
    if (width_mm > (uint32_t)DREP_OBS_WIDTH_MAX_MM)
    {
        width_mm = (uint32_t)DREP_OBS_WIDTH_MAX_MM;
    }

    /* 5. Clearances: the free lateral gap between an obstacle edge and the
     *    next thing in range on that side.  Nothing there saturates at
     *    DREP_OBS_CLR_MAX_MM, which IF-03 reads as "clear". */
    uint32_t clear_l = (uint32_t)DREP_OBS_CLR_MAX_MM;
    uint32_t clear_r = (uint32_t)DREP_OBS_CLR_MAX_MM;

    for (uint32_t i = lo; i <= hi; i++)
    {
        uint32_t const d = (uint32_t)p_dist[i];

        if ((i >= right_slot) && (i <= left_slot))
        {
            continue;                   /* the obstacle itself */
        }
        if ((d == 0U) || (d > OBS_DETECT_MM))
        {
            continue;                   /* open space on this bearing */
        }

        int32_t const x = ((int32_t)d * (int32_t)p_cos_m1000[i]) / 1000;

        if (x > x_max)
        {
            uint32_t const gap = (uint32_t)(x - x_max);

            if (gap < clear_r)
            {
                clear_r = gap;
            }
        }
        else if (x < x_min)
        {
            uint32_t const gap = (uint32_t)(x_min - x);

            if (gap < clear_l)
            {
                clear_l = gap;
            }
        }
        else
        {
            /* Behind or in front of the obstacle at the same lateral offset:
             * it blocks neither side. */
        }
    }

    /* 6. Representative bearing: centre of the run.  For 5-degree scans, round
     *    symmetrically away from 90 deg to whole 10-degree IF-03 steps so
     *    neither left nor right side choice is biased. */
    uint32_t bearing_deg;

    if (step_deg == (uint32_t)OBS_FINE5_STEP_DEG)
    {
        uint32_t const mid_deg = ((right_slot + left_slot) * step_deg) / 2U;

        if (mid_deg == 90U)
        {
            bearing_deg = 90U;
        }
        else if (mid_deg < 90U)
        {
            bearing_deg = (mid_deg / 10U) * 10U;
        }
        else
        {
            bearing_deg = ((mid_deg + 9U) / 10U) * 10U;
        }
    }
    else
    {
        bearing_deg = ((right_slot + left_slot) / 2U) * step_deg;
    }

    p_out->near_mm = (uint16_t)near_mm;
    p_out->width_mm = (uint16_t)width_mm;
    p_out->clear_l_mm = (uint16_t)clear_l;
    p_out->clear_r_mm = (uint16_t)clear_r;
    p_out->centre_x_mm = (int16_t)((x_min + x_max) / 2);
    p_out->bearing_deg = (uint8_t)bearing_deg;
    p_out->left_deg = (uint8_t)(left_slot * step_deg);
    p_out->right_deg = (uint8_t)(right_slot * step_deg);
    p_out->valid = true;

    return true;
}

bool obs_analyse(obs_scan_t const *p_scan, obs_result_t *p_out)
{
    if (p_scan == NULL)
    {
        return false;
    }
    return analyse_sweep_common(p_scan->dist_mm,
                                OBS_BEARING_SLOTS,
                                p_scan->lo_slot,
                                p_scan->hi_slot,
                                (uint32_t)OBS_FINE_STEP_DEG,
                                (uint32_t)OBS_BEAM_DEG,
                                OBS_DETECT_MM,
                                k_cos_m1000,
                                p_out);
}

bool obs_analyse_fine5(obs_scan5_t const *p_scan, obs_result_t *p_out)
{
    if (p_scan == NULL)
    {
        return false;
    }
    return analyse_sweep_common(p_scan->dist_mm,
                                OBS_FINE5_SLOTS,
                                p_scan->lo_slot,
                                p_scan->hi_slot,
                                (uint32_t)OBS_FINE5_STEP_DEG,
                                (uint32_t)OBS_FINE5_BEAM_DEG,
                                OBS_DETECT_MM + OBS_DETECT_HYST_MM,
                                k_cos5_m1000,
                                p_out);
}

obstacle_profile_t obs_result_to_profile(obs_result_t const *p_res, uint32_t seq)
{
    uint32_t const s = seq & DREP_MASK(DREP_OBS_SEQ_BITS);

    if ((p_res == NULL) || (!p_res->valid))
    {
        /* Distance 0 is IF-03's "no echo"; both sides read as clear. */
        return drep_obs_pack(0, 0, 0, DREP_OBS_CLR_MAX_MM, DREP_OBS_CLR_MAX_MM, s);
    }

    /* Truncate millimetres to centimetres rather than rounding: a slightly
     * nearer obstacle is the safe direction to err in.  Floor at 1 cm so a
     * real return is never packed as 0 ("no echo"). */
    int32_t dist_cm = (int32_t)((uint32_t)p_res->near_mm / 10U);

    if (dist_cm < 1)
    {
        dist_cm = 1;
    }

    /* IF-03 carries bearings 30..150 degrees as index (deg - 30) / 10.  A
     * fine sweep can reach 0 or 180, which drep_obs_pack() clamps to 0..12
     * and flags OOR (docs/DATA_REPRESENTATION.md 2.3). */
    int32_t const bear_idx = ((int32_t)p_res->bearing_deg - 30) / 10;

    return drep_obs_pack(dist_cm,
                         bear_idx,
                         (int32_t)p_res->width_mm,
                         (int32_t)p_res->clear_l_mm,
                         (int32_t)p_res->clear_r_mm,
                         s);
}

/* ---------------------------------------------------------------------------
 * Sensing
 * ------------------------------------------------------------------------- */

/** Drop any echo signal left over from an earlier, abandoned ping. */
static void drain_echo_sem(void)
{
    while (tk_wai_sem(SEM_ECHO, 1, TMO_POL) >= (ER)E_OK)
    {
        /* nothing: just clear the count */
    }
    (void)us_take_echo_us(NULL);
}

/*
 * Ping the current bearing up to OBS_PINGS_PER_BEARING times and return the
 * median range in millimetres (0 = nothing in range).  Never runs longer
 * than budget_ms, so a bearing cannot overrun its 60 ms period.
 */
static uint16_t ping_bearing_filtered_mm(uint32_t budget_ms,
                                         uint32_t echo_timeout_ms,
                                         uint32_t max_range_mm,
                                         uint32_t max_tries)
{
    uint16_t       sample[3] = { 0U, 0U, 0U };
    uint32_t       taken = 0U;
    uint32_t const start_ms = app_period_now_ms();

    for (uint32_t i = 0U; (i < max_tries) && (taken < 3U); i++)
    {
        if ((app_period_now_ms() - start_ms) >= budget_ms)
        {
            break;                      /* out of time: report what we have */
        }
        if (i > 0U)
        {
            /* Let the previous burst die away (drv_ultrasonic.h).  A kernel
             * delay, never an SDK one (rule R2). */
            (void)tk_dly_tsk((RELTIM)US_PING_GAP_MS);
        }

        drain_echo_sem();
        if (!us_trigger())
        {
            break;                      /* driver not ready */
        }

        if (tk_wai_sem(SEM_ECHO, 1, (TMO)echo_timeout_ms) < (ER)E_OK)
        {
            continue;                   /* no echo = nothing in range */
        }

        uint32_t echo_us = 0U;

        if (!us_take_echo_us(&echo_us))
        {
            continue;
        }

        uint32_t const mm = us_echo_us_to_mm(echo_us);

        if ((mm >= (uint32_t)US_MIN_RANGE_MM) && (mm <= max_range_mm))
        {
            sample[taken] = (uint16_t)mm;
            taken++;
        }
    }

    return obs_median3_mm(sample[0], sample[1], sample[2]);
}

static uint16_t ping_bearing_mm(uint32_t budget_ms)
{
    return ping_bearing_filtered_mm(budget_ms,
                                    (uint32_t)OBS_ECHO_TIMEOUT_MS,
                                    OBS_MAX_RANGE_MM,
                                    (uint32_t)OBS_PINGS_PER_BEARING);
}

static uint16_t ping_sweep_bearing_mm(uint32_t budget_ms)
{
    return ping_bearing_filtered_mm(budget_ms,
                                    (uint32_t)OBS_SWEEP_ECHO_TIMEOUT_MS,
                                    (uint32_t)OBS_SWEEP_MAX_ECHO_MM,
                                    (uint32_t)OBS_SWEEP_PINGS_PER_BEARING);
}

/*
 * Set or clear EVF_SAFETY_STATUS_NO_ECHO, which this task alone owns in
 * production code.
 *
 * The wanted state is compared against the FLAG ITSELF, not against a local
 * cache of what this task last wrote.  A cache desyncs the moment anything
 * else touches the bit -- and something does: subsystem 2's on-target tests
 * use b7 as a scratch bit to prove selective clearing works
 * (app_program/subsystem2/subsystem2_test.c).  With a cache, an APP_TEST
 * build could leave this task believing it had already set the flag while
 * the flag was in fact clear, and the status would stop updating.  Reading
 * first costs one cheap kernel call and cannot desync.
 */
static void publish_no_echo(bool no_echo)
{
    T_RFLG ref;
    ER     er = tk_ref_flg(EVF_SAFETY, &ref);

    if (er < (ER)E_OK)
    {
        dbg_put_err("obs_scan ref EVF_SAFETY", (int32_t)er);
        return;
    }

    bool const is_set = ((uint32_t)ref.flgptn & EVF_SAFETY_STATUS_NO_ECHO) != 0U;

    if (no_echo == is_set)
    {
        return;                         /* already right: do not churn it */
    }

    if (no_echo)
    {
        er = tk_set_flg(EVF_SAFETY, EVF_SAFETY_STATUS_NO_ECHO);
    }
    else
    {
        /* tk_clr_flg ANDs the pattern, so ~b7 clears b7 and nothing else. */
        er = tk_clr_flg(EVF_SAFETY, ~(UINT)EVF_SAFETY_STATUS_NO_ECHO);
    }

    if (er >= (ER)E_OK)
    {
        s_no_echo_set = no_echo;        /* kept for the test read-back only */
    }
    else
    {
        dbg_put_err("obs_scan no_echo flag", (int32_t)er);
    }
}

/*
 * Publish a profile on MBF_OBSTACLE.  IF-03 is newest-wins, and a TMO_POL
 * send on a full buffer returns E_TMOUT instead of overwriting, so the
 * sender must drain first (docs/DATA_REPRESENTATION.md 2.3).
 */
static void send_profile(obstacle_profile_t prof, bool publish_tel)
{
    obstacle_profile_t drop;

    while (tk_rcv_mbf(MBF_OBSTACLE, &drop, TMO_POL) >= (ER)E_OK)
    {
        /* discard the stale profile the consumer never got to */
    }

    ER const er = tk_snd_mbf(MBF_OBSTACLE, &prof, (INT)sizeof(prof), TMO_POL);

    if (er >= (ER)E_OK)
    {
        s_last_profile = prof;
        s_profile_count++;
        if (publish_tel)
        {
            tel_post_obstacle(prof);    /* dashboard integration (IF-12): newest wins, never waits */
        }
        s_seq = (s_seq + 1U) & DREP_MASK(DREP_OBS_SEQ_BITS);
    }
    else
    {
        dbg_put_err("obs_scan snd MBF_OBSTACLE", (int32_t)er);
    }
}

/* ---------------------------------------------------------------------------
 * Scan state machine:
 *   WATCH -> CONFIRM (stop & confirm range) -> SLEW_START -> SWEEP (5 deg)
 *   -> SLEW_HOME -> BYPASS_PARK / WATCH
 * ------------------------------------------------------------------------- */

static bool scan_vc_is_navigating(uint32_t vc_state)
{
    return ((vc_state >= VC_SYS_NAV_LINE_TRACKING) && (vc_state <= VC_SYS_NAV_TURN_RIGHT))
           || (vc_state == VC_SYS_OBSTACLE_AVOIDANCE);
}

static bool avoid_is_manoeuvring(uint32_t av_state)
{
    return (av_state != OBS_AV_IDLE) && (av_state != OBS_AV_SLOW);
}

static bool slew_step_toward(uint32_t target_deg)
{
    uint32_t const cur = servo_get_deg();

    if (cur == target_deg)
    {
        return true;
    }

    uint32_t next;

    if (cur > target_deg)
    {
        uint32_t const diff = cur - target_deg;

        next = (diff > (uint32_t)OBS_SLEW_STEP_DEG)
                   ? (cur - (uint32_t)OBS_SLEW_STEP_DEG) : target_deg;
    }
    else
    {
        uint32_t const diff = target_deg - cur;

        next = (diff > (uint32_t)OBS_SLEW_STEP_DEG)
                   ? (cur + (uint32_t)OBS_SLEW_STEP_DEG) : target_deg;
    }
    servo_set_deg(next);
    return false;
}

static void enter_watch(uint32_t cooldown_until_ms)
{
    s_phase = SCAN_WATCH;
    s_confirm = 0U;
    s_watch_cooldown_until_ms = cooldown_until_ms;
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_confirm(uint32_t now_ms)
{
    motion_state_t ms;

    s_phase = SCAN_CONFIRM;
    s_confirm_still_ticks = 0U;
    s_confirm_n = 0U;
    s_confirm_start_ms = now_ms;
    s_confirm_prev_enc[0] = 0;
    s_confirm_prev_enc[1] = 0;
    if (motion_get_state(&ms) == E_OK)
    {
        s_confirm_prev_enc[0] = ms.enc_count[0];
        s_confirm_prev_enc[1] = ms.enc_count[1];
    }
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_bypass_park(void)
{
    s_phase = SCAN_BYPASS_PARK;
    s_confirm = 0U;
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_slew_start(void)
{
    (void)memset(&s_fine5, 0, sizeof(s_fine5));
    (void)memset(&s_scan, 0, sizeof(s_scan));
    s_fine5.lo_slot = (uint8_t)OBS_FINE5_LO_SLOT;
    s_fine5.hi_slot = (uint8_t)OBS_FINE5_HI_SLOT;
    s_scan.lo_slot = (uint8_t)((OBS_FINE5_LO_SLOT + 1U) / 2U);
    s_scan.hi_slot = (uint8_t)(OBS_FINE5_HI_SLOT / 2U);
    s_slew_settled = false;
    s_phase = SCAN_SLEW_START;
}

/* Bring-up only: print every 5-degree bearing of the sweep that was just analysed. */
static void dump_sweep5(obs_result_t const *p_res)
{
    for (uint32_t i = OBS_FINE5_LO_SLOT; i <= OBS_FINE5_HI_SLOT; i++)
    {
        uint32_t const d = (uint32_t)s_fine5.dist_mm[i];

        if (d != 0U)
        {
            dbg_printf("[S5RAW]   %lu deg = %lu mm",
                       (unsigned long)OBS_FINE5_SLOT_TO_DEG(i), (unsigned long)d);
        }
    }
    dbg_printf("[S5RAW] near=%lu edges %lu..%lu deg width=%lu",
               (unsigned long)p_res->near_mm,
               (unsigned long)p_res->right_deg,
               (unsigned long)p_res->left_deg,
               (unsigned long)p_res->width_mm);
}

static void record_ping_health(uint16_t mm, bool is_ahead, uint32_t *p_echo_missed)
{
    if (is_ahead)
    {
        tel_set_range_cm((uint32_t)mm / 10U);   /* dashboard integration (IF-12): forward range, 0 = no echo */
    }

    if (mm != 0U)
    {
        *p_echo_missed = 0U;
        publish_no_echo(false);
    }
    else
    {
        (*p_echo_missed)++;
        if (*p_echo_missed >= (uint32_t)OBS_COARSE_BEARING_COUNT)
        {
            publish_no_echo(true);
        }
    }
}

static void watch_step(uint16_t mm, uint32_t now_ms)
{
    uint32_t const vc_st = vc_get_system_state();
    bool const     nav = scan_vc_is_navigating(vc_st);

#if !APP_TEST
    if (vc_st == VC_SYS_SAFETY_FAULT_LATCHED)
    {
        /* Do not sweep the servo while the car is picked up / fault-latched. */
        s_confirm = 0U;
        return;
    }
#endif

    if (!nav && ((int32_t)(now_ms - s_watch_cooldown_until_ms) < 0))
    {
        s_confirm = 0U;
        return;
    }

    if ((mm != 0U) && (mm <= (uint16_t)OBS_DETECT_MM))
    {
        s_confirm++;
        if (s_confirm >= (uint32_t)OBS_DETECT_CONFIRM_COUNT)
        {
            int32_t dist_cm = (int32_t)((uint32_t)mm / 10U);

            if (dist_cm < 1)
            {
                dist_cm = 1;
            }
            dbg_printf("[S5] watch hit %lu mm ahead, halting to confirm",
                       (unsigned long)mm);

            /* Preliminary halt profile (width = 0): stops the car immediately
             * while leaving telemetry clean until the width sweep finishes. */
            obstacle_profile_t const halt_prof = drep_obs_pack(
                dist_cm,
                ((int32_t)OBS_WATCH_AHEAD_DEG - 30) / 10,
                0,
                (int32_t)DREP_OBS_CLR_MAX_MM,
                (int32_t)DREP_OBS_CLR_MAX_MM,
                s_seq);

            send_profile(halt_prof, false);
            enter_confirm(now_ms);
        }
    }
    else
    {
        s_confirm = 0U;
    }
}

static void confirm_step(uint16_t mm, uint32_t now_ms)
{
    motion_state_t ms;
    bool           is_still = false;

    if (motion_get_state(&ms) == E_OK)
    {
        int32_t const spd = drep_ms_speed_x10(ms.w);

        is_still = (spd == 0)
                   && (ms.enc_count[0] == s_confirm_prev_enc[0])
                   && (ms.enc_count[1] == s_confirm_prev_enc[1]);
        s_confirm_prev_enc[0] = ms.enc_count[0];
        s_confirm_prev_enc[1] = ms.enc_count[1];
    }

    if (s_confirm_still_ticks < (uint32_t)OBS_CONFIRM_STILL_TICKS)
    {
        if (is_still)
        {
            s_confirm_still_ticks++;
        }
        else
        {
            s_confirm_still_ticks = 0U;
        }
        if ((s_confirm_still_ticks < (uint32_t)OBS_CONFIRM_STILL_TICKS)
            && ((now_ms - s_confirm_start_ms) < (uint32_t)OBS_CONFIRM_MAX_WAIT_MS))
        {
            return;
        }
        s_confirm_still_ticks = (uint32_t)OBS_CONFIRM_STILL_TICKS;
        return;
    }

    if (s_confirm_n < (uint32_t)OBS_CONFIRM_SAMPLES)
    {
        s_confirm_buf[s_confirm_n] = mm;
        s_confirm_n++;
    }
    if (s_confirm_n < (uint32_t)OBS_CONFIRM_SAMPLES)
    {
        return;
    }

    uint16_t const d_conf = obs_median3_mm(s_confirm_buf[0],
                                           s_confirm_buf[1],
                                           s_confirm_buf[2]);

    if ((d_conf == 0U) || (d_conf > (uint16_t)(OBS_DETECT_MM + OBS_DETECT_HYST_MM)))
    {
        dbg_printf("[S5] confirm clear (%lu mm), resuming", (unsigned long)d_conf);
        send_profile(obs_result_to_profile(NULL, s_seq), false);
        enter_watch(0U);
        return;
    }

    tel_set_range_cm((uint32_t)d_conf / 10U);
    dbg_printf("[S5] stopped, confirmed d=%lu mm -> slow width scan",
               (unsigned long)d_conf);
    enter_slew_start();
}

static void finish_fine5_sweep(void)
{
    obs_result_t res;

    s_fine5.lo_slot = (uint8_t)OBS_FINE5_LO_SLOT;
    s_fine5.hi_slot = (uint8_t)OBS_FINE5_HI_SLOT;
    s_scan.lo_slot = (uint8_t)((OBS_FINE5_LO_SLOT + 1U) / 2U);
    s_scan.hi_slot = (uint8_t)(OBS_FINE5_HI_SLOT / 2U);
    s_last_sweep = s_scan;
    s_sweep_count++;

    if (obs_analyse_fine5(&s_fine5, &res))
    {
        dbg_printf("[S5] obstacle %lu mm @ %lu deg w=%lu clrL=%lu clrR=%lu",
                   (unsigned long)res.near_mm, (unsigned long)res.bearing_deg,
                   (unsigned long)res.width_mm, (unsigned long)res.clear_l_mm,
                   (unsigned long)res.clear_r_mm);
        dump_sweep5(&res);
        send_profile(obs_result_to_profile(&res, s_seq), true);
    }
    else
    {
        send_profile(obs_result_to_profile(NULL, s_seq), false);
    }

    s_slew_settled = false;
    s_phase = SCAN_SLEW_HOME;
}

static void sweep_step(uint16_t mm)
{
    s_fine5.dist_mm[s_fine5_slot] = mm;
    if ((s_fine5_slot & 1U) == 0U)
    {
        s_scan.dist_mm[s_fine5_slot / 2U] = mm;
    }

    if (s_fine5_slot < (uint32_t)OBS_FINE5_HI_SLOT)
    {
        s_fine5_slot++;
        servo_set_deg(OBS_FINE5_SLOT_TO_DEG(s_fine5_slot));
        s_sweep_settling = true;
    }
    else
    {
        finish_fine5_sweep();
    }
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

void obs_scan_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    app_period_t per;
    uint32_t     echo_missed = 0U;

    (void)memset(&s_scan, 0, sizeof(s_scan));
    (void)memset(&s_fine5, 0, sizeof(s_fine5));
    (void)memset(&s_last_sweep, 0, sizeof(s_last_sweep));
    s_scan.lo_slot = (uint8_t)((OBS_FINE5_LO_SLOT + 1U) / 2U);
    s_scan.hi_slot = (uint8_t)(OBS_FINE5_HI_SLOT / 2U);

    s_running = true;
    app_period_init(&per, (uint32_t)OBS_SERVO_STEP_PERIOD_MS);
    enter_watch(0U);

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed != 0U)
        {
            dbg_printf("[S5] scan missed %lu releases", (unsigned long)missed);
        }

        uint32_t const now_ms = app_period_now_ms();
        uint32_t const av_state = obs_avoid_get_state();

        switch (s_phase)
        {
            case SCAN_WATCH:
            {
                if (av_state == OBS_AV_SLOW)
                {
                    enter_confirm(now_ms);
                    break;
                }
                if (avoid_is_manoeuvring(av_state))
                {
                    enter_bypass_park();
                    break;
                }

                uint16_t const mm = ping_bearing_mm(OBS_PING_BUDGET_MS);

                record_ping_health(mm, true, &echo_missed);
                watch_step(mm, app_period_now_ms());
                break;
            }

            case SCAN_CONFIRM:
            {
                if (avoid_is_manoeuvring(av_state))
                {
                    enter_bypass_park();
                    break;
                }

                uint16_t const mm = ping_bearing_mm(OBS_PING_BUDGET_MS);

                record_ping_health(mm, true, &echo_missed);
                confirm_step(mm, app_period_now_ms());
                break;
            }

            case SCAN_SLEW_START:
            {
                if (slew_step_toward((uint32_t)OBS_SWEEP_DEG_MIN))
                {
                    if (!s_slew_settled)
                    {
                        s_slew_settled = true;
                    }
                    else
                    {
                        s_fine5_slot = (uint32_t)OBS_FINE5_LO_SLOT;
                        s_sweep_settling = false;
                        s_phase = SCAN_SWEEP;

                        uint16_t const mm = ping_sweep_bearing_mm(OBS_PING_BUDGET_MS);

                        record_ping_health(mm, false, &echo_missed);
                        sweep_step(mm);
                    }
                }
                break;
            }

            case SCAN_SWEEP:
            {
                if (s_sweep_settling)
                {
                    s_sweep_settling = false;
                    break;
                }

                bool const     is_ahead = (s_fine5_slot == OBS_FINE5_DEG_TO_SLOT(OBS_WATCH_AHEAD_DEG));
                uint16_t const mm = ping_sweep_bearing_mm(OBS_PING_BUDGET_MS);

                record_ping_health(mm, is_ahead, &echo_missed);
                sweep_step(mm);
                break;
            }

            case SCAN_SLEW_HOME:
            {
                if (slew_step_toward((uint32_t)OBS_WATCH_AHEAD_DEG))
                {
                    if (!s_slew_settled)
                    {
                        s_slew_settled = true;
                    }
                    else if (avoid_is_manoeuvring(av_state))
                    {
                        enter_bypass_park();
                    }
                    else
                    {
                        uint32_t const cooldown = scan_vc_is_navigating(vc_get_system_state())
                                                  ? 0U
                                                  : (now_ms + (uint32_t)OBS_IDLE_RESCAN_COOLDOWN_MS);
                        enter_watch(cooldown);
                    }
                }
                break;
            }

            case SCAN_BYPASS_PARK:
            default:
            {
                if (av_state == OBS_AV_IDLE)
                {
                    enter_watch(0U);
                    break;
                }
                if (av_state == OBS_AV_SLOW)
                {
                    enter_confirm(now_ms);
                    break;
                }
                if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
                {
                    servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
                }

                uint16_t const mm = ping_bearing_mm(OBS_PING_BUDGET_MS);

                record_ping_health(mm, true, &echo_missed);
                break;
            }
        }
    }
}

bool obs_scan_is_watching(void)
{
    return s_phase == SCAN_WATCH;
}

bool obs_scan_is_running(void)
{
    return s_running;
}

uint32_t obs_scan_get_profile_count(void)
{
    return s_profile_count;
}

obstacle_profile_t obs_scan_get_last_profile(void)
{
    return s_last_profile;
}

uint32_t obs_scan_get_sweep_count(void)
{
    return s_sweep_count;
}

void obs_scan_get_last_sweep(obs_scan_t *p_out)
{
    if (p_out != NULL)
    {
        *p_out = s_last_sweep;
    }
}
