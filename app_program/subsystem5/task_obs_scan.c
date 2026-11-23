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
 *      OBS_CONFIRM_SAMPLES pings at 90 deg, then slew smoothly to 51 deg
 *      and sweep slowly across 51..129 deg in 3-degree steps with dedicated
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

/** Shorter echo timeout (8 ms = ~1.37 m) and range cap (400 mm) used during
 *  the stationary 3-deg width sweep (SCAN_SWEEP) so oblique pings that bounce
 *  off a wide flat block into a distant room wall never outvote a true block
 *  return, a missed oblique ping never consumes the 60 ms period, and at least
 *  OBS_SWEEP_MIN_HITS echoes are required to reject single-ping sidelobes. */
#define OBS_SWEEP_ECHO_TIMEOUT_MS   (8U)
#define OBS_SWEEP_BUDGET_MS         (40U)
#define OBS_SWEEP_MAX_ECHO_MM       (400U)
#define OBS_SWEEP_PINGS_PER_BEARING (4U)
#define OBS_SWEEP_MIN_HITS          (2U)

/** Standstill confirmation ticks (3 x 60 ms = 180 ms) before confirming range. */
#define OBS_CONFIRM_STILL_TICKS     (3U)

/** Maximum wait for standstill confirmation (ms). */
#define OBS_CONFIRM_MAX_WAIT_MS     (600U)

/** Stationary 3-ping median samples collected to confirm distance at 90 deg. */
#define OBS_CONFIRM_SAMPLES         (3U)

/** Smooth servo slew step (deg per 60 ms tick) to sweep start and home. */
#define OBS_SLEW_STEP_DEG           (8U)

/** Bench/idle cooldown between scans when VC is not navigating (ms). */
#define OBS_IDLE_RESCAN_COOLDOWN_MS (2000U)

/** Stationary 3-degree width-sweep bounds (51..129 deg, strictly inside
 *  the mechanical 50..130 deg stop and symmetric around 90 deg). */
#define OBS_SWEEP_DEG_MIN           (51U)
#define OBS_SWEEP_DEG_MAX           (129U)
#define OBS_FINE3_LO_SLOT           OBS_FINE3_DEG_TO_SLOT(OBS_SWEEP_DEG_MIN)
#define OBS_FINE3_HI_SLOT           OBS_FINE3_DEG_TO_SLOT(OBS_SWEEP_DEG_MAX)

/** A bearing belongs to the same obstacle as the nearest return if it is
 *  within this much of it (trimmed from 100 to 65 mm so deep block side-wall
 *  reflections never widen a flat-face scan while admitting +-38 deg slant). */
#define OBS_EDGE_TOL_MM         (65U)

/** Largest jump in range, between one bearing and the next one already in
 *  the run, that still counts as the same surface. */
#define OBS_EDGE_STEP_MM        (35U)

/** Effective HC-SR04 beam width, subtracted from the measured angular span
 *  and from the measured lateral extent so a narrow post is not reported as
 *  a wide wall. */
#define OBS_BEAM_DEG            (40U)

/** Calibrated physical HC-SR04 beam width for the 1-hit 5-degree scan (S5-U32). */
#define OBS_FINE5_BEAM_DEG      (35U)

/** Calibrated physical HC-SR04 beam width for the 2-hit 3-degree stationary
 *  scan (OBS_SWEEP_MIN_HITS = 2 clips the noisy outer beam skirt; measured
 *  2026-11-22 across 145 mm runs at 235-252 mm with 1 mm resolution). */
#define OBS_FINE3_BEAM_DEG      (28U)

/** Maximum deconvolved effective span supported by k_2tan_half_m1000[]. */
#define OBS_FINE_EFF_MAX_DEG    (90U)

/** Smallest width an obstacle is ever reported with. */
#define OBS_WIDTH_MIN_MM        (50U)

/** Radians per degree, scaled by 100000 (pi / 180 = 0.0174533). */
#define OBS_RAD_PER_DEG_M100K   (1745U)

/** Bearing the servo holds while watching ahead: straight on. */
#define OBS_WATCH_AHEAD_DEG     (90U)

_Static_assert(OBS_BEARING_SLOTS == 19U, "k_cos5_quad_m1000 stride-2 lookup assumes 19 coarse slots");
_Static_assert(OBS_FINE5_SLOTS == 37U, "k_cos5_quad_m1000 assumes 37 fine-5 slots");
_Static_assert(OBS_FINE3_SLOTS == 61U, "k_cos3_quad_m1000 assumes 61 fine-3 slots");
_Static_assert((((uint64_t)(OBS_DETECT_MM + OBS_DETECT_HYST_MM) * 2000ULL) + 500ULL) <= (uint64_t)UINT32_MAX,
               "fine width_mm 32-bit numerator must not overflow");
_Static_assert(((uint64_t)OBS_DETECT_MM * 190ULL * (uint64_t)OBS_RAD_PER_DEG_M100K) <= (uint64_t)UINT32_MAX,
               "coarse w_angle 32-bit numerator must not overflow");

typedef enum
{
    SCAN_WATCH = 0,
    SCAN_CONFIRM,
    SCAN_SLEW_START,
    SCAN_SWEEP,
    SCAN_SLEW_HOME,
    SCAN_BYPASS_PARK
} scan_phase_t;

typedef struct
{
    volatile uint8_t   phase;             /* scan_phase_t (0..5) */
    volatile bool      running;
    bool               no_echo_set;       /* last state written; diagnostics only */
    bool               slew_settled;
    bool               sweep_settling;
    uint16_t           confirm_buf[OBS_CONFIRM_SAMPLES];
    obstacle_profile_t last_profile;
    volatile uint32_t  profile_count;
    volatile uint32_t  sweep_count;
    uint32_t           seq;
    uint32_t           confirm;
    uint32_t           confirm_still_ticks;
    uint32_t           confirm_n;
    uint32_t           confirm_start_ms;
    int32_t            confirm_prev_enc[2];
    uint32_t           fine3_slot;
    uint32_t           watch_cooldown_until_ms;
    /* Large sweep arrays last so hot scalars stay inside Thumb-1 small offsets. */
    obs_scan_t         scan;
    obs_scan3_t        fine3;
    obs_scan_t         last_sweep;
} obs_scan_ctx_t;

static obs_scan_ctx_t s_ctx;

/* ---------------------------------------------------------------------------
 * Pure geometry
 * ------------------------------------------------------------------------- */

/* Folded quadrant cosine tables (0..90 degrees, scaled by 1000).  Keeps the
 * task free of floating point and libm while cutting .rodata vs full tables. */

/* cos x 1000 for 0, 5, ... 90 degrees (19 entries, 38 B).
 * Also serves 10-degree cos (stride 2) and sin (mirrored about 90 deg). */
static uint16_t const k_cos5_quad_m1000[19] =
{
    1000U, 996U, 985U, 966U, 940U, 906U, 866U, 819U, 766U, 707U,
     643U, 574U, 500U, 423U, 342U, 259U, 174U,  87U,   0U
};

/* cos x 1000 for 0, 3, ... 90 degrees (31 entries, 62 B). */
static uint16_t const k_cos3_quad_m1000[31] =
{
    1000U, 999U, 995U, 988U, 978U, 966U, 951U, 934U, 914U, 891U,
     866U, 839U, 809U, 777U, 743U, 707U, 669U, 629U, 588U, 545U,
     500U, 454U, 407U, 358U, 309U, 259U, 208U, 156U, 105U,  52U,
       0U
};

static inline int32_t cos_from_quad(uint16_t const *p_quad, uint32_t max_slot,
                                    uint32_t stride, uint32_t slot)
{
    uint32_t const mid = max_slot / 2U;

    if (slot <= mid)
    {
        return (int32_t)p_quad[slot * stride];
    }
    return -(int32_t)p_quad[(max_slot - slot) * stride];
}

/* round(2000 * tan(deg / 2)) for effective deconvolved span deg = 0..90. */
static uint16_t const k_2tan_half_m1000[OBS_FINE_EFF_MAX_DEG + 1U] =
{
       0,   17,   35,   52,   70,   87,  105,  122,  140,  157,
     175,  193,  210,  228,  246,  263,  281,  299,  317,  335,
     353,  371,  389,  407,  425,  443,  462,  480,  499,  517,
     536,  555,  573,  592,  611,  631,  650,  669,  689,  708,
     728,  748,  768,  788,  808,  828,  849,  870,  890,  911,
     933,  954,  975,  997, 1019, 1041, 1063, 1086, 1109, 1132,
    1155, 1178, 1202, 1226, 1250, 1274, 1299, 1324, 1349, 1375,
    1400, 1427, 1453, 1480, 1507, 1535, 1563, 1591, 1620, 1649,
    1678, 1708, 1739, 1769, 1801, 1833, 1865, 1898, 1931, 1965,
    2000
};

int32_t obs_cos_m1000(uint32_t deg)
{
    uint32_t const slot = OBS_DEG_TO_SLOT((deg > 180U) ? 180U : deg);

    return cos_from_quad(k_cos5_quad_m1000, OBS_BEARING_SLOTS - 1U, 2U, slot);
}

int32_t obs_sin_m1000(uint32_t deg)
{
    uint32_t const slot = OBS_DEG_TO_SLOT((deg > 180U) ? 180U : deg);
    uint32_t const dist = (slot <= 9U) ? (9U - slot) : (slot - 9U);

    return (int32_t)k_cos5_quad_m1000[dist * 2U];
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
                                 uint16_t const *p_cos_quad,
                                 uint32_t        cos_stride,
                                 obs_result_t   *p_out)
{
    if ((p_dist == NULL) || (p_cos_quad == NULL) || (cos_stride == 0U)
        || (p_out == NULL) || (slot_count == 0U))
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
    bool const     is_fine = (step_deg < (uint32_t)OBS_FINE_STEP_DEG);
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
        else if (is_fine
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
        else if (is_fine
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

        int32_t const x = ((int32_t)d * cos_from_quad(p_cos_quad, slot_count - 1U, cos_stride, i)) / 1000;

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

    /* 4. Width.  Fine stationary sweeps (3-deg / 5-deg) use exact flat-face
     *    tangent geometry W = 2 * near_mm * tan(eff_deg / 2) after removing the
     *    calibrated beam width (beam_deg), avoiding super-linear d_i * cos(theta_i)
     *    growth on wide flat blocks.  Legacy 10-deg coarse sweeps keep max(w_angle, w_extent). */
    uint32_t const span_deg = ((left_slot - right_slot) * step_deg) + step_deg;
    uint32_t       eff_deg = (span_deg > beam_deg) ? (span_deg - beam_deg) : step_deg;
    uint32_t       width_mm;

    if (is_fine)
    {
        if (eff_deg > (uint32_t)OBS_FINE_EFF_MAX_DEG)
        {
            eff_deg = (uint32_t)OBS_FINE_EFF_MAX_DEG;
        }
        width_mm = ((near_mm * (uint32_t)k_2tan_half_m1000[eff_deg]) + 500U) / 1000U;
    }
    else
    {
        uint32_t const w_angle = (near_mm * eff_deg * (uint32_t)OBS_RAD_PER_DEG_M100K) / 100000U;
        uint32_t const beam_mm = (near_mm * beam_deg * (uint32_t)OBS_RAD_PER_DEG_M100K) / 100000U;
        uint32_t const raw_extent = (uint32_t)(x_max - x_min);
        uint32_t const w_extent = (raw_extent > beam_mm) ? (raw_extent - beam_mm) : 0U;

        width_mm = (w_angle > w_extent) ? w_angle : w_extent;
    }

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

        int32_t const x = ((int32_t)d * cos_from_quad(p_cos_quad, slot_count - 1U, cos_stride, i)) / 1000;

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

    /* 6. Representative bearing: centre of the run.  For fine scans (3/5 deg),
     *    round symmetrically away from 90 deg to whole 10-degree IF-03 steps so
     *    neither left nor right side choice is biased. */
    uint32_t bearing_deg;

    if (is_fine)
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
                                k_cos5_quad_m1000,
                                2U,
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
                                k_cos5_quad_m1000,
                                1U,
                                p_out);
}

bool obs_analyse_fine3(obs_scan3_t const *p_scan, obs_result_t *p_out)
{
    if (p_scan == NULL)
    {
        return false;
    }
    return analyse_sweep_common(p_scan->dist_mm,
                                OBS_FINE3_SLOTS,
                                p_scan->lo_slot,
                                p_scan->hi_slot,
                                (uint32_t)OBS_FINE3_STEP_DEG,
                                (uint32_t)OBS_FINE3_BEAM_DEG,
                                OBS_DETECT_MM + OBS_DETECT_HYST_MM,
                                k_cos3_quad_m1000,
                                1U,
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
 * Ping the current bearing up to max_tries times, keep up to 3 echoes and
 * return the median range in millimetres if at least min_hits echoes arrived
 * (0 = nothing reliably in range).  Never runs longer than budget_ms.
 */
static uint16_t ping_bearing_filtered_mm(uint32_t budget_ms,
                                         uint32_t echo_timeout_ms,
                                         uint32_t max_range_mm,
                                         uint32_t max_tries,
                                         uint32_t min_hits)
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

    if (taken < min_hits)
    {
        return 0U;
    }
    return obs_median3_mm(sample[0], sample[1], sample[2]);
}

static uint16_t ping_bearing_mm(uint32_t budget_ms)
{
    return ping_bearing_filtered_mm(budget_ms,
                                    (uint32_t)OBS_ECHO_TIMEOUT_MS,
                                    OBS_MAX_RANGE_MM,
                                    (uint32_t)OBS_PINGS_PER_BEARING,
                                    1U);
}

static uint16_t ping_sweep_bearing_mm(uint32_t budget_ms)
{
    return ping_bearing_filtered_mm(budget_ms,
                                    (uint32_t)OBS_SWEEP_ECHO_TIMEOUT_MS,
                                    (uint32_t)OBS_SWEEP_MAX_ECHO_MM,
                                    (uint32_t)OBS_SWEEP_PINGS_PER_BEARING,
                                    (uint32_t)OBS_SWEEP_MIN_HITS);
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
        s_ctx.no_echo_set = no_echo;        /* kept for the test read-back only */
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
        s_ctx.last_profile = prof;
        s_ctx.profile_count++;
        if (publish_tel)
        {
            tel_post_obstacle(prof);    /* dashboard integration (IF-12): newest wins, never waits */
        }
        s_ctx.seq = (s_ctx.seq + 1U) & DREP_MASK(DREP_OBS_SEQ_BITS);
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
    s_ctx.phase = (uint8_t)SCAN_WATCH;
    s_ctx.confirm = 0U;
    s_ctx.watch_cooldown_until_ms = cooldown_until_ms;
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_confirm(uint32_t now_ms)
{
    motion_state_t ms;

    s_ctx.phase = (uint8_t)SCAN_CONFIRM;
    s_ctx.confirm_still_ticks = 0U;
    s_ctx.confirm_n = 0U;
    s_ctx.confirm_start_ms = now_ms;
    s_ctx.confirm_prev_enc[0] = 0;
    s_ctx.confirm_prev_enc[1] = 0;
    if (motion_get_state(&ms) == E_OK)
    {
        s_ctx.confirm_prev_enc[0] = ms.enc_count[0];
        s_ctx.confirm_prev_enc[1] = ms.enc_count[1];
    }
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_bypass_park(void)
{
    s_ctx.phase = (uint8_t)SCAN_BYPASS_PARK;
    s_ctx.confirm = 0U;
    if (servo_get_deg() != (uint32_t)OBS_WATCH_AHEAD_DEG)
    {
        servo_set_deg((uint32_t)OBS_WATCH_AHEAD_DEG);
    }
}

static void enter_slew_start(void)
{
    (void)memset(&s_ctx.fine3, 0, sizeof(s_ctx.fine3));
    (void)memset(&s_ctx.scan, 0, sizeof(s_ctx.scan));
    s_ctx.fine3.lo_slot = (uint8_t)OBS_FINE3_LO_SLOT;
    s_ctx.fine3.hi_slot = (uint8_t)OBS_FINE3_HI_SLOT;
    s_ctx.scan.lo_slot = (uint8_t)OBS_DEG_TO_SLOT(50U);
    s_ctx.scan.hi_slot = (uint8_t)OBS_DEG_TO_SLOT(130U);
    s_ctx.slew_settled = false;
    s_ctx.phase = (uint8_t)SCAN_SLEW_START;
}

/* Bring-up only: print every 3-degree bearing of the sweep that was just analysed. */
static void dump_sweep3(obs_result_t const *p_res)
{
    for (uint32_t i = OBS_FINE3_LO_SLOT; i <= OBS_FINE3_HI_SLOT; i++)
    {
        uint32_t const d = (uint32_t)s_ctx.fine3.dist_mm[i];

        if (d != 0U)
        {
            dbg_printf("[S5RAW]   %lu deg = %lu mm",
                       (unsigned long)OBS_FINE3_SLOT_TO_DEG(i), (unsigned long)d);
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
        s_ctx.confirm = 0U;
        return;
    }
#endif

    if (!nav && ((int32_t)(now_ms - s_ctx.watch_cooldown_until_ms) < 0))
    {
        s_ctx.confirm = 0U;
        return;
    }

    if ((mm != 0U) && (mm <= (uint16_t)OBS_DETECT_MM))
    {
        s_ctx.confirm++;
        if (s_ctx.confirm >= (uint32_t)OBS_DETECT_CONFIRM_COUNT)
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
                s_ctx.seq);

            send_profile(halt_prof, false);
            enter_confirm(now_ms);
        }
    }
    else
    {
        s_ctx.confirm = 0U;
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
                   && (ms.enc_count[0] == s_ctx.confirm_prev_enc[0])
                   && (ms.enc_count[1] == s_ctx.confirm_prev_enc[1]);
        s_ctx.confirm_prev_enc[0] = ms.enc_count[0];
        s_ctx.confirm_prev_enc[1] = ms.enc_count[1];
    }

    if (s_ctx.confirm_still_ticks < (uint32_t)OBS_CONFIRM_STILL_TICKS)
    {
        if (is_still)
        {
            s_ctx.confirm_still_ticks++;
        }
        else
        {
            s_ctx.confirm_still_ticks = 0U;
        }
        if ((s_ctx.confirm_still_ticks < (uint32_t)OBS_CONFIRM_STILL_TICKS)
            && ((now_ms - s_ctx.confirm_start_ms) < (uint32_t)OBS_CONFIRM_MAX_WAIT_MS))
        {
            return;
        }
        s_ctx.confirm_still_ticks = (uint32_t)OBS_CONFIRM_STILL_TICKS;
        return;
    }

    if (s_ctx.confirm_n < (uint32_t)OBS_CONFIRM_SAMPLES)
    {
        s_ctx.confirm_buf[s_ctx.confirm_n] = mm;
        s_ctx.confirm_n++;
    }
    if (s_ctx.confirm_n < (uint32_t)OBS_CONFIRM_SAMPLES)
    {
        return;
    }

    uint16_t const d_conf = obs_median3_mm(s_ctx.confirm_buf[0],
                                           s_ctx.confirm_buf[1],
                                           s_ctx.confirm_buf[2]);

    if ((d_conf == 0U) || (d_conf > (uint16_t)(OBS_DETECT_MM + OBS_DETECT_HYST_MM)))
    {
        dbg_printf("[S5] confirm clear (%lu mm), resuming", (unsigned long)d_conf);
        send_profile(obs_result_to_profile(NULL, s_ctx.seq), false);
        enter_watch(0U);
        return;
    }

    tel_set_range_cm((uint32_t)d_conf / 10U);
    dbg_printf("[S5] stopped, confirmed d=%lu mm -> slow width scan",
               (unsigned long)d_conf);
    enter_slew_start();
}

static void finish_fine3_sweep(void)
{
    obs_result_t res;

    s_ctx.fine3.lo_slot = (uint8_t)OBS_FINE3_LO_SLOT;
    s_ctx.fine3.hi_slot = (uint8_t)OBS_FINE3_HI_SLOT;
    s_ctx.scan.lo_slot = (uint8_t)OBS_DEG_TO_SLOT(50U);
    s_ctx.scan.hi_slot = (uint8_t)OBS_DEG_TO_SLOT(130U);
    s_ctx.last_sweep = s_ctx.scan;
    s_ctx.sweep_count++;

    if (obs_analyse_fine3(&s_ctx.fine3, &res))
    {
        dbg_printf("[S5] obstacle %lu mm @ %lu deg w=%lu clrL=%lu clrR=%lu",
                   (unsigned long)res.near_mm, (unsigned long)res.bearing_deg,
                   (unsigned long)res.width_mm, (unsigned long)res.clear_l_mm,
                   (unsigned long)res.clear_r_mm);
        dump_sweep3(&res);
        send_profile(obs_result_to_profile(&res, s_ctx.seq), true);
    }
    else
    {
        send_profile(obs_result_to_profile(NULL, s_ctx.seq), false);
    }

    s_ctx.slew_settled = false;
    s_ctx.phase = (uint8_t)SCAN_SLEW_HOME;
}

static void sweep_step(uint16_t mm)
{
    s_ctx.fine3.dist_mm[s_ctx.fine3_slot] = mm;

    uint32_t const deg = OBS_FINE3_SLOT_TO_DEG(s_ctx.fine3_slot);
    uint32_t const rem = deg % 10U;

    if ((rem <= 1U) || (rem >= 9U))
    {
        uint32_t const slot10 = (deg + 5U) / 10U;

        if (slot10 < OBS_BEARING_SLOTS)
        {
            s_ctx.scan.dist_mm[slot10] = mm;
        }
    }

    if (s_ctx.fine3_slot < (uint32_t)OBS_FINE3_HI_SLOT)
    {
        s_ctx.fine3_slot++;
        servo_set_deg(OBS_FINE3_SLOT_TO_DEG(s_ctx.fine3_slot));
        s_ctx.sweep_settling = true;
    }
    else
    {
        finish_fine3_sweep();
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

    (void)memset(&s_ctx.scan, 0, sizeof(s_ctx.scan));
    (void)memset(&s_ctx.fine3, 0, sizeof(s_ctx.fine3));
    (void)memset(&s_ctx.last_sweep, 0, sizeof(s_ctx.last_sweep));
    s_ctx.scan.lo_slot = (uint8_t)OBS_DEG_TO_SLOT(50U);
    s_ctx.scan.hi_slot = (uint8_t)OBS_DEG_TO_SLOT(130U);
    s_ctx.fine3_slot = (uint32_t)OBS_FINE3_LO_SLOT;

    s_ctx.running = true;
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

        switch ((scan_phase_t)s_ctx.phase)
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
                    if (!s_ctx.slew_settled)
                    {
                        s_ctx.slew_settled = true;
                    }
                    else
                    {
                        s_ctx.fine3_slot = (uint32_t)OBS_FINE3_LO_SLOT;
                        s_ctx.sweep_settling = false;
                        s_ctx.phase = (uint8_t)SCAN_SWEEP;

                        uint16_t const mm = ping_sweep_bearing_mm(OBS_SWEEP_BUDGET_MS);

                        record_ping_health(mm, false, &echo_missed);
                        sweep_step(mm);
                    }
                }
                break;
            }

            case SCAN_SWEEP:
            {
                if (s_ctx.sweep_settling)
                {
                    s_ctx.sweep_settling = false;
                    break;
                }

                bool const     is_ahead = (s_ctx.fine3_slot == OBS_FINE3_DEG_TO_SLOT(OBS_WATCH_AHEAD_DEG));
                uint16_t const mm = ping_sweep_bearing_mm(OBS_SWEEP_BUDGET_MS);

                record_ping_health(mm, is_ahead, &echo_missed);
                sweep_step(mm);
                break;
            }

            case SCAN_SLEW_HOME:
            {
                if (slew_step_toward((uint32_t)OBS_WATCH_AHEAD_DEG))
                {
                    if (!s_ctx.slew_settled)
                    {
                        s_ctx.slew_settled = true;
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
    return s_ctx.phase == (uint8_t)SCAN_WATCH;
}

bool obs_scan_is_running(void)
{
    return s_ctx.running;
}

uint32_t obs_scan_get_profile_count(void)
{
    return s_ctx.profile_count;
}

obstacle_profile_t obs_scan_get_last_profile(void)
{
    return s_ctx.last_profile;
}

uint32_t obs_scan_get_sweep_count(void)
{
    return s_ctx.sweep_count;
}

void obs_scan_get_last_sweep(obs_scan_t *p_out)
{
    if (p_out != NULL)
    {
        *p_out = s_ctx.last_sweep;
    }
}
