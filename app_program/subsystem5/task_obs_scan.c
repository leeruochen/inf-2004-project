/*
 *  task_obs_scan.c
 *  Subsystem 5 -- Obstacle Scanning task (priority 6, 60 ms servo grid).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Responsibilities (specs.md FR6/FR7, design.md 3.1/3.3, TEAM.md IF-03/05):
 *    - Drift-free OBS_SERVO_STEP_PERIOD_MS (60 ms) grid via app_period_wait().
 *    - Sweeps the servo coarsely (OBS_COARSE_BEARING_COUNT bearings,
 *      OBS_COARSE_STEP_DEG apart, 30..150 degrees), ping-ponging so the horn
 *      never has to cross the whole span between sweeps.
 *    - Pings up to OBS_PINGS_PER_BEARING times per bearing and keeps the
 *      median, which rejects a single spurious echo.
 *    - After OBS_DETECT_CONFIRM_COUNT consecutive coarse sweeps that see
 *      something inside OBS_D_DETECT_CM, runs a fine sweep of
 *      OBS_FINE_STEP_DEG steps over +-OBS_FINE_HALF_SPAN_DEG around the
 *      detection bearing, then profiles the obstacle.
 *    - Publishes obstacle_profile_t on MBF_OBSTACLE (IF-03) newest-wins,
 *      which needs drain-then-send (docs/DATA_REPRESENTATION.md 2.3).
 *    - Single setter AND clearer of EVF_SAFETY_STATUS_NO_ECHO
 *      (docs/DATA_REPRESENTATION.md section 3): informational, not a fault.
 *
 *  RESOLVED: "settle time vs 60 ms step" (HANDOVER.md section 3, Buddy 5's
 *  open decisions).  OBS_SERVO_SETTLE_MS (50 ms) plus three 30 ms echo
 *  timeouts cannot fit in one 60 ms period, so settling and pinging are
 *  separate passes over the grid: a period either settles or pings, never
 *  both.  The settle allowance is scaled by how far the horn actually has to
 *  travel (OBS_SETTLE_PER_STEP_MS per OBS_COARSE_STEP_DEG), so a 10 degree
 *  fine step settles in about 17 ms and still leaves its period free to
 *  ping, while a 30 degree coarse step takes a period of its own.  Pinging is
 *  additionally capped at OBS_PING_BUDGET_MS so a no-echo bearing (one 30 ms
 *  timeout) cannot overrun its period.
 *  Measured sweep times to record in testing/subsystem5/TEST_LOG.md:
 *  coarse about 10 periods (600 ms), fine about 7 periods (420 ms).
 */

#include "task_obs_scan.h"

#include <string.h>

#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "src/debug/dbg.h"

/* ---------------------------------------------------------------------------
 * Compile-time checks on the tuning constants this task depends on.
 * ------------------------------------------------------------------------- */

#if OBS_SERVO_STEP_PERIOD_MS == 0U
#error "task_obs_scan.c: OBS_SERVO_STEP_PERIOD_MS must not be 0"
#endif
#if (OBS_FINE_STEP_DEG == 0U) || ((OBS_FINE_STEP_DEG % 10U) != 0U)
#error "task_obs_scan.c: OBS_FINE_STEP_DEG must be a non-zero multiple of 10"
#endif
#if (OBS_COARSE_STEP_DEG == 0U) || ((OBS_COARSE_STEP_DEG % 10U) != 0U)
#error "task_obs_scan.c: OBS_COARSE_STEP_DEG must be a non-zero multiple of 10"
#endif
#if ((OBS_FINE_HALF_SPAN_DEG % OBS_FINE_STEP_DEG) != 0U)
#error "task_obs_scan.c: OBS_FINE_HALF_SPAN_DEG must be a whole number of fine steps"
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
#define OBS_DETECT_MM           ((uint32_t)OBS_D_DETECT_CM * 10U)

/** Largest range the IF-03 distance field can carry, in millimetres. */
#define OBS_MAX_RANGE_MM        ((uint32_t)DREP_OBS_DIST_MAX_CM * 10U)

/** Servo settle allowance per OBS_COARSE_STEP_DEG of travel.  TUNE:
 *  promote to config/thresholds.h once measured (S5-U13). */
#define OBS_SETTLE_PER_STEP_MS  ((uint32_t)OBS_SERVO_SETTLE_MS)

/** Floor and ceiling on the scaled settle allowance. */
#define OBS_SETTLE_MIN_MS       (8U)
#define OBS_SETTLE_MAX_MS       (250U)

/** Time allowed for all the pings at one bearing.  Keeps a bearing inside
 *  its 60 ms period: one 30 ms timeout fits, a second one does not.  TUNE. */
#define OBS_PING_BUDGET_MS      (45U)

/** A bearing belongs to the same obstacle as the nearest return if it is
 *  within this much of it.  TUNE: promote to thresholds.h after S5-U14. */
#define OBS_EDGE_TOL_MM         (100U)

/** Nominal HC-SR04 beam width, subtracted from the measured angular span so
 *  a narrow post is not reported as a wide wall.  TUNE. */
#define OBS_BEAM_DEG            (15U)

/** Radians per degree, scaled by 100000 (pi / 180 = 0.0174533). */
#define OBS_RAD_PER_DEG_M100K   (1745U)

/** Fine sweep bearings: +-half span in whole steps, plus the centre. */
#define OBS_FINE_SLOT_SPAN      (OBS_FINE_HALF_SPAN_DEG / 10U)

static bool               s_running = false;
static obs_scan_t         s_scan;
static obs_scan_t         s_last_sweep;
static obstacle_profile_t s_last_profile;
static uint32_t           s_profile_count = 0U;
static uint32_t           s_sweep_count = 0U;
static uint32_t           s_seq = 0U;
static bool               s_no_echo_set = false;

/* Sweep cursor. */
static bool     s_fine_mode = false;
static uint32_t s_slot = OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN);
static int32_t  s_dir = 1;                  /* +1 towards the left, -1 right */
static uint32_t s_fine_lo_slot = 0U;
static uint32_t s_fine_hi_slot = 0U;
static uint32_t s_confirm = 0U;
static uint32_t s_settle_until_ms = 0U;

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

/** True if slot i of the sweep is part of the obstacle run (within thresh). */
static bool slot_is_obstacle(obs_scan_t const *p_scan, uint32_t i, uint32_t thresh_mm)
{
    uint32_t const d = (uint32_t)p_scan->dist_mm[i];

    return (d != 0U) && (d <= thresh_mm);
}

bool obs_analyse(obs_scan_t const *p_scan, obs_result_t *p_out)
{
    if ((p_scan == NULL) || (p_out == NULL))
    {
        return false;
    }
    (void)memset(p_out, 0, sizeof(*p_out));

    uint32_t const lo = (p_scan->lo_slot < OBS_BEARING_SLOTS) ? p_scan->lo_slot : 0U;
    uint32_t const hi = (p_scan->hi_slot < OBS_BEARING_SLOTS) ? p_scan->hi_slot
                                                              : (OBS_BEARING_SLOTS - 1U);
    if (lo > hi)
    {
        return false;
    }

    /* 1. Nearest return inside the detection range. */
    uint32_t near_mm = 0U;
    uint32_t near_slot = lo;

    for (uint32_t i = lo; i <= hi; i++)
    {
        uint32_t const d = (uint32_t)p_scan->dist_mm[i];

        if ((d == 0U) || (d > OBS_DETECT_MM))
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
    uint32_t const thresh_mm = near_mm + OBS_EDGE_TOL_MM;
    uint32_t       right_slot = near_slot;
    uint32_t       left_slot = near_slot;

    while ((right_slot > lo) && slot_is_obstacle(p_scan, right_slot - 1U, thresh_mm))
    {
        right_slot--;
    }
    while ((left_slot < hi) && slot_is_obstacle(p_scan, left_slot + 1U, thresh_mm))
    {
        left_slot++;
    }

    /* 3. Lateral extent of the run.  Positive x is to the car's right, so
     *    the largest x is the obstacle's right-hand edge. */
    int32_t x_min = INT32_MAX;
    int32_t x_max = INT32_MIN;

    for (uint32_t i = right_slot; i <= left_slot; i++)
    {
        if (!slot_is_obstacle(p_scan, i, thresh_mm))
        {
            continue;
        }

        int32_t const x = obs_lateral_mm((uint32_t)p_scan->dist_mm[i], OBS_SLOT_TO_DEG(i));

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
    /* One extra fine step covers the sample at each end of the run.  Only a
     * fine sweep is ever packed into a profile, so the fine step is the right
     * sample pitch to add here. */
    uint32_t const span_deg = ((left_slot - right_slot) * 10U) + OBS_FINE_STEP_DEG;
    /* A run no wider than the beam cannot be resolved any finer than the
     * beam itself, so the beam width is the floor.  Flooring at the (smaller)
     * sample pitch instead would under-report a one-bearing hit, and
     * docs/DATA_REPRESENTATION.md 2.3 requires the width to err wide. */
    uint32_t const eff_deg = (span_deg > OBS_BEAM_DEG) ? (span_deg - OBS_BEAM_DEG)
                                                       : OBS_BEAM_DEG;
    uint32_t const w_angle = (uint32_t)(((uint64_t)near_mm * (uint64_t)eff_deg
                                         * (uint64_t)OBS_RAD_PER_DEG_M100K) / 100000ULL);
    uint32_t const w_extent = (uint32_t)(x_max - x_min);
    uint32_t       width_mm = (w_angle > w_extent) ? w_angle : w_extent;

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
        uint32_t const d = (uint32_t)p_scan->dist_mm[i];

        if ((i >= right_slot) && (i <= left_slot))
        {
            continue;                   /* the obstacle itself */
        }
        if ((d == 0U) || (d > OBS_DETECT_MM))
        {
            continue;                   /* open space on this bearing */
        }

        int32_t const x = obs_lateral_mm(d, OBS_SLOT_TO_DEG(i));

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

    p_out->near_mm = (uint16_t)near_mm;
    p_out->width_mm = (uint16_t)width_mm;
    p_out->clear_l_mm = (uint16_t)clear_l;
    p_out->clear_r_mm = (uint16_t)clear_r;
    p_out->centre_x_mm = (int16_t)((x_min + x_max) / 2);
    /* Report the centre of the run, not near_slot: with several bearings at
     * the same range near_slot is just the first one scanned, which would put
     * a centred obstacle off to one side and mislead obs_choose_side(). */
    p_out->bearing_deg = (uint8_t)OBS_SLOT_TO_DEG((right_slot + left_slot) / 2U);
    p_out->left_deg = (uint8_t)OBS_SLOT_TO_DEG(left_slot);
    p_out->right_deg = (uint8_t)OBS_SLOT_TO_DEG(right_slot);
    p_out->valid = true;

    return true;
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
static uint16_t ping_bearing_mm(uint32_t budget_ms)
{
    uint16_t       sample[3] = { 0U, 0U, 0U };
    uint32_t       taken = 0U;
    uint32_t const start_ms = app_period_now_ms();

    for (uint32_t i = 0U; i < (uint32_t)OBS_PINGS_PER_BEARING; i++)
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

        if (tk_wai_sem(SEM_ECHO, 1, (TMO)OBS_ECHO_TIMEOUT_MS) < (ER)E_OK)
        {
            continue;                   /* no echo = nothing in range */
        }

        uint32_t echo_us = 0U;

        if (!us_take_echo_us(&echo_us))
        {
            continue;
        }

        uint32_t const mm = us_echo_us_to_mm(echo_us);

        if ((mm >= (uint32_t)US_MIN_RANGE_MM) && (mm <= OBS_MAX_RANGE_MM)
            && (taken < 3U))
        {
            sample[taken] = (uint16_t)mm;
            taken++;
        }
    }

    return obs_median3_mm(sample[0], sample[1], sample[2]);
}

/** Set or clear EVF_SAFETY_STATUS_NO_ECHO, which this task alone owns. */
static void publish_no_echo(bool no_echo)
{
    if (no_echo == s_no_echo_set)
    {
        return;                         /* no change: do not churn the flag */
    }

    ER er;

    if (no_echo)
    {
        er = tk_set_flg(EVF_SAFETY, EVF_SAFETY_STATUS_NO_ECHO);
    }
    else
    {
        er = tk_clr_flg(EVF_SAFETY, ~(UINT)EVF_SAFETY_STATUS_NO_ECHO);
    }

    if (er >= (ER)E_OK)
    {
        s_no_echo_set = no_echo;
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
static void send_profile(obstacle_profile_t prof)
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
        s_seq = (s_seq + 1U) & DREP_MASK(DREP_OBS_SEQ_BITS);
    }
    else
    {
        dbg_put_err("obs_scan snd MBF_OBSTACLE", (int32_t)er);
    }
}

/* ---------------------------------------------------------------------------
 * Sweep cursor
 * ------------------------------------------------------------------------- */

/** Command the horn to a slot and charge the settle allowance for the move. */
static void move_to_slot(uint32_t slot, uint32_t now_ms)
{
    uint32_t const from_deg = servo_get_deg();
    uint32_t const to_deg = OBS_SLOT_TO_DEG(slot);
    uint32_t const travel = (to_deg > from_deg) ? (to_deg - from_deg) : (from_deg - to_deg);
    uint32_t       settle = ((travel * OBS_SETTLE_PER_STEP_MS) + (OBS_COARSE_STEP_DEG - 1U))
                           / OBS_COARSE_STEP_DEG;

    if (settle < OBS_SETTLE_MIN_MS)
    {
        settle = OBS_SETTLE_MIN_MS;
    }
    if (settle > OBS_SETTLE_MAX_MS)
    {
        settle = OBS_SETTLE_MAX_MS;
    }

    s_slot = slot;
    servo_set_deg(to_deg);
    s_settle_until_ms = now_ms + settle;
}

/*
 * True if slot lo is the nearer end of [lo, hi] from the horn's present
 * bearing.  Signed on purpose: the horn can sit outside [lo, hi] (a fine
 * window near one extreme, or the coarse span after a fine sweep), and
 * unsigned arithmetic would wrap and pick the far end.
 */
static bool lo_end_is_nearer(uint32_t lo, uint32_t hi)
{
    int32_t const here = (int32_t)OBS_DEG_TO_SLOT(servo_get_deg());
    int32_t const d_lo = (here > (int32_t)lo) ? (here - (int32_t)lo) : ((int32_t)lo - here);
    int32_t const d_hi = (here > (int32_t)hi) ? (here - (int32_t)hi) : ((int32_t)hi - here);

    return d_lo <= d_hi;
}

/** Widen the analysis window to the coarse span plus whatever the fine sweep
 *  covered, so clearances can use the context either side of the obstacle. */
static void set_analysis_window(uint32_t lo_slot, uint32_t hi_slot)
{
    uint32_t const coarse_lo = OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN);
    uint32_t const coarse_hi = OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX);

    s_scan.lo_slot = (uint8_t)((lo_slot < coarse_lo) ? lo_slot : coarse_lo);
    s_scan.hi_slot = (uint8_t)((hi_slot > coarse_hi) ? hi_slot : coarse_hi);
}

/** Enter the fine sweep centred on a coarse detection bearing. */
static void enter_fine(uint32_t centre_deg, uint32_t now_ms)
{
    uint32_t const centre_slot = OBS_DEG_TO_SLOT(centre_deg);
    uint32_t       lo = (centre_slot > OBS_FINE_SLOT_SPAN) ? (centre_slot - OBS_FINE_SLOT_SPAN) : 0U;
    uint32_t       hi = centre_slot + OBS_FINE_SLOT_SPAN;

    if (hi > (OBS_BEARING_SLOTS - 1U))
    {
        hi = OBS_BEARING_SLOTS - 1U;
    }

    s_fine_mode = true;
    s_fine_lo_slot = lo;
    s_fine_hi_slot = hi;

    /* Start the fine sweep from whichever end the horn is already nearest. */
    if (lo_end_is_nearer(lo, hi))
    {
        s_dir = 1;
        move_to_slot(lo, now_ms);
    }
    else
    {
        s_dir = -1;
        move_to_slot(hi, now_ms);
    }
}

/** Leave the fine sweep and resume coarse ping-ponging. */
static void enter_coarse(uint32_t now_ms)
{
    s_fine_mode = false;
    s_confirm = 0U;

    uint32_t const lo = OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN);
    uint32_t const hi = OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX);

    /* Resume at the nearer end of the coarse span, sweeping inwards. */
    if (lo_end_is_nearer(lo, hi))
    {
        s_dir = 1;
        move_to_slot(lo, now_ms);
    }
    else
    {
        s_dir = -1;
        move_to_slot(hi, now_ms);
    }
}

/** A coarse sweep has just finished: decide whether to profile it. */
static void finish_coarse_sweep(uint32_t now_ms)
{
    obs_result_t res;

    set_analysis_window(OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN),
                        OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX));
    s_last_sweep = s_scan;
    s_sweep_count++;

    if (obs_analyse(&s_scan, &res) && (res.near_mm <= OBS_DETECT_MM))
    {
        s_confirm++;
        if (s_confirm >= (uint32_t)OBS_DETECT_CONFIRM_COUNT)
        {
            dbg_printf("[S5] coarse hit %lu mm @ %lu deg, fine scan",
                       (unsigned long)res.near_mm, (unsigned long)res.bearing_deg);
            enter_fine(res.bearing_deg, now_ms);
            return;
        }
    }
    else
    {
        s_confirm = 0U;
    }

    /* Keep ping-ponging the coarse span. */
    s_dir = -s_dir;
    move_to_slot((s_dir > 0) ? OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN)
                             : OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX),
                 now_ms);
}

/** A fine sweep has just finished: profile it and publish. */
static void finish_fine_sweep(uint32_t now_ms)
{
    obs_result_t res;

    set_analysis_window(s_fine_lo_slot, s_fine_hi_slot);
    s_last_sweep = s_scan;
    s_sweep_count++;

    if (obs_analyse(&s_scan, &res))
    {
        dbg_printf("[S5] obstacle %lu mm @ %lu deg w=%lu clrL=%lu clrR=%lu",
                   (unsigned long)res.near_mm, (unsigned long)res.bearing_deg,
                   (unsigned long)res.width_mm, (unsigned long)res.clear_l_mm,
                   (unsigned long)res.clear_r_mm);
        send_profile(obs_result_to_profile(&res, s_seq));
    }
    else
    {
        /* The obstacle went away between the coarse hit and the fine sweep:
         * tell Avoidance so it can release, rather than leaving it waiting. */
        send_profile(obs_result_to_profile(NULL, s_seq));
    }

    enter_coarse(now_ms);
}

/** Step the cursor after a bearing has been measured. */
static void advance_cursor(uint32_t now_ms)
{
    uint32_t const lo = s_fine_mode ? s_fine_lo_slot : OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN);
    uint32_t const hi = s_fine_mode ? s_fine_hi_slot : OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX);
    uint32_t const step = s_fine_mode ? (OBS_FINE_STEP_DEG / 10U) : (OBS_COARSE_STEP_DEG / 10U);

    if (s_dir > 0)
    {
        if ((s_slot + step) <= hi)
        {
            move_to_slot(s_slot + step, now_ms);
            return;
        }
    }
    else
    {
        if (s_slot >= (lo + step))
        {
            move_to_slot(s_slot - step, now_ms);
            return;
        }
    }

    /* Reached the far end: the sweep is complete. */
    if (s_fine_mode)
    {
        finish_fine_sweep(now_ms);
    }
    else
    {
        finish_coarse_sweep(now_ms);
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
    (void)memset(&s_last_sweep, 0, sizeof(s_last_sweep));
    set_analysis_window(OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN),
                        OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX));

    s_running = true;
    s_dir = 1;

    app_period_init(&per, (uint32_t)OBS_SERVO_STEP_PERIOD_MS);
    move_to_slot(OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN), app_period_now_ms());

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed != 0U)
        {
            dbg_printf("[S5] scan missed %lu releases", (unsigned long)missed);
        }

        uint32_t const now_ms = app_period_now_ms();

        /* A period either settles or pings, never both (file header). */
        if ((int32_t)(now_ms - s_settle_until_ms) < 0)
        {
            continue;
        }

        uint16_t const mm = ping_bearing_mm(OBS_PING_BUDGET_MS);

        s_scan.dist_mm[s_slot] = mm;

        if (mm != 0U)
        {
            echo_missed = 0U;
            publish_no_echo(false);
        }
        else
        {
            echo_missed++;
            /* A whole sweep with nothing in range is the documented
             * "no echo" status, not a fault (IF-06 b7). */
            if (echo_missed >= (uint32_t)OBS_COARSE_BEARING_COUNT)
            {
                publish_no_echo(true);
            }
        }

        advance_cursor(app_period_now_ms());
    }
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
