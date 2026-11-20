/*
 *  task_obs_avoid.c
 *  Subsystem 5 -- Obstacle Avoidance task (priority 7).
 *  Kernel side: kernel and project headers only, never SDK headers
 *  (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Responsibilities (specs.md FR8/FR9, design.md 3.1/3.3, TEAM.md IF-01/03/07):
 *    - Sole consumer of MBF_OBSTACLE (IF-03); validates the sequence number
 *      so a profile that was already acted on is not acted on twice.
 *    - Gates avoidance on active autonomous navigation (vc_get_system_state()).
 *    - Halts the car as soon as watch-mode confirms an obstacle ahead, then
 *      waits for the stationary 5-degree width scan before planning the bypass.
 *    - Chooses the bypass side from the measured clearances (FR8).
 *    - Drives the four-turn 45-degree bypass using encoder-closed turn
 *      completion and encoder-measured straight legs (hardcoded 10 cm / 100 mm
 *      obstacle depth), with pre-turn stops to eliminate turn skid.
 *    - Advances the axle over the line after EVF_NAV_LINE_FOUND, straightens
 *      by 45 deg (TURN_STRAIGHT), and hands control cleanly back to LF.
 *    - Aborts to a stop if EVF_SAFETY says the car is in a fault or
 *      has been lifted / overturned, or if the manoeuvre runs past its budget.
 */

#include "task_obs_avoid.h"

#include "../subsystem2/subsystem2.h"   /* vc_get_system_state(), motion_get_state() */
#include "config/thresholds.h"
#include "config/timing.h"
#include "src/debug/dbg.h"

/* ---------------------------------------------------------------------------
 * Timing and geometry constants
 * ------------------------------------------------------------------------- */

/** Idle wake-up period while there is no obstacle. */
#define OBS_IDLE_POLL_MS        (50U)

/** Active control poll period for straight legs and CONTINUE refresh
 *  (< MOTION_T_HOLD_MS / 2 = 50 ms so unbounded legs never drop). */
#define OBS_CTRL_POLL_MS        (40U)

/** Poll period during turns and directional line-rejoin (matches Motion 20 ms). */
#define OBS_TURN_POLL_MS        (20U)

/** Max time to wait in OBS_AV_SLOW for standstill confirm + slow 5-deg sweep. */
#define OBS_SCAN_WAIT_MAX_MS    (10000U)

/** Hardcoded obstacle length/depth along the track (10 cm per test spec). */
#define OBS_OBSTACLE_DEPTH_MM   (100U)

/** Small geometric extra margin on the out-leg (encoder-closed legs need none
 *  of the old open-loop under-travel compensation). */
#define OBS_OUT_EXTRA_MM        (20U)

/** Car length behind ultrasonic sensor + rear clearance margin (compact trapezoid). */
#define OBS_ALONG_CHASSIS_MM    (190U)

/** Minimum along-leg travel regardless of stand-off distance. */
#define OBS_ALONG_MIN_MM        (OBS_OBSTACLE_DEPTH_MM + (uint32_t)OBS_SIDE_MARGIN_MM)

/** Distance past EVF_NAV_LINE_FOUND so driven axle (~120 mm behind IR) reaches
 *  the line before the final 45-degree straighten pivot. */
#define OBS_REJOIN_ALIGN_MM     (125U)

/** Encoder count thresholds for early line-lock and directional fallback rejoin. */
#define OBS_STRAIGHT_LOCK_MIN_CNT   (410U)
#define OBS_REJOIN_NUDGE_CNT        (235U)
#define OBS_REJOIN_NUDGE2_CNT       (470U)
#define OBS_REJOIN_UNNUDGE_LOCK_CNT (115U)
#define OBS_REJOIN_NUDGE_MAX_MS     (1500U)
#define OBS_REJOIN_NUDGE2_MAX_MS    (2500U)
#define OBS_REJOIN_FWD1_MM          (250U)
#define OBS_REJOIN_FWD2_MM          (350U)
#define OBS_REJOIN_SEARCH_ALIGN_MM  (115U)

/** Hysteresis matching scanner's stationary detection cap (250 + 30 = 280 mm). */
#define OBS_DETECT_HYST_MM      (30U)

/** Brief dead-stop settle before pivoting 45 deg after a straight leg, letting
 *  Motion Control brake and reset PID integral state. */
#define OBS_PRE_TURN_STOP_MS    (100U)

/** Encoder standstill confirmation window after a closed-loop pivot finishes. */
#define OBS_TURN_STILL_MS       (120U)

/** Minimum encoder delta before standstill detection can declare a turn done. */
#define OBS_TURN_MIN_MOVE_CNT   (20U)

/** Safety timeout cap for a 45-degree bypass turn (ms). */
#define OBS_TURN_MAX_MS         ((800U * 25U) / (uint32_t)MOTION_SPEED_BYPASS_CMPS)

/** Reverse dwell cap when the car is boxed in (15 cm reverse at bypass speed). */
#define OBS_BLOCKED_DWELL_MS    ((1200U * 25U) / (uint32_t)MOTION_SPEED_BYPASS_CMPS)

/** Longest wait for subsystem 3 to report the line again on LEG_BACK. */
#define OBS_LEG_BACK_MAX_MS     (((uint32_t)LF_T_SEARCH_MS * 25U) \
                                 / (uint32_t)MOTION_SPEED_BYPASS_CMPS)

/** The Vehicle Controller turns this much per bypass turn command. */
#define OBS_BYPASS_TURN_DEG     (45U)

/** Holdoff used only if obs_avoid_abandon() is explicitly called mid-run. */
#define OBS_START_GRACE_MS      (2000U)

/** Diagonal multiplier at 45 degrees: sqrt(2) x 1000. */
#define OBS_DIAG_PER_LATERAL_M1000  (1414U)

/** MBF_NAV_IN send timeout: 5 ms, per TEAM.md section 5 IF-01. */
#define OBS_NAV_SEND_TMO_MS     (5U)

#if OBS_BYPASS_CLEAR_MIN_MM == 0U
#error "task_obs_avoid.c: OBS_BYPASS_CLEAR_MIN_MM must not be 0"
#endif
#if MOTION_SPEED_BYPASS_CMPS == 0U
#error "task_obs_avoid.c: MOTION_SPEED_BYPASS_CMPS must not be 0"
#endif
#if (ENC_CPR == 0U) || (WHEEL_CIRC_MM == 0U)
#error "task_obs_avoid.c: ENC_CPR and WHEEL_CIRC_MM must be non-zero in config/thresholds.h"
#endif
#if OBS_BYPASS_TURN_DEG != 45U
#error "task_obs_avoid.c: OBS_DIAG_PER_LATERAL_M1000 assumes a 45 degree bypass"
#endif

static bool     s_running = false;
static uint32_t s_state = OBS_AV_IDLE;
static uint32_t s_side = OBS_SIDE_LEFT;
static uint32_t s_step_deadline_ms = 0U;
static uint32_t s_budget_deadline_ms = 0U;
static uint32_t s_grace_until_ms = 0U;
static uint32_t s_leg_out_mm = 0U;
static uint32_t s_leg_along_mm = 0U;
static uint32_t s_leg_target_cnt = 0U;
static int32_t  s_leg_start_enc[2] = { 0, 0 };
static bool     s_leg_stopping = false;
static bool     s_line_aligning = false;
static int32_t  s_turn_start_enc[2] = { 0, 0 };
static int32_t  s_turn_prev_enc[2] = { 0, 0 };
static uint32_t s_turn_start_ms = 0U;
static uint32_t s_turn_last_poll_ms = 0U;
static uint32_t s_turn_still_ms = 0U;
static bool     s_turn_moved = false;
static bool     s_was_nav = false;
static bool     s_have_seq = false;
static uint32_t s_last_seq = 0U;
static uint32_t s_bypass_count = 0U;
static uint32_t s_abort_count = 0U;
static uint32_t s_profile_count = 0U;
static uint32_t s_sent_count = 0U;

typedef enum
{
    REJOIN_NONE = 0,
    REJOIN_NUDGE1,
    REJOIN_FWD1,
    REJOIN_NUDGE2,
    REJOIN_FWD2,
    REJOIN_ALIGN,
    REJOIN_UNNUDGE
} rejoin_sub_t;

static rejoin_sub_t s_rejoin_sub = REJOIN_NONE;
static uint32_t     s_rejoin_unnudge_side = OBS_SIDE_LEFT;

/* ---------------------------------------------------------------------------
 * Pure decisions
 * ------------------------------------------------------------------------- */

uint32_t obs_leg_out_mm(uint32_t width_mm)
{
    uint32_t const lateral_mm = (width_mm / 2U)
                                + ((uint32_t)CAR_WIDTH_MM / 2U)
                                + (uint32_t)OBS_SIDE_MARGIN_MM
                                + (uint32_t)OBS_OUT_EXTRA_MM;

    return (lateral_mm * OBS_DIAG_PER_LATERAL_M1000) / 1000U;
}

uint32_t obs_leg_along_mm(uint32_t dist_mm, uint32_t width_mm)
{
    uint32_t const lateral_mm = (width_mm / 2U)
                                + ((uint32_t)CAR_WIDTH_MM / 2U)
                                + (uint32_t)OBS_SIDE_MARGIN_MM
                                + (uint32_t)OBS_OUT_EXTRA_MM;
    uint32_t const needed_mm = dist_mm + (uint32_t)OBS_OBSTACLE_DEPTH_MM
                               + (uint32_t)OBS_ALONG_CHASSIS_MM;
    uint32_t const along_mm = (needed_mm > lateral_mm) ? (needed_mm - lateral_mm) : 0U;

    return (along_mm < (uint32_t)OBS_ALONG_MIN_MM) ? (uint32_t)OBS_ALONG_MIN_MM : along_mm;
}

uint32_t obs_mm_to_counts(uint32_t distance_mm)
{
    return (uint32_t)(((uint64_t)distance_mm * (uint64_t)ENC_QUAD_MULT * (uint64_t)ENC_CPR)
                      / (uint64_t)WHEEL_CIRC_MM);
}

uint32_t obs_mm_to_ms(uint32_t distance_mm)
{
    /* MOTION_SPEED_BYPASS_CMPS cm/s is 10 x that many mm/s. */
    return (distance_mm * 100U) / (uint32_t)MOTION_SPEED_BYPASS_CMPS;
}

bool obs_choose_side(obstacle_profile_t prof, uint32_t *p_side)
{
    uint32_t const clear_l = drep_obs_clear_l_mm(prof);
    uint32_t const clear_r = drep_obs_clear_r_mm(prof);
    bool const     left_ok = (clear_l >= (uint32_t)OBS_BYPASS_CLEAR_MIN_MM);
    bool const     right_ok = (clear_r >= (uint32_t)OBS_BYPASS_CLEAR_MIN_MM);

    if (!left_ok && !right_ok)
    {
        return false;                   /* boxed in */
    }
    if (p_side == NULL)
    {
        return true;
    }

    if (left_ok && right_ok)
    {
        /* Both sides have room: go the way the obstacle is not.  Bearings
         * above 90 degrees are to the car's left (drv_servo.h), so an
         * obstacle on the left is passed on the right. */
        uint32_t const bearing_deg = drep_obs_bearing_deg(prof);

        if (bearing_deg > 90U)
        {
            *p_side = OBS_SIDE_RIGHT;
        }
        else if (bearing_deg < 90U)
        {
            *p_side = OBS_SIDE_LEFT;
        }
        else
        {
            /* Dead ahead: take the roomier side, left on a tie. */
            *p_side = (clear_r > clear_l) ? OBS_SIDE_RIGHT : OBS_SIDE_LEFT;
        }
    }
    else
    {
        *p_side = left_ok ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT;
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Talking to the Vehicle Controller
 * ------------------------------------------------------------------------- */

static void send_nav(nav_input_t msg)
{
    ER const er = tk_snd_mbf(MBF_NAV_IN, &msg, (INT)sizeof(msg),
                             (TMO)OBS_NAV_SEND_TMO_MS);

    if (er >= (ER)E_OK)
    {
        s_sent_count++;
    }
    else
    {
        dbg_put_err("obs_avoid snd MBF_NAV_IN", (int32_t)er);
    }
}

static void send_bypass(uint32_t bypass)
{
    send_nav(drep_nav_avoid(DREP_NAV_AVOID_BYPASS, bypass));
}

static bool rejoin_is_turn(rejoin_sub_t sub)
{
    return (sub == REJOIN_NONE) || (sub == REJOIN_NUDGE1)
           || (sub == REJOIN_NUDGE2) || (sub == REJOIN_UNNUDGE);
}

static bool rejoin_is_leg(rejoin_sub_t sub)
{
    return (sub == REJOIN_FWD1) || (sub == REJOIN_FWD2) || (sub == REJOIN_ALIGN);
}

static bool state_is_turn_or_blocked(uint32_t state)
{
    return (state == OBS_AV_TURN_OUT) || (state == OBS_AV_TURN_IN1)
           || (state == OBS_AV_TURN_IN2) || (state == OBS_AV_TURN_STRAIGHT)
           || (state == OBS_AV_BLOCKED);
}

static bool state_uses_turn_poll(uint32_t state, rejoin_sub_t sub)
{
    if (state == OBS_AV_TURN_STRAIGHT)
    {
        return rejoin_is_turn(sub);
    }
    return (state == OBS_AV_TURN_OUT) || (state == OBS_AV_TURN_IN1)
           || (state == OBS_AV_TURN_IN2) || (state == OBS_AV_BLOCKED);
}

static bool state_uses_leg_poll(uint32_t state, rejoin_sub_t sub)
{
    if (state == OBS_AV_TURN_STRAIGHT)
    {
        return rejoin_is_leg(sub);
    }
    return (state == OBS_AV_LEG_OUT) || (state == OBS_AV_LEG_ALONG)
           || (state == OBS_AV_LEG_BACK);
}

static bool vc_is_navigating(uint32_t vc_state)
{
    return ((vc_state >= VC_SYS_NAV_LINE_TRACKING) && (vc_state <= VC_SYS_NAV_TURN_RIGHT))
           || (vc_state == VC_SYS_OBSTACLE_AVOIDANCE);
}

/** The opposite side, for the turns that come back towards the line. */
static uint32_t other_side(uint32_t side)
{
    return (side == OBS_SIDE_LEFT) ? OBS_SIDE_RIGHT : OBS_SIDE_LEFT;
}

static uint32_t abs_diff_i32(int32_t now, int32_t prev)
{
    int32_t const d = (int32_t)((uint32_t)now - (uint32_t)prev);

    return (d < 0) ? (0U - (uint32_t)d) : (uint32_t)d;
}

static uint32_t leg_timeout_ms(uint32_t dist_mm)
{
    return (obs_mm_to_ms(dist_mm) * 2U) + 1500U;
}

static void drain_obstacle_mbf(void)
{
    obstacle_profile_t drop;

    while (tk_rcv_mbf(MBF_OBSTACLE, &drop, TMO_POL) >= (ER)E_OK)
    {
        /* discard stale profiles queued before navigation started */
    }
}

/* ---------------------------------------------------------------------------
 * Event flags and encoders
 * ------------------------------------------------------------------------- */

static uint32_t read_flags(ID flgid)
{
    T_RFLG ref;

    if (tk_ref_flg(flgid, &ref) < (ER)E_OK)
    {
        return 0U;
    }
    return (uint32_t)ref.flgptn;
}

static void set_nav_flag(uint32_t ptn)
{
    ER const er = tk_set_flg(EVF_NAV, (UINT)ptn);

    if (er < (ER)E_OK)
    {
        dbg_put_err("obs_avoid set EVF_NAV", (int32_t)er);
    }
}

static void clear_nav_flag(uint32_t ptn)
{
    ER const er = tk_clr_flg(EVF_NAV, ~(UINT)ptn);

    if (er < (ER)E_OK)
    {
        dbg_put_err("obs_avoid clr EVF_NAV", (int32_t)er);
    }
}

/** True while the car must not be manoeuvred: safety has latched a fault. */
static bool safety_blocks(void)
{
    uint32_t const flags = read_flags(EVF_SAFETY);

    return (flags & (EVF_SAFETY_LIFTED | EVF_SAFETY_FAULT_LATCHED)) != 0U;
}

static bool line_is_found(void)
{
    return (read_flags(EVF_NAV) & EVF_NAV_LINE_FOUND) != 0U;
}

static void snapshot_encoders(int32_t enc[2])
{
    motion_state_t ms;

    if (motion_get_state(&ms) == E_OK)
    {
        enc[0] = ms.enc_count[0];
        enc[1] = ms.enc_count[1];
    }
    else
    {
        enc[0] = 0;
        enc[1] = 0;
    }
}

static uint32_t leg_fwd_counts(void)
{
    motion_state_t ms;

    if (motion_get_state(&ms) != E_OK)
    {
        return 0U;
    }

    int32_t const dl = (int32_t)((uint32_t)ms.enc_count[0] - (uint32_t)s_leg_start_enc[0]);
    int32_t const dr = (int32_t)((uint32_t)ms.enc_count[1] - (uint32_t)s_leg_start_enc[1]);
    int32_t const mean = (dl + dr) / 2;

    return (mean > 0) ? (uint32_t)mean : 0U;
}

/* ---------------------------------------------------------------------------
 * State machine
 * ------------------------------------------------------------------------- */

static char const *state_name(uint32_t state)
{
    static char const *const k_names[OBS_AV_STATE_COUNT] =
    {
        "IDLE", "SLOW", "TURN_OUT", "LEG_OUT", "TURN_IN1",
        "LEG_ALONG", "TURN_IN2", "LEG_BACK", "TURN_STRAIGHT", "BLOCKED"
    };

    return (state < OBS_AV_STATE_COUNT) ? k_names[state] : "?";
}

static void enter_state(uint32_t state, uint32_t dwell_ms, uint32_t now_ms)
{
    s_state = state;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_step_deadline_ms = now_ms + dwell_ms;
    dbg_printf("[S5] avoid -> %s (%lu ms)", state_name(state), (unsigned long)dwell_ms);
}

static void begin_bounded_manoeuvre(uint32_t state, uint32_t bypass_cmd,
                                    uint32_t max_ms, uint32_t now_ms)
{
    s_state = state;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_turn_moved = false;
    s_turn_still_ms = 0U;
    s_turn_start_ms = now_ms;
    s_turn_last_poll_ms = now_ms;
    snapshot_encoders(s_turn_start_enc);
    s_turn_prev_enc[0] = s_turn_start_enc[0];
    s_turn_prev_enc[1] = s_turn_start_enc[1];
    s_step_deadline_ms = now_ms + max_ms;
    dbg_printf("[S5] avoid -> %s (%lu ms max)", state_name(state), (unsigned long)max_ms);
    send_bypass(bypass_cmd);
}

static void begin_turn(uint32_t state, uint32_t side, uint32_t now_ms)
{
    uint32_t const cmd = (side == OBS_SIDE_LEFT) ? DREP_BYPASS_LEFT : DREP_BYPASS_RIGHT;

    begin_bounded_manoeuvre(state, cmd, OBS_TURN_MAX_MS, now_ms);
}

static void begin_leg(uint32_t state, uint32_t dist_mm, uint32_t now_ms)
{
    uint32_t const tmo_ms = leg_timeout_ms(dist_mm);

    s_state = state;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_leg_target_cnt = obs_mm_to_counts(dist_mm);
    snapshot_encoders(s_leg_start_enc);
    s_step_deadline_ms = now_ms + tmo_ms;
    dbg_printf("[S5] avoid -> %s (%lu mm / %lu cnt, max %lu ms)",
               state_name(state), (unsigned long)dist_mm,
               (unsigned long)s_leg_target_cnt, (unsigned long)tmo_ms);
    send_bypass(DREP_BYPASS_CONTINUE);
}

static void begin_leg_back(uint32_t now_ms)
{
    s_state = OBS_AV_LEG_BACK;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_leg_target_cnt = 0U;
    s_step_deadline_ms = now_ms + OBS_LEG_BACK_MAX_MS;
    dbg_printf("[S5] avoid -> %s (%lu ms max)",
               state_name(OBS_AV_LEG_BACK), (unsigned long)OBS_LEG_BACK_MAX_MS);
    send_bypass(DREP_BYPASS_CONTINUE);
}

static void begin_pre_turn_stop(uint32_t now_ms)
{
    s_leg_stopping = true;
    s_step_deadline_ms = now_ms + (uint32_t)OBS_PRE_TURN_STOP_MS;
    send_bypass(DREP_BYPASS_STOP);
}

static bool poll_turn(uint32_t now_ms, uint32_t *p_mean_cnt)
{
    motion_state_t ms;

    if (p_mean_cnt != NULL)
    {
        *p_mean_cnt = 0U;
    }
    if (motion_get_state(&ms) != E_OK)
    {
        return false;
    }

    uint32_t const dt_ms = (now_ms >= s_turn_last_poll_ms) ? (now_ms - s_turn_last_poll_ms) : 0U;
    uint32_t const step_dl = abs_diff_i32(ms.enc_count[0], s_turn_prev_enc[0]);
    uint32_t const step_dr = abs_diff_i32(ms.enc_count[1], s_turn_prev_enc[1]);
    uint32_t const tot_dl = abs_diff_i32(ms.enc_count[0], s_turn_start_enc[0]);
    uint32_t const tot_dr = abs_diff_i32(ms.enc_count[1], s_turn_start_enc[1]);
    uint32_t const mean_cnt = (tot_dl + tot_dr) / 2U;

    if (p_mean_cnt != NULL)
    {
        *p_mean_cnt = mean_cnt;
    }

    s_turn_last_poll_ms = now_ms;
    s_turn_prev_enc[0] = ms.enc_count[0];
    s_turn_prev_enc[1] = ms.enc_count[1];

    if (!s_turn_moved && ((tot_dl >= (uint32_t)OBS_TURN_MIN_MOVE_CNT)
                          || (tot_dr >= (uint32_t)OBS_TURN_MIN_MOVE_CNT)))
    {
        s_turn_moved = true;
    }

    if ((step_dl > 0U) || (step_dr > 0U))
    {
        s_turn_still_ms = 0U;
    }
    else if (s_turn_moved)
    {
        s_turn_still_ms += dt_ms;
        if (s_turn_still_ms >= (uint32_t)OBS_TURN_STILL_MS)
        {
            dbg_printf("[S5] %s done: %lu cnt in %lu ms",
                       state_name(s_state),
                       (unsigned long)mean_cnt,
                       (unsigned long)(now_ms - s_turn_start_ms));
            return true;
        }
    }
    else
    {
        /* Waiting for turn motion to start. */
    }
    return false;
}

/** Give authority back to line following and go idle. */
static void release(uint32_t now_ms, bool reacquired, bool resume_lf)
{
    (void)now_ms;

    if (reacquired || resume_lf)
    {
        set_nav_flag(EVF_NAV_LINE_REACQUIRED);
    }
    if (reacquired)
    {
        s_bypass_count++;
    }
    send_nav(drep_nav_avoid(DREP_NAV_AVOID_RELEASE, 0U));
    clear_nav_flag(EVF_NAV_AVOID_ACTIVE);

    s_state = OBS_AV_IDLE;
    s_rejoin_sub = REJOIN_NONE;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_step_deadline_ms = 0U;
    s_budget_deadline_ms = 0U;
    dbg_printf("[S5] avoid -> IDLE (reacquired=%d)", reacquired ? 1 : 0);
}

/** Stop the car, count the abort and release authority on a safety fault. */
static void abort_manoeuvre(uint32_t now_ms, char const *p_why)
{
    dbg_printf("[S5] avoid ABORT in %s: %s", state_name(s_state), p_why);
    send_bypass(DREP_BYPASS_STOP);
    s_abort_count++;
    release(now_ms, false, false);
}

/** Hand authority back WITHOUT stopping, letting LF run its own search. */
static void handoff_no_line(uint32_t now_ms, char const *p_why)
{
    dbg_printf("[S5] avoid handoff in %s, line not found: %s",
               state_name(s_state), p_why);
    s_abort_count++;
    release(now_ms, false, true);
}

static void start_rejoin_turn(rejoin_sub_t sub, uint32_t side,
                              uint32_t max_ms, uint32_t now_ms)
{
    uint32_t const cmd = (side == OBS_SIDE_LEFT) ? DREP_BYPASS_LEFT : DREP_BYPASS_RIGHT;

    s_rejoin_sub = sub;
    begin_bounded_manoeuvre(OBS_AV_TURN_STRAIGHT, cmd, max_ms, now_ms);
}

static void start_rejoin_leg(rejoin_sub_t sub, uint32_t dist_mm, uint32_t now_ms)
{
    s_rejoin_sub = sub;
    begin_leg(OBS_AV_TURN_STRAIGHT, dist_mm, now_ms);
}

/** Step TURN_STRAIGHT (early line lock) and the directional fallback rejoin. */
static void handle_turn_straight(uint32_t cur_ms)
{
    if (s_leg_stopping)
    {
        if ((int32_t)(cur_ms - s_step_deadline_ms) < 0)
        {
            return;
        }
        s_leg_stopping = false;
        switch (s_rejoin_sub)
        {
            case REJOIN_NUDGE1:
                start_rejoin_leg(REJOIN_FWD1, (uint32_t)OBS_REJOIN_FWD1_MM, cur_ms);
                break;

            case REJOIN_FWD1:
                start_rejoin_turn(REJOIN_NUDGE2, s_side,
                                  (uint32_t)OBS_REJOIN_NUDGE2_MAX_MS, cur_ms);
                break;

            case REJOIN_NUDGE2:
                start_rejoin_leg(REJOIN_FWD2, (uint32_t)OBS_REJOIN_FWD2_MM, cur_ms);
                break;

            case REJOIN_ALIGN:
                start_rejoin_turn(REJOIN_UNNUDGE, s_rejoin_unnudge_side,
                                  (uint32_t)OBS_REJOIN_NUDGE_MAX_MS, cur_ms);
                break;

            case REJOIN_NONE:
            case REJOIN_FWD2:
            case REJOIN_UNNUDGE:
            default:
                if (line_is_found())
                {
                    release(cur_ms, true, true);
                }
                else
                {
                    handoff_no_line(cur_ms, "rejoin state mismatch");
                }
                break;
        }
        return;
    }

    uint32_t mean_cnt = 0U;

    switch (s_rejoin_sub)
    {
        case REJOIN_NONE:
        {
            bool const turn_stopped = poll_turn(cur_ms, &mean_cnt);

            if ((mean_cnt >= (uint32_t)OBS_STRAIGHT_LOCK_MIN_CNT) && line_is_found())
            {
                dbg_printf("[S5] straight lock at %lu cnt", (unsigned long)mean_cnt);
                release(cur_ms, true, true);
            }
            else if (turn_stopped || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                if (line_is_found())
                {
                    release(cur_ms, true, true);
                }
                else
                {
                    dbg_puts("[S5] TURN_STRAIGHT ended off-line -> nudge1");
                    start_rejoin_turn(REJOIN_NUDGE1, other_side(s_side),
                                      (uint32_t)OBS_REJOIN_NUDGE_MAX_MS, cur_ms);
                }
            }
            break;
        }

        case REJOIN_NUDGE1:
        {
            bool const turn_stopped = poll_turn(cur_ms, &mean_cnt);

            if (line_is_found())
            {
                dbg_printf("[S5] rejoin line hit in nudge1 at %lu cnt",
                           (unsigned long)mean_cnt);
                release(cur_ms, true, true);
            }
            else if ((mean_cnt >= (uint32_t)OBS_REJOIN_NUDGE_CNT)
                     || turn_stopped || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                begin_pre_turn_stop(cur_ms);
            }
            break;
        }

        case REJOIN_FWD1:
            if (line_is_found())
            {
                s_rejoin_unnudge_side = s_side;
                start_rejoin_leg(REJOIN_ALIGN, (uint32_t)OBS_REJOIN_SEARCH_ALIGN_MM, cur_ms);
            }
            else if ((leg_fwd_counts() >= s_leg_target_cnt)
                     || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                dbg_puts("[S5] rejoin fwd1 done, trying other side");
                begin_pre_turn_stop(cur_ms);
            }
            else
            {
                send_bypass(DREP_BYPASS_CONTINUE);
            }
            break;

        case REJOIN_NUDGE2:
        {
            bool const turn_stopped = poll_turn(cur_ms, &mean_cnt);

            if (line_is_found())
            {
                dbg_printf("[S5] rejoin line hit in nudge2 at %lu cnt",
                           (unsigned long)mean_cnt);
                release(cur_ms, true, true);
            }
            else if ((mean_cnt >= (uint32_t)OBS_REJOIN_NUDGE2_CNT)
                     || turn_stopped || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                begin_pre_turn_stop(cur_ms);
            }
            break;
        }

        case REJOIN_FWD2:
            if (line_is_found())
            {
                s_rejoin_unnudge_side = other_side(s_side);
                start_rejoin_leg(REJOIN_ALIGN, (uint32_t)OBS_REJOIN_SEARCH_ALIGN_MM, cur_ms);
            }
            else if ((leg_fwd_counts() >= s_leg_target_cnt)
                     || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                handoff_no_line(cur_ms, "rejoin search exhausted");
            }
            else
            {
                send_bypass(DREP_BYPASS_CONTINUE);
            }
            break;

        case REJOIN_ALIGN:
            if ((leg_fwd_counts() >= s_leg_target_cnt)
                || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                begin_pre_turn_stop(cur_ms);
            }
            else
            {
                send_bypass(DREP_BYPASS_CONTINUE);
            }
            break;

        case REJOIN_UNNUDGE:
        default:
        {
            bool const turn_stopped = poll_turn(cur_ms, &mean_cnt);

            if (((mean_cnt >= (uint32_t)OBS_REJOIN_UNNUDGE_LOCK_CNT) && line_is_found())
                || (mean_cnt >= (uint32_t)OBS_REJOIN_NUDGE_CNT)
                || turn_stopped || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                dbg_printf("[S5] rejoin unnudge done at %lu cnt", (unsigned long)mean_cnt);
                release(cur_ms, true, true);
            }
            break;
        }
    }
}

/** Begin a bypass from a fresh completed width profile. */
static void start_manoeuvre(obstacle_profile_t prof, uint32_t now_ms)
{
    uint32_t side = OBS_SIDE_LEFT;

    s_rejoin_sub = REJOIN_NONE;
    set_nav_flag(EVF_NAV_AVOID_ACTIVE);
    clear_nav_flag(EVF_NAV_LINE_REACQUIRED);
    s_budget_deadline_ms = now_ms + (uint32_t)OBS_BYPASS_BUDGET_MS + OBS_LEG_BACK_MAX_MS;

    if (!obs_choose_side(prof, &side))
    {
        dbg_printf("[S5] blocked: clrL=%lu clrR=%lu need %lu",
                   (unsigned long)drep_obs_clear_l_mm(prof),
                   (unsigned long)drep_obs_clear_r_mm(prof),
                   (unsigned long)OBS_BYPASS_CLEAR_MIN_MM);
        send_nav(drep_nav_avoid(DREP_NAV_AVOID_BLOCKED, 0U));
        begin_bounded_manoeuvre(OBS_AV_BLOCKED, DREP_BYPASS_REVERSE,
                                OBS_BLOCKED_DWELL_MS, now_ms);
        return;
    }

    uint32_t const dist_mm = drep_obs_distance_cm(prof) * 10U;
    uint32_t const width_mm = drep_obs_width_mm(prof);

    s_side = side;
    s_leg_out_mm = obs_leg_out_mm(width_mm);
    s_leg_along_mm = obs_leg_along_mm(dist_mm, width_mm);

    dbg_printf("[S5] bypass %s: d=%lu cm w=%lu mm out=%lu mm (%lu cnt) along=%lu mm (%lu cnt)",
               (s_side == OBS_SIDE_LEFT) ? "LEFT" : "RIGHT",
               (unsigned long)drep_obs_distance_cm(prof),
               (unsigned long)width_mm,
               (unsigned long)s_leg_out_mm,
               (unsigned long)obs_mm_to_counts(s_leg_out_mm),
               (unsigned long)s_leg_along_mm,
               (unsigned long)obs_mm_to_counts(s_leg_along_mm));

    begin_turn(OBS_AV_TURN_OUT, s_side, now_ms);
}

/** One manoeuvre step completed or timed out: advance to the next. */
static void step_expired(uint32_t now_ms)
{
    switch (s_state)
    {
        case OBS_AV_SLOW:
            /* Scan timed out or object gone: resume line following. */
            release(now_ms, false, true);
            break;

        case OBS_AV_TURN_OUT:
            begin_leg(OBS_AV_LEG_OUT, s_leg_out_mm, now_ms);
            break;

        case OBS_AV_LEG_OUT:
            if (!s_leg_stopping)
            {
                begin_pre_turn_stop(now_ms);
            }
            else
            {
                begin_turn(OBS_AV_TURN_IN1, other_side(s_side), now_ms);
            }
            break;

        case OBS_AV_TURN_IN1:
            begin_leg(OBS_AV_LEG_ALONG, s_leg_along_mm, now_ms);
            break;

        case OBS_AV_LEG_ALONG:
            if (!s_leg_stopping)
            {
                begin_pre_turn_stop(now_ms);
            }
            else
            {
                begin_turn(OBS_AV_TURN_IN2, other_side(s_side), now_ms);
            }
            break;

        case OBS_AV_TURN_IN2:
            begin_leg_back(now_ms);
            break;

        case OBS_AV_LEG_BACK:
            if (s_leg_stopping)
            {
                begin_turn(OBS_AV_TURN_STRAIGHT, s_side, now_ms);
            }
            else if (s_line_aligning)
            {
                begin_pre_turn_stop(now_ms);
            }
            else
            {
                handoff_no_line(now_ms, "LEG_BACK dwell elapsed");
            }
            break;

        case OBS_AV_TURN_STRAIGHT:
            release(now_ms, true, true);
            break;

        case OBS_AV_BLOCKED:
            /* Backed off; stop and wait for stationary confirmation & re-scan. */
            send_bypass(DREP_BYPASS_STOP);
            enter_state(OBS_AV_SLOW, (uint32_t)OBS_SCAN_WAIT_MAX_MS, now_ms);
            break;

        case OBS_AV_IDLE:
        default:
            break;
    }
}

/** A profile has arrived while VC is navigating. */
static void on_profile(obstacle_profile_t prof, uint32_t now_ms)
{
    uint32_t const seq = drep_obs_seq(prof);

    s_profile_count++;

    if (s_have_seq && (seq == s_last_seq))
    {
        return;
    }
    s_have_seq = true;
    s_last_seq = seq;

    if (drep_obs_no_echo(prof))
    {
        if (s_state == OBS_AV_SLOW)
        {
            release(now_ms, false, true);
        }
        return;
    }

    if ((s_state == OBS_AV_IDLE)
        && ((int32_t)(now_ms - s_grace_until_ms) < 0))
    {
        return;
    }

    uint32_t const dist_mm = drep_obs_distance_cm(prof) * 10U;

    if (s_state == OBS_AV_IDLE)
    {
        if (dist_mm > ((uint32_t)OBS_D_DETECT_CM * 10U))
        {
            return;
        }
        /* Watch-mode hit: take AVOID authority and halt immediately so the
         * stationary confirmation and slow 5-deg width scan run at rest. */
        set_nav_flag(EVF_NAV_AVOID_ACTIVE);
        clear_nav_flag(EVF_NAV_LINE_REACQUIRED);
        send_nav(drep_nav_avoid(DREP_NAV_AVOID_SLOW, 0U));
        send_bypass(DREP_BYPASS_STOP);
        s_budget_deadline_ms = now_ms + (uint32_t)OBS_BYPASS_BUDGET_MS
                               + OBS_LEG_BACK_MAX_MS;
        enter_state(OBS_AV_SLOW, (uint32_t)OBS_SCAN_WAIT_MAX_MS, now_ms);
        return;
    }

    if ((s_state == OBS_AV_SLOW) || (s_state == OBS_AV_BLOCKED))
    {
        uint32_t const width_mm = drep_obs_width_mm(prof);

        /* Ignore preliminary halt (width = 0) while waiting for the slow sweep. */
        if (width_mm == 0U)
        {
            return;
        }
        if (dist_mm > (((uint32_t)OBS_D_DETECT_CM * 10U) + (uint32_t)OBS_DETECT_HYST_MM))
        {
            if (s_state == OBS_AV_SLOW)
            {
                release(now_ms, false, true);
            }
            return;
        }
        start_manoeuvre(prof, now_ms);
    }
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

void obs_avoid_init(void)
{
    s_state = OBS_AV_IDLE;
    s_rejoin_sub = REJOIN_NONE;
    s_side = OBS_SIDE_LEFT;
    s_rejoin_unnudge_side = OBS_SIDE_LEFT;
    s_step_deadline_ms = 0U;
    s_budget_deadline_ms = 0U;
    s_grace_until_ms = 0U;
    s_leg_out_mm = 0U;
    s_leg_along_mm = 0U;
    s_leg_target_cnt = 0U;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_turn_moved = false;
    s_turn_still_ms = 0U;
    s_was_nav = false;
    s_have_seq = false;
    s_last_seq = 0U;
}

void obs_avoid_abandon(void)
{
    if (s_state != OBS_AV_IDLE)
    {
        dbg_printf("[S5] avoid abandon in %s: run starting", state_name(s_state));
    }
    clear_nav_flag(EVF_NAV_AVOID_ACTIVE | EVF_NAV_LINE_REACQUIRED);
    s_state = OBS_AV_IDLE;
    s_rejoin_sub = REJOIN_NONE;
    s_leg_stopping = false;
    s_line_aligning = false;
    s_step_deadline_ms = 0U;
    s_budget_deadline_ms = 0U;
    s_grace_until_ms = app_period_now_ms() + (uint32_t)OBS_START_GRACE_MS;
}

void obs_avoid_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    s_running = true;

    for (;;)
    {
        uint32_t const vc_pre = vc_get_system_state();
        bool const     nav_pre = vc_is_navigating(vc_pre);

        if (nav_pre && !s_was_nav)
        {
            drain_obstacle_mbf();
            s_have_seq = false;
            clear_nav_flag(EVF_NAV_AVOID_ACTIVE | EVF_NAV_LINE_REACQUIRED);
            s_was_nav = true;
        }

        uint32_t const now_ms = app_period_now_ms();
        uint32_t       wait_ms = OBS_IDLE_POLL_MS;

        if (s_state != OBS_AV_IDLE)
        {
            int32_t const  left = (int32_t)(s_step_deadline_ms - now_ms);
            uint32_t const poll_cap_ms = state_uses_turn_poll(s_state, s_rejoin_sub)
                                         ? (uint32_t)OBS_TURN_POLL_MS
                                         : (state_uses_leg_poll(s_state, s_rejoin_sub)
                                            ? (uint32_t)OBS_CTRL_POLL_MS : 0U);

            wait_ms = (left > 0) ? (uint32_t)left : 0U;
            if ((poll_cap_ms != 0U) && !s_leg_stopping && (wait_ms > poll_cap_ms))
            {
                wait_ms = poll_cap_ms;
            }
        }

        obstacle_profile_t prof;
        INT const          size = tk_rcv_mbf(MBF_OBSTACLE, &prof, (TMO)wait_ms);
        uint32_t const     woke_ms = app_period_now_ms();
        bool const         nav_now = vc_is_navigating(vc_get_system_state());

        if (!nav_now)
        {
            if (s_state != OBS_AV_IDLE)
            {
                clear_nav_flag(EVF_NAV_AVOID_ACTIVE | EVF_NAV_LINE_REACQUIRED);
                s_state = OBS_AV_IDLE;
                s_rejoin_sub = REJOIN_NONE;
                s_leg_stopping = false;
                s_line_aligning = false;
                s_step_deadline_ms = 0U;
                s_budget_deadline_ms = 0U;
                dbg_puts("[S5] avoid reset: VC idle");
            }
            s_was_nav = false;
            continue;
        }

        if (!s_was_nav)
        {
            clear_nav_flag(EVF_NAV_AVOID_ACTIVE | EVF_NAV_LINE_REACQUIRED);
            s_have_seq = false;
            s_was_nav = true;
        }

        /* Safety outranks everything (design.md section 4 precedence). */
        if ((s_state != OBS_AV_IDLE) && safety_blocks())
        {
            abort_manoeuvre(woke_ms, "safety fault latched");
            continue;
        }

        if ((size >= 0) && ((size_t)size >= sizeof(prof)))
        {
            on_profile(prof, woke_ms);
        }
        else if (size >= 0)
        {
            dbg_printf("[S5] short profile: %d B", (int)size);
        }
        else
        {
            /* E_TMOUT: normal step poll / deadline wakeup. */
        }

        if (s_state == OBS_AV_IDLE)
        {
            continue;
        }

        uint32_t const cur_ms = app_period_now_ms();

        /* Whole-manoeuvre safety budget. */
        if ((int32_t)(cur_ms - s_budget_deadline_ms) >= 0)
        {
            if (s_state == OBS_AV_BLOCKED)
            {
                abort_manoeuvre(cur_ms, "blocked, budget spent");
            }
            else
            {
                handoff_no_line(cur_ms, "bypass budget exhausted");
            }
            continue;
        }

        if (s_state == OBS_AV_TURN_STRAIGHT)
        {
            handle_turn_straight(cur_ms);
            continue;
        }

        if (state_is_turn_or_blocked(s_state))
        {
            if (poll_turn(cur_ms, NULL) || ((int32_t)(cur_ms - s_step_deadline_ms) >= 0))
            {
                step_expired(cur_ms);
            }
            continue;
        }

        if (s_state == OBS_AV_SLOW)
        {
            if ((int32_t)(cur_ms - s_step_deadline_ms) >= 0)
            {
                step_expired(cur_ms);
            }
            continue;
        }

        /* Straight legs: LEG_OUT, LEG_ALONG, LEG_BACK. */
        if (s_leg_stopping)
        {
            if ((int32_t)(cur_ms - s_step_deadline_ms) >= 0)
            {
                step_expired(cur_ms);
            }
            continue;
        }

        if ((s_state == OBS_AV_LEG_BACK) && !s_line_aligning && line_is_found())
        {
            snapshot_encoders(s_leg_start_enc);
            s_line_aligning = true;
            s_leg_target_cnt = obs_mm_to_counts((uint32_t)OBS_REJOIN_ALIGN_MM);
            s_step_deadline_ms = cur_ms + leg_timeout_ms((uint32_t)OBS_REJOIN_ALIGN_MM);
            dbg_printf("[S5] line found on LEG_BACK, aligning axle %lu mm",
                       (unsigned long)OBS_REJOIN_ALIGN_MM);
        }

        if ((s_state == OBS_AV_LEG_OUT) || (s_state == OBS_AV_LEG_ALONG) || s_line_aligning)
        {
            if (leg_fwd_counts() >= s_leg_target_cnt)
            {
                dbg_printf("[S5] %s leg target %lu cnt reached -> stopping",
                           state_name(s_state), (unsigned long)s_leg_target_cnt);
                begin_pre_turn_stop(cur_ms);
                continue;
            }
        }

        if ((int32_t)(cur_ms - s_step_deadline_ms) >= 0)
        {
            step_expired(cur_ms);
        }
        else
        {
            /* Keep Motion Control's unbounded hold alive during straight legs. */
            send_bypass(DREP_BYPASS_CONTINUE);
        }
    }
}

bool obs_avoid_is_running(void)
{
    return s_running;
}

uint32_t obs_avoid_get_state(void)
{
    return s_state;
}

uint32_t obs_avoid_get_side(void)
{
    return s_side;
}

uint32_t obs_avoid_get_bypass_count(void)
{
    return s_bypass_count;
}

uint32_t obs_avoid_get_abort_count(void)
{
    return s_abort_count;
}

uint32_t obs_avoid_get_profile_count(void)
{
    return s_profile_count;
}

uint32_t obs_avoid_get_sent_count(void)
{
    return s_sent_count;
}
