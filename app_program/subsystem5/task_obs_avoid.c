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
 *    - Chooses the bypass side from the measured clearances (FR8).
 *    - Drives the four-turn bypass by sending nav_input_t bypass commands to
 *      MBF_NAV_IN (IF-01) with a 5 ms timeout, which the Vehicle Controller
 *      (S2, task_vc.c) turns into motion commands.
 *    - Reacquires the line after the bypass (FR9) by waiting for subsystem
 *      3's EVF_NAV_LINE_FOUND, then sets EVF_NAV_LINE_REACQUIRED.
 *    - Single setter of EVF_NAV_AVOID_ACTIVE and EVF_NAV_LINE_REACQUIRED
 *      (docs/DATA_REPRESENTATION.md section 3).
 *    - Aborts to a stop if EVF_SAFETY says the car is in a fault or
 *      collision state, or if the manoeuvre runs past its budget.
 *
 *  Scheduling: this task is NOT periodic.  It blocks in tk_rcv_mbf() on
 *  MBF_OBSTACLE for exactly the time left in the current manoeuvre step, so
 *  a fresh profile wakes it immediately and an idle car costs one wake-up
 *  every OBS_IDLE_POLL_MS.
 *
 *  TUNE, and the reason there is no better value here: OBS_TURN_DWELL_MS is
 *  how long the Vehicle Controller plus Motion Control need to finish one
 *  45 degree bypass turn.  That is a property of the car on the real floor
 *  (S2 measured 114 mm effective track), so it has to be measured on the
 *  track and recorded in testing/subsystem5/TEST_LOG.md (S5-U21, INT-S5-01),
 *  not derived at a desk.  Every leg time below is either computed from the
 *  measured obstacle geometry or marked TUNE for the same reason.
 */

#include "task_obs_avoid.h"

#include "config/thresholds.h"
#include "config/timing.h"
#include "src/debug/dbg.h"

/* ---------------------------------------------------------------------------
 * Timing
 * ------------------------------------------------------------------------- */

/** Idle wake-up period while there is no obstacle. */
#define OBS_IDLE_POLL_MS        (100U)

/** Time for the Vehicle Controller to complete one 45 degree bypass turn.
 *
 *  MEASURED on the car, 2026-11-04 (S5-U32): six in-place 45 degree turns
 *  timed from the encoder counts settling (Buddy 2's turns are closed-loop
 *  on the encoders, task_motion.c), on the track mat at the bring-up speeds:
 *
 *      LEFT  530, 470, 500 ms
 *      RIGHT 500, 470, 470 ms      mean 490 ms, spread 60 ms
 *
 *  550 ms is the mean plus about one spread, so a slow turn still finishes
 *  inside its dwell.  The previous value, 700 ms, was a guess and cost
 *  630 ms of the 3.0 s NFR14 budget across the three turns for nothing.
 *  TUNE: re-measure if the bypass speed or the surface changes. */
#define OBS_TURN_DWELL_MS       (550U)

/** Straight run alongside the obstacle, past its depth.  The ultrasonic only
 *  ever sees the obstacle's FRONT FACE -- it cannot measure depth -- so this
 *  is the whole allowance for how far back the obstacle extends.
 *
 *  Was CAR_WIDTH_MM (120 mm), i.e. "no deeper than it is wide, plus a car
 *  width".  Reduced to OBS_SIDE_MARGIN_MM on 2026-11-04 to fit NFR14: at the
 *  measured turn time the three turns alone are 1650 ms of the 3.0 s budget,
 *  and this leg was 1133 ms of the rest.
 *
 *  RISK (owner-to-confirm, buddy 5): the assumption is now "no deeper than it
 *  is wide, plus 50 mm".  A deeper obstacle than that will be clipped on the
 *  way back in.  Measure the real obstacles on the track and raise this again
 *  if any of them are deep; correctness wins over the grade-A target.  TUNE */
#define OBS_ALONG_EXTRA_MM      ((uint32_t)OBS_SIDE_MARGIN_MM)

/** Longest wait for subsystem 3 to report the line again. */
#define OBS_LEG_BACK_MAX_MS     ((uint32_t)LF_T_SEARCH_MS)

/** Reverse dwell when the car is boxed in (VC reverses VC_BYPASS_REV_CM).
 *  TODO(owner-to-confirm, buddy 5 with buddy 2): task_vc.c keeps
 *  VC_BYPASS_REV_CM = 15 private, so this dwell is an estimate until the
 *  reverse leg is timed on the car (S5-U22). */
#define OBS_BLOCKED_DWELL_MS    (1200U)

/** The Vehicle Controller turns this much per bypass turn command.
 *  Mirrors VC_BYPASS_TURN_DEG in app_program/subsystem2/task_vc.c, which is
 *  private to subsystem 2.  TODO(owner-to-confirm, buddy 5 with buddy 2):
 *  keep the two in step, or have S2 export it. */
#define OBS_BYPASS_TURN_DEG     (45U)

/** tan and sin of the bypass angle, scaled by 1000.  At 45 degrees the
 *  diagonal is sqrt(2) times the lateral offset it buys. */
#define OBS_DIAG_PER_LATERAL_M1000  (1414U)

/** MBF_NAV_IN send timeout: 5 ms, per TEAM.md section 5 IF-01. */
#define OBS_NAV_SEND_TMO_MS     (5U)

#if OBS_BYPASS_CLEAR_MIN_MM == 0U
#error "task_obs_avoid.c: OBS_BYPASS_CLEAR_MIN_MM must not be 0"
#endif
#if MOTION_SPEED_BYPASS_CMPS == 0U
#error "task_obs_avoid.c: MOTION_SPEED_BYPASS_CMPS must not be 0"
#endif
#if OBS_BYPASS_TURN_DEG != 45U
#error "task_obs_avoid.c: OBS_DIAG_PER_LATERAL_M1000 assumes a 45 degree bypass"
#endif

static bool     s_running = false;
static uint32_t s_state = OBS_AV_IDLE;
static uint32_t s_side = OBS_SIDE_LEFT;
static uint32_t s_step_deadline_ms = 0U;
static uint32_t s_budget_deadline_ms = 0U;
static uint32_t s_leg_out_ms = 0U;
static uint32_t s_leg_along_ms = 0U;
static bool     s_have_seq = false;
static uint32_t s_last_seq = 0U;
static uint32_t s_bypass_count = 0U;
static uint32_t s_abort_count = 0U;
static uint32_t s_profile_count = 0U;
static uint32_t s_sent_count = 0U;

/* ---------------------------------------------------------------------------
 * Pure decisions
 * ------------------------------------------------------------------------- */

uint32_t obs_leg_out_mm(uint32_t width_mm)
{
    /* Half the obstacle, half the car, plus the margin, is how far the car
     * must move sideways.  At 45 degrees the diagonal is sqrt(2) times that. */
    uint32_t const lateral_mm = (width_mm / 2U)
                                + ((uint32_t)CAR_WIDTH_MM / 2U)
                                + (uint32_t)OBS_SIDE_MARGIN_MM;

    return (lateral_mm * OBS_DIAG_PER_LATERAL_M1000) / 1000U;
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

static void send_turn(uint32_t side)
{
    send_bypass((side == OBS_SIDE_LEFT) ? DREP_BYPASS_LEFT : DREP_BYPASS_RIGHT);
}

/** The opposite side, for the turns that come back towards the line. */
static uint32_t other_side(uint32_t side)
{
    return (side == OBS_SIDE_LEFT) ? OBS_SIDE_RIGHT : OBS_SIDE_LEFT;
}

/* ---------------------------------------------------------------------------
 * Event flags
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

    return (flags & (EVF_SAFETY_COLLISION | EVF_SAFETY_FAULT_LATCHED)) != 0U;
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
    s_step_deadline_ms = now_ms + dwell_ms;
    dbg_printf("[S5] avoid -> %s (%lu ms)", state_name(state), (unsigned long)dwell_ms);
}

/** Give authority back to line following and go idle. */
static void release(uint32_t now_ms, bool reacquired)
{
    (void)now_ms;

    if (reacquired)
    {
        set_nav_flag(EVF_NAV_LINE_REACQUIRED);
        s_bypass_count++;
    }
    send_nav(drep_nav_avoid(DREP_NAV_AVOID_RELEASE, 0U));
    clear_nav_flag(EVF_NAV_AVOID_ACTIVE);

    s_state = OBS_AV_IDLE;
    s_step_deadline_ms = 0U;
    s_budget_deadline_ms = 0U;
    dbg_printf("[S5] avoid -> IDLE (reacquired=%d)", reacquired ? 1 : 0);
}

/** Stop the car, count the abort and release authority. */
static void abort_manoeuvre(uint32_t now_ms, char const *p_why)
{
    dbg_printf("[S5] avoid ABORT in %s: %s", state_name(s_state), p_why);
    send_bypass(DREP_BYPASS_STOP);
    s_abort_count++;
    release(now_ms, false);
}

/** Begin a bypass from a fresh profile. */
static void start_manoeuvre(obstacle_profile_t prof, uint32_t now_ms)
{
    uint32_t side = OBS_SIDE_LEFT;

    set_nav_flag(EVF_NAV_AVOID_ACTIVE);
    clear_nav_flag(EVF_NAV_LINE_REACQUIRED);
    s_budget_deadline_ms = now_ms + (uint32_t)OBS_BYPASS_BUDGET_MS + OBS_LEG_BACK_MAX_MS;

    if (!obs_choose_side(prof, &side))
    {
        /* No room either side: stop, back off, and let the next profile
         * decide again (specs.md FR8 "avoiding collisions" first). */
        dbg_printf("[S5] blocked: clrL=%lu clrR=%lu need %lu",
                   (unsigned long)drep_obs_clear_l_mm(prof),
                   (unsigned long)drep_obs_clear_r_mm(prof),
                   (unsigned long)OBS_BYPASS_CLEAR_MIN_MM);
        send_nav(drep_nav_avoid(DREP_NAV_AVOID_BLOCKED, 0U));
        send_bypass(DREP_BYPASS_REVERSE);
        enter_state(OBS_AV_BLOCKED, OBS_BLOCKED_DWELL_MS, now_ms);
        return;
    }

    s_side = side;
    s_leg_out_ms = obs_mm_to_ms(obs_leg_out_mm(drep_obs_width_mm(prof)));
    s_leg_along_ms = obs_mm_to_ms(drep_obs_width_mm(prof) + OBS_ALONG_EXTRA_MM);

    dbg_printf("[S5] bypass %s: d=%lu cm w=%lu mm out=%lu ms along=%lu ms",
               (s_side == OBS_SIDE_LEFT) ? "LEFT" : "RIGHT",
               (unsigned long)drep_obs_distance_cm(prof),
               (unsigned long)drep_obs_width_mm(prof),
               (unsigned long)s_leg_out_ms,
               (unsigned long)s_leg_along_ms);

    send_turn(s_side);
    enter_state(OBS_AV_TURN_OUT, OBS_TURN_DWELL_MS, now_ms);
}

/** One manoeuvre step has run out of time: move to the next. */
static void step_expired(uint32_t now_ms)
{
    switch (s_state)
    {
        case OBS_AV_SLOW:
            /* No fresh profile arrived while slowed: the obstacle is gone. */
            release(now_ms, false);
            break;

        case OBS_AV_TURN_OUT:
            send_bypass(DREP_BYPASS_CONTINUE);
            enter_state(OBS_AV_LEG_OUT, s_leg_out_ms, now_ms);
            break;

        case OBS_AV_LEG_OUT:
            send_turn(other_side(s_side));
            enter_state(OBS_AV_TURN_IN1, OBS_TURN_DWELL_MS, now_ms);
            break;

        case OBS_AV_TURN_IN1:
            send_bypass(DREP_BYPASS_CONTINUE);
            enter_state(OBS_AV_LEG_ALONG, s_leg_along_ms, now_ms);
            break;

        case OBS_AV_LEG_ALONG:
            send_turn(other_side(s_side));
            enter_state(OBS_AV_TURN_IN2, OBS_TURN_DWELL_MS, now_ms);
            break;

        case OBS_AV_TURN_IN2:
            send_bypass(DREP_BYPASS_CONTINUE);
            enter_state(OBS_AV_LEG_BACK, OBS_LEG_BACK_MAX_MS, now_ms);
            break;

        case OBS_AV_LEG_BACK:
            /* Never found the line inside LF_T_SEARCH_MS. */
            abort_manoeuvre(now_ms, "line not reacquired");
            break;

        case OBS_AV_TURN_STRAIGHT:
            release(now_ms, true);
            break;

        case OBS_AV_BLOCKED:
            /* Backed off; wait for the scanner's next profile. */
            send_bypass(DREP_BYPASS_STOP);
            enter_state(OBS_AV_SLOW, (uint32_t)OBS_BYPASS_BUDGET_MS, now_ms);
            break;

        case OBS_AV_IDLE:
        default:
            break;
    }
}

/** A profile has arrived.  Only acted on when it says something new. */
static void on_profile(obstacle_profile_t prof, uint32_t now_ms)
{
    uint32_t const seq = drep_obs_seq(prof);

    s_profile_count++;

    /* The scanner wraps a 2-bit sequence per profile.  Ignoring a repeat
     * stops one profile being acted on twice if the buffer is re-read.
     * DECISION (Buddy 5, HANDOVER.md open item): the sequence number is
     * KEPT, exactly for this check. */
    if (s_have_seq && (seq == s_last_seq))
    {
        return;
    }
    s_have_seq = true;
    s_last_seq = seq;

    if (drep_obs_no_echo(prof))
    {
        /* Nothing in range any more.  If a manoeuvre is already past the
         * turn-out it has to finish, or the car would stop mid-bypass with
         * the obstacle beside it. */
        if (s_state == OBS_AV_SLOW)
        {
            release(now_ms, false);
        }
        return;
    }

    uint32_t const dist_mm = drep_obs_distance_cm(prof) * 10U;

    if (dist_mm > ((uint32_t)OBS_D_DETECT_CM * 10U))
    {
        return;                         /* seen, but not close enough to act */
    }

    if (s_state == OBS_AV_IDLE)
    {
        /* Slow down first and let one more profile confirm the geometry
         * before committing to a side (specs.md FR8). */
        set_nav_flag(EVF_NAV_AVOID_ACTIVE);
        send_nav(drep_nav_avoid(DREP_NAV_AVOID_SLOW, 0U));
        /* Arm the whole-manoeuvre budget HERE, not only in
         * start_manoeuvre(): the task loop checks it in every state but
         * OBS_AV_IDLE, so leaving it at its previous value makes the budget
         * look already spent and aborts the manoeuvre before the next
         * profile can choose a side.  The car then drives straight into the
         * obstacle it had just slowed down for. */
        s_budget_deadline_ms = now_ms + (uint32_t)OBS_BYPASS_BUDGET_MS
                               + OBS_LEG_BACK_MAX_MS;
        enter_state(OBS_AV_SLOW, (uint32_t)OBS_BYPASS_BUDGET_MS, now_ms);
        return;
    }

    if ((s_state == OBS_AV_SLOW) || (s_state == OBS_AV_BLOCKED))
    {
        start_manoeuvre(prof, now_ms);
    }

    /* Mid-manoeuvre profiles are ignored on purpose: the scanner is looking
     * sideways at the obstacle being passed, so its ranges say nothing
     * useful about the path ahead. */
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

void obs_avoid_init(void)
{
    s_state = OBS_AV_IDLE;
    s_side = OBS_SIDE_LEFT;
    s_step_deadline_ms = 0U;
    s_budget_deadline_ms = 0U;
    s_leg_out_ms = 0U;
    s_leg_along_ms = 0U;
    s_have_seq = false;
    s_last_seq = 0U;
}

void obs_avoid_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    s_running = true;

    for (;;)
    {
        uint32_t const now_ms = app_period_now_ms();
        uint32_t       wait_ms = OBS_IDLE_POLL_MS;

        if (s_state != OBS_AV_IDLE)
        {
            int32_t const left = (int32_t)(s_step_deadline_ms - now_ms);

            wait_ms = (left > 0) ? (uint32_t)left : 0U;
        }

        obstacle_profile_t prof;
        INT const          size = tk_rcv_mbf(MBF_OBSTACLE, &prof, (TMO)wait_ms);
        uint32_t const     woke_ms = app_period_now_ms();

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
            /* E_TMOUT is the normal path: the step's dwell has elapsed. */
        }

        if (s_state == OBS_AV_IDLE)
        {
            continue;
        }

        /* Whole-manoeuvre budget (NFR14). */
        if ((int32_t)(app_period_now_ms() - s_budget_deadline_ms) >= 0)
        {
            abort_manoeuvre(app_period_now_ms(), "bypass budget exhausted");
            continue;
        }

        /* Line found during the run back: cut the leg short and straighten. */
        if (s_state == OBS_AV_LEG_BACK)
        {
            if ((read_flags(EVF_NAV) & EVF_NAV_LINE_FOUND) != 0U)
            {
                send_turn(s_side);
                enter_state(OBS_AV_TURN_STRAIGHT, OBS_TURN_DWELL_MS,
                            app_period_now_ms());
                continue;
            }
        }

        if ((int32_t)(app_period_now_ms() - s_step_deadline_ms) >= 0)
        {
            step_expired(app_period_now_ms());
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
