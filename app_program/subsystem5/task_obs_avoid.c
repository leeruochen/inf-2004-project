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
 *      has been lifted / overturned, or if the manoeuvre runs past its budget.
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

/** How often a straight-line leg re-sends its drive command.
 *
 *  Motion Control holds an UNBOUNDED command for only MOTION_T_HOLD_MS
 *  (100 ms) and then stops the car (task_motion.c, stop_internal).  A
 *  command counts as a bounded manoeuvre, and so gets the much longer
 *  MOTION_MANOEUVRE_MAX_MS hold, only when its argument is non-zero: the
 *  bypass turns carry OBS_BYPASS_TURN_DEG and are therefore bounded, but
 *  DREP_BYPASS_CONTINUE packs DREP_MC_FWD with argument 0 and is not.
 *
 *  MEASURED on the car, 2026-11-13 (INT-S5-01): with one CONTINUE per leg
 *  the car moved about 2 cm -- 100 ms at the bypass speed -- and stopped,
 *  never reaching an obstacle 30 cm ahead.  The turns worked throughout,
 *  which is why the fault looked like "it rotates but will not drive".
 *
 *  The legs are open-loop timed, so the command must be refreshed faster
 *  than the hold expires.  Half of MOTION_T_HOLD_MS leaves a whole period
 *  of margin for scheduling jitter at this task's priority. */
#define OBS_LEG_REFRESH_MS      ((uint32_t)MOTION_T_HOLD_MS / 2U)

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
 *  600 ms of the 3.0 s NFR14 budget across the four turns for nothing.
 *
 *  SCALED WITH THE SPEED, 2026-11-19.  Those six turns were timed at the
 *  then 25 cm/s bypass speed.  The turn itself is closed-loop on the
 *  encoders, so the ANGLE does not change with speed -- but the TIME it
 *  takes is inversely proportional to it.  When the bypass speed dropped
 *  to 5 cm/s the dwell stayed at 550 ms, so this task sent the next
 *  command about 1.9 s before the car had finished turning: MEASURED on
 *  the car, none of the four turns completed, the path collapsed and the
 *  car ended up back at the obstacle.
 *
 *  Deriving it from the speed keeps the two in step whatever the speed is
 *  set to: 550 ms at 25 cm/s, 2750 ms at 5 cm/s.
 *  TUNE: re-measure on a different surface -- this scaling assumes only
 *  that turn time goes as 1 / speed, which the encoder closed loop makes
 *  reasonable but which has not been measured below 25 cm/s. */
#define OBS_TURN_DWELL_MS       ((550U * 25U) \
                                 / (uint32_t)MOTION_SPEED_BYPASS_CMPS)

/** Straight run alongside the obstacle, past its depth.  The ultrasonic only
 *  ever sees the obstacle's FRONT FACE -- it cannot measure depth -- so this
 *  is the whole allowance for how far back the obstacle extends.
 *
 *  Was CAR_WIDTH_MM (120 mm), i.e. "no deeper than it is wide, plus a car
 *  width".  Reduced to OBS_SIDE_MARGIN_MM on 2026-11-04 to fit NFR14: at the
 *  measured turn time the four turns alone are 2200 ms of the 3.0 s budget,
 *  and this leg was 1133 ms of the rest.
 *
 *  2026-11-06: the team raised MOTION_SPEED_BYPASS_CMPS to 25 cm/s, which
 *  buys back about 772 ms of leg time: the sum up to the start of LEG_BACK
 *  falls from 4056 ms to about 2810 ms.  Not enough to restore 120 mm --
 *  NFR14 also covers LEG_BACK and TURN_STRAIGHT, so 2810 ms is a floor, not
 *  the whole interval, and LEG_BACK's real duration is unmeasured until the
 *  line-reacquisition integration test (INT-S5-02) runs.  Left at 50 mm.
 *
 *  RISK (owner-to-confirm, buddy 5): the assumption is now "no deeper than it
 *  is wide, plus 50 mm".  A deeper obstacle than that will be clipped on the
 *  way back in.  Measure the real obstacles on the track and raise this again
 *  if any of them are deep; correctness wins over the grade-A target.  TUNE */
/** How deep the obstacle is, along the car's direction of travel.
 *
 *  HARDCODED, and it has to be: the ultrasonic only ever sees the front
 *  face, so depth cannot be measured from a sweep at any resolution.
 *  Width comes from the fine sweep and sets the out-leg; depth is the one
 *  number that has to be told to the car.
 *
 *  150 mm: the obstacle measured for the final test (Buddy 5, 2026-11-19).
 *  TUNE: measure the real obstacle and set this.  Too small clips the
 *  obstacle on the way back in; too large only wastes a little travel. */
#define OBS_OBSTACLE_DEPTH_MM   (150U)

/** Extra sideways swing on the out-leg, beyond the geometric minimum.
 *
 *  MEASURED on the car, 2026-11-19: with the minimum only, the car did not
 *  get far enough out, so the turn back in pointed it at the obstacle
 *  rather than past it.  The legs are open-loop timed and under-travel at
 *  5 cm/s.  TUNE: reduce once the legs are closed-loop on the encoders. */
#define OBS_OUT_EXTRA_MM        (100U)

/** Allowance added to the depth for the car's own length and a margin.
 *
 *  The sensor is at the FRONT of the car, so the car is not past the
 *  obstacle when the sensor is -- its rear still has to clear it.  About
 *  150 mm of chassis sits behind the sensor on this car, plus
 *  OBS_SIDE_MARGIN_MM of margin.
 *
 *  MEASURED on the car, 2026-11-19: with the along-leg at width + 50 mm
 *  (150 mm for a 100 mm obstacle) the car turned back in too early and did
 *  not clear the object.  Was OBS_SIDE_MARGIN_MM alone.
 *
 *  TUNE (owner-to-confirm, buddy 5): the 150 mm of chassis behind the
 *  sensor is an estimate, not a measurement.  Measure from the ultrasonic
 *  face to the back of the car and set this to that plus the margin. */
#define OBS_ALONG_EXTRA_MM      (300U + (uint32_t)OBS_SIDE_MARGIN_MM)

/** Longest wait for subsystem 3 to report the line again.
 *
 *  SCALED WITH THE SPEED, 2026-11-19, for the same reason as
 *  OBS_TURN_DWELL_MS.  LF_T_SEARCH_MS is 2000 ms, which at the old 25 cm/s
 *  bypass speed was about 500 mm of travel -- far enough to get back
 *  across the line.  At 5 cm/s it is only 100 mm, and the car has roughly
 *  200 mm to come back diagonally, so the leg expired having covered half
 *  the distance and the line was never reached: MEASURED on the car, the
 *  turns were correct by then and it still reported "line not found".
 *
 *  Scaling keeps the DISTANCE this leg covers the same at any speed:
 *  2000 ms at 25 cm/s, 10000 ms at 5 cm/s.
 *
 *  This is only a cap -- the leg ends as soon as EVF_NAV_LINE_FOUND
 *  appears -- so a generous value costs nothing when the line is found and
 *  simply gives the car room to reach it when it has further to go. */
#define OBS_LEG_BACK_MAX_MS     (((uint32_t)LF_T_SEARCH_MS * 25U) \
                                 / (uint32_t)MOTION_SPEED_BYPASS_CMPS)

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

/** How long after a run starts obstacle action is held off.
 *
 *  A run should not begin with the car already inside OBS_DETECT_MM of
 *  something: it stops on the spot and the bypass geometry is computed
 *  from a range it never had room to act on.  Giving the car a moment to
 *  pull away from wherever it was placed makes the first detection a real
 *  approach rather than an artefact of the starting position.
 *
 *  2000 ms is about 100 mm at the 5 cm/s bring-up cruise.  Scanning,
 *  profiling and telemetry all continue through the grace: only the
 *  decision to act is held.
 *
 *  LIMIT: an obstacle genuinely closer than that at the start is driven
 *  into.  The grace is for bring-up, where the car is placed by hand on a
 *  known-clear line, not a substitute for starting on clear track. */
#define OBS_START_GRACE_MS      (2000U)

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
/** Obstacle action is held off until this time (OBS_START_GRACE_MS). */
static uint32_t s_grace_until_ms = 0U;
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
     * must move sideways.  At 45 degrees the diagonal is sqrt(2) times that.
     *
     * Plus OBS_OUT_EXTRA_MM: MEASURED on the car, 2026-11-19, the geometric
     * minimum was not enough in practice.  The legs are open-loop timed
     * from MOTION_SPEED_BYPASS_CMPS, and at the 5 cm/s bring-up speed the
     * car travels less than the commanded distance, so a leg sized to the
     * exact geometry falls short.  Swinging wider also leaves room for the
     * run back in to meet the line at a shallower angle. */
    uint32_t const lateral_mm = (width_mm / 2U)
                                + ((uint32_t)CAR_WIDTH_MM / 2U)
                                + (uint32_t)OBS_SIDE_MARGIN_MM
                                + (uint32_t)OBS_OUT_EXTRA_MM;

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

/** True in the states where the car is meant to be driving in a straight
 *  line under an unbounded DREP_MC_FWD, which Motion Control drops after
 *  MOTION_T_HOLD_MS unless it is re-sent (see OBS_LEG_REFRESH_MS). */
static bool state_is_straight_leg(uint32_t state)
{
    return (state == OBS_AV_LEG_OUT) || (state == OBS_AV_LEG_ALONG)
           || (state == OBS_AV_LEG_BACK);
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

    return (flags & (EVF_SAFETY_LIFTED | EVF_SAFETY_FAULT_LATCHED)) != 0U;
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

/** Stop the car, count the abort and release authority.
 *
 *  Only for a fault the car must not keep driving through: a latched safety
 *  fault or a lifted / overturned car.  Running out of bypass time is NOT one of those --
 *  see handoff_no_line(). */
static void abort_manoeuvre(uint32_t now_ms, char const *p_why)
{
    dbg_printf("[S5] avoid ABORT in %s: %s", state_name(s_state), p_why);
    send_bypass(DREP_BYPASS_STOP);
    s_abort_count++;
    release(now_ms, false);
}

/** Hand authority back WITHOUT stopping, having never seen the line.
 *
 *  design.md 5.5: "If the bypass exceeds its 3.0 s budget, the handoff still
 *  happens with 'line not found' and Line-Following starts its own search,
 *  limited by T_search, so the worst case from obstacle to TRACK_LOST stop is
 *  5.0 s."  So the car must still be ROLLING when authority changes hands --
 *  Line-Following cannot search for a line from a standstill, and the 5.0 s
 *  worst case in NFR14 is bypass budget + LF_T_SEARCH_MS, which only adds up
 *  if the search actually runs.
 *
 *  This used to call abort_manoeuvre(), which sends DREP_BYPASS_STOP first.
 *  MEASURED 2026-11-04: the car completed the bypass correctly and then sat
 *  still with the line under it, because LEG_BACK expiring took the stopping
 *  path.  Motion holds its last setpoint for T_hold across the authority
 *  change (design.md 5.5), so leaving the bypass speed in place is what keeps
 *  the car moving into Line-Following's search. */
static void handoff_no_line(uint32_t now_ms, char const *p_why)
{
    dbg_printf("[S5] avoid handoff in %s, line not found: %s",
               state_name(s_state), p_why);
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
    /* The along-leg is set by the obstacle's DEPTH, not its width: width
     * is what the sweep can see, depth is what the car has to drive past.
     * The out-leg is diagonal, so it has already carried the car about
     * one lateral offset forward; the rest is left as margin rather than
     * subtracted, which errs towards clearing the obstacle. */
    s_leg_along_ms = obs_mm_to_ms((uint32_t)OBS_OBSTACLE_DEPTH_MM
                                  + OBS_ALONG_EXTRA_MM);

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
            /* Never found the line inside LF_T_SEARCH_MS.  Hand over still
             * rolling so Line-Following can run its own search. */
            handoff_no_line(now_ms, "LEG_BACK dwell elapsed");
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

    /* Still pulling away from where the car was placed: keep scanning and
     * reporting, but do not act yet (OBS_START_GRACE_MS). */
    if ((s_state == OBS_AV_IDLE)
        && ((int32_t)(now_ms - s_grace_until_ms) < 0))
    {
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
        /* AVOID_SLOW first: this is the message the Vehicle Controller
         * takes AVOID authority on, so it has to go out before any bypass
         * command will be honoured. */
        send_nav(drep_nav_avoid(DREP_NAV_AVOID_SLOW, 0U));
        /* Then actually HALT, rather than crawling on.
         *
         * MEASURED on the car, 2026-11-19: slowing was not enough.  The
         * side is chosen on the NEXT profile, and a full coarse-plus-fine
         * scan cycle takes on the order of a second; at the 15 cm/s
         * bring-up cruise the car covered the remaining detect distance in
         * less than that and drove into the obstacle before it had decided
         * anything.  Stopping makes the decision distance independent of
         * how long the scan takes: the car halts, the scanner completes a
         * fresh sweep, obs_choose_side() picks the clear side from it, and
         * only then does the car move again. */
        send_bypass(DREP_BYPASS_STOP);
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

/** Abandon any manoeuvre in progress and return to OBS_AV_IDLE.
 *
 *  The scanner runs from boot, so if an obstacle is already in front of the
 *  car while the run has not started the avoidance state machine cycles
 *  through complete bypasses that nothing ever drives: the legs are
 *  open-loop timed and assume the car is moving.  MEASURED on the car,
 *  2026-11-19: at the moment the run began the state machine was in
 *  OBS_AV_LEG_ALONG, left over from the boot settle, so the first real
 *  detection could not open a fresh manoeuvre -- it ran the stale budget
 *  out ("bypass budget exhausted") and went idle without ever turning.
 *
 *  A manoeuvre started before the car could move carries no information
 *  about where the car now is, so the run must begin from a clean state.
 *  This also clears EVF_NAV_AVOID_ACTIVE, since no avoidance is active.
 *
 *  NOTE: the production fix belongs at the start of a run, which S5 cannot
 *  currently observe -- there is no run-state flag in EVF_NAV (ipc_types.h
 *  defines only AVOID_ACTIVE, LINE_FOUND and LINE_REACQUIRED).  Adding one
 *  changes a shared header and needs team agreement. */
void obs_avoid_abandon(void)
{
    if (s_state != OBS_AV_IDLE)
    {
        dbg_printf("[S5] avoid abandon in %s: run starting", state_name(s_state));
    }
    clear_nav_flag(EVF_NAV_AVOID_ACTIVE);
    clear_nav_flag(EVF_NAV_LINE_REACQUIRED);
    s_state = OBS_AV_IDLE;
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
        uint32_t const now_ms = app_period_now_ms();
        uint32_t       wait_ms = OBS_IDLE_POLL_MS;

        if (s_state != OBS_AV_IDLE)
        {
            int32_t const left = (int32_t)(s_step_deadline_ms - now_ms);

            wait_ms = (left > 0) ? (uint32_t)left : 0U;

            /* A straight leg must not sleep past the motion hold, or the
             * car stops mid-leg.  Wake early enough to re-send. */
            if (state_is_straight_leg(s_state) && (wait_ms > OBS_LEG_REFRESH_MS))
            {
                wait_ms = OBS_LEG_REFRESH_MS;
            }
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

        /* Whole-manoeuvre budget (NFR14).  Out of time is not a fault: hand
         * over rolling and let Line-Following search (design.md 5.5).  The
         * one exception is being boxed in -- OBS_AV_BLOCKED has deliberately
         * stopped or reversed the car, so it must not be sent onwards. */
        if ((int32_t)(app_period_now_ms() - s_budget_deadline_ms) >= 0)
        {
            if (s_state == OBS_AV_BLOCKED)
            {
                abort_manoeuvre(app_period_now_ms(), "blocked, budget spent");
            }
            else
            {
                handoff_no_line(app_period_now_ms(), "bypass budget exhausted");
            }
            continue;
        }

        /* Line found during the run back: cut the leg short and straighten. */
        if (s_state == OBS_AV_LEG_BACK)
        {
            if ((read_flags(EVF_NAV) & EVF_NAV_LINE_FOUND) != 0U)
            {
                /* Hand over AT ONCE, without straightening first.
                 *
                 * The car used to pivot back towards the line
                 * (OBS_AV_TURN_STRAIGHT) and release only once that turn
                 * had finished.  That dwell scales with the bypass speed,
                 * so at 5 cm/s it is 2750 ms: MEASURED on the car, the
                 * sensor crossed the line, the car then pivoted for most
                 * of three seconds, and Line Following took over with
                 * CENTRE reading 168 raw -- plain white, the line well
                 * behind it.  reacquired=1 was reported and the line was
                 * lost all the same.
                 *
                 * Line Following is the right owner of the approach: it
                 * has REACQUIRING and a recovery ramp for exactly this,
                 * and st_suspended() is written to resume from
                 * LINE_REACQUIRED and track as soon as CENTRE is on black.
                 * Releasing while the line is still under the sensor hands
                 * it the one thing it needs.
                 *
                 * TRADE-OFF: the car is still at the bypass angle when it
                 * takes over, so it meets the line off-axis rather than
                 * square.  That is what its recovery is for, and an
                 * off-axis line it can see beats a square approach to a
                 * line it has already driven past. */
                release(app_period_now_ms(), true);
                continue;
            }
        }

        if ((int32_t)(app_period_now_ms() - s_step_deadline_ms) >= 0)
        {
            step_expired(app_period_now_ms());
        }
        else if (state_is_straight_leg(s_state))
        {
            /* Still mid-leg: refresh the drive command before Motion
             * Control's hold expires and stops the car. */
            send_bypass(DREP_BYPASS_CONTINUE);
        }
        else
        {
            /* Turns are bounded manoeuvres and hold themselves. */
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
