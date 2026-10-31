/*
 *  task_vc.c
 *  Subsystem 2 -- Vehicle Controller task (priority 2) and 10-state System FSM.
 *  Kernel side: kernel and project headers only, never SDK headers (rule R1)
 *  and never blocking SDK waits (rule R2).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Responsibilities (design.md sections 3-5, HANDOVER.md section 3,
 *  docs/DATA_REPRESENTATION.md sections 2-4):
 *    - Sole reader of MBF_NAV_IN (IF-01); validates every message with
 *      drep_nav_valid(), counting and logging rejects.
 *    - Arbitrates competing navigation inputs in fixed precedence order:
 *      Safety > Avoid > Barcode > Line, and holds authority mode
 *      (VC_AUTH_LINE or VC_AUTH_AVOID).
 *    - Owns the 4-bit 10-state System FSM (VC_SYS_*), publishing state
 *      transitions to MBF_TELEMETRY (DREP_TEL_STATE, TMO_POL) and through
 *      vc_get_system_state().
 *    - Reads EVF_SAFETY (IF-06) with tk_ref_flg() only (TA_WSGL); commands
 *      immediate brake/stop when EVF_SAFETY_FAULT_LATCHED is set, and clears
 *      EVF_SAFETY_FAULT_LATCHED only on a remote RESET after the car has been
 *      stationary (speed == 0) for at least SAFETY_T_STILL_MS (500 ms).
 */

#include "task_vc.h"

#include "config/thresholds.h"
#include "config/timing.h"
#include "src/debug/dbg.h"

/** Turn angle (degrees) commanded for an obstacle bypass step. TUNE */
#define VC_BYPASS_TURN_DEG      (45)
/** Reverse distance (cm) commanded for a reverse-and-reattempt bypass. TUNE */
#define VC_BYPASS_REV_CM        (15)
/** Turn angle (degrees) commanded for a 90-degree junction turn. */
#define VC_JUNCTION_TURN_DEG    (90)

static volatile bool     s_task_running = false;
static volatile uint32_t s_sys_state = VC_SYS_INITIALISING;
static volatile uint32_t s_authority = VC_AUTH_LINE;
static volatile uint32_t s_reject_count = 0U;
static volatile uint32_t s_sent_cmd_count = 0U;
static motion_cmd_t      s_last_motion_cmd = { 0U };

static bool     s_stationary_valid = false;
static uint32_t s_stationary_since_ms = 0U;

static bool     s_bc_pending = false;
static uint32_t s_bc_cmd = DREP_BC_CMD_STRAIGHT;
static uint32_t s_bc_dist_cm = 0U;

static bool is_nav_substate(uint32_t state)
{
    return (state >= VC_SYS_NAV_LINE_TRACKING) && (state <= VC_SYS_NAV_TURN_RIGHT);
}

static void set_system_state(uint32_t new_state, uint32_t now_ms)
{
    uint32_t const clamped = new_state & DREP_MASK(DREP_FSM_SYSTEM_BITS);

    if (clamped >= VC_SYS_STATE_COUNT)
    {
        return;
    }
    if (s_sys_state != clamped)
    {
        tel_evt_t const evt = drep_tel_make(DREP_TEL_STATE, now_ms, clamped, false);

        s_sys_state = clamped;
        /* Non-blocking telemetry post (NFR1: drop if full). */
        (void)tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL);
    }
}

static void send_motion_cmd(motion_cmd_t cmd)
{
    s_last_motion_cmd = cmd;
    s_sent_cmd_count++;

    /* Drain-then-send on full 4-slot queue so the latest VC decision wins. */
    if (tk_snd_mbf(MBF_MOTION_CMD, &cmd, (INT)sizeof(cmd), TMO_POL) == E_TMOUT)
    {
        motion_cmd_t dropped;

        (void)tk_rcv_mbf(MBF_MOTION_CMD, &dropped, TMO_POL);
        (void)tk_snd_mbf(MBF_MOTION_CMD, &cmd, (INT)sizeof(cmd), TMO_POL);
    }
}

/* Update stationary tracking from motion_state_t (under MTX_MOTION) and
 * return the current cumulative distance in cm. */
static uint32_t update_stationary_tracker(uint32_t now_ms)
{
    motion_state_t ms;
    int32_t        speed_x10 = 0;
    uint32_t       dist_cm = 0U;

    if (motion_get_state(&ms) == E_OK)
    {
        speed_x10 = drep_ms_speed_x10(ms.w);
        dist_cm = drep_ms_distance_cm(ms.w);
    }

    if (speed_x10 == 0)
    {
        if (!s_stationary_valid)
        {
            s_stationary_since_ms = now_ms;
            s_stationary_valid = true;
        }
    }
    else
    {
        s_stationary_valid = false;
        s_stationary_since_ms = now_ms;
    }
    return dist_cm;
}

static bool is_stationary_for_reset(uint32_t now_ms)
{
    if (!s_stationary_valid)
    {
        return false;
    }
    return (uint32_t)(now_ms - s_stationary_since_ms) >= SAFETY_T_STILL_MS;
}

static bool is_fault_latched(void)
{
    T_RFLG rflg;

    if (tk_ref_flg(EVF_SAFETY, &rflg) != E_OK)
    {
        return false;
    }
    return (rflg.flgptn & EVF_SAFETY_FAULT_LATCHED) != 0U;
}

static void handle_remote_msg(nav_input_t msg, bool fault_latched, uint32_t now_ms)
{
    uint32_t const type = drep_nav_type(msg);

    if (type == DREP_NAV_REMOTE_RESET)
    {
        if (fault_latched)
        {
            if (is_stationary_for_reset(now_ms))
            {
                /* Clear ONLY the FAULT_LATCHED bit (ARCHITECTURE.md D9). */
                (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
                s_authority = VC_AUTH_LINE;
                s_bc_pending = false;
                set_system_state(VC_SYS_READY_WAITING, now_ms);
                send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
            }
            else
            {
                set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
                send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
            }
        }
        return;
    }

    if (fault_latched)
    {
        set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
        return;
    }

    if (type == DREP_NAV_REMOTE_STOP)
    {
        s_authority = VC_AUTH_LINE;
        s_bc_pending = false;
        set_system_state(VC_SYS_READY_WAITING, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
    }
    else if (type == DREP_NAV_REMOTE_START)
    {
        if (s_sys_state == VC_SYS_INITIALISING)
        {
            set_system_state(VC_SYS_READY_WAITING, now_ms);
        }
        if (s_sys_state == VC_SYS_READY_WAITING)
        {
            s_authority = VC_AUTH_LINE;
            s_bc_pending = false;
            set_system_state(VC_SYS_NAV_LINE_TRACKING, now_ms);
            send_motion_cmd(drep_mc_pack(DREP_MC_FWD, (int32_t)MOTION_SPEED_CRUISE_CMPS, 0));
        }
    }
}

static void handle_avoid_msg(nav_input_t msg, uint32_t now_ms)
{
    uint32_t const type = drep_nav_type(msg);

    if (type == DREP_NAV_AVOID_RELEASE)
    {
        s_authority = VC_AUTH_LINE;
        if (s_sys_state == VC_SYS_OBSTACLE_AVOIDANCE)
        {
            set_system_state(VC_SYS_NAV_LINE_TRACKING, now_ms);
            send_motion_cmd(drep_mc_pack(DREP_MC_FWD, (int32_t)MOTION_SPEED_CRUISE_CMPS, 0));
        }
        return;
    }

    /* Enter or stay in avoidance if currently navigating or authority is AVOID. */
    if ((!is_nav_substate(s_sys_state)) && (s_sys_state != VC_SYS_OBSTACLE_AVOIDANCE)
        && (s_authority != VC_AUTH_AVOID))
    {
        return;
    }

    s_authority = VC_AUTH_AVOID;
    set_system_state(VC_SYS_OBSTACLE_AVOIDANCE, now_ms);

    if (type == DREP_NAV_AVOID_SLOW)
    {
        send_motion_cmd(drep_mc_pack(DREP_MC_FWD, (int32_t)MOTION_SPEED_SLOW_CMPS, 0));
    }
    else if (type == DREP_NAV_AVOID_BLOCKED)
    {
        send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
    }
    else if (type == DREP_NAV_AVOID_BYPASS)
    {
        switch (drep_nav_bypass(msg))
        {
            case DREP_BYPASS_STOP:
                send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
                break;
            case DREP_BYPASS_CONTINUE:
                send_motion_cmd(drep_mc_pack(DREP_MC_FWD, (int32_t)MOTION_SPEED_BYPASS_CMPS, 0));
                break;
            case DREP_BYPASS_LEFT:
                send_motion_cmd(drep_mc_pack(DREP_MC_TURN_L,
                                             (int32_t)MOTION_SPEED_BYPASS_CMPS,
                                             VC_BYPASS_TURN_DEG));
                break;
            case DREP_BYPASS_RIGHT:
                send_motion_cmd(drep_mc_pack(DREP_MC_TURN_R,
                                             (int32_t)MOTION_SPEED_BYPASS_CMPS,
                                             VC_BYPASS_TURN_DEG));
                break;
            case DREP_BYPASS_REVERSE:
                send_motion_cmd(drep_mc_pack(DREP_MC_BACK,
                                             (int32_t)MOTION_SPEED_BYPASS_CMPS,
                                             VC_BYPASS_REV_CM));
                break;
            default:
                break;
        }
    }
}

static void handle_barcode_msg(nav_input_t msg, uint32_t cur_dist_cm)
{
    if ((s_authority != VC_AUTH_LINE) || (!is_nav_substate(s_sys_state)))
    {
        return;
    }
    s_bc_cmd = drep_nav_barcode_cmd(msg);
    s_bc_dist_cm = cur_dist_cm;
    s_bc_pending = true;
}

static void handle_line_msg(nav_input_t msg, uint32_t cur_dist_cm, uint32_t now_ms)
{
    uint32_t const type = drep_nav_type(msg);

    /* Precedence rule: AVOID outranks LINE (design.md section 4). */
    if (s_authority != VC_AUTH_LINE)
    {
        return;
    }
    if (!is_nav_substate(s_sys_state))
    {
        return;
    }

    /* Expire a pending barcode command if the car travelled past D_cmd (30 cm). */
    if (s_bc_pending && ((uint16_t)(cur_dist_cm - s_bc_dist_cm) > BC_D_CMD_CM))
    {
        s_bc_pending = false;
    }

    if (type == DREP_NAV_LINE_TRACK_LOST)
    {
        s_bc_pending = false;
        set_system_state(VC_SYS_READY_WAITING, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
        return;
    }

    if (type == DREP_NAV_LINE_JUNCTION)
    {
        if (s_bc_pending)
        {
            s_bc_pending = false;
            switch (s_bc_cmd)
            {
                case DREP_BC_CMD_LEFT:
                    set_system_state(VC_SYS_NAV_TURN_LEFT, now_ms);
                    send_motion_cmd(drep_mc_pack(DREP_MC_TURN_L,
                                                 (int32_t)MOTION_SPEED_SLOW_CMPS,
                                                 VC_JUNCTION_TURN_DEG));
                    return;
                case DREP_BC_CMD_RIGHT:
                    set_system_state(VC_SYS_NAV_TURN_RIGHT, now_ms);
                    send_motion_cmd(drep_mc_pack(DREP_MC_TURN_R,
                                                 (int32_t)MOTION_SPEED_SLOW_CMPS,
                                                 VC_JUNCTION_TURN_DEG));
                    return;
                case DREP_BC_CMD_UTURN:
                    set_system_state(VC_SYS_NAV_U_TURN, now_ms);
                    send_motion_cmd(drep_mc_pack(DREP_MC_UTURN,
                                                 (int32_t)MOTION_SPEED_SLOW_CMPS,
                                                 0));
                    return;
                case DREP_BC_CMD_STRAIGHT:
                default:
                    set_system_state(VC_SYS_NAV_GO_STRAIGHT, now_ms);
                    send_motion_cmd(drep_mc_pack(DREP_MC_FWD,
                                                 (int32_t)MOTION_SPEED_CRUISE_CMPS,
                                                 0));
                    return;
            }
        }
        set_system_state(VC_SYS_NAV_GO_STRAIGHT, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_FWD, (int32_t)MOTION_SPEED_CRUISE_CMPS, 0));
        return;
    }

    if (type == DREP_NAV_LINE_STEER)
    {
        int32_t const steer_q7 = drep_nav_steer_q7(msg);
        int32_t const steer_arg = steer_q7 * 2;     /* Q7 (-128..+127) -> -256..+254 */

        set_system_state(VC_SYS_NAV_LINE_TRACKING, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_STEER,
                                     (int32_t)MOTION_SPEED_CRUISE_CMPS,
                                     steer_arg));
    }
}

static void process_one_nav(nav_input_t msg, uint32_t cur_dist_cm, uint32_t now_ms)
{
    bool const fault_latched = is_fault_latched();

    if (!drep_nav_valid(msg))
    {
        s_reject_count++;
        dbg_puts("[VC] invalid nav_input_t rejected");
        return;
    }

    /* Remote commands are checked first because RESET is the only exit from
     * FAULT_LATCHED and STOP always returns to READY. */
    if (drep_nav_source(msg) == DREP_NAV_SRC_REMOTE)
    {
        handle_remote_msg(msg, fault_latched, now_ms);
        return;
    }

    /* Safety overrides every non-RESET navigation message (S2-U24). */
    if (fault_latched)
    {
        set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
        return;
    }

    switch (drep_nav_source(msg))
    {
        case DREP_NAV_SRC_AVOID:
            handle_avoid_msg(msg, now_ms);
            break;
        case DREP_NAV_SRC_BARCODE:
            handle_barcode_msg(msg, cur_dist_cm);
            break;
        case DREP_NAV_SRC_LINE:
            handle_line_msg(msg, cur_dist_cm, now_ms);
            break;
        default:
            break;
    }
}

void vc_init(void)
{
    uint32_t const now_ms = app_period_now_ms();

    s_sys_state = VC_SYS_INITIALISING;
    s_authority = VC_AUTH_LINE;
    s_reject_count = 0U;
    s_sent_cmd_count = 0U;
    s_last_motion_cmd.w = 0U;
    s_stationary_valid = true;
    s_stationary_since_ms = now_ms;
    s_bc_pending = false;
    s_bc_cmd = DREP_BC_CMD_STRAIGHT;
    s_bc_dist_cm = 0U;
}

void vc_complete_selftest(bool passed)
{
    uint32_t const now_ms = app_period_now_ms();

    if ((!passed) || is_fault_latched())
    {
        set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
    }
    else
    {
        set_system_state(VC_SYS_READY_WAITING, now_ms);
    }
}

static void vc_drain_mbf_internal(uint32_t cur_dist_cm, uint32_t now_ms)
{
    nav_input_t msg;
    bool        any_msg = false;

    for (;;)
    {
        INT const n = tk_rcv_mbf(MBF_NAV_IN, &msg, TMO_POL);

        if (n == (INT)E_TMOUT)
        {
            break;
        }
        if (n != (INT)sizeof(msg))
        {
            s_reject_count++;
            continue;
        }
        any_msg = true;
        process_one_nav(msg, cur_dist_cm, now_ms);
    }

    if ((!any_msg) && is_fault_latched()
        && (s_sys_state != VC_SYS_SAFETY_FAULT_LATCHED))
    {
        set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
        send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
    }
}

void vc_drain_and_step(uint32_t now_ms)
{
    uint32_t const cur_dist_cm = update_stationary_tracker(now_ms);

    /* If the VC task is running at TASK_PRI_VC (2) and waiting on MBF_NAV_IN,
     * a lower-priority caller (such as app_test_task at priority 11) has
     * already woken VC synchronously inside tk_snd_mbf().  Yield briefly if
     * the buffer is not yet empty; never call tk_rcv_mbf(MBF_NAV_IN) from two
     * tasks (design.md section 4: VC is the sole reader of MBF_NAV_IN). */
    if (s_task_running)
    {
        T_RMBF rmbf;

        while ((tk_ref_mbf(MBF_NAV_IN, &rmbf) == E_OK) && (rmbf.msgsz > 0))
        {
            (void)tk_dly_tsk(1);
        }
        if (is_fault_latched() && (s_sys_state != VC_SYS_SAFETY_FAULT_LATCHED))
        {
            set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
            send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
        }
        return;
    }

    vc_drain_mbf_internal(cur_dist_cm, now_ms);
}

void vc_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    vc_init();
    vc_complete_selftest(!is_fault_latched());
    s_task_running = true;

    for (;;)
    {
        nav_input_t msg;
        INT const   n = tk_rcv_mbf(MBF_NAV_IN, &msg, (TMO)MOTION_PERIOD_MS);
        uint32_t    now_ms = app_period_now_ms();
        uint32_t    cur_dist_cm = update_stationary_tracker(now_ms);

        if (n == (INT)sizeof(msg))
        {
            process_one_nav(msg, cur_dist_cm, now_ms);
            vc_drain_mbf_internal(cur_dist_cm, now_ms);
        }
        else if (is_fault_latched() && (s_sys_state != VC_SYS_SAFETY_FAULT_LATCHED))
        {
            set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
            send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
        }
    }
    /* Never calls tk_ext_tsk() (rule R6). */
}

uint32_t vc_get_system_state(void)
{
    return s_sys_state;
}

bool vc_is_task_running(void)
{
    return s_task_running;
}

uint32_t vc_get_authority(void)
{
    return s_authority;
}

void vc_set_authority(uint32_t auth)
{
    s_authority = (auth == VC_AUTH_AVOID) ? VC_AUTH_AVOID : VC_AUTH_LINE;
}

uint32_t vc_get_reject_count(void)
{
    return s_reject_count;
}

uint32_t vc_get_sent_cmd_count(void)
{
    return s_sent_cmd_count;
}

motion_cmd_t vc_get_last_motion_cmd(void)
{
    return s_last_motion_cmd;
}
