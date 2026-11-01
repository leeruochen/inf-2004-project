/*
 *  task_tel.c
 *  Subsystem 1 -- Telemetry task (priority 9) and the tel_ producer API.
 *  Kernel side: kernel and project headers only, never SDK headers (rule R1)
 *  and never blocking SDK waits (rule R2).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  One wake every TEL_WAKE_PERIOD_MS (20 ms, app_period_wait):
 *    1. net_poll()                 network work (WiFi build only)
 *    2. drain MBF_TELEMETRY        TMO_POL; events -> frames -> outbox
 *    3. newest obstacle profile    -> event_obstacle frame -> outbox
 *    4. flush the outbox           when the link can send (<= 8 frames)
 *    5. every 10th wake (200 ms)   telemetry_motion + telemetry_sensor
 *       every 50th wake (1 s)      heartbeat (also right after a state change)
 *    6. received commands          check -> MBF_NAV_IN (WiFi build only)
 *    7. link state machine         connect / teardown / backoff, WIFI_LOSS bit
 *
 *  NFR1: nothing here waits for the network.  Producers post with TMO_POL.
 *  The only kernel waits are the 20 ms period, the MTX_MOTION copy (held for
 *  a 16-byte copy by its owner) and a 5 ms MBF_NAV_IN send for a command.
 *  Link state never changes vehicle behaviour: WIFI_LOSS is a status bit
 *  that no control task acts on (DATA_REPRESENTATION.md section 3).
 */

#include "task_tel.h"

#include "../subsystem2/subsystem2.h"   /* motion_get_state(), vc_get_system_state() */
#include "config/timing.h"
#include "drv_wifi.h"
#include "tel_outbox.h"
#include "tel_wire.h"

_Static_assert(TEL_LINK_STATE_COUNT == DREP_FSM_WIFI_STATES, "tel_link states must match the 3-bit WiFi FSM");
_Static_assert((TEL_PUBLISH_EVERY_N_WAKES > 0U) && (TEL_HEARTBEAT_EVERY_N_WAKES > 0U), "telemetry rates");

/* Snapshot word layouts (IF-12).  Bit 31 = a producer has written it. */
#define TEL_SNAP_VALID          (1UL << 31)
#define TEL_SNAP_IR_BITS        (12U)
#define TEL_SNAP_IR_MAX         (4095U)
#define TEL_SNAP_RANGE_BITS     (9U)
#define TEL_SNAP_ATT_BITS       (8U)

/* ---------------------------------------------------------------------------
 * State.  Everything below is written by the Telemetry task only, except the
 * producer-side words marked "producer".
 * ------------------------------------------------------------------------- */

static volatile bool     s_running = false;
static tel_stats_t       s_stats;
static volatile uint32_t s_evt_post_drops = 0U;        /* producer (any task) */

static tel_link_t        s_link;
static tel_outbox_t      s_outbox;
static bool              s_publish_failed = false;      /* this wake */
static bool              s_hb_due = false;              /* state changed: heartbeat now */
static bool              s_wifi_loss_set = false;
static uint32_t          s_periodic_tick = 0U;
static uint32_t          s_heartbeat_tick = 0U;
static uint32_t          s_obs_published_seq = 0U;

/* IF-12 latest-value snapshot: one writer task per word. */
static volatile uint32_t s_snap_ir = 0U;                /* producer: Line-Following */
static volatile uint32_t s_snap_range = 0U;             /* producer: Obstacle Scanning */
static volatile uint32_t s_snap_att = 0U;               /* producer: IMU Processing */

/* Newest obstacle profile, sequence-locked: odd = being written. */
static volatile uint32_t s_obs_seq = 0U;                /* producer: one Obstacle task */
static volatile uint32_t s_obs_lo = 0U;
static volatile uint32_t s_obs_hi = 0U;

static tel_link_cfg_t const k_link_cfg =
{
    TEL_CONNECT_TIMEOUT_MS,
    TEL_KEEPALIVE_MS,
    TEL_BACKOFF_MIN_MS,
    TEL_BACKOFF_MAX_MS,
};

/* ===========================================================================
 * Producer API (subsystem1.h).  Any task.  Never waits.
 * ========================================================================= */

bool tel_post(tel_evt_t evt)
{
    if (tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL) != E_OK)
    {
        s_evt_post_drops++;     /* a lost increment under a race only undercounts */
        return false;
    }
    return true;
}

bool tel_post_event(uint32_t type, uint32_t payload, bool oor)
{
    return tel_post(drep_tel_make(type, app_period_now_ms(), payload, oor));
}

bool tel_post_hump(int32_t height_mm, int32_t pitch_half_deg)
{
    return tel_post(drep_tel_hump(app_period_now_ms(), height_mm, pitch_half_deg));
}

bool tel_post_barcode(uint32_t bc_cmd)
{
    return tel_post_event(DREP_TEL_BARCODE, bc_cmd & DREP_MASK(DREP_BC_CMD_BITS), bc_cmd > DREP_BC_CMD_UTURN);
}

bool tel_post_fault(uint32_t evf_safety_bits)
{
    return tel_post_event(DREP_TEL_FAULT, evf_safety_bits & EVF_SAFETY_ALL_MASK, false);
}

void tel_set_ir_raw(uint32_t ir_r_raw, uint32_t ir_c_raw)
{
    uint32_t const r = (ir_r_raw > TEL_SNAP_IR_MAX) ? TEL_SNAP_IR_MAX : ir_r_raw;
    uint32_t const c = (ir_c_raw > TEL_SNAP_IR_MAX) ? TEL_SNAP_IR_MAX : ir_c_raw;

    s_snap_ir = TEL_SNAP_VALID | (c << TEL_SNAP_IR_BITS) | r;
}

void tel_set_range_cm(uint32_t range_cm)
{
    uint32_t const cm = (range_cm > (uint32_t)DREP_OBS_DIST_MAX_CM) ? (uint32_t)DREP_OBS_DIST_MAX_CM : range_cm;

    s_snap_range = TEL_SNAP_VALID | cm;
}

void tel_set_attitude(int32_t pitch_half_deg, int32_t roll_half_deg)
{
    uint32_t const p = (uint32_t)drep_pitch_q1(pitch_half_deg, NULL) & DREP_MASK(TEL_SNAP_ATT_BITS);
    uint32_t const r = (uint32_t)drep_pitch_q1(roll_half_deg, NULL) & DREP_MASK(TEL_SNAP_ATT_BITS);

    s_snap_att = TEL_SNAP_VALID | (r << TEL_SNAP_ATT_BITS) | p;
}

void tel_post_obstacle(obstacle_profile_t prof)
{
    /* Single writer.  The reader (P9) is lower priority than any obstacle
     * task, so it can be preempted mid-read but never runs mid-write. */
    s_obs_seq = s_obs_seq + 1U;             /* odd: writing */
    DREP_COMPILER_BARRIER();
    s_obs_lo = prof.lo;
    s_obs_hi = prof.hi;
    DREP_COMPILER_BARRIER();
    s_obs_seq = s_obs_seq + 1U;             /* even: stable, new */
}

void tel_get_stats(tel_stats_t *p_out)
{
    if (p_out == NULL)
    {
        return;
    }
    *p_out = s_stats;
    p_out->evt_post_drops = s_evt_post_drops;
    p_out->link_state = tel_link_state(&s_link);
    p_out->outbox_dropped = s_outbox.dropped;
    p_out->outbox_peak = s_outbox.peak;
    p_out->connects = s_link.connects;
    p_out->link_failures = s_link.failures;
}

bool tel_is_running(void)
{
    return s_running;
}

#if APP_TEST
void tel_test_snapshot(uint32_t *p_ir, uint32_t *p_range, uint32_t *p_att)
{
    *p_ir = s_snap_ir;
    *p_range = s_snap_range;
    *p_att = s_snap_att;
}

bool tel_test_obstacle_pending(void)
{
    return s_obs_seq != s_obs_published_seq;
}

uint32_t tel_test_outbox_count(void)
{
    return tel_outbox_count(&s_outbox);
}
#endif

/* ===========================================================================
 * Backend: console (dbg_ ring) or WiFi/MQTT (drv_wifi.c)
 * ========================================================================= */

/* Publish one frame now.  Returns true if the backend took it. */
static bool backend_publish(uint8_t const *p_frame, uint32_t len)
{
    char const *p_topic = tel_wire_topic(p_frame[1]);

    if (p_topic == NULL)
    {
        return true;    /* not a telemetry type: nothing to send, never retried */
    }
#if TEL_BACKEND_WIFI
    {
        int32_t const rc = net_publish(p_topic, p_frame, len, false);

        if (rc != NET_OK)
        {
            s_stats.publish_fail++;
            s_publish_failed = true;
            return false;
        }
    }
#else
    {
        char hex[(DREP_FRAME_MAX_BYTES * 2U) + 1U];

        (void)tel_wire_hex(hex, (uint32_t)sizeof(hex), p_frame, len);
        dbg_printf("[TEL] %s %s", p_topic, hex);
    }
#endif
    s_stats.frames_sent++;
    return true;
}

static bool backend_net_up(void)
{
#if TEL_BACKEND_WIFI
    return net_wifi_up() && net_mqtt_up();
#else
    return true;    /* the console is always there */
#endif
}

static bool backend_attempt_failed(void)
{
#if TEL_BACKEND_WIFI
    return net_stage() == NET_STAGE_FAILED;
#else
    return false;
#endif
}

static void backend_actions(uint32_t act)
{
#if TEL_BACKEND_WIFI
    if ((act & TEL_LINK_ACT_TEARDOWN) != 0U)
    {
        bool const     leave = !net_wifi_up();
        uint32_t const stage = net_stage();     /* where the attempt stopped, before the reset */

        net_teardown(leave);
        dbg_printf("[TEL] teardown (stage %lu, status %ld%s), retry in %lu ms",
                   (unsigned long)stage, (long)net_last_status(), leave ? ", leave AP" : "",
                   (unsigned long)s_link.last_wait_ms);
    }
    if ((act & TEL_LINK_ACT_CONNECT) != 0U)
    {
        if (!net_is_initialised() && (net_init() != NET_OK))
        {
            dbg_puts("[TEL] CYW43 init failed; retry after backoff");
            return;
        }
        if (net_connect_begin() != NET_OK)
        {
            dbg_puts("[TEL] connect start failed; retry after backoff");
        }
    }
#else
    (void)act;
#endif
}

/* ===========================================================================
 * Receive path for commands
 * ========================================================================= */

int32_t tel_cmd_receive(char const *p_topic, uint8_t const *p_buf, uint32_t n)
{
    nav_input_t nav;
    int32_t     rc;
    ER          er;

    nav.w = 0U;
    rc = tel_wire_cmd(p_topic, p_buf, n, &nav);
    if (rc != DREP_FRAME_OK)
    {
        s_stats.cmd_rejected++;
        dbg_printf("[TEL] cmd rejected: %s (%lu B)", tel_wire_reject_name(rc), (unsigned long)n);
        return rc;
    }
    er = tk_snd_mbf(MBF_NAV_IN, &nav, (INT)sizeof(nav), (TMO)NAV_MBF_SEND_TMO_MS);
    if (er != E_OK)
    {
        s_stats.cmd_post_fail++;
        dbg_put_err("tel: MBF_NAV_IN send", (int32_t)er);
        return TEL_CMD_POST_FAIL;
    }
    s_stats.cmd_accepted++;
    dbg_printf("[TEL] cmd %s -> VC", p_topic);
    return DREP_FRAME_OK;
}

static void handle_commands(void)
{
#if TEL_BACKEND_WIFI
    net_rx_msg_t msg;
    uint32_t     i;

    for (i = 0U; (i < TEL_CMD_MAX_PER_WAKE) && net_rx_pop(&msg); i++)
    {
        /* Only the first NET_RX_DATA_MAX bytes are stored; a longer message
         * can never be a valid command, so reject it without reading on. */
        if (msg.len > NET_RX_DATA_MAX)
        {
            s_stats.cmd_rejected++;
            dbg_printf("[TEL] cmd rejected: BAD_LENGTH (%lu B)", (unsigned long)msg.len);
            continue;
        }
        (void)tel_cmd_receive(msg.topic, msg.data, msg.len);
    }
#endif
}

/* ===========================================================================
 * Frames
 * ========================================================================= */

static void outbox_add(uint8_t const *p_frame, uint32_t len)
{
    (void)tel_outbox_push(&s_outbox, p_frame, len);
}

/* One tel_evt_t from MBF_TELEMETRY. */
static void handle_event(tel_evt_t e)
{
    uint8_t  frame[DREP_FRAME_MAX_BYTES];
    uint32_t len = 0U;
    uint32_t ts;

    s_stats.evt_rx++;
    if (!drep_tel_valid(e))
    {
        s_stats.evt_invalid++;
        return;
    }
    ts = drep_tel_timestamp_ms(e);
    switch (drep_tel_type(e))
    {
        case DREP_TEL_HUMP:
            len = drep_frame_hump(frame, ts, e.payload & DREP_MASK(16U));
            break;
        case DREP_TEL_BARCODE:
            len = drep_frame_barcode(frame, ts, e.payload & DREP_MASK(DREP_BC_CMD_BITS));
            break;
        case DREP_TEL_FAULT:
            len = drep_frame_fault(frame, ts, e.payload & EVF_SAFETY_ALL_MASK);
            break;
        case DREP_TEL_STATE:
            s_hb_due = true;    /* the heartbeat carries the 4-bit system state */
            break;
        default:
            s_stats.evt_invalid++;
            break;
    }
    if (len != 0U)
    {
        outbox_add(frame, len);
    }
}

static void drain_events(void)
{
    tel_evt_t e;
    uint32_t  i;

    /* At most one queue's worth per wake, so producers can never keep this
     * loop going forever. */
    for (i = 0U; i < MBF_TELEMETRY_DEPTH; i++)
    {
        INT const n = tk_rcv_mbf(MBF_TELEMETRY, &e, TMO_POL);

        if (n != (INT)sizeof(e))
        {
            break;  /* E_TMOUT: empty */
        }
        handle_event(e);
    }
}

static void check_obstacle(void)
{
    uint32_t           seq1;
    uint32_t           seq2;
    obstacle_profile_t prof;
    uint8_t            frame[DREP_FRAME_MAX_BYTES];

    seq1 = s_obs_seq;
    if (((seq1 & 1U) != 0U) || (seq1 == s_obs_published_seq))
    {
        return;     /* being written (try next wake) or nothing new */
    }
    DREP_COMPILER_BARRIER();
    prof.lo = s_obs_lo;
    prof.hi = s_obs_hi;
    DREP_COMPILER_BARRIER();
    seq2 = s_obs_seq;
    if (seq1 != seq2)
    {
        return;     /* overwritten while reading: take the newer one next wake */
    }
    s_obs_published_seq = seq1;
    outbox_add(frame, drep_frame_obstacle(frame, app_period_now_ms(), prof));
}

static void flush_outbox(void)
{
    uint32_t i;

    for (i = 0U; i < TEL_FLUSH_MAX_PER_WAKE; i++)
    {
        tel_frame_t const *p_f = tel_outbox_peek(&s_outbox);

        if ((p_f == NULL) || !tel_link_can_send(&s_link))
        {
            break;
        }
        if (!backend_publish(p_f->buf, p_f->len))
        {
            break;  /* keep it; the link FSM sees publish_failed */
        }
        tel_outbox_pop(&s_outbox);
    }
}

/* Send a periodic frame now, or skip it (stale in 200 ms: never buffered). */
static void send_periodic(uint8_t const *p_frame, uint32_t len)
{
    if (!tel_link_can_send(&s_link) || s_publish_failed)
    {
        s_stats.offline_skipped++;
        return;
    }
    (void)backend_publish(p_frame, len);
}

static bool console_takes(uint32_t tick)
{
#if TEL_BACKEND_WIFI
    (void)tick;
    return true;
#else
    return (TEL_CONSOLE_PERIODIC_DIV != 0U) && ((tick % TEL_CONSOLE_PERIODIC_DIV) == 0U);
#endif
}

static void publish_periodic(void)
{
    uint8_t        frame[DREP_FRAME_MAX_BYTES];
    motion_state_t ms;
    uint32_t const ir = s_snap_ir;
    uint32_t const range = s_snap_range;
    uint32_t const att = s_snap_att;

    s_periodic_tick++;
    if (!console_takes(s_periodic_tick))
    {
        return;
    }

    if (motion_get_state(&ms) == E_OK)
    {
        send_periodic(frame, drep_frame_motion(frame, ms.tick_ms, ms.w, ms.enc_count[0], ms.enc_count[1]));
    }

    /* The sensor frame only once some producer has written a value. */
    if (((ir | range | att) & TEL_SNAP_VALID) != 0U)
    {
        send_periodic(frame,
                      drep_frame_sensor(frame, app_period_now_ms(),
                                        ir & DREP_MASK(TEL_SNAP_IR_BITS),
                                        (ir >> TEL_SNAP_IR_BITS) & DREP_MASK(TEL_SNAP_IR_BITS),
                                        range & DREP_MASK(TEL_SNAP_RANGE_BITS),
                                        drep_sext(att, TEL_SNAP_ATT_BITS),
                                        drep_sext(att >> TEL_SNAP_ATT_BITS, TEL_SNAP_ATT_BITS)));
    }
}

static uint32_t safety_bits(void)
{
    T_RFLG rflg;

    if (tk_ref_flg(EVF_SAFETY, &rflg) != E_OK)
    {
        return 0U;
    }
    return (uint32_t)rflg.flgptn & EVF_SAFETY_ALL_MASK;
}

static void publish_heartbeat(uint32_t now_ms, bool forced)
{
    uint8_t frame[DREP_FRAME_MAX_BYTES];

    if (!forced)
    {
        s_heartbeat_tick++;
        if (!console_takes(s_heartbeat_tick))
        {
            return;
        }
    }
    send_periodic(frame, drep_frame_heartbeat(frame, now_ms, vc_get_system_state(), safety_bits()));
}

/* ===========================================================================
 * Link supervision
 * ========================================================================= */

static void update_wifi_loss_bit(uint32_t now_ms)
{
    bool const lost = tel_link_is_lost(&s_link);

    if (lost == s_wifi_loss_set)
    {
        return;
    }
    s_wifi_loss_set = lost;
    if (lost)
    {
        uint8_t frame[DREP_FRAME_MAX_BYTES];

        (void)tk_set_flg(EVF_SAFETY, (UINT)EVF_SAFETY_FAULT_WIFI_LOSS);
        /* Buffered, so the dashboard learns when the outage started. */
        outbox_add(frame, drep_frame_fault(frame, now_ms, safety_bits()));
        dbg_puts("[TEL] link lost: FAULT_WIFI_LOSS set (vehicle unaffected)");
    }
    else
    {
        (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_WIFI_LOSS);
        dbg_puts("[TEL] link back: FAULT_WIFI_LOSS cleared");
    }
}

static void step_link(uint32_t now_ms)
{
    tel_link_in_t in;
    uint32_t      before;
    uint32_t      act;

    in.net_up = backend_net_up();
    in.attempt_failed = backend_attempt_failed();
    in.publish_failed = s_publish_failed;
    in.outbox_empty = (tel_outbox_count(&s_outbox) == 0U);

    before = tel_link_state(&s_link);
    act = tel_link_step(&s_link, now_ms, &in);
    if (tel_link_state(&s_link) != before)
    {
        dbg_printf("[TEL] link %s -> %s", tel_link_state_name(before),
                   tel_link_state_name(tel_link_state(&s_link)));
    }
    backend_actions(act);
    update_wifi_loss_bit(now_ms);
}

static void log_status(void)
{
    tel_stats_t st;

    tel_get_stats(&st);
    dbg_printf("[TEL] %s tx=%lu txf=%lu off=%lu ev=%lu bad=%lu qd=%lu od=%lu cmd=%lu/%lu",
               tel_link_state_name(st.link_state), (unsigned long)st.frames_sent,
               (unsigned long)st.publish_fail, (unsigned long)st.offline_skipped,
               (unsigned long)st.evt_rx, (unsigned long)st.evt_invalid,
               (unsigned long)st.evt_post_drops, (unsigned long)st.outbox_dropped,
               (unsigned long)st.cmd_accepted, (unsigned long)st.cmd_rejected);
}

/* ===========================================================================
 * Task
 * ========================================================================= */

static void tel_task(INT stacd, void *p_exinf)
{
    app_period_t   per;
    uint32_t const status_every = TEL_STATUS_LOG_PERIOD_MS / TEL_WAKE_PERIOD_MS;

    (void)stacd;
    (void)p_exinf;

    tel_outbox_init(&s_outbox);
    tel_link_init(&s_link, &k_link_cfg, app_period_now_ms());

#if TEL_BACKEND_WIFI
    /* Takes up to about a second (firmware download), preemptible; done
     * here, never on the 1 KB init stack. */
    if (net_init() != NET_OK)
    {
        dbg_puts("[TEL] CYW43 init failed; will retry");
    }
    dbg_puts("[TEL] backend: WiFi/MQTT");
#else
    dbg_puts("[TEL] backend: console ([TEL] <topic> <hex>)");
#endif

    s_running = true;
    app_period_init(&per, TEL_WAKE_PERIOD_MS);
    for (;;)
    {
        uint32_t now_ms;

        s_stats.wake_overruns += app_period_wait(&per);
        now_ms = app_period_now_ms();
        s_stats.wakes++;
        s_publish_failed = false;

#if TEL_BACKEND_WIFI
        net_poll(now_ms);
#endif
        drain_events();
        check_obstacle();
        flush_outbox();

        if ((s_stats.wakes % TEL_PUBLISH_EVERY_N_WAKES) == 0U)
        {
            publish_periodic();
        }
        if (s_hb_due)
        {
            s_hb_due = false;
            publish_heartbeat(now_ms, true);
        }
        else if ((s_stats.wakes % TEL_HEARTBEAT_EVERY_N_WAKES) == 0U)
        {
            publish_heartbeat(now_ms, false);
        }
        else
        {
            /* nothing periodic this wake */
        }

        handle_commands();
        step_link(now_ms);

        if ((s_stats.wakes % status_every) == 0U)
        {
            log_status();
        }
    }
}

int32_t tel_task_start(void)
{
    return app_start_task(tel_task, TASK_PRI_TEL, TASK_STACK_TEL_BYTES, "tel");
}
