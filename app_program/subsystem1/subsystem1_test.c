/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1 (telemetry).  APP_TEST builds only.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem1/logs/ and record the result in
 *  testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01 .. S1-U10.
 *
 *  Runs in the test task (TASK_PRI_TEST = 11) while the Telemetry task (9)
 *  and every other task run, so each check waits for the Telemetry task with
 *  tk_dly_tsk() and compares counters before and after.  Works with both
 *  backends; the WiFi build waits up to S1_TEST_CONNECT_WAIT_MS for the
 *  broker in S1-U01 (the broker must be running).
 *
 *  Side effects: S1-U05 sends a remote STOP to the Vehicle Controller (the
 *  car stays or becomes READY_WAITING; motors stop).  S1-U07 overwrites the
 *  sensor snapshot and S1-U08 posts one test obstacle profile.
 */

#include "subsystem1.h"
#include "../test_report.h"
#include "task_tel.h"
#include "tel_link.h"
#include "tel_wire.h"
#include "config/timing.h"
#include "src/rtos/port/hal.h"

#if APP_TEST

#define S1_TEST_SETTLE_MS           (100U)      /* > 2 Telemetry wakes */
#define S1_TEST_POLL_MS             (20U)
#define S1_TEST_CONNECT_WAIT_MS     (45000U)    /* WiFi: join + DHCP + broker, with one retry */
#define S1_TEST_FLOOD               (64U)       /* 2 x MBF_TELEMETRY_DEPTH */
#define S1_TEST_FLOOD_MAX_US        (2000U)     /* 64 non-blocking posts */

static void settle(void)
{
    (void)tk_dly_tsk((TMO)S1_TEST_SETTLE_MS);
}

/* S1-U01: task running, link CONNECTED (console at once; WiFi within the wait). */
static bool test_u01_connected(void)
{
    tel_stats_t st;
    uint32_t    waited = 0U;

    for (;;)
    {
        tel_get_stats(&st);
        if (tel_is_running() && (st.link_state == TEL_LINK_CONNECTED))
        {
            return true;
        }
        if (waited >= S1_TEST_CONNECT_WAIT_MS)
        {
            dbg_printf("[TEST] S1-U01 link state %s after %lu ms", tel_link_state_name(st.link_state),
                       (unsigned long)waited);
            return false;
        }
        (void)tk_dly_tsk((TMO)S1_TEST_POLL_MS);
        waited += S1_TEST_POLL_MS;
    }
}

/* S1-U02: a valid event is received and framed into the outbox, then sent. */
static bool test_u02_event_path(void)
{
    tel_stats_t before;
    tel_stats_t after;
    bool        posted;

    tel_get_stats(&before);
    posted = tel_post_barcode(DREP_BC_CMD_STRAIGHT);
    settle();
    tel_get_stats(&after);
    return posted && (after.evt_rx > before.evt_rx) && (after.frames_sent > before.frames_sent)
           && (after.evt_invalid == before.evt_invalid) && (tel_test_outbox_count() == 0U);
}

/* S1-U03: an invalid tel_evt_t (type 0) is counted and never framed. */
static bool test_u03_invalid_event(void)
{
    tel_stats_t     before;
    tel_stats_t     after;
    tel_evt_t const bad = drep_tel_make(0U, 1U, 0U, false);
    bool            posted;

    tel_get_stats(&before);
    posted = tel_post(bad);
    settle();
    tel_get_stats(&after);
    return posted && (after.evt_invalid == (before.evt_invalid + 1U)) && (tel_test_outbox_count() == 0U);
}

/* S1-U04 (NFR1): producers never wait.  64 posts finish fast; a full queue is
 * a counted drop, never a block.  Invalid events, so nothing is printed. */
static bool test_u04_never_blocks(void)
{
    tel_stats_t     before;
    tel_stats_t     after;
    tel_evt_t const bad = drep_tel_make(0U, 2U, 0U, false);
    uint32_t        accepted = 0U;
    uint32_t        t0;
    uint32_t        dt;
    uint32_t        i;

    tel_get_stats(&before);
    t0 = hal_time_us();
    for (i = 0U; i < S1_TEST_FLOOD; i++)
    {
        if (tel_post(bad))
        {
            accepted++;
        }
    }
    dt = hal_time_us() - t0;
    settle();
    tel_get_stats(&after);
    dbg_printf("[TEST] S1-U04 %lu posts in %lu us, %lu queued, %lu dropped", (unsigned long)S1_TEST_FLOOD,
               (unsigned long)dt, (unsigned long)accepted,
               (unsigned long)(after.evt_post_drops - before.evt_post_drops));
    return (dt < S1_TEST_FLOOD_MAX_US)
           && ((accepted + (after.evt_post_drops - before.evt_post_drops)) == S1_TEST_FLOOD)
           && (accepted >= (MBF_TELEMETRY_DEPTH / 2U))  /* other producers may hold a few slots */
           && (after.evt_invalid >= (before.evt_invalid + accepted));
}

/* S1-U05 (FR2): a valid STOP on car/cmd/stop reaches MBF_NAV_IN. */
static bool test_u05_cmd_accept(void)
{
    tel_stats_t before;
    tel_stats_t after;
    uint8_t     f[DREP_FRAME_MAX_BYTES];
    uint32_t    n;
    int32_t     rc;

    tel_get_stats(&before);
    n = drep_frame_cmd(f, DREP_FT_CMD_STOP);
    rc = tel_cmd_receive("car/cmd/stop", f, n);
    tel_get_stats(&after);
    return (rc == DREP_FRAME_OK) && (after.cmd_accepted == (before.cmd_accepted + 1U))
           && (after.cmd_rejected == before.cmd_rejected);
}

/* S1-U06 (D10): malformed commands are rejected and never posted. */
static bool test_u06_cmd_reject(void)
{
    tel_stats_t before;
    tel_stats_t after;
    uint8_t     f[DREP_FRAME_MAX_BYTES];
    uint32_t    n;
    uint32_t    bad = 0U;

    tel_get_stats(&before);

    n = drep_frame_cmd(f, DREP_FT_CMD_START);
    f[0] = (uint8_t)(DREP_WIRE_VER + 1U);                                   /* wrong version */
    bad += (tel_cmd_receive("car/cmd/start", f, n) == DREP_FRAME_BAD_VERSION) ? 1U : 0U;

    n = drep_frame_cmd(f, DREP_FT_CMD_START);                               /* trailing byte */
    bad += (tel_cmd_receive("car/cmd/start", f, n + 1U) == DREP_FRAME_BAD_LENGTH) ? 1U : 0U;

    f[1] = 0x55U;                                                           /* unknown type */
    bad += (tel_cmd_receive("car/cmd/start", f, n) == DREP_FRAME_UNKNOWN_TYPE) ? 1U : 0U;

    n = drep_frame_cmd(f, DREP_FT_CMD_START);                               /* START on the reset topic */
    bad += (tel_cmd_receive("car/cmd/reset", f, n) == TEL_WIRE_TOPIC_MISMATCH) ? 1U : 0U;

    n = drep_frame_heartbeat(f, 1U, 0U, 0U);                                /* telemetry, not a command */
    bad += (tel_cmd_receive("car/cmd/stop", f, n) == DREP_FRAME_NOT_COMMAND) ? 1U : 0U;

    tel_get_stats(&after);
    return (bad == 5U) && (after.cmd_rejected == (before.cmd_rejected + 5U))
           && (after.cmd_accepted == before.cmd_accepted);
}

/* S1-U07: snapshot setters clamp to their wire fields and mark the word valid. */
static bool test_u07_snapshot_clamp(void)
{
    uint32_t ir;
    uint32_t range;
    uint32_t att;

    tel_set_ir_raw(5000U, 123U);
    tel_set_range_cm(1000U);
    tel_set_attitude(-200, 37);
    tel_test_snapshot(&ir, &range, &att);

    return ((ir & 0xFFFU) == 4095U) && (((ir >> 12) & 0xFFFU) == 123U) && ((ir >> 31) == 1U)
           && ((range & 0x1FFU) == 400U) && ((range >> 31) == 1U)
           && (drep_sext(att, 8U) == -90) && (drep_sext(att >> 8, 8U) == 37) && ((att >> 31) == 1U);
}

/* S1-U08: a posted obstacle profile is framed once by the Telemetry task. */
static bool test_u08_obstacle(void)
{
    tel_stats_t before;
    tel_stats_t after;
    bool        pending_before;

    tel_get_stats(&before);
    tel_post_obstacle(drep_obs_pack(30, 6, 100, 200, 250, 1U));
    pending_before = tel_test_obstacle_pending();
    settle();
    tel_get_stats(&after);
    return pending_before && !tel_test_obstacle_pending() && (after.frames_sent > before.frames_sent);
}

/* S1-U09: FAULT_WIFI_LOSS (EVF_SAFETY b8) is clear while CONNECTED. */
static bool test_u09_wifi_loss_bit(void)
{
    tel_stats_t st;
    T_RFLG      rflg;

    tel_get_stats(&st);
    if ((st.link_state != TEL_LINK_CONNECTED) || (tk_ref_flg(EVF_SAFETY, &rflg) != E_OK))
    {
        return false;
    }
    return ((uint32_t)rflg.flgptn & EVF_SAFETY_FAULT_WIFI_LOSS) == 0U;
}

/* S1-U10: the Telemetry task keeps its 20 ms grid under the test load. */
static bool test_u10_period(void)
{
    tel_stats_t before;
    tel_stats_t after;
    uint32_t    wakes;

    tel_get_stats(&before);
    (void)tk_dly_tsk((TMO)1000);
    tel_get_stats(&after);
    wakes = after.wakes - before.wakes;
    dbg_printf("[TEST] S1-U10 %lu wakes in 1 s, %lu overruns", (unsigned long)wakes,
               (unsigned long)(after.wake_overruns - before.wake_overruns));
    return (wakes >= ((1000U / TEL_WAKE_PERIOD_MS) - 2U)) && (wakes <= ((1000U / TEL_WAKE_PERIOD_MS) + 2U))
           && (after.wake_overruns == before.wake_overruns);
}

void subsystem1_test(void)
{
    TEST_INIT();

    TEST_CHECK("S1-U01", test_u01_connected(), "Telemetry task running, link CONNECTED");
    TEST_CHECK("S1-U02", test_u02_event_path(), "event -> frame -> sent within 100 ms");
    TEST_CHECK("S1-U03", test_u03_invalid_event(), "invalid tel_evt_t counted, never framed");
    TEST_CHECK("S1-U04", test_u04_never_blocks(), "64 posts never block; full queue = counted drop");
    TEST_CHECK("S1-U05", test_u05_cmd_accept(), "car/cmd/stop accepted -> MBF_NAV_IN");
    TEST_CHECK("S1-U06", test_u06_cmd_reject(), "5 malformed commands rejected, none posted");
    TEST_CHECK("S1-U07", test_u07_snapshot_clamp(), "sensor snapshot clamps to wire fields");
    TEST_CHECK("S1-U08", test_u08_obstacle(), "obstacle profile framed once");
    TEST_CHECK("S1-U09", test_u09_wifi_loss_bit(), "FAULT_WIFI_LOSS clear while connected");
    TEST_CHECK("S1-U10", test_u10_period(), "20 ms wake grid held, no overruns");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
