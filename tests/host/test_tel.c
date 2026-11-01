/*
 *  Host test for subsystem 1's pure modules (Buddy 1):
 *    tel_link.c    connection state machine and reconnect backoff (FR3)
 *    tel_outbox.c  event buffer, drop-oldest (design 6.5, NFR1)
 *    tel_wire.c    topic tree and the command receive check (FR2)
 *  plus GOLDEN FRAMES: fixed byte strings built by the drep_frame_*()
 *  encoders.  tools/dashboard/test_tel_decode.py decodes the very same
 *  strings, so the firmware and the dashboard cannot drift apart unnoticed
 *  (Week 8 risk R3).  If an encoder changes on purpose, update both files.
 */

#include <stdint.h>
#include <string.h>

#include "app_program/subsystem1/tel_link.h"
#include "app_program/subsystem1/tel_outbox.h"
#include "app_program/subsystem1/tel_wire.h"
#include "test_check.h"

static tel_link_cfg_t const k_cfg = { 15000U, 5000U, 1000U, 30000U };

static uint32_t step(tel_link_t *p_link, uint32_t now_ms, bool net_up, bool pub_failed, bool outbox_empty)
{
    tel_link_in_t in;

    in.net_up = net_up;
    in.attempt_failed = false;
    in.publish_failed = pub_failed;
    in.outbox_empty = outbox_empty;
    return tel_link_step(p_link, now_ms, &in);
}

/* A definite failure (wrong password, broker refused) ends the attempt at
 * once instead of waiting for the connect timeout. */
static void test_link_attempt_failed(void)
{
    tel_link_t    link;
    tel_link_in_t in = { false, true, false, true };

    tel_link_init(&link, &k_cfg, 0U);
    (void)step(&link, 0U, false, false, true);          /* CONNECTING */
    CHECK(tel_link_step(&link, 500U, &in) == TEL_LINK_ACT_TEARDOWN);
    CHECK(tel_link_state(&link) == TEL_LINK_RECONNECTING);
    CHECK(link.last_wait_ms == 1000U);
    CHECK(link.retry_at_ms == 1500U);
    /* attempt_failed is ignored outside CONNECTING */
    CHECK(tel_link_step(&link, 1000U, &in) == TEL_LINK_ACT_NONE);
    CHECK(tel_link_step(&link, 1500U, &in) == TEL_LINK_ACT_CONNECT);
}

/* ------------------------------------------------------------ tel_link */

static void test_link_connect(void)
{
    tel_link_t link;

    tel_link_init(&link, &k_cfg, 0U);
    CHECK(tel_link_state(&link) == TEL_LINK_DISCONNECTED);
    CHECK(!tel_link_can_send(&link));
    CHECK(!tel_link_is_lost(&link));                    /* never connected: not a loss */

    CHECK(step(&link, 20U, false, false, true) == TEL_LINK_ACT_CONNECT);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTING);
    CHECK(step(&link, 40U, false, false, true) == TEL_LINK_ACT_NONE);
    CHECK(!tel_link_is_lost(&link));

    CHECK(step(&link, 3000U, true, false, true) == TEL_LINK_ACT_NONE);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTED);
    CHECK(tel_link_can_send(&link));
    CHECK(link.connects == 1U);
}

static void test_link_backoff(void)
{
    static uint32_t const k_waits[] = { 1000U, 2000U, 4000U, 8000U, 16000U, 30000U, 30000U };
    tel_link_t            link;
    uint32_t              now = 0U;
    uint32_t              i;

    tel_link_init(&link, &k_cfg, now);
    (void)step(&link, now, false, false, true);         /* -> CONNECTING */
    for (i = 0U; i < (sizeof(k_waits) / sizeof(k_waits[0])); i++)
    {
        /* one ms before the connect timeout: still connecting */
        CHECK(step(&link, now + k_cfg.connect_timeout_ms - 1U, false, false, true) == TEL_LINK_ACT_NONE);
        now += k_cfg.connect_timeout_ms;
        CHECK(step(&link, now, false, false, true) == TEL_LINK_ACT_TEARDOWN);
        CHECK(tel_link_state(&link) == TEL_LINK_RECONNECTING);
        CHECK(link.last_wait_ms == k_waits[i]);
        CHECK(step(&link, now + k_waits[i] - 1U, false, false, true) == TEL_LINK_ACT_NONE);
        now += k_waits[i];
        CHECK(step(&link, now, false, false, true) == TEL_LINK_ACT_CONNECT);
        CHECK(tel_link_state(&link) == TEL_LINK_CONNECTING);
    }
    CHECK(link.failures == (sizeof(k_waits) / sizeof(k_waits[0])));

    /* success resets the backoff to the minimum */
    (void)step(&link, now + 10U, true, false, true);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTED);
    CHECK(link.backoff_ms == k_cfg.backoff_min_ms);
}

static void test_link_degraded(void)
{
    tel_link_t link;
    uint32_t   t;

    tel_link_init(&link, &k_cfg, 0U);
    (void)step(&link, 0U, false, false, true);
    (void)step(&link, 100U, true, false, true);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTED);

    /* full output buffer, network still up: DEGRADED, but not "lost" */
    (void)step(&link, 200U, true, true, false);
    CHECK(tel_link_state(&link) == TEL_LINK_DEGRADED);
    CHECK(!tel_link_is_lost(&link));
    CHECK(tel_link_can_send(&link));                    /* keep flushing */
    /* flushing (net up, publish ok, outbox not empty): stays, T_ka restarts */
    (void)step(&link, 4000U, true, false, false);
    CHECK(tel_link_state(&link) == TEL_LINK_DEGRADED);
    (void)step(&link, 8000U, true, false, false);
    CHECK(tel_link_state(&link) == TEL_LINK_DEGRADED);  /* 8 s after degrade, but progressing */
    (void)step(&link, 8020U, true, false, true);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTED);

    /* network down: DEGRADED and lost; can_send false */
    t = 10000U;
    (void)step(&link, t, false, false, false);
    CHECK(tel_link_state(&link) == TEL_LINK_DEGRADED);
    CHECK(tel_link_is_lost(&link));
    CHECK(!tel_link_can_send(&link));
    /* back within T_ka: recovers without a reconnect */
    (void)step(&link, t + 2000U, true, false, true);
    CHECK(tel_link_state(&link) == TEL_LINK_CONNECTED);
    CHECK(!tel_link_is_lost(&link));
    CHECK(link.failures == 0U);

    /* down for T_ka: teardown, then backoff 1 s */
    t = 20000U;
    (void)step(&link, t, false, false, true);
    CHECK(step(&link, t + k_cfg.degrade_timeout_ms - 1U, false, false, true) == TEL_LINK_ACT_NONE);
    CHECK(step(&link, t + k_cfg.degrade_timeout_ms, false, false, true) == TEL_LINK_ACT_TEARDOWN);
    CHECK(tel_link_state(&link) == TEL_LINK_RECONNECTING);
    CHECK(link.last_wait_ms == 1000U);
    CHECK(tel_link_is_lost(&link));
}

static void test_link_wrap(void)
{
    tel_link_t     link;
    uint32_t const t0 = 0xFFFFF000U;                    /* 4 s before the 49.7-day wrap */

    tel_link_init(&link, &k_cfg, t0);
    (void)step(&link, t0, false, false, true);          /* CONNECTING at t0 */
    CHECK(step(&link, t0 + 10000U, false, false, true) == TEL_LINK_ACT_NONE);   /* wrapped, 10 s in */
    CHECK(step(&link, t0 + 15000U, false, false, true) == TEL_LINK_ACT_TEARDOWN);
    CHECK(step(&link, t0 + 15999U, false, false, true) == TEL_LINK_ACT_NONE);
    CHECK(step(&link, t0 + 16000U, false, false, true) == TEL_LINK_ACT_CONNECT);
}

static void test_link_robust(void)
{
    tel_link_t     link;
    tel_link_cfg_t bad = { 1000U, 1000U, 0U, 0U };      /* nonsense backoff */

    tel_link_init(&link, &bad, 0U);
    CHECK(link.cfg.backoff_min_ms >= 1U);
    CHECK(link.cfg.backoff_max_ms >= link.cfg.backoff_min_ms);

    link.state = 77U;                                    /* corrupted */
    (void)step(&link, 10U, false, false, true);
    CHECK(tel_link_state(&link) == TEL_LINK_DISCONNECTED);
    CHECK(step(&link, 20U, false, false, true) == TEL_LINK_ACT_CONNECT);

    CHECK(tel_link_step(NULL, 0U, NULL) == TEL_LINK_ACT_NONE);
    CHECK_STR(tel_link_state_name(TEL_LINK_DEGRADED), "DEGRADED");
    CHECK_STR(tel_link_state_name(99U), "?");
}

/* ------------------------------------------------------------ tel_outbox */

static void test_outbox(void)
{
    tel_outbox_t       box;
    uint8_t            f[DREP_FRAME_MAX_BYTES];
    tel_frame_t const *p_f;
    uint32_t           i;

    tel_outbox_init(&box);
    CHECK(tel_outbox_peek(&box) == NULL);
    CHECK(!tel_outbox_push(&box, f, 2U));               /* shorter than a header */
    CHECK(!tel_outbox_push(&box, f, DREP_FRAME_MAX_BYTES + 1U));
    CHECK(!tel_outbox_push(&box, NULL, 5U));

    /* fill past capacity: the oldest are dropped and counted */
    for (i = 0U; i < (TEL_OUTBOX_LEN + 3U); i++)
    {
        (void)drep_frame_barcode(f, i, DREP_BC_CMD_LEFT);
        CHECK(tel_outbox_push(&box, f, 7U));
    }
    CHECK(tel_outbox_count(&box) == TEL_OUTBOX_LEN);
    CHECK(box.dropped == 3U);
    CHECK(box.peak == TEL_OUTBOX_LEN);

    /* FIFO order, starting with the 4th frame pushed (ts = 3) */
    for (i = 3U; i < (TEL_OUTBOX_LEN + 3U); i++)
    {
        uint32_t pos = 3U * 8U;

        p_f = tel_outbox_peek(&box);
        CHECK(p_f != NULL);
        if (p_f == NULL)
        {
            break;
        }
        CHECK(p_f->len == 7U);
        CHECK(drep_bits_get(p_f->buf, &pos, 24U) == i);
        tel_outbox_pop(&box);
    }
    CHECK(tel_outbox_count(&box) == 0U);
    tel_outbox_pop(&box);                               /* pop on empty is harmless */
    CHECK(tel_outbox_count(&box) == 0U);
}

/* ------------------------------------------------------------ tel_wire */

static void test_wire_topics(void)
{
    uint32_t t;

    CHECK(tel_wire_topic(0U) == NULL);
    CHECK_STR(tel_wire_topic(DREP_FT_TELEMETRY_MOTION), "car/telemetry/motion");
    CHECK_STR(tel_wire_topic(DREP_FT_TELEMETRY_SENSOR), "car/telemetry/sensor");
    CHECK_STR(tel_wire_topic(DREP_FT_EVENT_BARCODE), "car/telemetry/barcode");
    CHECK_STR(tel_wire_topic(DREP_FT_EVENT_HUMP), "car/telemetry/hump");
    CHECK_STR(tel_wire_topic(DREP_FT_EVENT_OBSTACLE), "car/telemetry/obstacle");
    CHECK_STR(tel_wire_topic(DREP_FT_EVENT_FAULT), "car/telemetry/fault");
    CHECK_STR(tel_wire_topic(DREP_FT_HEARTBEAT), "car/telemetry/heartbeat");
    for (t = DREP_FT_CMD_START; t < (DREP_FT_COUNT + 3U); t++)
    {
        CHECK(tel_wire_topic(t) == NULL);               /* commands are never published */
    }
    for (t = 1U; t < DREP_FT_CMD_START; t++)
    {
        CHECK(strlen(tel_wire_topic(t)) <= TEL_TOPIC_MAX);
    }
    CHECK(tel_wire_is_periodic(DREP_FT_TELEMETRY_MOTION));
    CHECK(tel_wire_is_periodic(DREP_FT_HEARTBEAT));
    CHECK(!tel_wire_is_periodic(DREP_FT_EVENT_HUMP));
}

static void test_wire_cmd(void)
{
    static uint32_t const    k_type[] = { DREP_FT_CMD_START, DREP_FT_CMD_STOP, DREP_FT_CMD_RESET };
    static char const *const k_topic[] = { "car/cmd/start", "car/cmd/stop", "car/cmd/reset" };
    static uint32_t const    k_nav[] = { DREP_NAV_REMOTE_START, DREP_NAV_REMOTE_STOP, DREP_NAV_REMOTE_RESET };
    uint8_t                  f[DREP_FRAME_MAX_BYTES];
    nav_input_t              nav;
    nav_input_t const        untouched = { 0xA5A5A5A5U };
    uint32_t                 i;
    uint32_t                 n;

    for (i = 0U; i < 3U; i++)
    {
        n = drep_frame_cmd(f, k_type[i]);
        nav.w = 0U;
        CHECK(tel_wire_cmd(k_topic[i], f, n, &nav) == DREP_FRAME_OK);
        CHECK(drep_nav_source(nav) == DREP_NAV_SRC_REMOTE);
        CHECK(drep_nav_type(nav) == k_nav[i]);
        CHECK(drep_nav_valid(nav));
    }

    /* every reject leaves *p_nav alone */
    n = drep_frame_cmd(f, DREP_FT_CMD_STOP);
    nav = untouched;
    CHECK(tel_wire_cmd("car/cmd/start", f, n, &nav) == TEL_WIRE_TOPIC_MISMATCH);
    CHECK(tel_wire_cmd("car/cmd/stopp", f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd("car/cmd/sto", f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd("car/cmd/", f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd("car/cmdx/stop", f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd("", f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd(NULL, f, n, &nav) == TEL_WIRE_BAD_TOPIC);
    CHECK(tel_wire_cmd("car/cmd/stop", f, 2U, &nav) == DREP_FRAME_SHORT);
    CHECK(tel_wire_cmd("car/cmd/stop", f, n + 1U, &nav) == DREP_FRAME_BAD_LENGTH);
    f[0] = (uint8_t)(DREP_WIRE_VER + 1U);
    CHECK(tel_wire_cmd("car/cmd/stop", f, n, &nav) == DREP_FRAME_BAD_VERSION);
    f[0] = (uint8_t)DREP_WIRE_VER;
    f[1] = 0U;
    CHECK(tel_wire_cmd("car/cmd/stop", f, n, &nav) == DREP_FRAME_UNKNOWN_TYPE);
    f[1] = (uint8_t)DREP_FT_COUNT;
    CHECK(tel_wire_cmd("car/cmd/stop", f, n, &nav) == DREP_FRAME_UNKNOWN_TYPE);
    f[1] = (uint8_t)DREP_FT_CMD_STOP;
    f[2] = 1U;                                          /* len field lies */
    CHECK(tel_wire_cmd("car/cmd/stop", f, n, &nav) == DREP_FRAME_BAD_LENGTH);
    /* a valid TELEMETRY frame sent as a command */
    n = drep_frame_fault(f, 1U, 0U);
    CHECK(tel_wire_cmd("car/cmd/stop", f, n, &nav) == DREP_FRAME_NOT_COMMAND);
    CHECK(nav.w == untouched.w);

    CHECK_STR(tel_wire_reject_name(TEL_WIRE_TOPIC_MISMATCH), "TOPIC_MISMATCH");
    CHECK_STR(tel_wire_reject_name(DREP_FRAME_BAD_VERSION), "BAD_VERSION");
}

static void test_wire_hex(void)
{
    static uint8_t const k_bytes[] = { 0x01U, 0xABU, 0x00U, 0xFFU };
    char                 out[16];

    CHECK(tel_wire_hex(out, sizeof(out), k_bytes, 4U) == 8U);
    CHECK_STR(out, "01ab00ff");
    CHECK(tel_wire_hex(out, 6U, k_bytes, 4U) == 4U);    /* cut to fit, NUL kept */
    CHECK_STR(out, "01ab");
    CHECK(tel_wire_hex(out, 1U, k_bytes, 4U) == 0U);
    CHECK_STR(out, "");
}

/* ------------------------------------------------------------ golden frames */

static void check_hex(uint8_t const *p_buf, uint32_t n, char const *p_expected)
{
    char out[(DREP_FRAME_MAX_BYTES * 2U) + 1U];

    (void)tel_wire_hex(out, sizeof(out), p_buf, n);
    CHECK_STR(out, p_expected);
}

static void test_golden_frames(void)
{
    uint8_t            b[DREP_FRAME_MAX_BYTES];
    tel_evt_t const    hump = drep_tel_hump(5000U, 42, -31);
    obstacle_profile_t prof = drep_obs_pack(25, 6, 123, 340, 635, 2U);

    /* Same inputs and bytes as tools/dashboard/test_tel_decode.py GOLDEN. */
    check_hex(b, drep_frame_motion(b, 0x123456U, drep_ms_pack(-123, 4567U), -1000, 123456),
              "01010f563412855f4760f0ffff0389070000");
    check_hex(b, drep_frame_sensor(b, 1000U, 4095U, 2048U, 400U, -90, 45), "01020ae80300ff0f80904d5b00");
    check_hex(b, drep_frame_barcode(b, 777U, DREP_BC_CMD_UTURN), "01030409030003");
    check_hex(b, drep_frame_hump(b, 5000U, hump.payload & 0xFFFFU), "010405881300aa70");
    check_hex(b, drep_frame_obstacle(b, 123456U, prof), "01050840e201192c43d41f");
    check_hex(b, drep_frame_fault(b, 99U, 0x105U), "0106056300000501");
    check_hex(b, drep_frame_heartbeat(b, 0xDEADBEEFU, 9U, 0x3FFU), "010706efbeaddef93f");
    check_hex(b, drep_frame_cmd(b, DREP_FT_CMD_START), "010800");
    check_hex(b, drep_frame_cmd(b, DREP_FT_CMD_STOP), "010900");
    check_hex(b, drep_frame_cmd(b, DREP_FT_CMD_RESET), "010a00");
}

int main(void)
{
    test_link_connect();
    test_link_backoff();
    test_link_attempt_failed();
    test_link_degraded();
    test_link_wrap();
    test_link_robust();
    test_outbox();
    test_wire_topics();
    test_wire_cmd();
    test_wire_hex();
    test_golden_frames();
    return TEST_RESULT("tel");
}
