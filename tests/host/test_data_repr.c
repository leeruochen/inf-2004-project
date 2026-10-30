/*
 *  Host test for src/common/data_repr.h (Batch B stage B7, the packed
 *  formats).  Covers .agents/tasks/week8-impact-report.md section E.3:
 *  round trips at the limits and one past them (clamped, OOR set), sign
 *  extension, every motion_cmd_t type (HOST-MC-01..08), every nav_input_t
 *  source/type, the obstacle sentinel and saturation, the edge ring, and
 *  the MQTT frames (byte order and every reject rule).
 */

#include <stdint.h>
#include <string.h>

#include "src/common/data_repr.h"
#include "test_check.h"

/* ---------------------------------------------------------------- nav_input_t */

static void test_nav_steer(void)
{
    static int32_t const k_in[] = { 0, 1, -1, 127, -128, 128, -129, 100000, -100000 };
    static int32_t const k_out[] = { 0, 1, -1, 127, -128, 127, -128, 127, -128 };
    static bool const    k_oor[] = { false, false, false, false, false, true, true, true, true };
    uint32_t             i;

    for (i = 0U; i < (sizeof(k_in) / sizeof(k_in[0])); i++)
    {
        nav_input_t const m = drep_nav_steer(k_in[i]);

        CHECK(drep_nav_source(m) == DREP_NAV_SRC_LINE);
        CHECK(drep_nav_type(m) == DREP_NAV_LINE_STEER);
        CHECK(drep_nav_steer_q7(m) == k_out[i]);            /* sign-extended */
        CHECK(drep_nav_oor(m) == k_oor[i]);
        CHECK(drep_nav_bypass(m) == 0U);
        CHECK(drep_nav_valid(m));
    }
}

static void test_nav_sources(void)
{
    uint32_t c;
    uint32_t b;

    CHECK(drep_nav_type(drep_nav_line_event(DREP_NAV_LINE_JUNCTION)) == DREP_NAV_LINE_JUNCTION);
    CHECK(drep_nav_valid(drep_nav_line_event(DREP_NAV_LINE_TRACK_LOST)));
    CHECK(!drep_nav_valid(drep_nav_line_event(DREP_NAV_LINE_TYPES)));        /* unknown type */

    for (c = 0U; c < 4U; c++)
    {
        nav_input_t const m = drep_nav_barcode(c);

        CHECK(drep_nav_source(m) == DREP_NAV_SRC_BARCODE);
        CHECK(drep_nav_barcode_cmd(m) == c);
        CHECK(drep_nav_valid(m));
    }

    for (b = 0U; b < DREP_BYPASS_COUNT; b++)
    {
        nav_input_t const m = drep_nav_avoid(DREP_NAV_AVOID_BYPASS, b);

        CHECK(drep_nav_source(m) == DREP_NAV_SRC_AVOID);
        CHECK(drep_nav_bypass(m) == b);
        CHECK(drep_nav_valid(m));
    }
    CHECK(!drep_nav_valid(drep_nav_avoid(DREP_NAV_AVOID_BYPASS, 5U)));       /* reject 5-7 */
    CHECK(!drep_nav_valid(drep_nav_avoid(DREP_NAV_AVOID_BYPASS, 7U)));
    CHECK(drep_nav_bypass(drep_nav_avoid(DREP_NAV_AVOID_RELEASE, 3U)) == 0U); /* only with BYPASS */
    CHECK(drep_nav_valid(drep_nav_avoid(DREP_NAV_AVOID_BLOCKED, 0U)));
    CHECK(!drep_nav_valid(drep_nav_avoid(DREP_NAV_AVOID_TYPES, 0U)));

    CHECK(drep_nav_type(drep_nav_remote(DREP_NAV_REMOTE_RESET)) == DREP_NAV_REMOTE_RESET);
    CHECK(drep_nav_valid(drep_nav_remote(DREP_NAV_REMOTE_START)));
    CHECK(!drep_nav_valid(drep_nav_remote(DREP_NAV_REMOTE_TYPES)));

    {
        nav_input_t m = drep_nav_remote(DREP_NAV_REMOTE_STOP);

        m.w |= 1UL << DREP_NAV_TOTAL_BITS;                                   /* stray bit */
        CHECK(!drep_nav_valid(m));
    }
}

/* --------------------------------------------------------------- motion_cmd_t */

typedef struct
{
    uint32_t type;
    int32_t  min;
    int32_t  max;
} mc_case_t;

/* HOST-MC-01..08: each type at min, max, min - 1 and max + 1. */
static void test_motion_cmd_types(void)
{
    static mc_case_t const k_cases[8] =
    {
        { DREP_MC_STOP,   0,    0   },     /* HOST-MC-01 */
        { DREP_MC_FWD,    0,    511 },     /* HOST-MC-02 */
        { DREP_MC_BACK,   0,    511 },     /* HOST-MC-03 */
        { DREP_MC_TURN_L, 0,    180 },     /* HOST-MC-04 */
        { DREP_MC_TURN_R, 0,    180 },     /* HOST-MC-05 */
        { DREP_MC_UTURN,  0,    0   },     /* HOST-MC-06 */
        { DREP_MC_STEER,  -256, 255 },     /* HOST-MC-07 */
        { DREP_MC_BRAKE,  0,    0   },     /* HOST-MC-08 */
    };
    uint32_t i;

    for (i = 0U; i < 8U; i++)
    {
        mc_case_t const *p_c = &k_cases[i];
        motion_cmd_t     m;

        m = drep_mc_pack(p_c->type, 20, p_c->min);
        CHECK((drep_mc_type(m) == p_c->type) && (drep_mc_arg(m) == p_c->min) && !drep_mc_oor(m));
        CHECK(drep_mc_valid(m));

        m = drep_mc_pack(p_c->type, 20, p_c->max);
        CHECK((drep_mc_arg(m) == p_c->max) && !drep_mc_oor(m) && drep_mc_valid(m));

        m = drep_mc_pack(p_c->type, 20, p_c->min - 1);
        CHECK((drep_mc_arg(m) == p_c->min) && drep_mc_oor(m) && drep_mc_valid(m));

        m = drep_mc_pack(p_c->type, 20, p_c->max + 1);
        CHECK((drep_mc_arg(m) == p_c->max) && drep_mc_oor(m) && drep_mc_valid(m));

        CHECK(drep_mc_speed(m) == 20U);
    }
}

static void test_motion_cmd_speed(void)
{
    CHECK((drep_mc_speed(drep_mc_pack(DREP_MC_FWD, 0, 10)) == 0U) && !drep_mc_oor(drep_mc_pack(DREP_MC_FWD, 0, 10)));
    CHECK((drep_mc_speed(drep_mc_pack(DREP_MC_FWD, 50, 10)) == 50U) && !drep_mc_oor(drep_mc_pack(DREP_MC_FWD, 50, 10)));
    CHECK((drep_mc_speed(drep_mc_pack(DREP_MC_FWD, 51, 10)) == 50U) && drep_mc_oor(drep_mc_pack(DREP_MC_FWD, 51, 10)));
    CHECK((drep_mc_speed(drep_mc_pack(DREP_MC_FWD, -1, 10)) == 0U) && drep_mc_oor(drep_mc_pack(DREP_MC_FWD, -1, 10)));

    {
        motion_cmd_t bad = drep_mc_pack(DREP_MC_TURN_L, 10, 0);

        bad.w = drep_set(bad.w, DREP_MC_ARG_SHIFT, DREP_MC_ARG_BITS, 181U);      /* angle 181: reject */
        CHECK(!drep_mc_valid(bad));
        bad = drep_mc_pack(DREP_MC_STOP, 10, 0);
        bad.w = drep_set(bad.w, DREP_MC_ARG_SHIFT, DREP_MC_ARG_BITS, 1U);        /* STOP with arg: reject */
        CHECK(!drep_mc_valid(bad));
        bad = drep_mc_pack(DREP_MC_FWD, 10, 0);
        bad.w = drep_set(bad.w, DREP_MC_SPEED_SHIFT, DREP_MC_SPEED_BITS, 51U);   /* speed 51: reject */
        CHECK(!drep_mc_valid(bad));
    }
}

/* ----------------------------------------------------------- obstacle_profile_t */

static void test_obstacle(void)
{
    obstacle_profile_t p;

    p = drep_obs_pack(250, 6, 120, 300, 635, 2U);
    CHECK(drep_obs_distance_cm(p) == 250U);
    CHECK(drep_obs_bearing_idx(p) == 6U);
    CHECK(drep_obs_bearing_deg(p) == 90U);
    CHECK(drep_obs_width_mm(p) == 120U);
    CHECK(drep_obs_clear_l_mm(p) == 300U);
    CHECK(drep_obs_clear_r_mm(p) == 635U);
    CHECK(drep_obs_seq(p) == 2U);
    CHECK(!drep_obs_oor(p));
    CHECK(!drep_obs_no_echo(p));

    p = drep_obs_pack(0, 0, 0, 0, 0, 0U);                       /* 0 = no echo */
    CHECK(drep_obs_no_echo(p) && !drep_obs_oor(p));
    CHECK(drep_obs_bearing_deg(p) == 30U);

    p = drep_obs_pack(400, 12, 400, 0, 0, 0U);                  /* every max, no OOR */
    CHECK((drep_obs_distance_cm(p) == 400U) && (drep_obs_bearing_idx(p) == 12U));
    CHECK((drep_obs_width_mm(p) == 400U) && !drep_obs_oor(p));

    CHECK(drep_obs_oor(drep_obs_pack(401, 0, 0, 0, 0, 0U)));    /* max + 1 */
    CHECK(drep_obs_distance_cm(drep_obs_pack(401, 0, 0, 0, 0, 0U)) == 400U);
    CHECK(drep_obs_oor(drep_obs_pack(-1, 0, 0, 0, 0, 0U)));     /* min - 1 */
    CHECK(drep_obs_bearing_idx(drep_obs_pack(10, 13, 0, 0, 0, 0U)) == 12U);
    CHECK(drep_obs_oor(drep_obs_pack(10, 13, 0, 0, 0, 0U)));
    CHECK(drep_obs_width_mm(drep_obs_pack(10, 0, 401, 0, 0, 0U)) == 400U);
    CHECK(drep_obs_oor(drep_obs_pack(10, 0, 401, 0, 0, 0U)));

    /* Clearances saturate at 635 mm WITHOUT OOR. */
    p = drep_obs_pack(10, 0, 0, 10000, 636, 0U);
    CHECK((drep_obs_clear_l_mm(p) == 635U) && (drep_obs_clear_r_mm(p) == 635U) && !drep_obs_oor(p));

    /* Conservative rounding: width up, clearances down. */
    p = drep_obs_pack(10, 0, 121, 634, 4, 0U);
    CHECK(drep_obs_width_mm(p) == 125U);
    CHECK(drep_obs_clear_l_mm(p) == 630U);
    CHECK(drep_obs_clear_r_mm(p) == 0U);

    CHECK(drep_obs_seq(drep_obs_pack(10, 0, 0, 0, 0, 5U)) == 1U);   /* wraps */
    CHECK((drep_obs_pack(400, 12, 400, 635, 635, 3U).lo >> DREP_OBS_LO_BITS) == 0U);
    CHECK((drep_obs_pack(400, 12, 400, 635, 635, 3U).hi >> DREP_OBS_CLR_BITS) == 0U);
}

/* -------------------------------------------------------- motion_state_t word */

static void test_motion_state(void)
{
    static int32_t const k_in[] = { 0, 1, -1, 511, -511, 512, -512, 2000 };
    static int32_t const k_out[] = { 0, 1, -1, 511, -511, 511, -511, 511 };
    static bool const    k_oor[] = { false, false, false, false, false, true, true, true };
    uint32_t             i;

    for (i = 0U; i < (sizeof(k_in) / sizeof(k_in[0])); i++)
    {
        uint32_t const w = drep_ms_pack(k_in[i], 1234U);

        CHECK(drep_ms_speed_x10(w) == k_out[i]);                /* sign-extended */
        CHECK(drep_ms_oor(w) == k_oor[i]);
        CHECK(drep_ms_distance_cm(w) == 1234U);
    }
    CHECK(drep_ms_distance_cm(drep_ms_pack(0, 65535U)) == 65535U);
    CHECK(drep_ms_distance_cm(drep_ms_pack(0, 65536U)) == 0U);  /* wraps */
    CHECK(!drep_ms_oor(drep_ms_pack(0, 65536U)));
}

/* ------------------------------------------------------------- tel_evt_t, pitch */

static void test_telemetry_event(void)
{
    tel_evt_t e;
    bool      oor = false;

    CHECK((drep_pitch_q1(90, &oor) == 90) && !oor);
    CHECK((drep_pitch_q1(-90, &oor) == -90) && !oor);
    CHECK((drep_pitch_q1(91, &oor) == 90) && oor);
    oor = false;
    CHECK((drep_pitch_q1(-91, &oor) == -90) && oor);

    e = drep_tel_hump(0x123456U, 37, -21);
    CHECK(drep_tel_type(e) == DREP_TEL_HUMP);
    CHECK(drep_tel_timestamp_ms(e) == 0x123456U);
    CHECK(drep_hump_height_mm(e.payload) == 37U);
    CHECK(drep_hump_pitch_q1(e.payload) == -21);                 /* sign-extended */
    CHECK(!drep_tel_oor(e) && drep_tel_valid(e));

    e = drep_tel_hump(0x1000000U, 101, 0);                       /* timestamp wraps; height clamps */
    CHECK(drep_tel_timestamp_ms(e) == 0U);
    CHECK((drep_hump_height_mm(e.payload) == 100U) && drep_tel_oor(e));
    CHECK(((e.payload >> DREP_HUMP_OOR_SHIFT) & 1U) == 1U);

    CHECK(!drep_tel_valid(drep_tel_make(0U, 0U, 0U, false)));
    CHECK(!drep_tel_valid(drep_tel_make(DREP_TEL_TYPE_COUNT, 0U, 0U, false)));
    CHECK(drep_tel_valid(drep_tel_make(DREP_TEL_FAULT, 5U, EVF_SAFETY_FAULT_STALL, false)));
}

/* ------------------------------------------------------------------- edge ring */

static void test_edge_ring(void)
{
    static drep_edge_ring_t ring;
    uint32_t                i;
    uint32_t                gap = 0U;
    bool                    ok = true;

    drep_edge_init(&ring);
    CHECK(sizeof(ring.lo) + sizeof(ring.hi) == 160U);
    CHECK(!drep_edge_pop(&ring, &gap));                          /* empty */

    /* Nibble neighbours: writing slot 2k never changes slot 2k+1 and back. */
    drep_edge_store(&ring, 4U, 0xABCDEU);
    drep_edge_store(&ring, 5U, 0x12345U);
    CHECK(drep_edge_load(&ring, 4U) == 0xABCDEU);
    CHECK(drep_edge_load(&ring, 5U) == 0x12345U);
    drep_edge_store(&ring, 4U, 0x00001U);
    CHECK(drep_edge_load(&ring, 5U) == 0x12345U);
    drep_edge_store(&ring, 5U, 0xF0000U);
    CHECK(drep_edge_load(&ring, 4U) == 0x00001U);

    /* Saturation at 0xFFFFF. */
    drep_edge_store(&ring, 7U, 0xFFFFFU);
    CHECK(drep_edge_load(&ring, 7U) == 0xFFFFFU);
    drep_edge_store(&ring, 7U, 0x100000U);
    CHECK(drep_edge_load(&ring, 7U) == 0xFFFFFU);
    drep_edge_store(&ring, 7U, 1000000U);                        /* BC_CAPTURE_TIMEOUT_MS in us */
    CHECK(drep_edge_load(&ring, 7U) == 1000000U);

    /* Full and overflow: 64 fit, the 65th is counted and dropped. */
    drep_edge_init(&ring);
    for (i = 0U; i < 64U; i++)
    {
        ok = ok && drep_edge_push(&ring, (i * 16411U) & 0xFFFFFU);
    }
    CHECK(ok);
    CHECK(drep_edge_count(&ring) == 64U);
    CHECK(!drep_edge_push(&ring, 1U));
    CHECK(ring.overflow == 1U);
    ok = true;
    for (i = 0U; i < 64U; i++)
    {
        ok = ok && drep_edge_pop(&ring, &gap) && (gap == ((i * 16411U) & 0xFFFFFU));
    }
    CHECK(ok);
    CHECK(drep_edge_count(&ring) == 0U);

    /* Wrap: the free-running uint8 indices pass 255 many times. */
    ok = true;
    for (i = 0U; i < 1000U; i++)
    {
        uint32_t const v = (i * 7919U) & 0xFFFFFU;

        ok = ok && drep_edge_push(&ring, v) && drep_edge_push(&ring, v ^ 0x80000U);
        ok = ok && drep_edge_pop(&ring, &gap) && (gap == v);
        ok = ok && drep_edge_pop(&ring, &gap) && (gap == (v ^ 0x80000U));
    }
    CHECK(ok);
    CHECK(ring.overflow == 1U);
}

/* -------------------------------------------------------------- MQTT frames */

static void test_frames_encode(void)
{
    uint8_t  f[DREP_FRAME_MAX_BYTES];
    uint32_t n;
    uint32_t pos;

    /* Byte order: header, then LSB-first little-endian payload. */
    memset(f, 0xEE, sizeof f);
    n = drep_frame_barcode(f, 0x030201U, DREP_BC_CMD_UTURN);
    CHECK(n == 7U);
    CHECK((f[0] == DREP_WIRE_VER) && (f[1] == DREP_FT_EVENT_BARCODE) && (f[2] == 4U));
    CHECK((f[3] == 0x01U) && (f[4] == 0x02U) && (f[5] == 0x03U));   /* timestamp, little-endian */
    CHECK(f[6] == 0x03U);                                           /* command 3 in bits 24-25, rest 0 */
    CHECK(drep_frame_check(f, n) == DREP_FRAME_OK);

    n = drep_frame_motion(f, 0xABCDEFU, drep_ms_pack(-5, 0xBEEFU), -2, 0x12345678);
    CHECK(n == 18U);
    pos = 24U;
    CHECK(drep_bits_get(f, &pos, 24U) == 0xABCDEFU);
    CHECK(drep_sext(drep_bits_get(f, &pos, 10U), 10U) == -5);
    CHECK(drep_bits_get(f, &pos, 16U) == 0xBEEFU);
    CHECK((int32_t)drep_bits_get(f, &pos, 32U) == -2);
    CHECK(drep_bits_get(f, &pos, 32U) == 0x12345678U);
    CHECK(pos == (114U + 24U));
    CHECK(drep_frame_check(f, n) == DREP_FRAME_OK);

    {
        obstacle_profile_t const p = drep_obs_pack(321, 11, 95, 600, 45, 3U);

        n = drep_frame_obstacle(f, 7U, p);
        CHECK(n == 11U);
        pos = 24U + 24U;
        CHECK(drep_bits_get(f, &pos, DREP_OBS_LO_BITS) == p.lo);
        CHECK(drep_bits_get(f, &pos, DREP_OBS_CLR_BITS) == p.hi);
    }

    CHECK(drep_frame_sensor(f, 1U, 4095U, 0U, 400U, -90, 90) == 13U);
    CHECK(drep_frame_hump(f, 1U, drep_tel_hump(1U, 50, 10).payload) == 8U);
    CHECK(drep_frame_fault(f, 1U, EVF_SAFETY_ALL_MASK) == 8U);
    CHECK(drep_frame_heartbeat(f, 0xFFFFFFFFU, 9U, EVF_SAFETY_FAULT_LATCHED) == 9U);
    pos = 24U;
    CHECK(drep_bits_get(f, &pos, 32U) == 0xFFFFFFFFU);
    CHECK(drep_bits_get(f, &pos, 4U) == 9U);
    CHECK(drep_bits_get(f, &pos, 10U) == EVF_SAFETY_FAULT_LATCHED);
}

static void test_frames_reject(void)
{
    uint8_t     f[DREP_FRAME_MAX_BYTES + 2U];
    nav_input_t nav = { 0xDEADBEEFU };
    uint32_t    n;

    /* Valid commands become remote nav inputs. */
    n = drep_frame_cmd(f, DREP_FT_CMD_START);
    CHECK((n == 3U) && (drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_OK));
    CHECK((drep_nav_source(nav) == DREP_NAV_SRC_REMOTE) && (drep_nav_type(nav) == DREP_NAV_REMOTE_START));
    n = drep_frame_cmd(f, DREP_FT_CMD_RESET);
    CHECK((drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_OK) && (drep_nav_type(nav) == DREP_NAV_REMOTE_RESET));

    /* Every reject leaves nav untouched: it never reaches the VC. */
    nav.w = 0xDEADBEEFU;
    n = drep_frame_cmd(f, DREP_FT_CMD_STOP);
    f[0] = (uint8_t)(DREP_WIRE_VER + 1U);
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_BAD_VERSION);
    f[0] = DREP_WIRE_VER;
    CHECK(drep_cmd_to_nav(f, n - 1U, &nav) == DREP_FRAME_SHORT);      /* short */
    CHECK(drep_cmd_to_nav(f, n + 1U, &nav) == DREP_FRAME_BAD_LENGTH); /* long */
    f[2] = 1U;
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_BAD_LENGTH);      /* wrong len field */
    f[2] = 0U;
    f[1] = 0U;
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_UNKNOWN_TYPE);
    f[1] = (uint8_t)DREP_FT_COUNT;
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_UNKNOWN_TYPE);
    f[1] = 0xFFU;
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_UNKNOWN_TYPE);
    n = drep_frame_barcode(f, 1U, 0U);                                /* valid, not a command */
    CHECK(drep_cmd_to_nav(f, n, &nav) == DREP_FRAME_NOT_COMMAND);
    CHECK(nav.w == 0xDEADBEEFU);
}

/* ---------------------------------------------------------- the layouts agree */

static void test_layouts(void)
{
    /* Every field at its max sets only its own bits. */
    nav_input_t const  n = drep_nav_make(3U, 7U, 0xFFU, 7U, true);
    motion_cmd_t const m = drep_mc_pack(DREP_MC_STEER, 63, -1);

    CHECK(n.w == DREP_MASK(DREP_NAV_TOTAL_BITS));
    CHECK((m.w >> DREP_MC_TOTAL_BITS) == 0U);
    CHECK(drep_frame_len(DREP_FT_TELEMETRY_MOTION) == DREP_FRAME_MAX_PAYLOAD);
    CHECK(drep_frame_len(DREP_FT_COUNT) == 0U);
}

int main(void)
{
    test_nav_steer();
    test_nav_sources();
    test_motion_cmd_types();
    test_motion_cmd_speed();
    test_obstacle();
    test_motion_state();
    test_telemetry_event();
    test_edge_ring();
    test_frames_encode();
    test_frames_reject();
    test_layouts();
    return TEST_RESULT("data_repr");
}
