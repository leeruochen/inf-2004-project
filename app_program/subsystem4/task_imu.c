/*
 *  task_imu.c
 *  Subsystem 4 -- IMU Processing task (priority 8, 10 ms periodic).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Responsibilities (design.md 3.1/3.2, HANDOVER.md section 3):
 *    - Drift-free 10 ms period via app_period_wait() (IMU_SAMPLE_PERIOD_MS).
 *    - Sole user of I2C1 after init: reads the accelerometer via drv_imu.
 *    - Pitch from the low-passed accelerometer only (no gyro, D4), measured
 *      from a level reference taken after boot once the car has been still
 *      and flat for 0.5 s (falls back to 0 after 10 s).
 *    - Sets EVF_SAFETY_IMU_ALIVE after every good sample (watchdog input).
 *    - Sets EVF_SAFETY_LIFTED once per pick-up when the car is lifted or
 *      overturned (imu_lift.c; replaces collision detection, 11 Nov 2026).
 *    - Sends one DREP_TEL_HUMP event per hump (pitch above IMU_HUMP_PITCH_DEG)
 *      with its estimated height (imu_hump.c, FR11), when the hump is over.
 *    - Sets EVF_SAFETY_FAULT_IMU_I2C after repeated I2C failures.
 */

#include "task_imu.h"

#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_imu.h"
#include "imu_hump.h"
#include "imu_lift.h"
#include "../subsystem1/subsystem1.h"   /* tel_set_attitude (IF-12, dashboard integration) */
#include "../subsystem2/subsystem2.h"   /* motion_get_state: hump distance (IF-08) */

/* Consecutive I2C failures before FAULT_IMU_I2C (5 x 10 ms). */
#define IMU_I2C_FAIL_THRESHOLD      (5U)

/* Lift tuning log: once a second, print the peaks if the car moved up faster
 * than this or tilted past 15 deg (debug console only; quiet at rest). */
#define IMU_LIFT_LOG_SAMPLES        (100U)      /* 1 s */
#define IMU_LIFT_LOG_V_MMPS         (40)
#define IMU_LIFT_LOG_COS_X1000      (966)

/* "Turning" for the hump estimator: the two wheels' travel differs by more
 * than half their total (an in-place turn is 100 %, straight is ~0 %). */
#define IMU_TURN_DIFF_PCT           (50)
/* Treat the car as still turning this long after the wheels last moved
 * against each other: Motion publishes every 20 ms (every other sample
 * sees no travel) and the 80 ms filter still holds the turn's tilt. */
#define IMU_TURN_HOLD_SAMPLES       (20U)       /* 200 ms */

/* "Speed change" for the hump estimator: the wheel speed changed by more
 * than this between two Motion periods (20 mm/s in 20 ms = 1 m/s^2, which
 * reads as ~6 deg of pitch).  Pulling away and braking at 22 cm/s read
 * +-13 deg; the slowdown on the 10 mm hump itself is ~12 mm/s (S4-U26
 * traces, 19 Nov 2026).  Held for 200 ms: the 80 ms filter still shows
 * the acceleration after it ends. TUNE */
#define IMU_SPEED_STEP_MMPS         (20)
#define IMU_SPEED_HOLD_SAMPLES      (20U)       /* 200 ms */

/* Hump trace for tuning (debug console): the last IMU_TRACE_LEN samples of
 * pitch (half-deg) and wheel travel (0.25 mm), printed after every hump,
 * from IMU_TRACE_PRE samples before it started, one line per 10 ms so the
 * dbg_ ring (2 KB) never overflows.  32 samples per line, 2 hex digits each
 * (signed 8 bit, two's complement). */
#define IMU_TRACE_LEN               (256U)
#define IMU_TRACE_PRE               (30U)
#define IMU_TRACE_PER_LINE          (32U)

/* Low-pass for pitch: each sample moves the filtered value 1/8 of the way
 * (time constant about 8 x 10 ms = 80 ms).  Motor and servo vibration
 * otherwise swings the raw pitch by +-15 deg on the car.  The lift detector
 * uses the same filtered vector. */
#define IMU_FILTER_DIV              (8)

/* Boot level reference, so a board not mounted level still reads 0 on flat
 * ground.  After the filter settles, average a 0.5 s window, accepted only
 * if the pitch stayed within IMU_LEVEL_STILL_SPAN (car not shaking or held)
 * and the average is within IMU_LEVEL_MAX_TILT (car not tilted); otherwise
 * start a new window.  If no window is accepted in IMU_LEVEL_TIMEOUT
 * samples, use 0: the board measured 0 when flat on 2 Nov 2026.  The same
 * window gives the lift detector its "up" vector. */
#define IMU_LEVEL_SKIP_SAMPLES      (20U)       /* 200 ms filter settling */
#define IMU_LEVEL_AVG_SAMPLES       (50U)       /* 500 ms window */
#define IMU_LEVEL_STILL_SPAN        (6)         /* 3 deg max-min in a window */
#define IMU_LEVEL_MAX_TILT          (20)        /* 10 deg */
#define IMU_LEVEL_TIMEOUT           (1000U)     /* 10 s */

static volatile bool     s_running;
static volatile int32_t  s_pitch_half_deg;
static volatile int32_t  s_pitch_offset_half_deg;
static volatile bool     s_level_done;
static volatile uint32_t s_accel_mag_mg;
static volatile int32_t  s_z_mg;
static volatile uint32_t s_release_count;
static volatile uint32_t s_good_count;
static volatile uint32_t s_i2c_error_count;
static volatile uint32_t s_lift_count;
static volatile uint32_t s_last_lift_event;
static volatile uint32_t s_hump_count;
static volatile int32_t  s_last_hump_pitch;
static volatile int32_t  s_last_hump_height_mm;
static volatile int32_t  s_max_hump_height_mm;
static volatile uint32_t s_hump_dropped_count;

static uint32_t s_consecutive_fail;
static imu_hump_t s_hump;
static bool       s_enc_primed;
static int32_t    s_enc_prev[2];
static uint32_t   s_turn_hold;
static uint32_t   s_speed_hold;
static uint32_t   s_prev_tick;
static int32_t    s_prev_v_mmps;
static int8_t     s_tr_pitch[IMU_TRACE_LEN];
static int8_t     s_tr_ds[IMU_TRACE_LEN];
static uint32_t   s_tr_head;        /* next write index */
static uint32_t   s_tr_since;       /* samples since the hump started */
static uint32_t   s_tr_out_left;    /* samples still to print */
static uint32_t   s_tr_out_pos;     /* index of the next sample to print */
static uint32_t   s_tr_out_n;       /* samples in the dump being printed */
static bool       s_tr_out_ds;      /* printing the travel series */
static imu_lift_t s_lift;
static uint32_t   s_lift_log_n;
static bool     s_filter_primed;
static int32_t  s_fx_mg;
static int32_t  s_fy_mg;
static int32_t  s_fz_mg;
static uint32_t s_level_count;     /* samples since start */
static uint32_t s_level_win_n;     /* samples in the current window */
static int32_t  s_level_sum;
static int32_t  s_level_sum_x;     /* filtered vector sums, for the up vector */
static int32_t  s_level_sum_y;
static int32_t  s_level_sum_z;
static int32_t  s_level_min;
static int32_t  s_level_max;
static uint32_t s_level_retries;

bool imu_is_task_running(void)
{
    return s_running;
}

int32_t imu_get_pitch_half_deg(void)
{
    return s_pitch_half_deg;
}

int32_t imu_get_pitch_offset_half_deg(void)
{
    return s_pitch_offset_half_deg;
}

bool imu_is_level_done(void)
{
    return s_level_done;
}

uint32_t imu_get_accel_mag_mg(void)
{
    return s_accel_mag_mg;
}

int32_t imu_get_z_mg(void)
{
    return s_z_mg;
}

uint32_t imu_get_release_count(void)
{
    return s_release_count;
}

uint32_t imu_get_good_sample_count(void)
{
    return s_good_count;
}

uint32_t imu_get_i2c_error_count(void)
{
    return s_i2c_error_count;
}

uint32_t imu_get_lift_count(void)
{
    return s_lift_count;
}

uint32_t imu_get_last_lift_event(void)
{
    return s_last_lift_event;
}

bool imu_is_lift_armed(void)
{
    return s_lift.armed;
}

uint32_t imu_get_hump_count(void)
{
    return s_hump_count;
}

int32_t imu_get_last_hump_pitch_half_deg(void)
{
    return s_last_hump_pitch;
}

int32_t imu_get_last_hump_height_mm(void)
{
    return s_last_hump_height_mm;
}

int32_t imu_get_max_hump_height_mm(void)
{
    return s_max_hump_height_mm;
}

uint32_t imu_get_hump_dropped_count(void)
{
    return s_hump_dropped_count;
}

bool imu_task_init(void)
{
    s_running          = false;
    s_pitch_half_deg   = 0;
    s_accel_mag_mg     = 0U;
    s_z_mg             = 0;
    s_release_count    = 0U;
    s_good_count       = 0U;
    s_i2c_error_count  = 0U;
    s_lift_count       = 0U;
    s_last_lift_event  = (uint32_t)IMU_LIFT_NONE;
    s_hump_count       = 0U;
    s_last_hump_pitch  = 0;
    s_last_hump_height_mm = 0;
    s_max_hump_height_mm  = 0;
    s_hump_dropped_count  = 0U;
    s_consecutive_fail = 0U;
    imu_hump_init(&s_hump);
    s_enc_primed       = false;
    s_enc_prev[0]      = 0;
    s_enc_prev[1]      = 0;
    s_turn_hold        = 0U;
    s_speed_hold       = 0U;
    s_prev_tick        = 0U;
    s_prev_v_mmps      = 0;
    s_tr_head          = 0U;
    s_tr_since         = 0U;
    s_tr_out_left      = 0U;
    s_tr_out_pos       = 0U;
    s_tr_out_n         = 0U;
    s_tr_out_ds        = false;
    imu_lift_init(&s_lift);
    s_lift_log_n       = 0U;
    s_filter_primed    = false;
    s_fx_mg            = 0;
    s_fy_mg            = 0;
    s_fz_mg            = 0;
    s_level_count      = 0U;
    s_level_win_n      = 0U;
    s_level_sum        = 0;
    s_level_sum_x      = 0;
    s_level_sum_y      = 0;
    s_level_sum_z      = 0;
    s_level_min        = 0;
    s_level_max        = 0;
    s_level_retries    = 0U;
    s_level_done       = false;
    s_pitch_offset_half_deg = 0;

    return imu_init();
}

/* One line listing every address that answered on I2C1 at boot. */
static void log_bus_scan(void)
{
    static char const hex[] = "0123456789ABCDEF";
    char     line[DBG_LINE_MAX + 1U];
    uint32_t n     = 0U;
    uint32_t found = 0U;
    char const *p_head = "[IMU] I2C1 scan (SDA GP2, SCL GP3):";

    while ((*p_head != '\0') && (n < DBG_LINE_MAX))
    {
        line[n++] = *p_head++;
    }

    for (uint32_t addr = 0x08U; (addr <= 0x77U) && ((n + 5U) <= DBG_LINE_MAX); addr++)
    {
        if (imu_scan_found(addr))
        {
            line[n++] = ' ';
            line[n++] = '0';
            line[n++] = 'x';
            line[n++] = hex[(addr >> 4U) & 0xFU];
            line[n++] = hex[addr & 0xFU];
            found++;
        }
    }

    if (found == 0U)
    {
        char const *p_none = " nothing answered (check wiring)";

        while ((*p_none != '\0') && (n < DBG_LINE_MAX))
        {
            line[n++] = *p_none++;
        }
    }

    line[n] = '\0';
    dbg_puts(line);
}

/* Which chip was found; if none, the ID registers of each device that answered. */
static void log_chip(void)
{
    uint32_t const chip = imu_chip();

    if (chip == IMU_CHIP_LSM303DLHC)
    {
        dbg_printf("[IMU] chip LSM303DLHC, accel at 0x%02lX", (unsigned long)imu_accel_addr());
        return;
    }

    if (chip == IMU_CHIP_LSM303D)
    {
        dbg_printf("[IMU] chip LSM303D, accel+mag at 0x%02lX", (unsigned long)imu_accel_addr());
        return;
    }

    for (uint32_t addr = 0x08U; addr <= 0x77U; addr++)
    {
        if (imu_scan_found(addr))
        {
            /* WHO_AM_I candidates: ST 0x0F, NXP 0x0D, ADI 0x00, InvenSense 0x75. */
            dbg_printf("[IMU] unknown chip at 0x%02lX: r0F=%ld r0D=%ld r00=%ld r75=%ld",
                       (unsigned long)addr,
                       (long)imu_read_id_reg(addr, 0x0FU), (long)imu_read_id_reg(addr, 0x0DU),
                       (long)imu_read_id_reg(addr, 0x00U), (long)imu_read_id_reg(addr, 0x75U));
        }
    }
}

static void send_tel(tel_evt_t evt)
{
    (void)tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL);
}

static void check_lift(void)
{
    imu_lift_meas_t        meas;
    bool const             was_armed = s_lift.armed;
    imu_lift_event_t const evt       = imu_lift_step(&s_lift, s_fx_mg, s_fy_mg, s_fz_mg, &meas);

    if (evt != IMU_LIFT_NONE)
    {
        s_last_lift_event = (uint32_t)evt;
        s_lift_count++;
        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_LIFTED);
        dbg_printf("[IMU] LIFTED (%s): cos_tilt=%ld/1000 up_accel=%ld mg v=%ld mm/s",
                   (evt == IMU_LIFT_TILTED) ? ((meas.cos_tilt_x1000 < 0) ? "overturned" : "tilted")
                                            : "raised",
                   (long)meas.cos_tilt_x1000, (long)meas.up_accel_mg, (long)meas.v_mmps);
    }
    else if (!was_armed && s_lift.armed)
    {
        dbg_puts("[IMU] lift detection re-armed (car level and still)");
    }

    s_lift_log_n++;

    if (s_lift_log_n >= IMU_LIFT_LOG_SAMPLES)
    {
        imu_lift_meas_t peak;

        s_lift_log_n = 0U;
        imu_lift_take_peaks(&s_lift, &peak);

        if ((peak.v_mmps >= IMU_LIFT_LOG_V_MMPS) || (peak.cos_tilt_x1000 < IMU_LIFT_LOG_COS_X1000))
        {
            dbg_printf("[IMU] lift peak 1s: v=%ld/%lu mm/s acc=%ld mg cos=%ld/%lu%s",
                       (long)peak.v_mmps, (unsigned long)IMU_LIFT_V_MMPS, (long)peak.up_accel_mg,
                       (long)peak.cos_tilt_x1000, (unsigned long)IMU_LIFT_COS_TILT_X1000,
                       s_lift.armed ? "" : " [disarmed]");
        }
    }
}

/* Wheel travel since the last call (um, mean of both wheels).  Motion
 * publishes its state every 20 ms, so every other call sees no travel.
 * Also notes when the wheels move against each other (a turn). */
static int32_t wheel_travel_um(void)
{
    motion_state_t st;

    if (motion_get_state(&st) != E_OK)
    {
        return 0;
    }

    if (!s_enc_primed)
    {
        s_enc_prev[0] = st.enc_count[0];
        s_enc_prev[1] = st.enc_count[1];
        s_prev_tick   = st.tick_ms;
        s_enc_primed  = true;
        return 0;
    }

    if (st.tick_ms == s_prev_tick)
    {
        return 0;               /* Motion has not published since the last call */
    }

    /* Wrap-safe deltas of the free-running counts. */
    int32_t const dl = (int32_t)((uint32_t)st.enc_count[0] - (uint32_t)s_enc_prev[0]);
    int32_t const dr = (int32_t)((uint32_t)st.enc_count[1] - (uint32_t)s_enc_prev[1]);

    s_enc_prev[0] = st.enc_count[0];
    s_enc_prev[1] = st.enc_count[1];

    int32_t const abs_l = (dl >= 0) ? dl : -dl;
    int32_t const abs_r = (dr >= 0) ? dr : -dr;
    int32_t const diff  = ((dl - dr) >= 0) ? (dl - dr) : (dr - dl);

    if (((diff * 100) > ((abs_l + abs_r) * IMU_TURN_DIFF_PCT)) && (diff > 2))
    {
        s_turn_hold = IMU_TURN_HOLD_SAMPLES;
    }

    /* Speed over this Motion period (um per ms = mm/s), compared with the
     * last period's. */
    int32_t const  um    = imu_hump_counts_to_um((dl + dr) / 2);
    uint32_t const dt_ms = st.tick_ms - s_prev_tick;
    int32_t const  v     = (dt_ms > 0U) ? (um / (int32_t)dt_ms) : 0;
    int32_t const  dv    = v - s_prev_v_mmps;

    s_prev_tick   = st.tick_ms;
    s_prev_v_mmps = v;
    if ((dv > IMU_SPEED_STEP_MMPS) || (dv < -IMU_SPEED_STEP_MMPS))
    {
        s_speed_hold = IMU_SPEED_HOLD_SAMPLES;
    }

    return um;
}

static int8_t clamp_s8(int32_t v)
{
    return (int8_t)((v > 127) ? 127 : ((v < -127) ? -127 : v));
}

/* Start printing the trace of the hump that just ended. */
static void trace_dump_start(void)
{
    uint32_t n = s_tr_since + IMU_TRACE_PRE;

    n = (n > IMU_TRACE_LEN) ? IMU_TRACE_LEN : n;
    s_tr_out_n    = n;
    s_tr_out_left = n;
    s_tr_out_pos  = (s_tr_head + IMU_TRACE_LEN - n) % IMU_TRACE_LEN;
    s_tr_out_ds   = false;
    dbg_printf("[HTR] %lu samples, 10 ms apart: p = pitch half-deg, d = travel 0.25 mm",
               (unsigned long)n);
}

/* Print at most one trace line (called every sample). */
static void trace_dump_line(void)
{
    static char const hex[] = "0123456789abcdef";
    char              line[IMU_TRACE_PER_LINE * 2U + 1U];
    uint32_t          k = 0U;
    uint32_t const    first = s_tr_out_n - s_tr_out_left;

    if (s_tr_out_left == 0U)
    {
        return;
    }

    while ((k < IMU_TRACE_PER_LINE) && (s_tr_out_left > 0U))
    {
        uint8_t const v = (uint8_t)(s_tr_out_ds ? s_tr_ds[s_tr_out_pos] : s_tr_pitch[s_tr_out_pos]);

        line[k * 2U]      = hex[v >> 4];
        line[k * 2U + 1U] = hex[v & 0x0FU];
        k++;
        s_tr_out_left--;
        s_tr_out_pos = (s_tr_out_pos + 1U) % IMU_TRACE_LEN;
    }
    line[k * 2U] = '\0';
    dbg_printf("[HTR] %c%03lu %s", s_tr_out_ds ? 'd' : 'p', (unsigned long)first, line);

    if ((s_tr_out_left == 0U) && !s_tr_out_ds)
    {
        /* Pitch done: now the same samples' travel. */
        s_tr_out_ds   = true;
        s_tr_out_left = s_tr_out_n;
        s_tr_out_pos  = (s_tr_head + IMU_TRACE_LEN - s_tr_out_n) % IMU_TRACE_LEN;
    }
}

static void check_hump(int32_t pitch)
{
    int32_t const          ds_um   = wheel_travel_um();
    /* Not trustworthy while turning, or while the car is (or was just)
     * lifted: a hump in progress is then dropped, so carrying the car
     * (e.g. the end-of-run pick-up) never reports a hump. */
    bool const             turning = (s_turn_hold > 0U) || !s_lift.armed;
    imu_hump_report_t      rep;
    bool const             speed_change = (s_speed_hold > 0U);
    imu_hump_event_t const ev    = imu_hump_step(&s_hump, pitch, ds_um, turning, speed_change, &rep);

    s_turn_hold  = (s_turn_hold > 0U) ? (s_turn_hold - 1U) : 0U;
    s_speed_hold = (s_speed_hold > 0U) ? (s_speed_hold - 1U) : 0U;

    /* Trace: a new dump overwrites one still printing (the newer hump wins). */
    if (s_tr_out_left == 0U)
    {
        s_tr_pitch[s_tr_head] = clamp_s8(pitch);
        s_tr_ds[s_tr_head]    = clamp_s8(ds_um / 250);
        s_tr_head             = (s_tr_head + 1U) % IMU_TRACE_LEN;
    }
    s_tr_since = (ev == IMU_HUMP_START) ? 0U : (s_tr_since + 1U);

    if (ev == IMU_HUMP_START)
    {
        dbg_printf("[IMU] hump start pitch=%ld half-deg", (long)pitch);
    }
    else if (ev == IMU_HUMP_END)
    {
        s_last_hump_pitch     = rep.pitch_half_deg;
        s_last_hump_height_mm = rep.height_mm;
        if (rep.height_mm > s_max_hump_height_mm)
        {
            s_max_hump_height_mm = rep.height_mm;
        }
        s_hump_count++;
        dbg_printf("[IMU] hump h=%ld mm (tilt %ld climb %ld) held=%ld spike=%ld hd max=%ld",
                   (long)rep.height_mm, (long)rep.tilt_mm, (long)rep.climb_mm,
                   (long)rep.pitch_half_deg, (long)rep.spike_half_deg, (long)s_max_hump_height_mm);
        send_tel(drep_tel_hump(app_period_now_ms(), rep.height_mm, rep.pitch_half_deg));
        trace_dump_start();
    }
    else if (ev == IMU_HUMP_DROPPED)
    {
        s_hump_dropped_count++;
        dbg_puts("[IMU] hump dropped: the car turned or was lifted during it");
        trace_dump_start();
    }
    else
    {
        trace_dump_line();
    }
}

static void finish_level(int32_t offset, int32_t up_x, int32_t up_y, int32_t up_z)
{
    s_pitch_offset_half_deg = offset;
    imu_lift_set_up(&s_lift, up_x, up_y, up_z);
    s_level_done            = true;
}

static void update_level(int32_t board_pitch)
{
    s_level_count++;

    if (s_level_count <= IMU_LEVEL_SKIP_SAMPLES)
    {
        return;
    }

    if (s_level_count > IMU_LEVEL_TIMEOUT)
    {
        /* Not still and flat: assume the board's Z axis points up or down. */
        finish_level(0, 0, 0, (s_fz_mg >= 0) ? 1000 : -1000);
        dbg_printf("[IMU] level reference: car not still and flat in 10 s (%lu tries), using 0",
                   (unsigned long)s_level_retries);
        return;
    }

    if (s_level_win_n == 0U)
    {
        s_level_sum   = 0;
        s_level_sum_x = 0;
        s_level_sum_y = 0;
        s_level_sum_z = 0;
        s_level_min = board_pitch;
        s_level_max = board_pitch;
    }

    s_level_win_n++;
    s_level_sum += board_pitch;
    s_level_sum_x += s_fx_mg;
    s_level_sum_y += s_fy_mg;
    s_level_sum_z += s_fz_mg;
    s_level_min = (board_pitch < s_level_min) ? board_pitch : s_level_min;
    s_level_max = (board_pitch > s_level_max) ? board_pitch : s_level_max;

    if ((s_level_max - s_level_min) > IMU_LEVEL_STILL_SPAN)
    {
        s_level_win_n = 0U;     /* moving or shaking: start a new window */
        s_level_retries++;
        return;
    }

    if (s_level_win_n < IMU_LEVEL_AVG_SAMPLES)
    {
        return;
    }

    int32_t const avg     = s_level_sum / (int32_t)IMU_LEVEL_AVG_SAMPLES;
    int32_t const abs_avg = (avg >= 0) ? avg : -avg;

    s_level_win_n = 0U;

    if (abs_avg > IMU_LEVEL_MAX_TILT)
    {
        s_level_retries++;      /* steady but tilted: not flat yet */
        return;
    }

    finish_level(avg, s_level_sum_x / (int32_t)IMU_LEVEL_AVG_SAMPLES,
                 s_level_sum_y / (int32_t)IMU_LEVEL_AVG_SAMPLES,
                 s_level_sum_z / (int32_t)IMU_LEVEL_AVG_SAMPLES);
    dbg_printf("[IMU] level reference: board at %ld half-deg when flat (%lu tries)",
               (long)avg, (unsigned long)s_level_retries);
}

void imu_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    app_period_t per;

    app_period_init(&per, IMU_SAMPLE_PERIOD_MS);
    s_running = true;
    dbg_puts(imu_is_ready() ? "[IMU] task started" : "[IMU] task started, IMU NOT READY");
    log_bus_scan();
    log_chip();

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            dbg_printf("[IMU] missed %lu releases", (unsigned long)missed);
        }

        s_release_count++;

        imu_accel_raw_t raw;

        if (!imu_read_accel(&raw))
        {
            s_i2c_error_count++;

            if (s_consecutive_fail < IMU_I2C_FAIL_THRESHOLD)
            {
                s_consecutive_fail++;

                if (s_consecutive_fail == IMU_I2C_FAIL_THRESHOLD)
                {
                    (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_IMU_I2C);
                    dbg_puts("[IMU] I2C fault");
                }
            }

            /* No IMU_ALIVE: the Safety watchdog brakes after 50 ms. */
            continue;
        }

        if (s_consecutive_fail > 0U)
        {
            /* The sensor answers again after failed reads.  If it lost power
             * it is powered down and returns stale data, so configure it
             * again and drop this sample. */
            if (!imu_reconfigure())
            {
                s_i2c_error_count++;
                continue;
            }

            if (s_consecutive_fail >= IMU_I2C_FAIL_THRESHOLD)
            {
                (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_IMU_I2C);
                dbg_puts("[IMU] I2C back, sensor configured again");
            }

            s_consecutive_fail = 0U;
            s_filter_primed    = false;
            continue;
        }

        s_consecutive_fail = 0U;

        int32_t x_mg;
        int32_t y_mg;
        int32_t z_mg;

        imu_raw_to_mg(&raw, &x_mg, &y_mg, &z_mg);

        if (!s_filter_primed)
        {
            s_fx_mg = x_mg;
            s_fy_mg = y_mg;
            s_fz_mg = z_mg;
            s_filter_primed = true;
        }
        else
        {
            s_fx_mg += (x_mg - s_fx_mg) / IMU_FILTER_DIV;
            s_fy_mg += (y_mg - s_fy_mg) / IMU_FILTER_DIV;
            s_fz_mg += (z_mg - s_fz_mg) / IMU_FILTER_DIV;
        }

        int32_t const  board_pitch = imu_pitch_half_deg(s_fx_mg, s_fy_mg, s_fz_mg);

        s_accel_mag_mg = imu_accel_magnitude_mg(s_fx_mg, s_fy_mg, s_fz_mg);
        s_z_mg         = s_fz_mg;
        s_good_count++;

        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_IMU_ALIVE);

        if (!s_level_done)
        {
            update_level(board_pitch);
            continue;
        }

        int32_t const pitch = board_pitch - s_pitch_offset_half_deg;

        s_pitch_half_deg = pitch;
        /* Dashboard integration (IF-12): newest attitude for the 200 ms
         * sensor frame; one aligned store, never waits. */
        tel_set_attitude(pitch, imu_roll_half_deg(s_fx_mg, s_fy_mg, s_fz_mg));
        check_lift();
        check_hump(pitch);
    }
}
