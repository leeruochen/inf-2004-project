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
 *    - Sets EVF_SAFETY_COLLISION once per spike above IMU_G_COLLISION_X1000.
 *    - Sends one DREP_TEL_HUMP event per hump (pitch above IMU_HUMP_PITCH_DEG).
 *    - Sets EVF_SAFETY_FAULT_IMU_I2C after repeated I2C failures.
 */

#include "task_imu.h"

#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_imu.h"

/* Consecutive I2C failures before FAULT_IMU_I2C (5 x 10 ms). */
#define IMU_I2C_FAIL_THRESHOLD      (5U)

/* Samples above the hump pitch before a hump is reported (debounce). */
#define IMU_HUMP_CONFIRM_SAMPLES    (3U)

#define IMU_HUMP_PITCH_HALF_DEG     ((int32_t)IMU_HUMP_PITCH_DEG * 2)
#define IMU_COLLISION_MG            ((uint32_t)IMU_G_COLLISION_X1000)

/* Re-arm collision detection once the magnitude falls below this (1.5 g). */
#define IMU_COLLISION_REARM_MG      (1500U)

#if IMU_COLLISION_REARM_MG >= IMU_G_COLLISION_X1000
#error "task_imu.c: collision re-arm level must be below g_collision"
#endif

/* A hump ends once |pitch| falls below half the hump threshold (hysteresis). */
#define IMU_HUMP_EXIT_HALF_DEG      (IMU_HUMP_PITCH_HALF_DEG / 2)

/* Low-pass for pitch: each sample moves the filtered value 1/8 of the way
 * (time constant about 8 x 10 ms = 80 ms).  Motor and servo vibration
 * otherwise swings the raw pitch by +-15 deg on the car.  Collision uses the
 * unfiltered magnitude so a sharp hit is not smoothed away. */
#define IMU_FILTER_DIV              (8)

/* Boot level reference, so a board not mounted level still reads 0 on flat
 * ground.  After the filter settles, average a 0.5 s window, accepted only
 * if the pitch stayed within IMU_LEVEL_STILL_SPAN (car not shaking or held)
 * and the average is within IMU_LEVEL_MAX_TILT (car not tilted); otherwise
 * start a new window.  If no window is accepted in IMU_LEVEL_TIMEOUT
 * samples, use 0: the board measured 0 when flat on 2 Nov 2026. */
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
static volatile uint32_t s_collision_count;
static volatile uint32_t s_hump_count;
static volatile int32_t  s_last_hump_pitch;

static uint32_t s_consecutive_fail;
static uint32_t s_hump_confirm;
static bool     s_hump_active;
static bool     s_collision_armed;
static bool     s_filter_primed;
static int32_t  s_fx_mg;
static int32_t  s_fy_mg;
static int32_t  s_fz_mg;
static uint32_t s_level_count;     /* samples since start */
static uint32_t s_level_win_n;     /* samples in the current window */
static int32_t  s_level_sum;
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

uint32_t imu_get_collision_count(void)
{
    return s_collision_count;
}

uint32_t imu_get_hump_count(void)
{
    return s_hump_count;
}

int32_t imu_get_last_hump_pitch_half_deg(void)
{
    return s_last_hump_pitch;
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
    s_collision_count  = 0U;
    s_hump_count       = 0U;
    s_last_hump_pitch  = 0;
    s_consecutive_fail = 0U;
    s_hump_confirm     = 0U;
    s_hump_active      = false;
    s_collision_armed  = true;
    s_filter_primed    = false;
    s_fx_mg            = 0;
    s_fy_mg            = 0;
    s_fz_mg            = 0;
    s_level_count      = 0U;
    s_level_win_n      = 0U;
    s_level_sum        = 0;
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

static void check_collision(uint32_t mag)
{
    if (s_collision_armed && (mag >= IMU_COLLISION_MG))
    {
        s_collision_armed = false;
        s_collision_count++;
        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_COLLISION);
        dbg_printf("[IMU] COLLISION mag=%lu mg", (unsigned long)mag);
    }
    else if (!s_collision_armed && (mag < IMU_COLLISION_REARM_MG))
    {
        s_collision_armed = true;
    }
}

static void check_hump(int32_t pitch)
{
    int32_t const abs_pitch = (pitch >= 0) ? pitch : -pitch;

    if (abs_pitch < IMU_HUMP_EXIT_HALF_DEG)
    {
        s_hump_confirm = 0U;
        s_hump_active  = false;
        return;
    }

    if (abs_pitch < IMU_HUMP_PITCH_HALF_DEG)
    {
        s_hump_confirm = 0U;    /* between exit and entry level: no change */
        return;
    }

    if (s_hump_confirm < IMU_HUMP_CONFIRM_SAMPLES)
    {
        s_hump_confirm++;
    }

    if ((s_hump_confirm >= IMU_HUMP_CONFIRM_SAMPLES) && !s_hump_active)
    {
        s_hump_active = true;
        s_last_hump_pitch = pitch;
        s_hump_count++;
        dbg_printf("[IMU] hump pitch=%ld half-deg", (long)pitch);
        send_tel(drep_tel_hump(app_period_now_ms(), 0, pitch));
    }
}

static void finish_level(int32_t offset)
{
    s_pitch_offset_half_deg = offset;
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
        finish_level(0);
        dbg_printf("[IMU] level reference: car not still and flat in 10 s (%lu tries), using 0",
                   (unsigned long)s_level_retries);
        return;
    }

    if (s_level_win_n == 0U)
    {
        s_level_sum = 0;
        s_level_min = board_pitch;
        s_level_max = board_pitch;
    }

    s_level_win_n++;
    s_level_sum += board_pitch;
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

    finish_level(avg);
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

        uint32_t const raw_mag   = imu_accel_magnitude_mg(x_mg, y_mg, z_mg);
        int32_t const  board_pitch = imu_pitch_half_deg(s_fx_mg, s_fy_mg, s_fz_mg);

        s_accel_mag_mg = imu_accel_magnitude_mg(s_fx_mg, s_fy_mg, s_fz_mg);
        s_z_mg         = s_fz_mg;
        s_good_count++;

        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_IMU_ALIVE);

        check_collision(raw_mag);

        if (!s_level_done)
        {
            update_level(board_pitch);
            continue;
        }

        int32_t const pitch = board_pitch - s_pitch_offset_half_deg;

        s_pitch_half_deg = pitch;
        check_hump(pitch);
    }
}
