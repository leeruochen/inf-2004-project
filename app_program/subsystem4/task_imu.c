/*
 *  task_imu.c
 *  Subsystem 4 -- IMU Processing task (priority 8, 10 ms periodic).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Responsibilities (design.md 3.1/3.2, HANDOVER.md section 3):
 *    - Drift-free 10 ms periodic execution via app_period_wait()
 *      (IMU_SAMPLE_PERIOD_MS).
 *    - Reads the LSM303DLHC accelerometer via drv_imu.
 *    - Computes pitch from accelerometer only (no gyro, D4).
 *    - Sets EVF_SAFETY_IMU_ALIVE every cycle as a watchdog heartbeat.
 *    - Detects collision (magnitude > IMU_G_COLLISION_X1000 mg) and
 *      sets EVF_SAFETY_COLLISION.
 *    - Detects humps (pitch exceeds IMU_HUMP_PITCH_DEG) and sends a
 *      tel_evt_t (DREP_TEL_HUMP) to MBF_TELEMETRY.
 *    - Sets EVF_SAFETY_FAULT_IMU_I2C on repeated I2C failures.
 */

#include "task_imu.h"

#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_imu.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

/* Consecutive I2C failures before setting the fault flag. */
#define IMU_I2C_FAIL_THRESHOLD      (5U)

/* Hump detection: pitch must exceed the threshold for this many consecutive
 * samples before a hump event is emitted (debounce at 10 ms). */
#define IMU_HUMP_CONFIRM_SAMPLES    (3U)

/* Hump pitch threshold in half-degrees (Q1). */
#define IMU_HUMP_PITCH_HALF_DEG     ((int32_t)IMU_HUMP_PITCH_DEG * 2)

/* Collision threshold in milli-g. */
#define IMU_COLLISION_MG            ((uint32_t)IMU_G_COLLISION_X1000)

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */

static volatile bool     s_running;
static volatile int32_t  s_pitch_half_deg;
static volatile uint32_t s_accel_mag_mg;
static volatile uint32_t s_release_count;
static volatile uint32_t s_i2c_error_count;

static uint32_t s_consecutive_fail;
static uint32_t s_hump_confirm;
static bool     s_hump_active;

/* ---------------------------------------------------------------------------
 * Public accessors (read-only from other tasks; no lock needed for
 * single-word volatile reads).
 * ------------------------------------------------------------------------- */

bool imu_is_task_running(void)
{
    return s_running;
}

int32_t imu_get_pitch_half_deg(void)
{
    return s_pitch_half_deg;
}

uint32_t imu_get_accel_mag_mg(void)
{
    return s_accel_mag_mg;
}

uint32_t imu_get_release_count(void)
{
    return s_release_count;
}

uint32_t imu_get_i2c_error_count(void)
{
    return s_i2c_error_count;
}

/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

bool imu_task_init(void)
{
    s_running           = false;
    s_pitch_half_deg    = 0;
    s_accel_mag_mg      = 0U;
    s_release_count     = 0U;
    s_i2c_error_count   = 0U;
    s_consecutive_fail  = 0U;
    s_hump_confirm      = 0U;
    s_hump_active       = false;

    return imu_init();
}

/* ---------------------------------------------------------------------------
 * Task entry
 * ------------------------------------------------------------------------- */

void imu_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    app_period_t per;

    app_period_init(&per, IMU_SAMPLE_PERIOD_MS);
    s_running = true;
    dbg_puts("[IMU] task started");

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            dbg_printf("[IMU] missed %lu releases", (unsigned long)missed);
        }

        s_release_count++;

        /* --- Read accelerometer --- */
        imu_accel_raw_t raw;

        if (!imu_read_accel(&raw))
        {
            s_i2c_error_count++;
            s_consecutive_fail++;

            if (s_consecutive_fail >= IMU_I2C_FAIL_THRESHOLD)
            {
                (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_IMU_I2C);
            }

            continue;
        }

        s_consecutive_fail = 0U;

        /* --- Convert to milli-g --- */
        int32_t x_mg;
        int32_t y_mg;
        int32_t z_mg;

        imu_raw_to_mg(&raw, &x_mg, &y_mg, &z_mg);

        /* --- Compute pitch and magnitude --- */
        int32_t const pitch = imu_pitch_half_deg(x_mg, z_mg);
        uint32_t const mag  = imu_accel_magnitude_mg(x_mg, y_mg, z_mg);

        s_pitch_half_deg = pitch;
        s_accel_mag_mg   = mag;

        /* --- Watchdog heartbeat: signal IMU_ALIVE every cycle --- */
        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_IMU_ALIVE);

        /* --- Collision detection --- */
        if (mag >= IMU_COLLISION_MG)
        {
            (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_COLLISION);
            dbg_printf("[IMU] COLLISION mag=%lu mg", (unsigned long)mag);

            /* Send a fault telemetry event. */
            uint32_t const now = app_period_now_ms();
            tel_evt_t const evt = drep_tel_make(DREP_TEL_FAULT, now,
                                                EVF_SAFETY_COLLISION, false);

            (void)tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL);
        }

        /* --- Hump detection (debounced) --- */
        int32_t abs_pitch = (pitch >= 0) ? pitch : -pitch;

        if (abs_pitch >= IMU_HUMP_PITCH_HALF_DEG)
        {
            s_hump_confirm++;

            if ((s_hump_confirm >= IMU_HUMP_CONFIRM_SAMPLES) && !s_hump_active)
            {
                s_hump_active = true;
                dbg_printf("[IMU] hump pitch=%ld half-deg", (long)pitch);

                uint32_t const now = app_period_now_ms();
                tel_evt_t const evt = drep_tel_hump(now, 0, pitch);

                (void)tk_snd_mbf(MBF_TELEMETRY, &evt,
                                 (INT)sizeof(evt), TMO_POL);
            }
        }
        else
        {
            s_hump_confirm = 0U;
            s_hump_active  = false;
        }
    }
}
