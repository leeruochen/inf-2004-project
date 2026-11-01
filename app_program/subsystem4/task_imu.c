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
 *    - Pitch from the accelerometer only (no gyro, D4).
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

static volatile bool     s_running;
static volatile int32_t  s_pitch_half_deg;
static volatile uint32_t s_accel_mag_mg;
static volatile int32_t  s_z_mg;
static volatile uint32_t s_release_count;
static volatile uint32_t s_good_count;
static volatile uint32_t s_i2c_error_count;
static volatile uint32_t s_collision_count;

static uint32_t s_consecutive_fail;
static uint32_t s_hump_confirm;
static bool     s_hump_active;
static bool     s_collision_armed;

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
    s_consecutive_fail = 0U;
    s_hump_confirm     = 0U;
    s_hump_active      = false;
    s_collision_armed  = true;

    return imu_init();
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

    if (abs_pitch < IMU_HUMP_PITCH_HALF_DEG)
    {
        s_hump_confirm = 0U;
        s_hump_active  = false;
        return;
    }

    if (s_hump_confirm < IMU_HUMP_CONFIRM_SAMPLES)
    {
        s_hump_confirm++;
    }

    if ((s_hump_confirm >= IMU_HUMP_CONFIRM_SAMPLES) && !s_hump_active)
    {
        s_hump_active = true;
        dbg_printf("[IMU] hump pitch=%ld half-deg", (long)pitch);
        send_tel(drep_tel_hump(app_period_now_ms(), 0, pitch));
    }
}

void imu_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    app_period_t per;

    app_period_init(&per, IMU_SAMPLE_PERIOD_MS);
    s_running = true;
    dbg_puts(imu_is_ready() ? "[IMU] task started" : "[IMU] task started, IMU NOT READY");

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

        s_consecutive_fail = 0U;

        int32_t x_mg;
        int32_t y_mg;
        int32_t z_mg;

        imu_raw_to_mg(&raw, &x_mg, &y_mg, &z_mg);

        int32_t const  pitch = imu_pitch_half_deg(x_mg, y_mg, z_mg);
        uint32_t const mag   = imu_accel_magnitude_mg(x_mg, y_mg, z_mg);

        s_pitch_half_deg = pitch;
        s_accel_mag_mg   = mag;
        s_z_mg           = z_mg;
        s_good_count++;

        (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_IMU_ALIVE);

        check_collision(mag);
        check_hump(pitch);
    }
}
