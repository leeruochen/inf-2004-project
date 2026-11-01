/*
 *  subsystem5.c
 *  Subsystem 5 -- obstacle scanning and avoidance: brings up the servo and
 *  the ultrasonic sensor, creates its objects and starts its tasks.
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Two-phase start (app.h): subsystem5_init() runs after ipc_init(), so
 *  SEM_ECHO already exists and can be handed to the SDK-side echo handler.
 *  Neither task needs a cyclic handler: Obstacle Scanning keeps its 60 ms
 *  grid with app_period_wait() (HANDOVER.md section 3 step 7) and Obstacle
 *  Avoidance is driven by MBF_OBSTACLE with a timeout.
 */

#include "subsystem5.h"

#include "config/thresholds.h"
#include "drv_servo.h"          /* kernel-free driver interface */
#include "drv_ultrasonic.h"     /* kernel-free driver interface */
#include "task_obs_avoid.h"
#include "task_obs_scan.h"

/** Tolerance on the servo PWM frequency read-back, in Hz. */
#define S5_PWM_FREQ_TOL_HZ      (2U)

static bool s_sensors_ok = false;

bool subsystem5_sensors_ok(void)
{
    return s_sensors_ok;
}

uint32_t subsystem5_get_avoid_state(void)
{
    return obs_avoid_get_state();
}

bool subsystem5_avoid_is_active(void)
{
    return obs_avoid_get_state() != OBS_AV_IDLE;
}

int32_t subsystem5_init(void)
{
    /* 1. Servo: park the horn straight ahead before anything can move. */
    servo_init();

    uint32_t const freq = servo_get_pwm_freq_hz();
    bool const     servo_ok = servo_is_ready()
                              && (freq >= ((uint32_t)SERVO_PWM_FREQ_HZ - S5_PWM_FREQ_TOL_HZ))
                              && (freq <= ((uint32_t)SERVO_PWM_FREQ_HZ + S5_PWM_FREQ_TOL_HZ))
                              && servo_chan_a_is_low();

    if (!servo_ok)
    {
        dbg_printf("[S5] servo PWM %lu Hz, expected %lu Hz",
                   (unsigned long)freq, (unsigned long)SERVO_PWM_FREQ_HZ);
    }

    /* 2. Ultrasonic sensor and its echo handler on IO_IRQ_BANK0. */
    int32_t const us_er = us_init();

    APP_CHECK(us_er, "s5 us_init");

    /*
     * 3. An echo line that is already HIGH means the sensor is unpowered,
     *    miswired, or wired straight to 5 V without the divider that GP5
     *    needs (TEAM.md section 2, ARCHITECTURE.md A8).  Report it loudly:
     *    every range reading would otherwise be silently wrong.
     */
    bool const echo_idle = !us_echo_pin_is_high();

    if (!echo_idle)
    {
        dbg_puts("[S5] echo line stuck HIGH: check wiring and the 3.3 V divider");
    }

    /* 4. Let the SDK-side echo handler signal IF-05.  SEM_ECHO is valid
     *    because ipc_init() ran in usermain step 1 (app.h). */
    us_set_echo_sem((int32_t)SEM_ECHO);

    obs_avoid_init();

    s_sensors_ok = servo_ok && (us_er >= (int32_t)E_OK) && echo_idle;

    if (!s_sensors_ok)
    {
        dbg_puts("[S5] sensor check FAILED");
    }

    return (int32_t)E_OK;
}

int32_t subsystem5_start(void)
{
    int32_t const scan_id = app_start_task(obs_scan_task_entry,
                                           TASK_PRI_OBS_SCAN,
                                           TASK_STACK_OBS_SCAN_BYTES,
                                           "obs_scan");
    if (scan_id < (int32_t)E_OK)
    {
        return scan_id;
    }

    int32_t const avoid_id = app_start_task(obs_avoid_task_entry,
                                            TASK_PRI_OBS_AVOID,
                                            TASK_STACK_OBS_AVOID_BYTES,
                                            "obs_avoid");
    if (avoid_id < (int32_t)E_OK)
    {
        return avoid_id;
    }

    return (int32_t)E_OK;
}
