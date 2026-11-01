/*
 *  task_obs_scan.h
 *  Subsystem 5 -- internal interface to the Obstacle Scanning task (P6).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Kernel side only: included by task_obs_scan.c, task_obs_avoid.c,
 *  subsystem5.c and subsystem5_test.c.  Never included by a drv_*.c
 *  (rule R1).
 *
 *  The task sweeps the servo, pings the ultrasonic sensor at each bearing,
 *  turns a sweep into an obstacle_profile_t and publishes it on
 *  MBF_OBSTACLE (IF-03).  It is the single setter and clearer of
 *  EVF_SAFETY_STATUS_NO_ECHO (docs/DATA_REPRESENTATION.md section 3).
 *
 *  Bearings are whole tens of degrees, 0 (full right) .. 180 (full left),
 *  90 = straight ahead (drv_servo.h).  A sweep is held in an obs_scan_t
 *  indexed by deg / 10, so a fine sweep can reuse the coarse readings
 *  around it.
 */

#ifndef TASK_OBS_SCAN_H
#define TASK_OBS_SCAN_H

#include "subsystem5.h"

/** Bearing slots in a sweep: 0, 10, ... 180 degrees. */
#define OBS_BEARING_SLOTS       (19U)

/** Bearing (degrees) of slot i, and the slot of a bearing. */
#define OBS_SLOT_TO_DEG(i)      ((uint32_t)(i) * 10U)
#define OBS_DEG_TO_SLOT(d)      ((uint32_t)(d) / 10U)

/** Coarse sweep limits: the 5 x 30 degree sweep of design.md runs 30..150. */
#define OBS_COARSE_DEG_MIN      (30U)
#define OBS_COARSE_DEG_MAX      (150U)

/** One sweep's readings.  dist_mm[i] is the filtered range at OBS_SLOT_TO_DEG(i),
 *  or 0 for "no echo / nothing in range".  Only lo_slot..hi_slot were pinged. */
typedef struct
{
    uint16_t dist_mm[OBS_BEARING_SLOTS];
    uint8_t  lo_slot;
    uint8_t  hi_slot;
} obs_scan_t;

/** What a sweep says about the obstacle in front of the car.
 *  Lateral sign convention: positive is to the car's RIGHT. */
typedef struct
{
    uint16_t near_mm;       /**< range at the nearest obstacle bearing */
    uint16_t width_mm;      /**< estimated obstacle width across the track */
    uint16_t clear_l_mm;    /**< free lateral gap left of the obstacle */
    uint16_t clear_r_mm;    /**< free lateral gap right of the obstacle */
    int16_t  centre_x_mm;   /**< lateral centre of the obstacle (+ = right) */
    uint8_t  bearing_deg;   /**< representative bearing: the CENTRE of the
                                 obstacle's angular run, not the bearing that
                                 happened to hold the minimum range.  A wall
                                 spanning 60..120 degrees reports 90. */
    uint8_t  left_deg;      /**< leftmost (largest) obstacle bearing */
    uint8_t  right_deg;     /**< rightmost (smallest) obstacle bearing */
    bool     valid;         /**< false when the sweep found nothing in range */
} obs_result_t;

/*
 * Obstacle Scanning RTOS task entry (TASK_PRI_OBS_SCAN = 6).
 * Drift-free OBS_SERVO_STEP_PERIOD_MS grid via app_period_wait().
 * Never calls tk_ext_tsk() (rule R6).
 */
void obs_scan_task_entry(INT stacd, void *p_exinf);

/** True once obs_scan_task_entry() has started. */
bool obs_scan_is_running(void);

/* ---------------------------------------------------------------------------
 * Pure geometry and analysis.  No kernel calls, no hardware: the on-target
 * tests (S5-U06..S5-U12) drive these with synthetic sweeps.
 * ------------------------------------------------------------------------- */

/** cos and sin of a whole-ten-degree bearing, scaled by 1000 (table lookup). */
int32_t obs_cos_m1000(uint32_t deg);
int32_t obs_sin_m1000(uint32_t deg);

/** Lateral offset of a return, in mm.  Positive is to the car's RIGHT. */
int32_t obs_lateral_mm(uint32_t dist_mm, uint32_t deg);

/** Forward distance of a return, in mm (always >= 0 for 0..180 degrees). */
int32_t obs_forward_mm(uint32_t dist_mm, uint32_t deg);

/** Median of up to three samples, ignoring zeros ("no echo").  0 if all zero. */
uint16_t obs_median3_mm(uint16_t a, uint16_t b, uint16_t c);

/*
 * Turn one sweep into an obstacle description.  Returns false and clears
 * p_out when no bearing in the sweep saw anything inside the detection
 * range; p_out is never read on entry.
 */
bool obs_analyse(obs_scan_t const *p_scan, obs_result_t *p_out);

/** Pack an analysed sweep into the IF-03 wire format (seq wraps, 0..3). */
obstacle_profile_t obs_result_to_profile(obs_result_t const *p_res, uint32_t seq);

/* ---------------------------------------------------------------------------
 * Test read-backs
 * ------------------------------------------------------------------------- */

/** Profiles successfully sent on MBF_OBSTACLE since start-up. */
uint32_t obs_scan_get_profile_count(void);

/** Last profile sent (all-zero before the first one). */
obstacle_profile_t obs_scan_get_last_profile(void);

/** Completed sweeps since start-up (coarse and fine together). */
uint32_t obs_scan_get_sweep_count(void);

/** Copy of the most recently completed sweep. */
void obs_scan_get_last_sweep(obs_scan_t *p_out);

#endif /* TASK_OBS_SCAN_H */
