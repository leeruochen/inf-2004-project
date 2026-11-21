/*
 *  imu_hump.h
 *  Subsystem 4 -- hump detector and height estimator (pure C).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_imu_hump.c builds it unchanged.  The IMU
 *  Processing task calls imu_hump_step() once per 10 ms sample with the
 *  pitch and the distance the wheels moved; it sends the telemetry.
 *
 *  Requirement: FR11 (detect humps, estimate the height from pitch and
 *  encoder distance, report the highest peak), NFR11 (height within +-20 %
 *  or +-5 mm of the measured height).  Demo rubric: "measure hump height".
 *
 *  How the height is estimated.  The car touches the ground at two points,
 *  the driven wheels and the front contact, IMU_HUMP_WHEELBASE_MM apart.
 *  Two estimates, the larger is reported:
 *    TILT   wheelbase x sin(largest |pitch| held for IMU_HUMP_HOLD_SAMPLES).  Exact while one end is on
 *           the hump and the other on the floor: a short hump (shorter than
 *           the car), or a book under the front wheels while standing still.
 *    CLIMB  the highest point of sum(sin(pitch) x distance), design.md A6.
 *           Exact for a long hump (longer than the car) with a flat top,
 *           where both ends are on it at once and the pitch drops back to
 *           the ramp angle.  For a short hump it reads low (about the
 *           hump's cross-section area / wheelbase), so TILT wins there.
 *  Both can only read low, never high, on a hump of any shape: TILT because
 *  one end cannot rise more than the hump above the other, CLIMB because
 *  the car never climbs higher than the top.
 *
 *  Double-integrating vertical acceleration was rejected in design.md
 *  (drift about 1.2 cm per 0.5 s at 0.01 g bias).
 *
 *  One hump = one report.  A hump starts after IMU_HUMP_CONFIRM_SAMPLES
 *  above IMU_HUMP_PITCH_DEG and ends once the car has been level again for
 *  a wheelbase of travel, or for IMU_HUMP_END_SAMPLES standing still.  So a
 *  short hump (front wheels up, level, rear wheels up) is one report, not
 *  two.  Samples taken while the car turns (the wheels moving against each
 *  other) are not used: centripetal acceleration tilts the accelerometer by
 *  up to ~11 deg in an in-place turn (S4 log, 13 Nov 2026).  A hump that
 *  overlaps a turn is dropped.
 */

#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <stdbool.h>
#include <stdint.h>

#include "config/thresholds.h"

/** Samples above the hump pitch before a hump starts.  The 10 mm track
 *  hump shows for only ~2 samples above 3 deg (S4-U26 traces, 19 Nov
 *  2026), so one is enough; start/stop and turns are gated out instead. TUNE */
#define IMU_HUMP_CONFIRM_SAMPLES    (1U)
/** Level samples without wheel travel that end a hump when the car has
 *  stopped (0.5 s). */
#define IMU_HUMP_END_SAMPLES        (50U)
/** TILT uses only a pitch HELD this many samples in a row (the smallest of
 *  the window, same sign).  1 = the peak sample itself: replaying the two
 *  S4-U26 traces of the 10 mm hump (19 Nov 2026) gives 9 and 10 mm with 1,
 *  5 and 8 mm with 2.  Raise it if noise spikes set false heights. TUNE */
#define IMU_HUMP_HOLD_SAMPLES       (1U)
/** Entry and exit pitch, half-degrees.  Exit stays above +-2 deg (4 half-deg)
 *  cruise noise so flat-floor vibration does not keep resetting level_um. */
#define IMU_HUMP_ENTRY_HALF_DEG     ((int32_t)IMU_HUMP_PITCH_DEG * 2)
#define IMU_HUMP_EXIT_HALF_DEG      ((((IMU_HUMP_ENTRY_HALF_DEG * 2) / 3) < 5) ? 5 : ((IMU_HUMP_ENTRY_HALF_DEG * 2) / 3))

typedef enum
{
    IMU_HUMP_NONE = 0,      /**< nothing new */
    IMU_HUMP_START,         /**< a hump started (no height yet) */
    IMU_HUMP_END,           /**< a hump ended; the report is filled in */
    IMU_HUMP_DROPPED        /**< a hump ended but overlapped a turn; no report */
} imu_hump_event_t;

typedef struct
{
    bool     active;        /**< inside a hump (confirmed) */
    bool     turned;        /**< the car turned during this hump */
    uint32_t confirm;
    uint32_t level_n;       /**< level samples since the pitch dropped */
    int32_t  level_um;      /**< level travel since the pitch dropped, um */
    int32_t  climb_um;      /**< sum(sin(pitch) x distance), um */
    int32_t  climb_peak_um;
    int32_t  spike_pitch;   /**< instantaneous pitch with the largest |value| */
    int32_t  peak_pitch;    /**< held pitch with the largest |value|, half-deg */
    int16_t  hold[IMU_HUMP_HOLD_SAMPLES];   /**< last pitches (kept across humps) */
    uint32_t hold_n;
    uint32_t hold_i;
} imu_hump_t;

typedef struct
{
    int32_t height_mm;      /**< max(tilt_mm, climb_mm) */
    int32_t tilt_mm;        /**< wheelbase x sin(|peak held pitch|) */
    int32_t climb_mm;       /**< highest point of the climb integral */
    int32_t pitch_half_deg; /**< peak held pitch, signed (positive = nose up) */
    int32_t spike_half_deg; /**< peak instantaneous pitch, signed (for the log) */
} imu_hump_report_t;

/** sin of an angle in half-degrees, x 10000 (integer table, no FPU). */
int32_t imu_hump_sin_x10000(int32_t half_deg);

/** Encoder counts (X4) to micrometres of wheel travel. */
int32_t imu_hump_counts_to_um(int32_t counts);

void imu_hump_init(imu_hump_t *p_hump);

/**
 * One 10 ms sample.
 *   pitch_half_deg  pitch from level, half-degrees, positive = nose up
 *   ds_um           wheel travel since the last sample, um (negative = reverse)
 *   turning         true if the wheels moved against each other this sample
 *                   (or the car is lifted): a hump in progress is dropped
 *   speed_change    true while the car speeds up or slows down (and shortly
 *                   after): the accelerometer then reads the acceleration as
 *                   pitch (+13 deg pulling away, -13 deg braking at 22 cm/s,
 *                   S4-U26 trace, 19 Nov 2026), so the sample is ignored
 *   p_report        filled in on IMU_HUMP_END (may be NULL)
 */
imu_hump_event_t imu_hump_step(imu_hump_t *p_hump, int32_t pitch_half_deg, int32_t ds_um,
                               bool turning, bool speed_change, imu_hump_report_t *p_report);

#endif /* IMU_HUMP_H */
