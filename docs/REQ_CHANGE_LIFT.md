# Requirement change: IMU lift/overturn detection replaces collision detection

**Date of instruction:** Wed 11 Nov 2026
**Changed by:** Buddy 4 (Vincent, `2500709`), branch `buddy4/lift-detection`
**Affects:** FR12, NFR10, NFR12, design.md 3.1/3.2/5/6.4/A4/A7, IF-06 bit b0

## The instruction

Prof Fauzi agreed that collision avoidance (obstacle bypass, FR8) and IMU
collision detection conflict, and simplified the requirement:

- **Collision detection using the IMU is removed.**
- The IMU must instead detect when the robot has been **lifted or overturned**.
  When that happens, the motors must stop immediately.
- At the demonstration, **lifting the robot signals the end of the run**, so the
  robot must detect it and stop all motor movement.
- Implementation and documentation are to be updated accordingly.

## What changed

| Area | Before | After |
|---|---|---|
| Detection (`app_program/subsystem4/`) | `check_collision()` in `task_imu.c`: raw magnitude >= 2.5 g | New pure module `imu_lift.c/.h`, called from `task_imu.c` every 10 ms on the low-passed vector |
| Event flag b0 (`src/common/ipc_types.h`) | `EVF_SAFETY_COLLISION` | `EVF_SAFETY_LIFTED` (same bit, same single setter, same Safety wait mask) |
| Safety task | Brake + FAULT_LATCHED on COLLISION | Brake + FAULT_LATCHED on LIFTED (unchanged path, new trigger) |
| Thresholds (`config/thresholds.h`) | `IMU_G_COLLISION_X1000` 2.5 g | `IMU_LIFT_TILT_DEG` 30 deg (`IMU_LIFT_COS_TILT_X1000` 866), `IMU_LIFT_ACCEL_G_X1000` 0.3 g |
| Subsystem 5 (`task_obs_avoid.c`) | Aborts a bypass on COLLISION or FAULT_LATCHED | Aborts on LIFTED or FAULT_LATCHED (identifier rename only) |
| Dashboard (`tools/dashboard/tel_codec.py`) | bit 0 named COLLISION | bit 0 named LIFTED |
| Tests | S4-U13 (knock the car) | S4-U13 dropped; S4-U17 (no false lift at rest), S4-U18..U20 (lift, re-arm, tip over, operator-guided), S4-U21 (host), S4-U22 (no false lift over humps, on track); SYS-06 redefined |

## How the detector works

The accelerometer has no gyro, so a car held still and level in the air reads
1 g, the same as a car on the floor. What the IMU can see is the act of
picking it up or turning it over. At boot, the IMU task already takes a level
reference once the car has been flat and still for 0.5 s; the same window now
also stores the gravity vector as "up". Every 10 ms, the low-passed (80 ms)
acceleration is compared with "up":

- **Tilted** more than 30 deg in any direction. People almost always tilt a car
  they pick up, and an overturned car is past 90 deg. Humps tilt the car only
  about 10-15 deg (S4 run, 13 Nov 2026), so they stay well below.
- **Raised**: acceleration along "up" more than 0.3 g above rest, for a lift
  that stays level.

Five suspicious samples in a row (50 ms) give one event: the IMU task sets
`EVF_SAFETY_LIFTED` and logs `[IMU] LIFTED (tilted|overturned|raised)`. The
P1 Safety task brakes the motors and sets FAULT_LATCHED. The detector then
disarms until the car has stood level and still for 0.5 s, so one pick-up is
one event. Because lifting ends the run, the car stays stopped until a remote
`car/cmd/reset` while stationary (unchanged Vehicle Controller rule).

## Points for the team

- **Carrying the car after power-on also latches the fault.** Power the car on
  where it will start, or send `car/cmd/reset` after placing it.
- The lift detector is off for the first moments after boot, until the level
  reference is taken (needs the car flat and still for 0.5 s; falls back to the
  board's Z axis after 10 s).
- Thresholds are initial values (TUNE). S4-U22 and design.md A4 check that
  driving over humps never triggers a false stop.
- Edits outside Subsystem 4 (`src/common/ipc_types.h`, `config/*.h`,
  `TEAM.md`, Subsystem 5's `task_obs_avoid.c` / `subsystem5.h`, the dashboard)
  are an identifier rename and comments only, and need the team's agreement
  (HANDOVER.md section 0, rule 5) before this branch is merged.
