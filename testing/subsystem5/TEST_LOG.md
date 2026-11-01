# Subsystem 5 — test log

**Subsystem:** Obstacle scanning and avoidance  ·  **Student ID:** 2500596 (Chay Han)  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Hardware set-up before any S5 test

| Item | Requirement |
|---|---|
| Ultrasonic | HC-SR04 on Grove 3: TRIG `GP4`, ECHO `GP5`. **ECHO must pass through a 3.3 V divider** (e.g. 1 kΩ / 2 kΩ) or use a 3.3 V-native module — a bare 5 V echo will damage GP5 (`TEAM.md` section 2, `ARCHITECTURE.md` A8) |
| Servo | Signal on `GP15` (PWM slice 7 B). `GP12`/`GP13`/`GP14` stay unused by S5. Servo powered from the Robo Pico servo rail, not the Pico 3V3 |
| Mounting | Sensor on the servo horn, facing forward with the horn at its centre position (90°). Bearing convention: 0° = full right, 90° = straight ahead, 180° = full left |
| Build | `-DAPP_TEST=ON -DDBG_BACKEND=USB`, console captured with `tools/serial_capture.ps1` |

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Servo angle → pulse width mapping | `SERVO_PULSE_MIN_US`, `SERVO_PULSE_MAX_US` | auto | Call `servo_deg_to_pulse_us()` at 0°, 90°, 180° and at an out-of-range 500° | 1000 µs, 1500 µs, 2000 µs; 500° clamps to 2000 µs |
| S5-U02 | Servo PWM frequency | `SERVO_PWM_FREQ_HZ` = 50 Hz, pin GP15 | auto | After `servo_init()`, read back slice 7 TOP and DIV and compute the frequency | 50 Hz ± 2 Hz |
| S5-U03 | Slice 7 channel A left alone | `TEAM.md` section 2 (GP14 is another owner's spare) | auto | Read back slice 7 channel A compare level after `servo_init()` | Level 0 (GP14 never driven HIGH by S5) |
| S5-U04 | Echo width → distance conversion | 343 m/s, `US_ECHO_MAX_US` | auto | `us_echo_us_to_mm()` with 0, 1160 µs, and `US_ECHO_MAX_US + 1` | 0; 199 mm ± 5 mm; 0 (over-range rejected) |
| S5-U05 | Echo handler registered, line idle | IF-05, `hal_gpio_irq_register()` | auto | After `us_init()`, check the return code and read GP5 | Registered OK; echo line reads LOW (a HIGH means missing divider or unpowered sensor) |
| S5-U06 | cos/sin lookup tables | internal geometry | auto | Check 0°, 90°, 180° in both tables | cos: 1000, 0, −1000. sin: 0, 1000, 0 |
| S5-U07 | Lateral sign convention | internal geometry | auto | `obs_lateral_mm(1000, θ)` at 30°, 90°, 150° | Positive at 30° (right), 0 at 90°, negative at 150° (left) |
| S5-U08 | 3-sample median filter | NFR15, `OBS_PINGS_PER_BEARING` | auto | Feed (0,0,0), (150,0,0), (150,900,0), (900,150,160) | 0; 150; 150 (nearer of two); 160 (outlier 900 rejected) |
| S5-U09 | Empty sweep is not an obstacle | FR6 | auto | `obs_analyse()` on a sweep of all zeros | Returns false, result marked invalid |
| S5-U10 | Single-bearing obstacle | FR6, FR7 | auto | One return of 150 mm at 90° | Valid; range 150 mm; bearing 90°; width > 0 (beam-limited, ≈39 mm) |
| S5-U11 | Wide obstacle span and centring | FR7 | auto | Returns of 200 mm at every bearing 60°…120° | Span 60°…120°; **bearing 90°** (the centre, not the first tied minimum); width ≥ 200 mm |
| S5-U12 | Clearances saturate in open space | IF-03 (`127` = "≥ 635 mm, clear") | auto | One return at 90°, nothing either side | `clear_L` = `clear_R` = 635 mm |
| S5-U13 | A second object cuts one clearance | FR7, FR8 | auto | 150 mm at 90° plus 220 mm at 60° | `clear_R` ≈ 110 mm; `clear_L` stays 635 mm |
| S5-U14 | IF-03 profile round-trip | IF-03, `docs/DATA_REPRESENTATION.md` 2.3 | auto | Analyse a 200 mm return at 90°, pack with sequence 2, unpack | 20 cm, 90°, sequence 2, no-echo false, `clear_L` 635 mm |
| S5-U15 | Out-of-range bearing clamps | IF-03 bearing field (0…12 = 30°…150°) | auto | Pack a result whose bearing is 0° | Index clamps to 0 **and** the OOR bit is set |
| S5-U16 | Invalid result packs as "no echo" | IF-03 (distance 0 = no echo) | auto | `obs_result_to_profile(NULL, 1)` | Distance 0, `drep_obs_no_echo()` true |
| S5-U17 | Side choice: obstacle on the left | FR8, `OBS_BYPASS_CLEAR_MIN_MM` | auto | Profile at bearing 120°, both clearances 635 mm | Chooses **right** (goes the way the obstacle is not) |
| S5-U18 | Side choice: obstacle on the right | FR8 | auto | Profile at bearing 60°, both clearances 635 mm | Chooses **left** |
| S5-U19 | Side choice: only one side fits | FR8 | auto | `clear_L` = 635 mm, `clear_R` = `OBS_BYPASS_CLEAR_MIN_MM` − 5 | Chooses **left** |
| S5-U20 | Blocked when neither side fits | FR8 ("avoiding collisions") | auto | Both clearances `OBS_BYPASS_CLEAR_MIN_MM` − 5 | Returns false → Avoidance reports `BLOCKED` and reverses |
| S5-U21 | Out-leg distance and duration | FR8, `CAR_WIDTH_MM`, `OBS_SIDE_MARGIN_MM` | auto | `obs_leg_out_mm()` at widths 0/100/300; `obs_mm_to_ms()` at 0/500/1000 mm | Both strictly increase with width/distance; `obs_mm_to_ms(0)` = 0 |
| S5-U22 | Power-up sensor check | `subsystem5_sensors_ok()` | auto | Read it after `subsystem5_init()` | True (servo PWM up, echo handler registered, echo line idle) |
| S5-U23 | Both tasks start | `TASK_PRI_OBS_SCAN` (6), `TASK_PRI_OBS_AVOID` (7) | auto | Check both "is running" flags from the test task | Both true |
| S5-U24 | Avoidance starts clean | IF-07 | auto | Read the manoeuvre state and abort counter at start-up | State `OBS_AV_IDLE`, 0 aborts |
| S5-U25 | Coarse sweep completes on the grid | `OBS_SERVO_STEP_PERIOD_MS` (60 ms) | auto | Wait ~2 sweep times, read `obs_scan_get_sweep_count()` | ≥ 1 completed sweep; 0 missed releases logged |
| S5-U26 | Live echo capture | FR6, IF-05 | auto (manual set-up) | **Place a flat object ~15 cm in front of the sensor**, then run the test build | `us_echo_count()` ≥ 1; console shows a `[S5] coarse hit …` line |
| S5-U27 | Servo sweeps the full coarse span | FR7 | manual | Watch the horn during a coarse sweep | Horn steps 30°→150° and back, pausing at 5 bearings, no buzzing or end-stop grinding |
| S5-U28 | Distance accuracy against a tape measure | FR6, NFR15 | manual | Flat target at 10, 15, 20, 25, 30, 40 cm dead ahead; log the reported distance at each | Error ≤ ±1 cm up to 30 cm; record the table and any bias |
| S5-U29 | Detection threshold and confirmation | `OBS_D_DETECT_CM` (25), `OBS_DETECT_CONFIRM_COUNT` (2) | manual | Move a target slowly in from 40 cm | Fine scan starts only after 2 consecutive coarse sweeps see ≤ 25 cm |
| S5-U30 | No-echo status flag | IF-06 b7 `STATUS_NO_ECHO` | manual | Point the sensor at open space for a full sweep, then re-introduce a target | Flag sets after a whole sweep with no echo, clears on the next echo |
| S5-U31 | Servo settle time measurement | `OBS_SERVO_SETTLE_MS` (TUNE) | manual | Command a 30° step, scope or video the horn, time it to rest | Record the measured settle time; promote to `config/thresholds.h` if it differs from 50 ms |
| S5-U32 | 45° bypass turn duration | `OBS_TURN_DWELL_MS` (TUNE) | manual | On the floor, send one `DREP_BYPASS_LEFT` and time the turn to completion | Record the measured time; update `OBS_TURN_DWELL_MS` in `task_obs_avoid.c` |
| S5-U33 | Width estimate against a known box | FR7 | manual | Box of known width at 20 cm; run a fine scan | Reported width ≥ true width, within +50 % (it is required to err wide) |

### Integration tests (with other subsystems)

| ID | What is tested | Needs | Method | Expected result |
|---|---|---|---|---|
| INT-S5-01 | Full bypass on the floor | S2 (motion + VC) | manual | Obstacle on a straight line, car driving; let the manoeuvre run | Car slows, swings out, passes, comes back, within `OBS_AVOID_WORST_CASE_MS` (5 s) |
| INT-S5-02 | Line reacquisition after a bypass | S3 (line following) | manual | As INT-S5-01 on the real track | S3 sets `LINE_FOUND`; S5 sets `LINE_REACQUIRED`, clears `AVOID_ACTIVE`, S3 regains steering |
| INT-S5-03 | Safety abort mid-manoeuvre | S4 (safety) | manual | Trigger `EVF_SAFETY_FAULT_LATCHED` during a bypass | Manoeuvre aborts, car stops, abort counter increments, authority released |
| INT-S5-04 | Blocked obstacle (no way past) | S2 | manual | Obstacle wider than the clear gap both sides | Car stops, reverses, re-scans; never collides |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S5-U06…S5-U21 | `buddy5/obstacle` | longchayhan03-eng | PASS (16/16) | Host stub build, not on target | Pure geometry/decision logic only, compiled with the project's `-Wall -Wextra -Werror`. **Must be re-run on the Pico W** as part of the `APP_TEST` build before these count as passed |

## Open items for the owner

1. **Everything above is desk-verified only.** The logic tests were compiled and run against stub
   headers on a PC; nothing has been on the Pico W. Flash an `APP_TEST` build and re-run S5-U01…S5-U26
   before claiming any of them.
2. **Measure, then promote.** `OBS_TURN_DWELL_MS` (`task_obs_avoid.c`), `OBS_SERVO_SETTLE_MS`,
   `OBS_EDGE_TOL_MM` and `OBS_BEAM_DEG` (`task_obs_scan.c`) are marked TUNE. Measure them
   (S5-U31, S5-U32, S5-U33) and promote the measured values into `config/thresholds.h` /
   `config/timing.h`, the way Subsystem 2 promoted its calibrated constants.
3. **`CAR_WIDTH_MM` is still an assumption** (120 mm in `config/thresholds.h`). Measure the car at
   its widest point, including wheels, and update it — it feeds `OBS_BYPASS_CLEAR_MIN_MM`.
4. **Ask Buddy 2** to confirm `VC_BYPASS_TURN_DEG` (45°) and `VC_BYPASS_REV_CM` (15 cm), which are
   private to `task_vc.c` but mirrored in `task_obs_avoid.c`, or to export them.
5. **Ask Buddy 1** whether Telemetry wants an obstacle event type. `DREP_TEL_TYPE_COUNT` is 5 today
   with no obstacle type, so S5 sends no telemetry. Adding one is a change to shared
   `src/common/data_repr.h` and needs team agreement.
