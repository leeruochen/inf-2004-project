# Subsystem 2 roadmap (Buddy 2, Ruo Chen) - rev 3

> **Canonical copy: `app_program/subsystem2/subsystem2_roadmap.md` (in the repo).** Update it after every completed step, then commit it. A new device or agent: read HANDOVER.md, then this file, and continue at the first step marked NEXT.

## Progress

| Step | State | Notes |
|---|---|---|
| S0 setup | DONE 31 Oct 2026 | Branch `buddy2/motion` from main ce15ff3. Sanity build: 0 warnings, check_rules PASS, robocar.uf2 built. Test cases S2-U01..S2-U28 defined in `testing/subsystem2/TEST_LOG.md`. HANDOVER row updated |
| S1 motor driver | DONE (code + compile) 31 Oct 2026 | `drv_motor.h/.c` created, registered in `sources.cmake`, S2-U01/S2-U02 auto checks added in `subsystem2_test.c`. Both `build/s0b` and `build/test` (`APP_TEST=ON`) pass with 0 warnings, `check_rules: PASS` (72 files). Hardware checks S2-U01..U04 pending on board |
| S2 emergency brake | DONE (code + compile) 31 Oct 2026 | `motion_emergency_brake()` defined in `subsystem2.c` calling `motor_brake_all()`; `motor_are_all_braked()` readback and S2-U06 auto check added (verifies brake both before `motor_init()` and after driving). 0 warnings, `check_rules: PASS` |
| S3 encoder driver | DONE (code + compile) 31 Oct 2026 | `quadrature_encoder.pio` (24 instructions, `.origin 0`, BSD-3-Clause header preserved) added; `drv_encoder.h/.c` implemented on PIO0 (SM0 right GP0/1, SM1 left GP16/17, non-blocking `enc_get_count()`); `pico_generate_pio_header` added in `CMakeLists.txt` (shared file: notify team); S2-U09 auto check added. Hardware checks S2-U07..U11 (`ENC_x_DIR_INVERT`, `ENC_CPR`, `WHEEL_CIRC_MM`) pending on board |
| S4 Motion task | DONE (code + compile) 31 Oct 2026 | `task_motion.h/.c` implemented (P3, 20 ms `app_period_wait()`, `MBF_MOTION_CMD` poll + `drep_mc_valid()`, `MOTION_T_HOLD_MS` = 100 ms hold, `MOTION_SPEED_MAX_CMPS` = 50 cm/s cap, `EVF_SAFETY` latch stop, PID + straight-line correction, `MTX_MOTION` + `motion_get_state()` publish, `#error` guards on `ENC_CPR`/`WHEEL_CIRC_MM`). Auto checks S2-U12..U16 added in `subsystem2_test.c`. 0 warnings, `check_rules: PASS` (75 files) |
| S5 Vehicle Controller | DONE (code + compile) 31 Oct 2026 | `task_vc.h/.c` implemented (P2, sole reader of `MBF_NAV_IN`, `drep_nav_valid()`, arbitration Safety > Avoid > Barcode > Line, `EVF_SAFETY` `tk_ref_flg` & `FAULT_LATCHED` reset only after `SAFETY_T_STILL_MS` = 500 ms at zero speed, 10-state 4-bit System FSM `VC_SYS_*` in `subsystem2.h`, `DREP_TEL_STATE` telemetry events). Auto checks S2-U22..U27 added in `subsystem2_test.c`. 0 warnings, `check_rules: PASS` (77 files) |
| S6 startup and API | NEXT | see Q3 |
| S7 tests and reports | not started | |

Build note (Windows): the sandbox cannot run the SDK compiler (illegal DLL relocation); run the HANDOVER section 5 build outside the sandbox. Build dirs: `build/s0b` is the S0 sanity build (gitignored).


Checked against HANDOVER.md, specs.md, TEAM.md, ARCHITECTURE.md, `docs/DATA_REPRESENTATION.md` and the repo headers.
Folders you may edit: `app_program/subsystem2/`, `testing/subsystem2/`. Anything else (CMakeLists.txt, TEAM.md, config/*.h) needs team agreement.

## Open decisions (settle before or during the step shown)

| # | Decision | Proposal | Step |
|---|---|---|---|
| Q1 | Who owns the 10-state system FSM? No document names an owner. Analysis: VC is the only task that sees every transition input (remote START/STOP/RESET, barcode turns, AVOID/RELEASE, TRACK_LOST, FAULT_LATCHED) and already holds authority and clears the latch | DECIDED (Ruo Chen, pending team notice): VC owns it as a private 4-bit state, and Telemetry reads a published copy for the heartbeat | S5 |
| Q2 | Motion period | DECIDED: `app_period_wait()` (HANDOVER is latest). Remove the cyclic-handler TODO. TEAM.md section 6 budgets 1 cyclic for S2: tell the team | S4 |
| Q3 | specs.md "Required APIs" (moveForward, ...) | OPEN (found in S0): `nav_input_t` cannot carry a distance or angle (4 sources, full 2-bit field, final since M2). Option (a, recommended): the API is the motion library, each call sends `drep_mc_pack(...)` to `MBF_MOTION_CMD`; in the car only VC calls it. Option (b): extend `nav_input_t`, needs team agreement | S6 |
| Q4 | Motor drive truth table (PWM on one input, other LOW) | DECIDED: confirm on the board in S1; HANDOVER only states brake = both LOW, coast = both HIGH | S1 |

## Steps

### S0 - Setup (no hardware)
- Pull `main`, branch `buddy2/motion`. Run the section 5 sanity build (0 warnings, `check_rules: PASS`).
- Fill `testing/subsystem2/TEST_LOG.md` with planned IDs `S2-U01...` before writing each `TEST_CHECK`.

### S1 - Motor driver `drv_motor.h/.c` (SDK headers only, never `subsystem2.h`)
- PWM slices 4 (GP8/9) and 5 (GP10/11) at `MOTOR_PWM_FREQ_HZ`; duty capped at `MOTOR_DUTY_MAX_PCT`.
- `MOTOR_L/R_DIR_INVERT`; brake = both LOW, coast = both HIGH.
- Register in `ROBOCAR_SOURCES`.
- Hardware: wheels off the ground, check direction, set `MOTOR_x_DIR_INVERT`. Settle Q4.

### S2 - `motion_emergency_brake()` (IF-11)
- All four channels LOW inside a critical section. Must be callable from Safety (P1) and work without Motion running.
- Test: call it while driving, wheels stop.

### S3 - Encoder driver `drv_encoder.c`
- Copy `quadrature_encoder.pio` from pico-examples tag `sdk-1.5.1` (keep header; NOTICE.md already covers it). It is not in the repo yet.
- PIO0 offset 0, SM0 right (GP0/1), SM1 left (GP16/17). Non-blocking `enc_get_count()` with `ENC_x_DIR_INVERT`.
- Add `pico_generate_pio_header` to CMakeLists.txt (shared file, notify team).
- Hardware: hand-spin then forward-drive, set `ENC_x_DIR_INVERT`.
- Measure `ENC_CPR` and `WHEEL_CIRC_MM` (both 0 now). Code using them must `#error` while 0.

### S4 - Motion task `task_motion.c` (P3, kernel headers only)
- `app_period_init`/`app_period_wait` at `MOTION_PERIOD_MS`; log missed releases through `dbg_`.
- Poll `MBF_MOTION_CMD` with `TMO_POL`; validate with `drep_mc_valid()`.
- Per-wheel PID on encoder deltas (two's-complement, wrap-safe); straight-line correction between wheels.
- Speed and distance estimation; encoder-based turns (angle to count difference).
- Hold last setpoint for `MOTION_T_HOLD_MS`, then stop. Speed capped at `MOTION_SPEED_MAX_CMPS` (NFR2).
- Publish `drep_ms_pack()` under `MTX_MOTION` and the counts (single writer); set/clear FAULT_STALL and FAULT_ENCODER bits.
- A task ends by returning, never `tk_ext_tsk` (R6).

### S5 - Vehicle Controller `task_vc.c` (P2)
- Sole reader of `MBF_NAV_IN`; drop anything failing `drep_nav_valid()` and count it.
- Arbitrate LINE / BARCODE / AVOID / REMOTE (design.md section 3) into one `motion_cmd_t` sent with `TMO_POL`.
- Read `EVF_SAFETY` with `tk_ref_flg` only (the flag is `TA_WSGL`). On FAULT_LATCHED, command stop.
- On remote RESET while stationary for `SAFETY_T_STILL_MS`, `tk_clr_flg(EVF_SAFETY, ~EVF_SAFETY_FAULT_LATCHED)`.
- System FSM (Q1): INITIALISING -> READY (self-test passed) -> AUTONOMOUS_NAVIGATION (remote START) with sub-states LINE_TRACKING, HUMP_SENSING, TURN_LEFT/RIGHT, GO_STRAIGHT, U_TURN; OBSTACLE_AVOIDANCE/BYPASS (AVOID_ACTIVE, back only via LINE_REACQUIRED); TRACK_LOST or STOP -> READY; FAULT_LATCHED -> SAFETY_COLLISION_LATCHED_FAULT, left only by RESET while stationary. States encoded as 4 bits through a drep_ accessor. Sources: design.md section 5.

### S6 - Startup and public API
- `subsystem2_init()`: nothing to create unless Q2 changes. `subsystem2_start()`: `app_start_task` for VC and Motion.
- Wrapper API in `subsystem2.h` (Q3): `moveForward(cm)`, `moveBackward(cm)`, `turnLeft(deg)`, `turnRight(deg)`, `stop()`, built on `drep_nav_*`/`drep_mc_pack` (C naming per Barr C).
- Update HANDOVER section 1 and 3 row, TEAM.md only by agreement.

### S7 - Tests and deliverables
- `subsystem2_test.c` `TEST_CHECK`s, one ID per case in TEST_LOG.md; results rows never edited.
- NFR2 count test: drive a measured distance at 50 cm/s, compare counts on both wheels (ARCHITECTURE.md D7).
- Deliverables from specs.md: motion control library, **PID tuning report**, **motion accuracy evaluation** (distance error and turn repeatability), saved in `testing/subsystem2/`.
- Measure `MOTOR_DUTY_MAX_PCT` and verify `POWER_GROVE_3V3_BUDGET_MA`.

## Rules to keep in mind
R1 no mixing kernel and SDK headers in one `.c`; R2 no blocking SDK calls in tasks; R6 no `tk_ext_tsk`; no bit-fields; use `drep_` functions for every message; every `.c` listed in `sources.cmake`; build must have 0 warnings before each push.

## Suggested order
S0, then S1 + S2 (motors safe first), S3, S4, S5, S6, with S7 running alongside each step. Start with S1.


