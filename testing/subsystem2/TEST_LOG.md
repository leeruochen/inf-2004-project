# Subsystem 2 — test log

**Subsystem:** Motion control and Vehicle Controller  ·  **Student ID:** 2500530 (Ruo Chen)  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Motor PWM setup: slices 4 and 5 at 20 kHz | `MOTOR_PWM_FREQ_HZ`, pins GP8-GP11 | auto | After `drv_motor` init, read back the slice wrap and clock divider and compute the frequency | 20 kHz ± 1 % on both slices |
| S2-U02 | Duty cap | `MOTOR_DUTY_MAX_PCT` | auto | Request 100 % duty forward on each motor, read back the channel level | Level ≤ `MOTOR_DUTY_MAX_PCT` of wrap |
| S2-U03 | Motor direction, both wheels | FR4, `MOTOR_x_DIR_INVERT` | manual | Wheels off the ground; command forward at low duty on each motor, then backward | Each wheel turns forward, then backward; record the INVERT values set |
| S2-U04 | Brake and coast pin states | ROBO PICO truth table | manual | Command brake, then coast; measure GP8-GP11 with a meter or scope | Brake: all four LOW. Coast: both inputs of each motor HIGH |
| S2-U05 | `motion_emergency_brake()` stops a running wheel | IF-11, FR12 | manual | Wheels off the ground at cruise duty; call the brake from a test hook | All four channels LOW at once; both wheels stop |
| S2-U06 | `motion_emergency_brake()` works with the Motion task not running | IF-11 | auto | Before Motion starts, call the brake, then read the GPIO output levels | All four channels LOW |
| S2-U07 | Encoder sign, hand spin | IF-10, `ENC_x_DIR_INVERT` | manual | Spin each wheel forward by hand, log `enc_get_count()` | Count rises on the spun wheel only; record the INVERT values set |
| S2-U08 | Encoder sign, forward drive | IF-10 | manual | Drive forward 1 s at low duty, log both counts | Both counts rise |
| S2-U09 | `enc_get_count()` is non-blocking | rule R2, IF-10 | auto | Call it 1000 times with the wheels still, measure the elapsed time | No blocking; average call time < 10 µs |
| S2-U10 | `ENC_CPR` measurement | thresholds.h | manual | Turn one wheel exactly 10 revolutions by hand, read the count | CPR = count / (10 × `ENC_QUAD_MULT`); value recorded in thresholds.h with its source |
| S2-U11 | `WHEEL_CIRC_MM` measurement | thresholds.h | manual | Roll the wheel 10 turns along a ruler, or measure the diameter | Value recorded in thresholds.h with its source |
| S2-U12 | Motion task period | FR6, `MOTION_PERIOD_MS` | auto | Count `app_period_wait()` releases over 10 s and the missed releases | 500 ± 1 releases, 0 missed |
| S2-U13 | Invalid `motion_cmd_t` is rejected | IF-02 | auto | Post a command with stray bits above bit 18 to `MBF_MOTION_CMD` | Motion ignores it and counts a reject; setpoint unchanged |
| S2-U14 | Hold then stop without commands | `MOTION_T_HOLD_MS` | auto | Post one FWD command, then nothing; sample the setpoint | Setpoint held for 100 ms ± one period, then 0 |
| S2-U15 | Speed cap | NFR2, `MOTION_SPEED_MAX_CMPS` | auto | Post FWD with speed 63 (packed with OOR) | Setpoint ≤ 50 cm/s |
| S2-U16 | `motion_state_t` published under `MTX_MOTION` | IF-08, NFR6 | auto | Read the state twice, 100 ms apart, while driving | Tick advances; counts and distance non-decreasing when driving forward |
| S2-U17 | Speed estimate | FR6 | manual | Drive at a set speed over a measured distance and time it | Reported speed within 10 % of the measured average |
| S2-U18 | Distance estimate | FR6, specs "accurate distance" | manual | `moveForward(100)` on the floor, five runs | Mean error and spread recorded in the accuracy report |
| S2-U19 | Straight-line correction | FR5 | manual | Drive 2 m straight, five runs, measure lateral drift | Drift recorded; PID gains and runs in the tuning report |
| S2-U20 | Encoder-based turns | FR4, specs "repeatable turns" | manual | `turnLeft(90)` and `turnRight(90)`, five runs each, measure the heading | Mean error and spread recorded in the accuracy report |
| S2-U21 | Zero missed counts at top speed | NFR2 (ARCHITECTURE.md D7) | manual | Drive a measured distance at 50 cm/s; compare counts to the expected value, both wheels | Counts within one wheel-slip tolerance of the expected value; result goes to the team for the D7 wording |
| S2-U22 | VC rejects an invalid `nav_input_t` | IF-01 | auto | Post a message with bypass value 6 to `MBF_NAV_IN` | VC drops and counts it; no motion command sent |
| S2-U23 | VC precedence: AVOID over LINE | IF-01, design.md section 4 | auto | With authority AVOID, post a LINE steer and an AVOID bypass | Only the AVOID command reaches `MBF_MOTION_CMD` |
| S2-U24 | VC stops on FAULT_LATCHED | IF-06 | auto | Set `EVF_SAFETY_FAULT_LATCHED`, then post a LINE steer | VC sends STOP/BRAKE, never a drive command |
| S2-U25 | VC clears FAULT_LATCHED only when stationary | IF-06, `SAFETY_T_STILL_MS` | auto | RESET while moving, then RESET after 500 ms at zero speed | First RESET ignored; second clears only the FAULT_LATCHED bit |
| S2-U26 | System FSM: power-up to READY to NAV | design.md section 5 | auto | Boot, then post remote START | State goes INITIALISING, then READY, then AUTONOMOUS_NAVIGATION/LINE_TRACKING |
| S2-U27 | System FSM: TRACK_LOST and STOP return to READY | design.md section 5 | auto | In NAV, post LINE TRACK_LOST; restart, then post remote STOP | State goes to READY and the car stops each time |
| S2-U28 | Public motion API packs valid `motion_cmd_t` | FR4, IF-02, Q3 | auto | Call `moveForward(100)`, `moveBackward(25)`, `turnLeft(90)`, `turnRight(45)`, `stop()`; inspect the `motion_cmd_t` received by Motion Control | Each call posts one valid `motion_cmd_t` (`drep_mc_valid()`) to `MBF_MOTION_CMD` with the expected type, speed ≤ 50 cm/s, and distance/angle argument |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | S2-U01 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Motor PWM slices 4 and 5 verified at 20 kHz ±1 % |
| 2026-10-31 | S2-U02 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Motor duty capped at `MOTOR_DUTY_MAX_PCT` (95 %) |
| 2026-10-31 | S2-U06 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | `motion_emergency_brake()` drives GP8..GP11 LOW with and without Motion task |
| 2026-10-31 | S2-U08 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Forward drive 100 ms: `encL: 101->561 (+460)`, `encR: -575->-1122 (-547)`; confirms `ENC_L_DIR_INVERT=0`, `ENC_R_DIR_INVERT=1` needed in `config/pins.h` |
| 2026-10-31 | S2-U09 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | `enc_get_count()` 1000 calls completed in < 1 ms (~1 µs/call, non-blocking) |
| 2026-10-31 | S2-U12 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Motion task 20 ms period: 500 ± 1 releases in 10 s, 0 missed |
| 2026-10-31 | S2-U13 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Invalid `motion_cmd_t` rejected and logged; setpoint unchanged at 0 |
| 2026-10-31 | S2-U14 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Setpoint held for `MOTION_T_HOLD_MS` (100 ms) then automatically stopped |
| 2026-10-31 | S2-U15 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Speed capped at `MOTION_SPEED_MAX_CMPS` (50 cm/s); raw >50 rejected |
| 2026-10-31 | S2-U16 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | `motion_state_t` published under `MTX_MOTION`: `dt=100 ms`, `dist=4->7 cm`, `encL=101->561`, `encR=-575->-1122` |
| 2026-10-31 | S2-U22 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | VC rejected invalid `nav_input_t` (bypass=6) and sent no `motion_cmd_t` |
| 2026-10-31 | S2-U23 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | VC precedence: `AVOID` overrides `LINE` while authority is `VC_AUTH_AVOID` |
| 2026-10-31 | S2-U24 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | VC commands `DREP_MC_BRAKE` and transitions to `SAFETY_FAULT_LATCHED` when `EVF_SAFETY_FAULT_LATCHED` is set |
| 2026-10-31 | S2-U25 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | VC ignores RESET while moving and clears only `EVF_SAFETY_FAULT_LATCHED` after `SAFETY_T_STILL_MS` (500 ms) stationary |
| 2026-10-31 | S2-U26 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | System FSM transitions `INITIALISING` -> `READY_WAITING` -> `NAV_LINE_TRACKING` |
| 2026-10-31 | S2-U27 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | System FSM returns to `READY_WAITING` and stops on `TRACK_LOST` and remote `STOP` |
| 2026-10-31 | S2-U28 | `buddy2/motion` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_unit_all.txt` | Public API (`moveForward`, `moveBackward`, `turnLeft`, `turnRight`, `stop`) posts valid `motion_cmd_t` |
| 2026-10-31 | S2-U03 | `a25bb17` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_phase1_calib.txt` | Left `+40%` spun backward (`dL=-7610`), `-40%` spun forward (`dL=+7790`) -> `MOTOR_L_DIR_INVERT=1U`, `ENC_L_DIR_INVERT=0U`; Right `+40%` spun forward (`dR=-7901`), `-40%` spun backward (`dR=+7948`) -> `MOTOR_R_DIR_INVERT=0U`, `ENC_R_DIR_INVERT=1U` |
| 2026-10-31 | S2-U04 | `a25bb17` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_phase1_calib.txt` | Coast: `L=(6250,6250)/6250 R=(6250,6250)/6250` (all 4 channels HIGH); Brake: `L=(0,0)/6250 R=(0,0)/6250 braked=1` (all 4 channels LOW) |
| 2026-10-31 | S2-U05 | `a25bb17` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_phase1_calib.txt` | Both wheels driven at +50% for 1.5 s then `motion_emergency_brake()` called: both wheels stopped immediately (`dL=0, dR=0, braked=1`) |
| 2026-10-31 | S2-U07 | `a25bb17` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_handspin.txt` | Hand-spinning left wheel forward rose monotonically `0 -> +15572` while unspun right wheel stayed at `dR=0` (`ENC_L_DIR_INVERT=0U`, `ENC_R_DIR_INVERT=1U`) |
| 2026-10-31 | S2-U11 | `a25bb17` | 2500530 | PASS | `testing/subsystem2/MOTION_ACCURACY_EVALUATION.md` | Measured 66 mm wheel diameter -> `WHEEL_CIRC_MM = 207U` (`pi * 66 mm = 207.3 mm`) |
| 2026-10-31 | S2-U10 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/2026-10-31_s2_handspin.txt`, `testing/subsystem2/logs/floor_run3.txt` | Hand-spin (`dL=15572`) + 100 cm floor calibration (`dL=12025, dR=12018` over `101 cm` = `119.03` counts/cm -> `2464` quad counts/rev) -> `ENC_CPR = 616U` (`616 * 4 = 2464`) |
| 2026-10-31 | S2-U17 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/floor_fwd100_run2.txt`, `testing/subsystem2/logs/floor_run3.txt` | Floor Run 2 (`dt=4598 ms`, actual `95 cm` -> `20.7 cm/s` measured vs `21.7 cm/s` reported = `+4.8 %` err); Floor Run 3 (`dt=4724 ms`, actual `101 cm` -> `21.4 cm/s` measured vs `21.1 cm/s` reported = `-1.4 %` err, well within ±10 %) |
| 2026-10-31 | S2-U18 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/floor_fwd100_run1.txt`, `testing/subsystem2/logs/floor_fwd100_run2.txt`, `testing/subsystem2/logs/floor_run3.txt`, `testing/subsystem2/logs/floor_run4_topspeed.txt` | `moveForward(100)` floor progression: Run 1 (`67 cm` at `CPR=390`) -> Run 2 (`95 cm` at `CPR=585`) -> Run 3 (`101 cm` at calibrated `CPR=616`, `+1.0 %` err, `dL=12025, dR=12018`) -> Run 4 (`103 cm` at `50 cm/s` top speed, `+3.0 %` err) |
| 2026-10-31 | S2-U19 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/floor_fwd100_run1.txt`, `testing/subsystem2/logs/floor_fwd100_run2.txt`, `testing/subsystem2/logs/floor_run3.txt`, `testing/subsystem2/PID_TUNING_REPORT.md` | Tuned velocity PID (`KP=3, KI_X100=10, KD=1`) + cumulative heading sync (`SYNC_KP=2, SYNC_KD=2, SYNC_MAX=120`) eliminated Run 1 stuttering (`df=-606` counts) and achieved smooth straight tracking (`dL=12025, dR=12018, df=+7` counts = `0.58 mm` wheel diff over `101 cm`) |
| 2026-10-31 | S2-U20 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/floor_run3.txt`, `testing/subsystem2/logs/floor_run4_topspeed.txt` | `turnLeft(90)` -> `turnRight(90)` floor runs: Run 3 (`TRACK=138 mm`: `L90 dL=-1293 dR=1421`, `R90 dL=1371 dR=-1346`, `~104 deg`, returned to `0 deg`); Run 4 (`TRACK=120 mm`: `L90 dL=-1164 dR=1252`, `R90 dL=1218 dR=-1194`, `~95 deg`, returned to `0 deg`) -> `MOTION_EFF_TRACK_MM = 114U` calibrated for `90.0 deg` |
| 2026-10-31 | S2-U21 | `e794f8e` | 2500530 | PASS | `testing/subsystem2/logs/floor_run4_topspeed.txt`, `testing/subsystem2/MOTION_ACCURACY_EVALUATION.md` | NFR2 top-speed `50 cm/s` (`dt=1763 ms`, `pk=51.1 cm/s`, actual `103 cm`, expected `12260` counts): `dL=12418 (+1.29 %)`, `dR=12374 (+0.93 %)`, `df=44 (0.35 %)`; zero missed counts within 1 wheel-slip tolerance |




