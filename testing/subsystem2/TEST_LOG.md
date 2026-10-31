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
