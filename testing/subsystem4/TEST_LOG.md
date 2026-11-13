# Subsystem 4 — test log

**Subsystem:** IMU processing and safety (lift/overturn stop)  ·  **Student ID:** 2500709 (Vincent)  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Accelerometer found and identified on I2C1 | IMU on I2C1 (GP2/GP3) | auto | `imu_init()` scans 0x08-0x77 at boot and identifies an LSM303DLHC (0x19) or an LSM303D (WHO_AM_I 0x49 at 0x1D/0x1E); check `imu_accel_found()` | PASS |
| S4-U02 | Magnetometer present | LSM303DLHC mag at 0x1E, or built into the LSM303D | auto | `imu_mag_found()` | PASS |
| S4-U03 | Accelerometer configured | drv_imu init | auto | `imu_is_ready()` after `subsystem4_init()` | PASS |
| S4-U04 | IMU task reads new samples | I2C1 data path | auto | Good-sample count rises over 100 ms | PASS |
| S4-U05 | Z-axis reads ~1 g at rest | Accel sanity | auto | Car flat and still; abs(z) in (800, 1200) mg | PASS |
| S4-U06 | Magnitude ~1 g at rest | Accel sanity | auto | Car flat and still; magnitude in (800, 1200) mg | PASS |
| S4-U07 | Pitch ~0 when flat | Pitch calculation, boot level reference | auto | After boot the level reference is taken once the car has been still (within 3 deg) and flat (within 10 deg) for 0.5 s, else 0 after 10 s; later, car still flat: abs(pitch) < 20 half-deg (10 deg) | PASS |
| S4-U08 | IMU task is running | Task start | auto | `imu_is_task_running()` | PASS |
| S4-U09 | Safety task is running | Task start | auto | `safety_is_task_running()` | PASS |
| S4-U10 | IMU period ~10 ms | IMU_SAMPLE_PERIOD_MS | auto | At least 8 good samples in 100 ms | PASS |
| S4-U11 | No I2C errors | I2C reliability | auto | `imu_get_i2c_error_count() == 0` | PASS |
| S4-U12 | No IMU watchdog timeouts with the IMU connected | NFR13, IF-06 | auto | `safety_get_wdog_timeout_count() == 0` | PASS |
| S4-U13 | ~~Collision triggers the emergency brake~~ **DROPPED 11 Nov 2026**: IMU collision detection removed from the requirements (lecturer's instruction; see docs/REQ_CHANGE_LIFT.md). Replaced by S4-U17..U21. Never run | ~~IF-06 EVF_SAFETY_COLLISION~~ | — | — | — |
| S4-U14 | IMU watchdog triggers the emergency brake | NFR13, IF-06 EVF_SAFETY_FAULT_IMU_WDOG | auto (operator-guided) | When asked, unplug the IMU cable, wait 2 s, plug it back in (20 s window) | Watchdog timeout counted, FAULT_LATCHED set |
| S4-U15 | Hump detected, pitch sign correct | IF-04 MBF_TELEMETRY (DREP_TEL_HUMP), D4 | auto (operator-guided) | When asked, put a ~2 cm book under the FRONT wheels for 2 s, then remove it (20 s window) | One hump reported with positive pitch (nose up) |
| S4-U16 | IMU recovers after it is plugged back in | IF-06 EVF_SAFETY_FAULT_IMU_I2C, robustness | auto (operator-guided) | Follows S4-U14: after the watchdog trips, plug the IMU back in (20 s window) | Good samples resume (>= 40 in 0.5 s), FAULT_IMU_I2C cleared, `[IMU] I2C back` logged |
| S4-U17 | No false lift event at rest; detector armed | FR12, NFR12 | auto | Car flat and still and **not touched** after power-on. `imu_is_lift_armed()` and `imu_get_lift_count() == 0` | PASS |
| S4-U18 | Lifting the robot triggers the emergency brake | FR12, NFR10, IF-06 EVF_SAFETY_LIFTED, IF-11 | auto (operator-guided) | When asked, lift the car ~10 cm off the desk, hold it 2 s, put it back down flat (20 s window) | Lift counted, Safety brake count rises, FAULT_LATCHED set |
| S4-U19 | One pick-up is one event; detector re-arms once put down | FR12 | auto (operator-guided) | Follows S4-U18: car back on the desk, level and still | Re-armed within 20 s, no second event; then the test asks the Vehicle Controller to reset the fault |
| S4-U20 | Overturning the robot triggers the emergency brake | FR12, IF-06 EVF_SAFETY_LIFTED | auto (operator-guided) | When asked, roll the car onto its side on the desk without lifting it (lifting first triggers RAISED before the tilt), 2 s, then put it back down flat (20 s window) | Event of kind TILTED, Safety brake count rises, FAULT_LATCHED set |
| S4-U21 | Lift/overturn detector logic (host) | FR12, NFR12 | auto (host) | `tests/host/test_imu_lift.c` (`ctest -R imu_lift`): thresholds 30 deg / 0.10 m/s / 50 ms, no events at rest, on 15 deg humps, 25 deg roll, 0.3 g forward acceleration, +-0.05 g hump bounces, +-0.5 g jolts or a 30 mg offset; tilt, smooth level lift (10 cm in 0.8 s, the S4-U18 case), quick lift, side, upside-down and free fall detected; one event per pick-up, re-arm after 0.5 s level and still; board mounted upside down or 5 deg tilted | All checks pass |
| S4-U22 | No false lift event while driving over humps | NFR12, design.md A4 | manual + log | Drive the course (line, humps, turns) with the console logging; then lift the car at the end | No `[IMU] LIFTED` line until the final lift; the final lift stops the motors |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…);
**auto (operator-guided)** = the console asks a person for one action and the `TEST_CHECK` detects it (only in builds configured with `-DS4_MANUAL_TESTS=ON`);
**auto (host)** = a host unit test in `tests/host/`, no board needed.

`testing/subsystem4/s4_test.ps1 -AutoOnly` builds, flashes (rebooting the Pico into BOOTSEL over USB when it already runs the firmware) and records S4-U01..U12 and U17 unattended. Without `-AutoOnly` it adds the operator-guided tests.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-13 | S4-U01 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | LSM303D at 0x1D (accel + mag) |
| 2026-11-13 | S4-U02 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U03 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U04 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U05 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U06 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U07 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Level reference: board at 8 half-deg (4 deg) when flat, 0 retries |
| 2026-11-13 | S4-U08 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U09 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U10 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U11 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U12 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | Unattended run (`s4_test.ps1 -AutoOnly`), car flat on a stand |
| 2026-11-13 | S4-U17 | `5b5adc1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1524_s4_unit.txt` | First run of the lift detector on the car; Subsystem 2 motor tests ran before it without a false lift |
| 2026-11-13 | S4-U21 | `5b5adc1` | Vincent (Vincent6430) | PASS | `tests/host/test_imu_lift.c` | Host test, 63 checks, 0 failed |
| 2026-11-13 | S4-U15 | `44541b1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | Front raised ~2 cm: hump at +14 half-deg (+7 deg), nose up = positive |
| 2026-11-13 | S4-U18 | `44541b1` | Vincent (Vincent6430) | FAIL | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | Car lifted ~10 cm smoothly and level: no lift event in the 20 s window. Tilt stayed below 5 deg (no hump event either) and the 0.3 g / 50 ms upward-acceleration rule was not met by a gentle lift. Fix: detect vertical speed instead (next commit) |
| 2026-11-13 | S4-U19 | `44541b1` | Vincent (Vincent6430) | FAIL | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | Consequence of S4-U18 (no event, so nothing to re-arm) |
| 2026-11-13 | S4-U20 | `44541b1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | Tipped onto its side: TILTED at cos 0.855 (~31 deg), Safety braked, FAULT_LATCHED set; re-armed once put down |
| 2026-11-13 | S4-U14 | `44541b1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | IMU unplugged: watchdog brake, FAULT_LATCHED set |
| 2026-11-13 | S4-U16 | `44541b1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | IMU plugged back in: readings resumed |
| 2026-11-13 | S4-U01..U12, U17 | `44541b1` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1530_s4_console.txt` (full window export; `logs/2026-11-13_1530_s4_unit.txt` lost lines 146-795 to a file lock) | Same guided run; all 13 automatic checks pass again |
| 2026-11-13 | S4-U01..U12, U17 | `1fded1a` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1554_s4_unit.txt` | All 13 automatic checks pass (twice: the run restarted after a crash, below) |
| 2026-11-13 | S4-U15 | `1fded1a` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1554_s4_unit.txt` | Hump reported, nose up = positive pitch (both boots) |
| 2026-11-13 | S4-U18 | `1fded1a` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1554_s4_unit.txt` | Fix verified: the same gentle, level lift that failed in the 15:30 run is now detected as RAISED at v=117 mm/s (limit 100), peak 145 mm/s; Safety braked, FAULT_LATCHED set |
| 2026-11-13 | S4-U19 | `1fded1a` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1554_s4_unit.txt` | Re-armed once put down, no second event |
| 2026-11-13 | S4-U20 | `1fded1a` | Vincent (Vincent6430) | FAIL | `logs/2026-11-13_1554_s4_unit.txt` | Operator lifted the car to tip it, so RAISED (v=123 mm/s) fired before the tilt; brake and latch worked, but the test expects a TILTED event. Prompt to be reworded: roll the car over on the desk without lifting it. The car was then turned upside down during the S4-U14 prompt: TILTED detected (cos 0.763 at trigger, about -0.85 while upside down) |
| 2026-11-13 | S4-U14 | `1fded1a` | Vincent (Vincent6430) | FAIL | `logs/2026-11-13_1554_s4_unit.txt` | Steps out of sync: the car was upside down during this prompt and the IMU cable was not unplugged, so no watchdog trip. Passed in the 15:30 run; code unchanged since |
| 2026-11-13 | S4-U16 | `1fded1a` | Vincent (Vincent6430) | FAIL | `logs/2026-11-13_1554_s4_unit.txt` | Consequence of S4-U14 (no trip, nothing to recover from) |
| 2026-11-13 | (run note) | `1fded1a` | Vincent (Vincent6430) | — | `logs/2026-11-13_1554_s4_unit.txt` | After S4-U15 the robot reset on a HardFault: `panic()` in the Pico SDK (pc in `__breakpoint` via `_exit`/`panic`, IPSR 47 = IRQ 31, the SDK USB stdio background IRQ), right after the car was handled. The USB console also dropped 4 times without a reset. Suspect the USB cable moving; not Subsystem 4 code. To investigate separately |
| 2026-11-13 | S4-U01..U12, U17 | `593de73` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1608_s4_unit.txt` | All 13 automatic checks pass |
| 2026-11-13 | S4-U15 | `593de73` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1608_s4_unit.txt` | Hump reported, nose up = positive pitch |
| 2026-11-13 | S4-U18 | `593de73` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1608_s4_unit.txt` | Gentle level lift detected as RAISED at v=158 mm/s (limit 100); brake + FAULT_LATCHED |
| 2026-11-13 | S4-U19 | `593de73` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1608_s4_unit.txt` | Re-armed once put down, no second event |
| 2026-11-13 | S4-U20 | `593de73` | Vincent (Vincent6430) | PASS | `logs/2026-11-13_1608_s4_unit.txt` | Rolled onto its side on the desk (reworded prompt): TILTED at cos 0.797 (~37 deg); brake + FAULT_LATCHED |
| 2026-11-13 | S4-U14 | `593de73` | Vincent (Vincent6430) | not performed | `logs/2026-11-13_1608_s4_unit.txt` | Skipped by the tester this run (the console prints FAIL when the 20 s window expires untouched). PASS on `44541b1` (15:30 run); watchdog and I2C recovery code unchanged since. Re-run in the full pre-demo test set |
| 2026-11-13 | S4-U16 | `593de73` | Vincent (Vincent6430) | not performed | `logs/2026-11-13_1608_s4_unit.txt` | Skipped with S4-U14 (see above). PASS on `44541b1` |
| 2026-11-13 | (run note) | `593de73` | Vincent (Vincent6430) | — | `logs/2026-11-13_1608_s4_unit.txt` | No HardFault this run; the USB console dropped 3 times while the car was handled and reconnected each time |
