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
| S4-U20 | Overturning the robot triggers the emergency brake | FR12, IF-06 EVF_SAFETY_LIFTED | auto (operator-guided) | When asked, tip the car onto its side (or upside down) for 2 s, then put it back down flat (20 s window) | Event of kind TILTED, Safety brake count rises, FAULT_LATCHED set |
| S4-U21 | Lift/overturn detector logic (host) | FR12, NFR12 | auto (host) | `tests/host/test_imu_lift.c` (`ctest -R imu_lift`): thresholds 30 deg / 0.3 g / 50 ms, no events at rest, on 15 deg humps, 25 deg roll or 0.3 g forward acceleration; tilt, raise, side, upside-down and free fall detected; one event per pick-up, re-arm after 0.5 s level and still; board mounted upside down or 5 deg tilted | All checks pass |
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
