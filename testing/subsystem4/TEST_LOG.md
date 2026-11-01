# Subsystem 4 — test log

**Subsystem:** IMU processing and safety  ·  **Student ID:** 2500709 (Vincent)  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | I2C bus scan finds accelerometer | IMU accel at 0x19 | auto | `imu_bus_scan_accel()` returns true | PASS |
| S4-U02 | I2C bus scan finds magnetometer | IMU mag at 0x1E | auto | `imu_bus_scan_mag()` returns true | PASS |
| S4-U03 | IMU driver initialises | drv_imu init | auto | `imu_is_ready()` returns true after init | PASS |
| S4-U04 | Accelerometer read succeeds | I2C1 data path | auto | `imu_read_accel()` returns true | PASS |
| S4-U05 | Z-axis reads ~1 g at rest | Accel sanity | auto | Abs(z_mg) in [800, 1200] with car flat | PASS |
| S4-U06 | Magnitude ~1 g at rest | Accel sanity | auto | `imu_accel_magnitude_mg()` in [800, 1200] | PASS |
| S4-U07 | Pitch ~0 at rest (car flat) | Pitch calculation | auto | Abs(pitch_half_deg) < 20 (~10 deg) | PASS |
| S4-U08 | IMU task is running | Task start | auto | `imu_is_task_running()` returns true | PASS |
| S4-U09 | Safety task is running | Task start | auto | `safety_is_task_running()` returns true | PASS |
| S4-U10 | IMU releases > 0 after 50 ms | Periodic execution | auto | `imu_get_release_count() > 0` after 50 ms delay | PASS |
| S4-U11 | No I2C errors after initial cycles | I2C reliability | auto | `imu_get_i2c_error_count() == 0` | PASS |
| S4-U12 | Public pitch accessor works | Subsystem interface | auto | `subsystem4_get_pitch_half_deg()` returns sane value | PASS |
| S4-U13 | Collision detection triggers brake | IF-06 EVF_SAFETY_COLLISION, IF-11 | manual | Tap the car firmly; observe `[SAFETY] BRAKE: collision` on console | PASS: brake count increments |
| S4-U14 | IMU watchdog triggers brake | IF-06 EVF_SAFETY_FAULT_IMU_WDOG | manual | Disconnect IMU I2C; observe `[SAFETY] BRAKE: IMU watchdog` within 50 ms | PASS: brake count increments |
| S4-U15 | Hump detection fires telemetry event | IF-04 MBF_TELEMETRY, DREP_TEL_HUMP | manual | Tilt the car > 5 deg pitch for > 30 ms; observe `[IMU] hump` on console | PASS: hump event logged |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
