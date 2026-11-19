# Subsystem tests with the dashboard

Dashboard integration, 19 Nov 2026 (branch `buddy3/dashboard-integration`, not merged).
It uses Buddy 1's `tel_dashboard.py` with per-subsystem views (`--view`) and
evidence checks (`check`) added in `tel_views.py`.

## What each label means

| Label | Meaning |
|---|---|
| MEASURED | read from a sensor: IR ADC, ultrasonic echo, encoder counts, accelerometer |
| DERIVED | computed on the car from measurements: pitch/roll, obstacle width and clearances, distance |
| REPORTED | a state or decision the firmware reports: VC state, fault bits, barcode command, hump event |
| SIMULATED | injected by a self-test: an APP_TEST build that ran the subsystem 1 tests |
| N/A | no producer in this firmware, or it has not written a value yet |

A check prints **PASS** only when the log contains the measurement behind it. The other results:
FAIL, NOT TESTED (you skipped that step), NO DATA (the firmware didn't send what the check needs),
NOT MEASURED (the firmware has no such measurement) and COMPARE (needs a person with a tape measure).
`check` exits with 1 if anything FAILs.

## The stationary firmware

This is the normal firmware (APP_TEST=OFF) with the console backend and status lines every 1 s.
**It never commands the drive motors.** The Vehicle Controller stays in READY_WAITING, and only a
`car/cmd/start` over MQTT can move it, which can't reach a console-backend build. The scanner
servo still sweeps.

Build (WSL, from the repo root):

```bash
. ~/pico-1.5.1/env.sh
cmake -S . -B ~/robocar-build/dash-stationary -G Ninja -DCMAKE_BUILD_TYPE=Debug \
      -DPICO_TOOLCHAIN_PATH=$PICO_TOOLCHAIN_PATH -DDBG_BACKEND=USB -DTEL_STATUS_PERIOD_MS=1000
cmake --build ~/robocar-build/dash-stationary
cp ~/robocar-build/dash-stationary/robocar.uf2 build/robocar-dash-stationary-$(git rev-parse --short HEAD).uf2
```

## IMU (Buddy 4): stationary, about 4 minutes

```powershell
powershell -ExecutionPolicy Bypass -File tools\dashboard\stationary_test.ps1 -Subsystem imu
```

1. Keep the car flat and still for 30 s after flashing. That covers the level reference and the flat readings.
2. Lift the front about 15° (front wheels on a 4 cm book), hold 3 s, then put it down. Pitch should go positive.
3. Lift the left side about 15°, hold 3 s, then put it down. Note the sign of roll.
4. Do this last: lift the whole car about 5 cm, hold 1 s, then put it down. Expect `[IMU] LIFTED` and `[SAFETY] BRAKE`. The fault stays latched until you replug the USB.

Expected: PASS for chip found, no I2C/watchdog fault, |accel| 900–1100 mg at rest, pitch within ±2°
when flat, pitch tilt, roll tilt, lift + brake, and motors never commanded. The hump event shows as
INFO, because a hand tilt also fires the pitch-only detector. Hump height shows as NOT MEASURED,
because the firmware always sends 0.

## Obstacle (Buddy 5): stationary, about 4 minutes

```powershell
powershell -ExecutionPolicy Bypass -File tools\dashboard\stationary_test.ps1 -Subsystem obstacle -ExpectCm 20,50 -ExpectWidthMm <box width>
```

1. Leave nothing within 60 cm in front or to the sides for 20 s.
2. Put the box straight ahead at 50 cm, measured from the sensor face, for 15 s.
3. Move it to 20 cm for 15 s. That's inside the 25 cm detection distance, so expect an obstacle profile.
4. Remove the box for 15 s.

Expected: PASS for sensors OK, forward readings, a stable reading within ±2 cm of each `--expect-cm`
value, obstacle detected, width ≥ the real box (the spec says the width errs wide), and motors never
commanded. Avoidance shows as NOT TESTED: it needs the car driving.

The forward range is sampled when the servo points straight ahead, about once per sweep. The console
shows one sensor frame per second.

## Other subsystems

| Subsystem | Command | Available now |
|---|---|---|
| Vehicle Controller | `decode-log <log> --view vc` | Any firmware: state changes from heartbeats, encoder motion |
| Line following | `decode-log <log> --view lf` | `[LF]` console lines from the LF branch logs. IR telemetry is N/A on `main`: subsystem 3 is a stub there |
| Barcode | `decode-log <log> --view barcode` | Nothing yet: no barcode task on `main` |
| MQTT | `listen --broker <ip> --save run.txt`, then `check mqtt run.txt` | Needs a WiFi build (`-DTEL_BACKEND_WIFI=ON`, `config/secrets.h`), a broker and a network |

`send start` now needs `--allow-motion`. START drives the motors, and on `main` it drives straight,
because there is no line-following task. Only use it with the car on the track and someone ready to catch it.
