# Subsystem tests with the dashboard

Updated **22 Nov 2026** (all subsystems merged on `main`). Uses Buddy 1's
`tel_dashboard.py` / `tel_web.py` with per-subsystem views (`--view`), evidence checks (`check`),
the Overview Demo Rubric, and the Barcode `A–Z` mapping tab.

## Three firmware configurations

| Mode | CMake Flags | Drive motors | Use it for |
|---|---|---|---|
| **Active Car Wi-Fi + Straddle** | `-DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON "-DCMAKE_C_FLAGS=-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1"` (optional `-DTEL_STATUS_PERIOD_MS=1000`) | Driven via MQTT (`START`) | Full course runs at `5 cm/s` bring-up (`OBS_BYPASS_SLOW_BRINGUP=1U`), live MQTT dashboard, barcode `car/cmd/bc_map` |
| **Stationary (USB console)** | `APP_TEST=OFF -DDBG_BACKEND=USB -DTEL_STATUS_PERIOD_MS=1000` | **Never commanded.** VC stays in `READY_WAITING` | IMU, obstacle scanner, IR sensor readings, VC/fault states at rest |
| **Controlled moving (`APP_TEST`)** | `-DAPP_TEST=ON -DAPP_TEST_ONLY=3 -DS3_DRIVE_MM=<mm> -DTEL_STATUS_PERIOD_MS=1000` | Driven by VC via `APP_TEST` | Controlled USB-tethered line following & obstacle bypass drives |

The car stops by itself at the distance or time limit (the test sends STOP), at TRACK_LOST (the VC
stops), or when lifted (Safety brakes and latches; the test sends RESET once the car is still).
Catching and lifting the car is always a safe stop.

Build both (WSL, repo root, on the branch you are testing):

```bash
. ~/pico-1.5.1/env.sh
H=$(git rev-parse --short HEAD)
C="-G Ninja -DCMAKE_BUILD_TYPE=Debug -DPICO_TOOLCHAIN_PATH=$PICO_TOOLCHAIN_PATH -DDBG_BACKEND=USB -DTEL_STATUS_PERIOD_MS=1000"
cmake -S . -B ~/robocar-build/team-stationary $C && cmake --build ~/robocar-build/team-stationary
cmake -S . -B ~/robocar-build/team-drive $C -DAPP_TEST=ON -DAPP_TEST_ONLY=3 -DS3_DRIVE_MM=3000 && cmake --build ~/robocar-build/team-drive
cp ~/robocar-build/team-stationary/robocar.uf2 build/robocar-team-stationary-$H.uf2
cp ~/robocar-build/team-drive/robocar.uf2 build/robocar-team-drive3000-$H.uf2
```

Flash: hold BOOTSEL, plug in the USB, release, then drag the `.uf2` onto the RPI-RP2 drive (or use
`stationary_test.ps1`, which flashes for you).

## The quickest way: the browser dashboard

Double-click `tools\dashboard\dashboard.cmd`, or run `python tools\dashboard\tel_web.py` from the repo root.

1. Flash the stationary firmware: drag `build\robocar-dash-stationary-<hash>.uf2` onto the RPI-RP2 drive.
2. **USB (Windows) → Start capture**. This runs `tools\serial_capture.ps1` into `build\dash-live-*.txt` and shows it live.
3. Open your subsystem's tab, follow its *Test procedure*, then press **Run check**.

You can also pick any saved log from the **Log** list (tick *follow* for a log that's still being
written, e.g. by `stationary_test.ps1`), or connect to **MQTT** with a WiFi build. An MQTT session
is saved to `build\dash-mqtt-*.txt`. Only one source can read the USB port at a time.

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

Expected: PASS for chip found (`LSM303D`/`LSM303DLHC`), no I2C/watchdog fault, `|accel|` 900–1100 mg at rest, pitch within ±2°
when flat, pitch tilt, roll tilt, lift + brake, and motors never commanded. Hump height estimation (`imu_hump.c`,
calibrated `95 mm` wheelbase, `3°` entry) reports real hump heights on drive runs over the track hump (`S4-U26`).

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

## Moving tests: VC, line following (`5 cm/s` straddle), obstacle avoidance

For live Wi-Fi track tests, build with `-DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON "-DCMAKE_C_FLAGS=-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1"` and connect over MQTT in `tel_web.py`. For USB-tethered controlled drives without Wi-Fi, use `APP_TEST=ON -DAPP_TEST_ONLY=3 -DS3_DRIVE_MM=3000` (add `-DS3_LF_STRADDLE=ON` to match the straddle bracket). Line following supports both edge mode (`lf_core.c`, default) and straddle mode (`lf_straddle.c`, with `tel_set_ir_raw()` live).

1. Browser dashboard: **Live car (MQTT)** or **USB capture → Start capture**, then check the *Overview* (Demo Rubric), *Barcode*, *Line calibration*, and *Subsystem tests* tabs.
2. At power-up, put the car on the line in the configured mount pose (CENTRE on black / RIGHT on white for edge mode; sensors straddling white/line for straddle mode), still, and wait for `[LF] boot calibration` and `[IMU] level reference`.
3. **Obstacle avoidance (Buddy 5):** put the obstacle on the line ahead (`>= 60 cm` after start). The car halts on watch-ahead confirmation, runs a stationary `5°` sweep (`50°–130°`), sends `event_obstacle`, drives an encoder-closed 4-turn bypass (`OBSTACLE_AVOIDANCE`, `5 cm/s` under `OBS_BYPASS_SLOW_BRINGUP=1U`), and hands back on `EVF_NAV_LINE_FOUND`.

## Per-subsystem CLI views (`main`)

| Subsystem | Command | Available on `main` |
|---|---|---|
| Vehicle Controller | `decode-log <log> --view vc` | Full state transitions from heartbeats (`0..9`) and encoder motion |
| Line following | `decode-log <log> --view lf` | Live IR_C / IR_R raw (`MEASURED` via `tel_set_ir_raw()` every `200 ms`) and `[LF]` state transitions |
| Barcode | `decode-log <log> --view barcode` | `A–Z` Code 39 decodes + `car/cmd/bc_map` mapping + VC turn telemetry |
| IMU & hump | `decode-log <log> --view imu` | Attitude (`tel_set_attitude()`), lift/watchdog faults, calibrated hump height (`95 mm` wheelbase) |
| Obstacle | `decode-log <log> --view obstacle` | Forward range (`tel_set_range_cm()`), `event_obstacle` width & clearances (`5°` sweep), avoid states |
| MQTT | `listen --broker <ip> --save run.txt`, then `check mqtt run.txt` | Full Wi-Fi/MQTT client (`-DTEL_BACKEND_WIFI=ON`, `config/secrets.h`), broker and dashboard |

`send start` needs `--allow-motion` on the CLI, because `START` drives the motors. Always run moving tests on a track with someone ready to catch (lifting the car ends the run via `EVF_SAFETY_LIFTED`).
