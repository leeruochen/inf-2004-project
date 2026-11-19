# Subsystem tests with the dashboard

Dashboard integration, 19 Nov 2026 (branch `buddy3/dashboard-integration`; line following and the
moving test mode on `buddy3/lf-integration`; neither merged yet). It uses Buddy 1's
`tel_dashboard.py` with per-subsystem views (`--view`) and evidence checks (`check`) added in
`tel_views.py`.

## Two test modes: pick the firmware first

| Mode | Firmware (build below) | Drive motors | Use it for |
|---|---|---|---|
| **Stationary** | `robocar-team-stationary-<hash>.uf2`: normal firmware, APP_TEST=OFF | **Never commanded.** The VC stays in READY_WAITING. A USB build has no START path, and line following only steers while the VC is driving | IMU, obstacle scanner, IR sensor readings, VC/fault states at rest |
| **Controlled moving** | `robocar-team-drive<mm>-<hash>.uf2`: APP_TEST=ON, APP_TEST_ONLY=3, S3_DRIVE_MM=<mm> | Driven by the VC only. The test starts each drive through the VC's input (START, STOP, RESET, as Telemetry would) after the start guard | VC, line following, obstacle avoidance and return to the line |

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

## Moving tests: VC, line following, obstacle avoidance

Firmware: `robocar-team-drive3000-<hash>.uf2` (3 drives of up to 3 m each). Battery connected,
with the USB cable overhead and slack. Clear 40 cm on both sides of the line. Line following is
the production version (`buddy3/lf3-link`). It follows straights, gentle curves and cross
junctions (straight on), but **stops at tight 90° curved tiles** (TRACK_LOST, by design a safe stop).
Start on a straight that has at least 1 m of line before any tight bend.

1. Browser dashboard: **USB (Windows) → Start capture**, then the *Vehicle Controller*, *Line following* and *Console* tabs.
2. At power-up, put the car on the line: CENTRE on black, RIGHT on white, still. Wait for `[LF] boot calibration` and `[IMU] level reference`.
3. **Pose A** prompt: CENTRE on black in the middle of the line, RIGHT on white, hands off.
   **Pose B** prompt: RIGHT over the middle of the line, CENTRE on white, hands off.
4. Each drive: place the car at the start mark, square to the line, CENTRE on black and RIGHT on white.
   It starts once CENTRE has been steady for 3 s, after the warning `HANDS OFF: the car DRIVES in 3 s`.
5. **Obstacle avoidance (Buddy 5):** put the obstacle on the line at least 60 cm after the start, with at
   least 1 m of straight line after it. During the bypass the VC shows OBSTACLE_AVOIDANCE, LF shows
   SUSPENDED, and the log shows `[LF] drive: obstacle avoidance took over` / `handed back`. Line
   following sets LINE_FOUND when CENTRE meets the line on the way back.
6. Results per drive: `[LF] drive N: <why it ended> at <mm>`, plus (d1) "ended with the wheels still" and
   (d2) "reached the distance limit on the line". A drive that ends at a tight bend fails (d2) and
   passes (d1). That is the expected result with the production line following.

## Other subsystems

| Subsystem | Command | Available now |
|---|---|---|
| Vehicle Controller | `decode-log <log> --view vc` | Any firmware: state changes from heartbeats, encoder motion |
| Line following | `decode-log <log> --view lf` | With `buddy3/lf-integration` firmware: IR CENTRE/RIGHT raw (MEASURED, every sensor frame) and the LF state (REPORTED, from the task's state-change lines). On `main` alone, subsystem 3 is a stub: N/A |
| Barcode | `decode-log <log> --view barcode` | Nothing yet: no barcode task on `main` |
| MQTT | `listen --broker <ip> --save run.txt`, then `check mqtt run.txt` | Needs a WiFi build (`-DTEL_BACKEND_WIFI=ON`, `config/secrets.h`), a broker and a network |

`send start` now needs `--allow-motion`, because START drives the motors. Without line following
(`main` alone) the car drives straight. Only use it with the car on the track and someone ready to
catch it. For moving tests over USB, use the controlled moving firmware above.
