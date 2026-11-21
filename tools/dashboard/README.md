# Telemetry dashboard (Buddy 1)

Laptop-side tool for the car's MQTT telemetry. Python 3.8+.

**In the browser (Windows):** double-click `tools\dashboard\dashboard.cmd`, or run
`python tools\dashboard\tel_web.py`. It opens http://127.0.0.1:8765. Pick a source at the top:
**Live car** (MQTT broker IP, e.g. `127.0.0.1` when the broker runs on this laptop), **USB
console** (a capture), or a **Saved log** (replay; tick *follow* for one still being written).
Nothing to install, except `pip install paho-mqtt` for MQTT. The status strip (broker, car
ONLINE/OFFLINE, heartbeat and data age, VC state) and the **EMERGENCY STOP** button stay at the top.

| Page | What it does |
|---|---|
| Overview | **10-item Demo Rubric** (`DONE` / `ACTIVE` / `WAITING` / `SIMULATED`), newest-first **Run Events** timeline, **Live Status**, quick **START / STOP / RESET** buttons, every sensor's health card (`VALID` / `STALE` / `INVALID` / `UNAVAILABLE` / `DISCONNECTED`), and VC state/speed/distance (`tel_demo.py`) |
| Drive | START / STOP / RESET over the existing MQTT commands, with a pre-START checklist. One press of START sends it; the dashboard still refuses START unless the car is ONLINE, its heartbeat is fresh and it is in READY_WAITING. Each command shows *broker acknowledged* and then **CONFIRMED** only when the car's heartbeat shows the result, else **NOT CONFIRMED** with the reason. PAUSE is shown disabled (see below) |
| Barcode | Live barcode detection badge, prominent decoded `A–Z` letter display with its assigned action and VC target state, decoded barcode history table, and an interactive **`A–Z` mapping editor** that sends a framed, non-retained `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`) packet over MQTT |
| Recording | One row per telemetry wake: time, car ms, IR centre/right, pitch, roll, speed, distance, range, VC state. Live or paged back through history; **Record** marks a span; CSV export of the span or of every row |
| Data gaps | One event per sensor each time it stops being VALID, with start, end, length and reason; CSV export |
| Line calibration | Guided capture over white and over the black line for both IR sensors; per-sensor verdict, span and raw thresholds using Buddy 3's rules (read from `lf_core.h` / `thresholds.h`); live zone (white / edge / black); the car's own measurement from its `[LF]` console lines |
| Subsystem tests | The per-subsystem readings, event feeds and evidence checks of the first web UI (MEASURED / DERIVED / REPORTED / SIMULATED / N/A) |
| Console | Every raw line of the source |

The page reads only what the car sends. It never shows a command as done, or a sensor as
valid, without the frame that proves it.

| File | What it is |
|---|---|
| `tel_codec.py` | Frame decoder/encoder (`FT_*` types `1..11` incl. `FT_CMD_BC_MAP` and `A–Z` barcode payload bits), a mirror of `src/common/data_repr.h` (standard library only) |
| `tel_dashboard.py` | CLI: `listen`, `send`, `decode-log`, `decode`, `check` |
| `test_tel_decode.py` | Unit tests, including the golden frames (`1..11`) shared with `tests/host/test_tel.c` |
| `tel_views.py` | Per-subsystem views (`--view`) with MEASURED/DERIVED/REPORTED/SIMULATED/N/A labels, and the evidence checks behind `check` |
| `test_tel_views.py` | Unit tests for the views and checks, with regressions on the hardware logs in `testing/` |
| `tel_demo.py` | Live Overview status tracker, obstacle/line/junction/pickup state inference, Run Events, and 10-item Demo Rubric (`test_tel_demo.py` unit tests) |
| `tel_web.py`, `tel_web.html`, `dashboard.cmd` | The browser dashboard server (standard library HTTP server on `127.0.0.1:8765`) |
| `test_tel_web.py` | Tests for the web API on a real local server (fake MQTT client) |
| `tel_record.py` | Recording table and CSV, sensor health, update rates, data-gap events |
| `tel_calib.py` | Line-sensor calibration helper on the laptop with Buddy 3's rules |
| `tel_control.py` | Command rules (preconditions) and confirmation from telemetry |
| `test_tel_record.py`, `test_tel_calib.py`, `test_tel_control.py`, `test_tel_demo.py` | Unit tests for recording, calibration, control, and Overview rubric (7 test files total) |
| `stationary_test.ps1`, `SUBSYSTEM_TESTS.md` | Flash, view live and check: IMU and obstacle tests with the motors never commanded |

```
pip install paho-mqtt                                   # only for listen / send / MQTT in web UI
python tel_dashboard.py listen --broker 192.168.1.10    # Ctrl+C prints the run summary (peak hump)
python tel_dashboard.py send start --broker 192.168.1.10 --allow-motion   # START drives the motors
python tel_dashboard.py decode-log saved_console_log.txt   # console backend, no broker
python tel_dashboard.py decode-log live.txt --view imu --follow   # live, while serial_capture.ps1 writes live.txt
python tel_dashboard.py check imu live.txt                 # also: obstacle (--expect-cm, --expect-width-mm), mqtt
python tel_dashboard.py listen --broker 192.168.1.10 --view obstacle --save run.txt
python tel_dashboard.py decode 010706efbeaddef93f
python -m unittest discover -s . -p "test_*.py" -v         # runs all 7 dashboard test files (or run from repo root with -s tools/dashboard)
python tel_web.py --open ../../build/dash-imu-20261119-1500.txt --follow   # browser, starting on a log
```

## Firmware status for dashboard features (22 Nov 2026)

- **Live IR values (`tel_set_ir_raw`):** **DONE on `main`** (`app_program/subsystem3/task_lf.c` calls `tel_set_ir_raw()` every `5 ms` sample).
- **Decoded `A–Z` barcode + runtime map (`car/cmd/bc_map`):** **DONE on `main`** (`tel_post_barcode_letter()`, `DREP_FT_CMD_BC_MAP = 11`, non-retained).
- **Optional future enhancements (not in current firmware):**

| Feature | Needs | Owner |
|---|---|---|
| Calibrate the car from the dashboard | The car calibrates at boot (`lf_cal_from_start_pose()`). Sending white/black to the car would need a new command and a hook in `task_lf.c` | Buddy 3, Buddy 1 |
| **PAUSE** | A `DREP_FT_CMD_PAUSE` frame type (`data_repr.h`, shared), a `DREP_NAV_REMOTE_PAUSE` remote command and a `VC_SYS_PAUSED` state in the Vehicle Controller: motors stopped, steering and avoidance ignored (as in READY_WAITING), START resumes the run, STOP ends it | Buddy 2 (VC), team (`data_repr.h`), Buddy 1 |
| Acknowledgement from the car itself | Today a command is confirmed from the next heartbeat. A small `car/cmd/ack` frame (command, accepted or rejected, reason) from `task_tel.c` would confirm receipt even when the state does not change | Buddy 1, team (`data_repr.h`) |

## Verification note

Unit tests (`7` files), replays of hardware logs in `testing/`, fake-client web tests, and live Pico W Wi-Fi/MQTT runs on track (Overview Demo Rubric, START/STOP/RESET, Barcode `A–Z` mapping, IMU hump height, obstacle width & bypass) have been exercised. Still to capture for formal sign-off: dedicated high-load Wi-Fi update-rate and reconnect logs (`S1-M03`, `S1-M04`, `S1-M05`).

If a frame layout in `data_repr.h` changes, update `tel_codec.py`, the golden
strings in `test_tel_decode.py` and the ones in `tests/host/test_tel.c`
together. Both test files fail until they agree.

Protocol, topics and setup: `app_program/subsystem1/MQTT.md`.
