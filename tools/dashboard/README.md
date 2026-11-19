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
| Overview | Every sensor's state (VALID / STALE / INVALID / UNAVAILABLE / DISCONNECTED), newest value, measured update rate and owner; VC state, speed, distance, last command |
| Drive | START / STOP / RESET over the existing MQTT commands, with a pre-START checklist. START needs **Arm motion** (one START per arming, 60 s) and a confirm dialog. Each command shows *broker acknowledged* and then **CONFIRMED** only when the car's heartbeat shows the result, else **NOT CONFIRMED** with the reason. PAUSE is shown disabled (see below) |
| Recording | One row per telemetry wake: time, car ms, IR centre/right, pitch, roll, speed, distance, range, VC state. Live or paged back through history; **Record** marks a span; CSV export of the span or of every row |
| Data gaps | One event per sensor each time it stops being VALID, with start, end, length and reason; CSV export |
| Line calibration | Guided capture over white and over the black line for both IR sensors; per-sensor verdict, span and raw thresholds using Buddy 3's rules (read from `lf_core.h` / `thresholds.h`); live zone (white / edge / black); the car's own measurement from its `[LF]` console lines |
| Subsystem tests | The per-subsystem readings, event feeds and evidence checks of the first web UI (MEASURED / DERIVED / REPORTED / SIMULATED / N/A) |
| Console | Every raw line of the source |

The page reads only what the car sends. It never shows a command as done, or a sensor as
valid, without the frame that proves it.

| File | What it is |
|---|---|
| `tel_codec.py` | Frame decoder/encoder, a mirror of `src/common/data_repr.h` (standard library only) |
| `tel_dashboard.py` | CLI: `listen`, `send`, `decode-log`, `decode`, `check` |
| `test_tel_decode.py` | Unit tests, including the golden frames shared with `tests/host/test_tel.c` |
| `tel_views.py` | Per-subsystem views (`--view`) with MEASURED/DERIVED/REPORTED/SIMULATED/N/A labels, and the evidence checks behind `check` (dashboard integration) |
| `test_tel_views.py` | Unit tests for the views and checks, with regressions on the hardware logs in `testing/` |
| `tel_web.py`, `tel_web.html`, `dashboard.cmd` | The same views and checks in the browser (standard library server on 127.0.0.1) |
| `test_tel_web.py` | Tests for the web API on a real local server (fake MQTT client) |
| `tel_record.py` | Dashboard v2: recording table and CSV, sensor health, update rates, data-gap events |
| `tel_calib.py` | Dashboard v2: line-sensor calibration on the laptop with Buddy 3's rules |
| `tel_control.py` | Dashboard v2: command rules (arming, preconditions) and confirmation from telemetry |
| `test_tel_record.py`, `test_tel_calib.py`, `test_tel_control.py` | Unit tests for the three, with replays of the hardware logs (S1-H06..H08) |
| `stationary_test.ps1`, `SUBSYSTEM_TESTS.md` | Flash, view live and check: IMU and obstacle tests with the motors never commanded |

```
pip install paho-mqtt                                   # only for listen / send
python tel_dashboard.py listen --broker 192.168.1.10    # Ctrl+C prints the run summary (peak hump)
python tel_dashboard.py send start --broker 192.168.1.10 --allow-motion   # START drives the motors
python tel_dashboard.py decode-log saved_console_log.txt   # console backend, no broker
python tel_dashboard.py decode-log live.txt --view imu --follow   # live, while serial_capture.ps1 writes live.txt
python tel_dashboard.py check imu live.txt                 # also: obstacle (--expect-cm, --expect-width-mm), mqtt
python tel_dashboard.py listen --broker 192.168.1.10 --view obstacle --save run.txt
python tel_dashboard.py decode 010706efbeaddef93f
python -m unittest -v test_tel_decode test_tel_views test_tel_web test_tel_record test_tel_calib test_tel_control
python tel_web.py --open ../../build/dash-imu-20261119-1500.txt --follow   # browser, starting on a log
```

## Needs firmware from teammates (dashboard v2)

The dashboard side is done; these need a change in someone else's code first. Until then the page
says what is missing instead of showing made-up values.

| Feature | Needs | Owner |
|---|---|---|
| Live IR values (calibration, table, health) | The Line-Following task calls `tel_set_ir_raw(ir_r, ir_c)` once per sample (IF-12, already in `subsystem1.h`). Today no task calls it, on `main` or on `buddy3/lf3-link` | Buddy 3 |
| Calibrate the car from the dashboard | The car calibrates only at boot. Sending white/black to the car needs a new command and a hook in the Line-Following task | Buddy 3, Buddy 1 |
| **PAUSE** | A `DREP_FT_CMD_PAUSE` frame type (`data_repr.h`, shared), a `DREP_NAV_REMOTE_PAUSE` remote command and a `VC_SYS_PAUSED` state in the Vehicle Controller: motors stopped, steering and avoidance ignored (as in READY_WAITING), START resumes the run, STOP ends it. Then Buddy 1 adds it to `tel_wire.c` and `tel_codec.py`, and the page enables the button | Buddy 2 (VC), team (`data_repr.h`), Buddy 1 |
| Acknowledgement from the car itself | Today a command is confirmed from the next heartbeat. A small `car/cmd/ack` frame (command, accepted or rejected, reason) from `task_tel.c` would confirm receipt even when the state does not change | Buddy 1, team (`data_repr.h`) |

## Not yet verified on the car

Checked with unit tests, the hardware logs in `testing/`, and end to end against a stand-in car on an
isolated local broker (real paho client and Mosquitto): START, E-STOP, confirmation, the live
table at 5 Hz, last-will detection and the data-gap table. Still to do on the real car: command
confirmation times, E-STOP while driving, update rates under WiFi load, and a calibration with live IR
values (needs the first row above).

If a frame layout in `data_repr.h` changes, update `tel_codec.py`, the golden
strings in `test_tel_decode.py` and the ones in `tests/host/test_tel.c`
together. Both test files fail until they agree.

Protocol, topics and setup: `app_program/subsystem1/MQTT.md`.
