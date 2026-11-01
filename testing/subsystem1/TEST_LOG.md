# Subsystem 1 — test log

**Subsystem:** Telemetry (WiFi, MQTT, dashboard link)  ·  **Student ID:** 2501299 (Damian)  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results. Protocol and
setup: `app_program/subsystem1/MQTT.md`.

## Test cases

Define a test here before running it. Test IDs never change.

*Method:* **auto** = `TEST_CHECK` in `subsystem1_test.c` (APP_TEST image), result printed on the console;
**host** = host unit test, no board (`ctest`, `python -m unittest`);
**manual** = a person runs the steps and checks the result (console, broker, dashboard, scope).

### On target, automatic (`-DAPP_TEST=ON`; works with either backend)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Telemetry task runs and the link comes up | FR1, statechart 6.5 | auto | Poll `tel_get_stats()` every 20 ms | Task running and link CONNECTED (console at once; WiFi within 45 s, broker running) |
| S1-U02 | Event path: `tel_evt_t` → frame → sent | IF-04, FR1 | auto | `tel_post_barcode(STRAIGHT)`, wait 100 ms | `evt_rx` and `frames_sent` rise, outbox empty, no invalid count |
| S1-U03 | An invalid `tel_evt_t` is never framed | IF-04, D8 | auto | Post an event with type 0, wait 100 ms | `evt_invalid` +1, outbox empty |
| S1-U04 | Producers never block; a full queue is a counted drop | NFR1 | auto | 64 back-to-back `tel_post()` calls (type 0), timed with `hal_time_us()` | All 64 return in < 2 ms; queued + dropped = 64; ≥ 16 queued |
| S1-U05 | A valid remote command reaches the Vehicle Controller | FR2, IF-01 | auto | `tel_cmd_receive("car/cmd/stop", STOP frame)` | Returns OK, `cmd_accepted` +1 (posted to MBF_NAV_IN) |
| S1-U06 | Malformed commands are rejected and never posted | D10, FR2 | auto | Wrong version, trailing byte, unknown type, START on `car/cmd/reset`, heartbeat frame on `car/cmd/stop` | 5 rejects with the right reason, `cmd_accepted` unchanged |
| S1-U07 | Sensor snapshot clamps to the wire fields | IF-12 (proposed) | auto | `tel_set_ir_raw(5000,123)`, `tel_set_range_cm(1000)`, `tel_set_attitude(-200,37)` | IR_R 4095, IR_C 123, range 400, pitch −90, roll 37, valid bits set |
| S1-U08 | An obstacle profile is framed exactly once | IF-12 (proposed), DATA_REPRESENTATION.md 10 | auto | `tel_post_obstacle()`, wait 100 ms | Pending before, not pending after, `frames_sent` rises |
| S1-U09 | FAULT_WIFI_LOSS is clear while connected | EVF_SAFETY b8 | auto | `tk_ref_flg(EVF_SAFETY)` while CONNECTED | Bit 8 clear |
| S1-U10 | The Telemetry task holds its 20 ms grid | D2, `TEL_WAKE_PERIOD_MS` | auto | Count wakes over 1 s during the test run | 48–52 wakes, 0 overruns |

### Host (no board)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-H01 | Link state machine, backoff 1-2-4-8-16-30-30 s, immediate retry on definite failure, DEGRADED recovery within T_ka, 49.7-day wrap, corrupted state | FR3, statechart 6.5 | host | `cmake -S tests -B build/host -G Ninja; cmake --build build/host; ctest --test-dir build/host` (test `tel`) | All checks pass |
| S1-H02 | Outbox: FIFO order, drop-oldest + count when full, bad arguments | NFR1, design 6.5 | host | as S1-H01 | All checks pass |
| S1-H03 | Topic tree and the command check (every reject reason; `*p_nav` untouched on reject) | FR2, D10 | host | as S1-H01 | All checks pass |
| S1-H04 | Golden frames: C encoders produce the agreed bytes | risk R3 | host | as S1-H01 (`test_golden_frames`) | All 10 frames match |
| S1-H05 | Dashboard decoder: same golden frames, reject rules, console-log parsing | risk R3 | host | `cd tools/dashboard; python -m unittest -v test_tel_decode` | 5 tests OK |

### On the car, manual (`-DTEL_BACKEND_WIFI=ON`, broker on the laptop)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-M01 | Telemetry reporting demo | FR1, specs deliverable | manual | Flash the WiFi build; `tel_dashboard.py listen`; push the car by hand, then drive it | `car/status` ONLINE; motion at 5/s, heartbeat at 1/s; encoder counts and distance follow the wheels; state changes appear at once |
| S1-M02 | Remote commands | FR2 | manual | `tel_dashboard.py send start`, then `stop`, then `reset`; also send a bad frame with `mosquitto_pub -t car/cmd/stop -m x` | Each command logged `[TEL] cmd car/cmd/... -> VC` and the heartbeat state follows; the bad frame gives `[TEL] cmd rejected` and nothing happens |
| S1-M03 | WiFi drop and recovery without control jitter | FR3, A3, NFR1 | manual | While driving, switch the access point off for 20 s, then on | `link CONNECTED -> DEGRADED -> RECONNECTING`, backoff 1, 2, 4 s ... in the log; dashboard shows OFFLINE (last will); reconnects within 30 s of the AP returning; the buffered fault frame (WIFI_LOSS) arrives; Motion keeps 500 releases per 10 s (S2-U12 counters), 0 missed |
| S1-M04 | 30-minute soak: kernel + CYW43/lwIP coexistence | A9 | manual | WiFi build with everything running; 30 min with at least 3 forced AP drops; save the console | No `[FAULT]`, no fault record at the next boot, no stall (heartbeat every 1 s on the dashboard); `[STACK] tel` high-water below the 4.5 KB stack; `wake_overruns` stays 0 |
| S1-M05 | Broker down: backoff and no impact on the car | FR3 | manual | Stop the broker for 2 min with the car running | Retries at 1, 2, 4, 8, 16, 30, 30 s (`teardown ... retry in` lines); the car drives normally; full recovery when the broker returns |
| S1-M06 | Peak hump reported at the end of the run | mission requirement 5 | manual (with Buddy 4) | Full run over the humps with `listen`; Ctrl+C at the end | Run summary `PEAK HUMP` equals the highest hump event |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S1-H01..S1-H04 | not committed yet (working tree) | Buddy 1 (prepared with AI assistance) | PASS | `ctest --test-dir build/host`: `tel: 243 checks, 0 failed`; also clean under `-fsanitize=address,undefined` | Linux host gcc. Re-run on the committed tree and replace this note with the commit |
| 2026-11-01 | S1-H05 | not committed yet (working tree) | Buddy 1 (prepared with AI assistance) | PASS | `python -m unittest -v test_tel_decode`: 5 tests OK | Also an end-to-end run against a local Mosquitto broker with a stand-in car (listen, send stop, wrong-topic reject, last will OFFLINE, no-heartbeat warning) |
