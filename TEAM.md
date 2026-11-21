# Team and subsystems

Ported from the course template (`ProjectTemplate/TEAM.md`) and filled in
with this project's data. Pins come from `config/pins.h`, priorities from
`config/priorities.h`, interfaces from design.md section 3.3 and object
limits from `config/mtk3/config.h`.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

Student IDs are copied from the Team Information table in
`INF 2004 Embedded Systems.md`. GitHub usernames come from the owner's
entries in design.md (Team Role and Subsystem Ownership table).

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Telemetry: WiFi, MQTT, dashboard link | 2501299 (Damian) | Damian-oliveiro | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control (motors, encoders, PID) and Vehicle Controller | 2500530 (Ruo Chen) | leeruochen | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following and barcode decode | 2503319 (Jin Yong) | jyong1101 | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU processing and safety / lift-overturn response (incl. `task_safety.c`) | 2500709 (Vincent) | Vincent6430 | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle scanning and obstacle avoidance / recovery | 2500596 (Chay Han) | longchayhan03-eng (TODO(owner-to-confirm): design.md reads "Chay Hanlongchayhan03-eng", no separator) | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`,
`app_program/test_report.h`, this file) are changed only by agreement of the
team. The exception is claiming an empty row in this file (a pin, a
peripheral, an interface ID), which anyone may do (section 2).

Platform code in `src/` (`src/app/main.c`, `src/rtos/port/`, `src/debug/`,
`src/common/`) and `config/` is maintained centrally (Batch A/B completed by
Buddy 2 Ruo Chen with Buddy 3 Jin Yong as helper).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

This is the Week 8 pin map v2 (ARCHITECTURE.md D5). `config/pins.h` stays
the compile-time source of truth: every pin there is checked for collisions
at build time.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio (CYW43). Never use |
| GP0 | S2 | ENC_R_A, right wheel encoder phase A (Grove 1), PIO0 SM0. A/B label provisional |
| GP1 | S2 | ENC_R_B, right wheel encoder phase B (Grove 1), PIO0 SM0 |
| GP2 | S4 | IMU_SDA, I2C1 (Maker port; shared with Grove 2, so Grove 2 stays unused) |
| GP3 | S4 | IMU_SCL, I2C1 |
| GP4 | S5 | US_TRIG, ultrasonic trigger (Grove 3) |
| GP5 | S5 | US_ECHO, ultrasonic echo, GPIO IRQ (Grove 3); must be 3.3 V-safe (A8) |
| GP6 | spare | Second lead of Grove 5. Line-sensor DO is not wired. Input only |
| GP7 | spare | Second lead of Grove 7. Line-sensor DO is not wired. Input only |
| GP8 | S2 | MOTOR_L_A, left motor PWM slice 4 A (onboard driver) |
| GP9 | S2 | MOTOR_L_B, left motor PWM slice 4 B |
| GP10 | S2 | MOTOR_R_A, right motor PWM slice 5 A |
| GP11 | S2 | MOTOR_R_B, right motor PWM slice 5 B |
| GP12 | spare | Servo header. Never the servo pin |
| GP13 | spare | Servo header |
| GP14 | spare | Servo header. PWM slice 7 A, shares the slice with the servo: 50 Hz PWM or plain GPIO only |
| GP15 | S5 | SERVO_SIG, scanning servo, PWM slice 7 B, 50 Hz |
| GP16 | S2 | ENC_L_A, left wheel encoder phase A (Grove 4), PIO0 SM1. A/B label provisional |
| GP17 | S2 | ENC_L_B, left wheel encoder phase B (Grove 4), PIO0 SM1 |
| GP18 | *reserved* | Onboard NeoPixel (WS2812 via PIO), not implemented |
| GP19 | | unclaimed |
| GP20 | *reserved* | Onboard button 0 |
| GP21 | *reserved* | Onboard button 1 |
| GP22 | *reserved* | Onboard piezo buzzer |
| GP26 | S3 | IR_R, RIGHT line sensor AO, ADC0 (Grove 5). Grove 6's GP26 lead is the same pin |
| GP27 | S3 | IR_BC, barcode sensor DO, digital edge IRQ (Grove 6). Never `adc_gpio_init()` |
| GP28 | S3 | IR_C, CENTRE line sensor AO, ADC2 (Grove 7; acts as LEFT sensor under `-DS3_LF_STRADDLE=ON`). The GP28 VBAT-sense jumper must stay OPEN |

### Hardware checklist

Check these on the car before power-on with the Week 8 wiring:

- [ ] Grove 6's GP26 lead is taped or capped. It is the same pin as Grove 5's
      GP26 (IR_R); firmware cannot disconnect it.
- [ ] The GP28 VBAT-sense jumper is OPEN (unsoldered), or ADC2 (IR_C) reads
      the battery divider.
- [x] Line sensors: only VCC, GND and AO are wired (`drv_ir.c` verified on track). DO is not wired, so GP6
      and GP7 stay spare (inputs only).
- [x] Barcode sensor: VCC, GND and DO only. Its AO is not wired.
- [x] Encoders are powered from 3V3, never VIN or the servo header V pins.
- [x] Ultrasonic ECHO is 3.3 V-safe for GP5 (`S5-U05` / `S5-U26` verified on car, assumption A8).
- [x] Encoder harness is green/red; polarity confirmed by `S2-U03`/`S2-U07`/`S2-U08` (`MOTOR_L_DIR_INVERT=1U`, `MOTOR_R_DIR_INVERT=0U`, `ENC_L_DIR_INVERT=0U`, `ENC_R_DIR_INVERT=1U`, `ENC_CPR=616U`, `WHEEL_CIRC_MM=207U`, `CAR_WIDTH_MM=130U`).
- [x] IMU GY-511 on I2C1 (`GP2`/`GP3`, Grove 2 unused): `drv_imu.c` auto-detects LSM303D (`0x1D`) or LSM303DLHC (`0x19`/`0x1E`).

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "line following and barcode", "motion control" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | Line-Following and Barcode Decode, both in subsystem 3 |
| **Peripheral** | one hardware block on the RP2040 | I2C1, PIO0, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: subsystem 4 contains the IMU I2C code and owns I2C1.
Write it as `drv_<name>.c` / `drv_<name>.h` in that subsystem's folder,
using the Pico SDK `hardware_*` API, and call it only from the peripheral's
owning task. The kernel's own drivers in `device/` are not used.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* the kernel runs on one core (core 1 is unused), but it is
preemptive. A higher-priority task can interrupt another task in the middle
of an I2C or ADC transfer. If both drive the same block, their transfers get
mixed up, and the SDK drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C1 | S4 | IMU Processing | GP2, GP3 |
| ADC | S3 | Line-Following (channels 0 and 2) | GP26, GP28 |
| PWM slices 4, 5 | S2 | Motion Control | GP8–GP11 |
| PWM slice 7 | S5 | Obstacle Scanning (servo, channel B) | GP15 |
| PIO0 | S2 | Motion Control (encoders: SM0 right, SM1 left) | GP0, GP1, GP16, GP17 |
| PIO1 | S1 | Telemetry / MQTT (`TEL_BACKEND_WIFI=ON`, CYW43 SPI PIO) | GP23–25, GP29 |
| DMA (2 channels) | S1 | Telemetry / MQTT (`TEL_BACKEND_WIFI=ON`, CYW43 SPI DMA) | |
| IO_IRQ_BANK0 handlers | S5 / S3 / S1 | echo GP5 (S5), barcode GP27 (S3), CYW43 GP24 (`TEL_BACKEND_WIFI=ON`). Dispatcher: `hal_gpio_irq_register()` (`src/rtos/port/port_irq.c`) | GP5, GP24, GP27 |
| USB (CDC) | platform | debug drain task (debug console, `DBG_ENABLE` builds) | USB port |
| UART0, UART1, I2C0, SPI0, SPI1 | | unused | |

Wi-Fi/MQTT ownership (`TEL_BACKEND_WIFI=ON`): `task_tel.c` (Priority 9) is the sole caller of `drv_wifi.c` (`pico_cyw43_arch_lwip_poll` + `pico_lwip_mqtt`), keeping all network work inside one task (`app_program/subsystem1/MQTT.md`). The template's `cyw43_utk`/lwIP/MQTT design is reference only (`docs/TEMPLATE_NOTES.md`).

## 4. Task priorities

1 is the highest. These are our priorities, from `config/priorities.h`.
Change them only by team agreement. Write down every task you create. There
is one core, so there is no core column.

| Subsystem | Tasks (name – priority) |
|---|---|
| 4 | Safety / Lift Response (`safety`) – 1, IMU Processing (`imu`, 10 ms) – 8 |
| 2 | Vehicle Controller (`vc`) – 2, Motion Control (`motion`, 20 ms) – 3 |
| 3 | Line-Following (`lf`, 5 ms) – 4, Barcode Decode (`bc`, edge-woken / 2 ms capture poll) – 5 |
| 5 | Obstacle Scanning (`obs_scan`, 60 ms) – 6, Obstacle Avoidance (`obs_avoid`) – 7 |
| 1 | Telemetry (`tel`, 20 ms wake / 200 ms publish / 1 s heartbeat) – 9 |
| platform | kernel init task (`usermain`) – 1, debug drain (`dbg_drain`, 10 ms, `DBG_ENABLE` only) – 10, test task (`app_test`, `APP_TEST` only) – 11 |

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **receiving or owning**
subsystem is the object's owner and lists it in its own `subsystemN.h`; the
other side only sends to it or waits on it. Agree on the data format before
coding. In this project every IPC object is created centrally by `ipc_init()`
in `usermain()` step 1, before any `subsystemN_init()`.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Status |
|---|---|---|---|---|---|
| IF-01 | S3, S5, S1 → S2 (VC) | message buffer, 8 slots (owned by S2) | `MBF_NAV_IN` | `nav_input_t`: steering (LF, `TMO_POL`), LF junction/track-lost events & bypass/barcode commands (`5 ms` timeout), remote START/STOP/RESET commands (`5 ms` timeout) | Active (`main`) |
| IF-02 | S2 VC → S2 Motion (internal) | message buffer, 4 slots (owned by S2) | `MBF_MOTION_CMD` | `motion_cmd_t`; VC sends `TMO_POL`, Motion polls every `20 ms` | Active (`main`) |
| IF-03 | S5 Scan → S5 Avoid (internal) | message buffer, 2 slots (owned by S5) | `MBF_OBSTACLE` | `obstacle_profile_t`; newest profile wins. Sender drains then sends (`DATA_REPRESENTATION.md` 2.3) | Active (`main`) |
| IF-04 | S2 VC, S3 BC, S4 IMU/Safety, S1 Tel → S1 Tel | message buffer, 32 slots (owned by S1) | `MBF_TELEMETRY` | `tel_evt_t` (`DREP_TEL_HUMP`, `BARCODE` incl. `A–Z` + unmapped bit, `FAULT`, `STATE`); senders use `TMO_POL`, overflow dropped and counted (NFR1). Obstacle profiles use IF-12 instead of 32-bit `tel_evt_t` | Active (`main`) |
| IF-05 | echo ISR → S5 Scan | semaphore (owned by S5) | `SEM_ECHO` | echo received; `30 ms` watch / `12 ms` sweep timeout = nothing in range. Signalled from `echo_isr` via `hal_isr_sig()` | Active (`main`) |
| IF-06 | S4 IMU → S4 Safety; S4 Safety, S2 Motion, S5 Scan, S1 Tel, S3 LF → readers; S2 VC clears `FAULT_LATCHED` | eventflag, `TA_WSGL` (owned by S4) | `EVF_SAFETY` | `b0 LIFTED` & `b1 IMU_ALIVE` (S4 IMU; Safety waits `TWF_ORW\|TWF_BITCLR`); `b2 FAULT_LATCHED` (Safety sets, VC clears on stationary RESET); `b3 FAULT_STALL` & `b4 FAULT_ENCODER` (S2 Motion); `b5 FAULT_IMU_I2C` (S4 IMU); `b6 FAULT_IMU_WDOG` (S4 Safety); `b7 STATUS_NO_ECHO` (S5 Scan, informational); `b8 FAULT_WIFI_LOSS` (S1 Tel); `b9 FAULT_IR_SENSOR` (S3 LF). S5 Avoid reads `LIFTED\|FAULT_LATCHED` | Active (`main`) |
| IF-07 | S5 Avoid, S3 LF → readers | eventflag, `TA_WMUL` (owned by S5) | `EVF_NAV` | `AVOID_ACTIVE`, `LINE_REACQUIRED` (S5 Avoid), `LINE_FOUND` (S3 LF); readers poll via `tk_ref_flg()` | Active (`main`) |
| IF-08 | S2 Motion → P2 VC, P4 LF, P5 BC, P6 Obs Scan, P7 Obs Avoid, P8 IMU, P9 Tel | shared data under a mutex, `TA_INHERIT` (owned by S2) | `MTX_MOTION`, `motion_state_t` | speed, distance, encoder counts `[0]=L, [1]=R`, `tick_ms`; copy through `motion_get_state()` and release at once. **P1 Safety never takes it** | Active (`main`) |
| IF-09 | barcode ISR → S3 BC (internal) | ISR ring buffer, 64 edges (owned by S3) | barcode edge ring (`drep_edge_ring_t`) | 20-bit bar/space edge times (`bc_edge.c`); overflow forces `INVALID_READ_RESET` | Active (`main`) |
| IF-10 | PIO0 → S2 Motion (internal) | hardware counters read via `enc_get_count()` (owned by S2) | encoder count words | free-running signed X4 counts written only by Motion task (`ARCHITECTURE.md` D7) | Active (`main`) |
| IF-11 | S4 Safety → S2 | function call through `subsystem2.h` | `motion_emergency_brake()` | drives all four motor channels (`GP8`–`GP11`) LOW inside a critical section | Active (`main`) |
| IF-12 | S3 LF, S4 IMU, S5 Scan/Avoid, S3 BC → S1 Tel | function calls through `subsystem1.h` into single-writer shared words / seq-locked slot | `tel_set_ir_raw()`, `tel_set_range_cm()`, `tel_set_attitude()`, `tel_post_obstacle()`, `tel_post_barcode_letter()` | newest IR raws (`ir_r`, `ir_c`), forward range (`cm`), pitch/roll (`0.5°`), newest 37-bit obstacle profile (seq-locked slot), and decoded `A–Z` barcode letter + action; non-blocking (single stores / `TMO_POL`), merged on `main` | Active (`main`) |

#### Cross-subsystem read & configuration helpers (through `subsystemN.h`)

| Caller → Callee | Header & Function | Purpose |
|---|---|---|
| S1 Tel, S3 LF, S5 Scan, S5 Avoid, S4 IMU, S2 Motion → S2 VC | `../subsystem2/subsystem2.h`: `vc_get_system_state()` | Reads the 4-bit System FSM state (`VC_SYS_*`) for heartbeats, nav gating, and straight-motion IMU hump checks. |
| S1 Tel → S3 BC | `../subsystem3/subsystem3.h`: `bc_task_set_map()` / `bc_task_get_map()` | Updates/reads the runtime 4-letter Code 39 map (`LEFT`, `RIGHT`, `STRAIGHT`, `UTURN` in `'A'..'Z'`) from MQTT `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP`). |
| S2 Motion → S4 IMU | `../subsystem4/subsystem4.h`: `subsystem4_get_pitch_half_deg()` | Provides instantaneous filtered IMU pitch (`0.5°` units) to S2's gradual hump climb speed/torque assist (`task_motion.c`). |
| S2 VC, S1 Tel → S5 | `../subsystem5/subsystem5.h`: `subsystem5_sensors_ok()`, `subsystem5_get_avoid_state()`, `subsystem5_avoid_is_active()` | Power-up readiness gate, telemetry status summary, and avoidance-active status. |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/mtk3/config.h` and checked in
`config/mtk3/config_check.h`. If the team needs more, raise the limit there.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | Platform |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 2 | 2 | 2 | 2 | 3 (init, debug drain, test task) |
| Semaphore | 16 | | | | | 1 | |
| Event flag | 16 | | | | 1 | 1 | |
| Mailbox | 4 | | | | | | |
| Mutex | 4 | | 1 | | | | |
| Message buffer | 8 | 1 | 2 | | | 1 | |
| Fixed memory pool | 4 | | | | | | |
| Variable memory pool | 4 | | | | | | |
| Cyclic handler | 8 | 0 (`app_period_wait`) | 0 (`app_period_wait`) | 0 (`app_period_wait`) | 0 (`app_period_wait`) | 0 (`app_period_wait`) | |
| Alarm handler | 4 | | | | | | |

All five periodic tasks (`motion`, `lf`, `imu`, `obs_scan`, `tel`) use
`app_period_wait()` (`src/common/period.h`, `ARCHITECTURE.md` T8 and D2), so
**0 cyclic handlers** are created at runtime.
