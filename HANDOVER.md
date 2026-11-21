# HANDOVER: where the project stands and where each member continues

Read this first (or start at **`START_GUIDE.md`** for a full index of all important files, **`INTEGRATION_PHASES.md`** for team integration and lab course testing, and **`docs/EMBEDDED_OPTIMIZATIONS.md`** for low-level ARMv6-M / SWAR / fixed-point / hardware optimizations across all subsystems). Section 3 is for every team member (where to continue on your
own subsystem). Section 0 is for whoever continues on another device or with an
AI agent. The detailed evidence is in `.agents/tasks/` (index in section 9).

Last updated: 22 Nov 2026 — All 5 subsystems integrated on `main` (`APP_TEST=OFF`, Wi-Fi/MQTT + browser dashboard live); host tests 14/14 PASS, dashboard tests 7/7 PASS, `check_rules: PASS`, software block diagram updated at `docs/software_block_diagram.svg`. Previous updates: 13 Nov 2026 (`OPT-01`..`OPT-04` and `docs/EMBEDDED_OPTIMIZATIONS.md` by Buddy 2 Ruo Chen); 2 Nov 2026 (S1/S3/S4/S5 update by Buddy 4 Vincent); 31 Oct 2026 (Batch B & S2 complete by Buddy 2 Ruo Chen).

---

## 0. Quick start for the next device or agent

1. **Repo root** = the folder that holds this file. Older reports in `.agents/tasks/` use historical local paths; substitute your root.
2. **Set up** as in section 5 (SDK 1.5.1 at the default path, submodule at `15ed232`). If `git submodule status` starts with `-`, run `git submodule update --init`.
3. **Check the state:**
   - `git log --oneline -n 5`. Platform (Batch B) and all five subsystems (`subsystem1..5`) are merged on `main`.
   - Builds and checks in section 5 must pass: `0` warnings, `check_rules: PASS`, `14/14` host C tests, and `7/7` dashboard Python test files.
4. **Platform status (Batch B):** **COMPLETE** (`src/app/main.c`, port layer in `src/rtos/port/`, `src/common/`, `src/debug/`). All platform milestones (`M1-lite`, `M1` / `SYS-01`, `M2`) are closed on `main`.
5. **Do not write another member's subsystem code.** Each member owns their own `app_program/subsystemN/` and `testing/subsystemN/` folders (section 3).
   - **Exception: Subsystem 2 is open to everyone (owner permission, Buddy 2 Ruo Chen, 4 Nov 2026).** Any member, or an agent working for one, may edit `app_program/subsystem2/` and `testing/subsystem2/` when integration needs it, without asking first — log the change in `app_program/subsystem2/subsystem2_roadmap.md` ("Changes by other members"), explain why in the commit message, and keep section 5 checks passing. Shared files (`config/*.h`, `CMakeLists.txt`, `TEAM.md`) still need team agreement.
6. **Rules in section 6 are binding.** Owner decisions in section 7 are binding.
7. When you finish a piece of work: run the checks in section 5, update section 1 and your own row in section 3 of this file, and record test results in `testing/`.

---

## 1. Where things stand

| Item | Status |
|---|---|
| Batch A & Template merge + Week 8 pin map v2 | **Done & verified on `main`** (`56826c7`, `559f479`, `8d083ca`). Reference template stays untouched in `ProjectTemplate/`. |
| **Batch B: platform (boot, kernel port, debug console, IPC objects)** | **COMPLETE (`main`, M1 & M2 closed):** SDK `main()` (`src/app/main.c`), port layer (`src/rtos/port/`), system-message ring + `.uninitialized_data` RAM fault record + `dbg_` drain, `app_period_wait()`, stack high-water painting, packed `src/common/data_repr.h` (Week 8 formats), `SYS-01` PASS (`testing/system/TEST_LOG.md`), and host tests passing (`tests/host/`). |
| **Buddy 1 (Subsystem 1 — Telemetry & Dashboard)** | **Integrated on `main`:** Console and Wi-Fi/MQTT (`-DTEL_BACKEND_WIFI=ON`, CYW43 + lwIP poll mode) backends, non-blocking `MBF_TELEMETRY` + IF-12 snapshots (`tel_set_*`, `tel_post_obstacle`, `tel_post_barcode_letter`), single buffered state-change heartbeats, framed non-retained `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`), web dashboard (`tools/dashboard/tel_web.py`, Overview Demo Rubric, Barcode `A–Z` tab, Recording/Gaps, Calibration, Drive, Subsystem checks). Host `test_tel` + `7/7` dashboard Python test files PASS; `S1-U01..U10` PASS; `S1-M01` partial & `S1-M02` PASS (`testing/subsystem1/TEST_LOG.md`). Remaining: `S1-M01` driving portion, `S1-M03..M06` soak/drop/hump logs. |
| **Buddy 2 (Subsystem 2 — Motion & Vehicle Controller)** | **Integrated on `main`:** All `28/28` HW tests (`S2-U01..U28`) PASS (`MOTOR_L_DIR_INVERT=1U`, `ENC_R_DIR_INVERT=1U`, `WHEEL_CIRC_MM=207U`, `ENC_CPR=616U`, `MOTION_EFF_TRACK_MM=114U`); PID tuning & motion accuracy reports complete; `OPT-01..04` (`docs/EMBEDDED_OPTIMIZATIONS.md`), `S2-U24/U25` fault-latch reset fix, `BC_D_CMD_CM=60U`, `-DMOTION_STEER_TARGETS=1` differential steering targets, and gradual IMU-pitch hump climb speed/torque assist (`MOTION_HUMP_*`) applied. |
| **Buddy 3 (Subsystem 3 — Line Following & Barcode)** | **Integrated on `main`:** `drv_ir.c`, `lf_core.c` (edge follower), `lf_straddle.c` (`-DS3_LF_STRADDLE=ON`, `-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1`), `lf_link.c`, `task_lf.c` (5 ms sampling, 20 ms aligned steering, `tel_set_ir_raw()`, `EVF_NAV_LINE_FOUND`), and `drv_bc.c` / `bc_edge.c` / `bc_frame.c` / `bc_decode.c` / `task_bc.c` (`A–Z` Code 39 decode, `BC_MIN_SPEED_MMPS = 20 mm/s`, runtime 4-letter map `bc_task_set_map()`, `tel_post_barcode_letter()`). All `7` S3 host tests PASS (`test_bc_decode`, `test_bc_edge`, `test_bc_frame`, `test_lf_sanity`, `test_lf_core`, `test_lf_link`, `test_lf_straddle`). |
| **Buddy 4 (Subsystem 4 — IMU & Safety)** | **Integrated on `main`:** `drv_imu.c` auto-detects LSM303D (`0x1D`) or LSM303DLHC (`0x19`/`0x1E`); lift/overturn (`imu_lift.c`, `> 30°` tilt or `> 0.10 m/s` vertical speed for `50 ms`, replacing collision on 11 Nov per `docs/REQ_CHANGE_LIFT.md`) verified stopping wheels in `134 ms` (`S4-U23`); hump detector & estimator (`imu_hump.c`, `IMU_HUMP_WHEELBASE_MM = 95 mm` calibrated 20 Nov, `3°` entry / `2.5°` exit, `120 mm` level-end, `15 mm` min travel, `9 mm` min height, `max(tilt, climb)`), `tel_set_attitude()`, `tel_post_hump()`, `50 ms` Safety watchdog. Both host tests (`imu_lift`, `imu_hump`) PASS. |
| **Buddy 5 (Subsystem 5 — Obstacle Scan & Avoid)** | **Integrated on `main`:** `CAR_WIDTH_MM = 130 mm` measured; `task_obs_scan.c` watches ahead at `90°`, commands stop (`DREP_NAV_AVOID_SLOW` + `DREP_BYPASS_STOP`), confirms standstill, slews smoothly, and runs a stationary `5°` width sweep across `50°–130°` (4 pings / `12 ms` / `400 mm` cap -> 3-echo median, oblique dropout bridging, `OBS_BEAM_DEG = 40°`), feeding `tel_set_range_cm()` & `tel_post_obstacle()`. `task_obs_avoid.c` runs encoder-closed 4-turn `45°` bypass (`OBS_OBSTACLE_DEPTH_MM = 100 mm`) + directional rejoin search. Currently uses `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5 cm/s`, `60 s` budget; `S5-U01..U31` and full bypass chain tested on car). |
| **Software block diagram & documentation** | Updated vector + raster diagram at `docs/software_block_diagram.svg` and `.png`; host tests `14/14` PASS and all `3` firmware builds (`wifi`, `usb`, `test`) + `check_rules: PASS`. |

**What is next:** Run and save formal console logs (`testing/integration/logs/`, `testing/system/logs/`, `testing/subsystem*/logs/`) and unlisted YouTube links (`videos/README.md`) for the final sign-off tests (`INT-01..05`, `SYS-02..06`, `S1-M03/M04/M05`, post-95mm `S4-U26` & `S4-U22`, `5°`-sweep `S5-U33` & `INT-S5-02..04`). If running the `NFR14` (`<= 5.0 s`) bypass timing check at nominal bypass speed, clear `OBS_BYPASS_SLOW_BRINGUP` (`0U`) and restore `MOTION_SPEED_BYPASS_BRINGUP_CMPS` (`25U`) / `MOTION_SPEED_BYPASS_MIN_CMPS` (`15U`) in `config/thresholds.h` and `config/timing.h`.

---

## 2. Project in one paragraph

Autonomous line-following car on a Raspberry Pi **Pico W (RP2040)** on a Cytron Robo Pico
board, in **C** with **Pico SDK 1.5.1** and **micro T-Kernel 3.0** (official single-core
`tron-forum/mtk3_bsp`, branch `pico_rp2040`, submodule `external/mtk3_bsp` pinned at
`15ed232c08f2d89e54e514db79de150e224307e3`). CMake + Ninja on Windows, Barr C style. The SDK
owns startup, linker script and vector table; the kernel is a static library (`mtk3`) behind a
port layer. Team code is in `app_program/subsystem1..5/`; platform code goes in `src/`;
laptop dashboard tooling is in `tools/dashboard/`. Baseline sources of truth: `design.md`
(team design), `specs.md` (brief), `INF 2004 Embedded Systems.md` (Week 8 submission, read only),
plus all active deltas in `ARCHITECTURE.md`.

---

## 3. Each member: where to continue

General workflow for everyone:

1. Pull `main`. Work on your own branch, for example `buddy2/motor-driver`, and open a pull
   request to `main`. **Only edit your own `app_program/subsystemN/` and
   `testing/subsystemN/` folders.** Shared files (`app_program/app.h`, `app_main.c`,
   `test_report.h`, `TEAM.md`, `config/*.h`, `CMakeLists.txt`) change only by team agreement.
   Claiming an empty row in `TEAM.md` is allowed.
2. Put SDK hardware code in `drv_<name>.c` + a kernel-free `drv_<name>.h` (`<stdint.h>` only).
   Put tasks in `task_<name>.c` (kernel headers only). A `drv_*.c` never includes
   `subsystemN.h`. List every new `.c` in your folder's `sources.cmake`:
   `drv_*.c` in `ROBOCAR_SOURCES`, everything else in `ROBOCAR_KERNEL_SOURCES`.
3. Pins come only from `config/pins.h`; values from `config/thresholds.h` and `config/timing.h`.
   Replace a `TODO(owner-to-confirm, buddy N)` value only with a measured or agreed one, and
   say where it came from in the comment.
4. Define each test in `testing/subsystemN/TEST_LOG.md` first (IDs `SN-U01`, ...), then add a
   `TEST_CHECK` with the same ID to `subsystemN_test.c`. On-target test builds work after
   Batch B.
5. Build before every push (section 5). The build fails on any warning or rule violation.
6. When you finish a step, update your row below.
7. **Running on the car (since M1-lite, 31 Oct 2026):** start each task with
   `app_start_task(entry, TASK_PRI_x, TASK_STACK_x_BYTES, "name")` in `subsystemN_start()`.
   A periodic task waits with `app_period_wait()` (`src/common/period.h`), never with a
   plain `tk_dly_tsk(P)` loop: that runs every P + 1 ms. A task ends by returning from its
   function, never with `tk_ext_tsk` (rule R6). Log with `dbg_puts` / `dbg_printf`; the
   console and debugging are in `docs/DEBUG.md`. Messages are packed words (stage B7):
   build and read them only with the `drep_` functions in `src/common/data_repr.h`,
   for example `nav_input_t m = drep_nav_steer(err_q7);` then
   `tk_snd_mbf(MBF_NAV_IN, &m, sizeof m, TMO_POL)`. Field table: `docs/DATA_REPRESENTATION.md`.

Every subsystem today has `subsystemN.c` (init/start stubs), `subsystemN.h` (its interfaces as
comments), `subsystemN_test.c` (empty test) and `sources.cmake`. Interfaces between
subsystems are IF-01..IF-11 in `TEAM.md` section 5. Data formats are in
`docs/DATA_REPRESENTATION.md`. Low-level ARMv6-M assembly, SWAR bit-hacks, 64-bit division
elimination, and RP2040 peripheral offloads for each subsystem are in
**`docs/EMBEDDED_OPTIMIZATIONS.md`** (adopt after your baseline car tests pass).

### Buddy 1: Damian (2501299, Damian-oliveiro), telemetry, `app_program/subsystem1/`

| | |
|---|---|
| Tasks | Telemetry (`tel`), priority 9 (`TASK_PRI_TEL`, `TASK_STACK_TEL_BYTES`), wakes every 20 ms (`app_period_wait`, D2), publishes motion/sensor every 200 ms, heartbeat every 1 s |
| Pins / peripherals | none with console backend. Wi-Fi build: CYW43 (`GP23`–`25`, `GP29`), `PIO1`, 2 DMA channels (`TEL_BACKEND_WIFI=ON`, poll mode) |
| Interfaces | owns `IF-04` `MBF_TELEMETRY` (`tel_post_*()` / `tel_post_barcode_letter()`, never wait); sends validated remote commands into `IF-01` `MBF_NAV_IN` (5 ms); applies runtime barcode map `car/cmd/bc_map` via `bc_task_set_map()`; reads `IF-08` through `motion_get_state()`; sets/clears `EVF_SAFETY` `b8 FAULT_WIFI_LOSS`; owns `IF-12` (`tel_set_ir_raw/range_cm/attitude`, `tel_post_obstacle`) |
| Docs | `app_program/subsystem1/MQTT.md`; laptop CLI & browser dashboard in `tools/dashboard/` (`README.md`, `SUBSYSTEM_TESTS.md`) |
| Status | **Integrated on `main` and live over Wi-Fi/MQTT.** Console backend and Wi-Fi/MQTT backend (`-DTEL_BACKEND_WIFI=ON`, needs `config/secrets.h`) both build with 0 warnings and `check_rules: PASS`; single buffered state-change heartbeat (`s_outbox`), framed non-retained `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`), full `A–Z` barcode event decoding, and browser dashboard (`tools/dashboard/tel_web.py` with Overview Demo Rubric & Barcode tab) are active. `test_tel` (`S1-H01..H04`), `7/7` dashboard test files (`test_tel_*.py`), and `S1-U01..U10` PASS; `S1-M01` (partial) & `S1-M02` PASS in `testing/subsystem1/TEST_LOG.md`. Remaining: full driving `S1-M01`, `S1-M03..M06` soak/reconnect/hump logs. |

### Buddy 2: Ruo Chen (2500530, leeruochen), motion + Vehicle Controller, `app_program/subsystem2/`

| | |
|---|---|
| Tasks | Vehicle Controller (`vc`), priority 2; Motion Control (`motion`), priority 3 (20 ms PID, `MOTION_PERIOD_MS`) |
| Pins | motors `GP8`/`GP9` (left, PWM slice 4), `GP10`/`GP11` (right, slice 5), 20 kHz; encoders `GP0`/`GP1` (right, `PIO0 SM0`), `GP16`/`GP17` (left, `PIO0 SM1`) |
| Interfaces | owns `IF-01` `MBF_NAV_IN`, `IF-02` `MBF_MOTION_CMD`, `IF-08` `MTX_MOTION` + `motion_state_t`, `IF-10` encoder counts, `IF-11` `motion_emergency_brake()`; VC clears `FAULT_LATCHED`; reads `subsystem4_get_pitch_half_deg()` for gradual hump climb assist |
| Measured geometry | `MOTOR_L_DIR_INVERT=1U`, `MOTOR_R_DIR_INVERT=0U`, `ENC_L_DIR_INVERT=0U`, `ENC_R_DIR_INVERT=1U`, `WHEEL_CIRC_MM=207U`, `ENC_CPR=616U` (`2464` quad counts/rev), `MOTION_EFF_TRACK_MM=114U`, `POWER_GROVE_3V3_BUDGET_MA` verified (`72 mA` typ / `110 mA` pk < `300 mA`) |
| Status | **Integrated on `main` (`28/28` HW tests `S2-U01..U28` PASS).** Deliverables complete (`PID_TUNING_REPORT.md`, `MOTION_ACCURACY_EVALUATION.md`, `subsystem2_roadmap.md`); `OPT-01..04` (`docs/EMBEDDED_OPTIMIZATIONS.md`), `S2-U24/U25` fault-latch reset fix (`INTEGRATION_FIXES.md`), `BC_D_CMD_CM=60U`, `-DMOTION_STEER_TARGETS=1`, and gradual IMU-pitch hump climb speed/torque assist (`MOTION_HUMP_*`) active. |

### Buddy 3: Jin Yong (2503319, jyong1101), line following + barcode, `app_program/subsystem3/`

| | |
|---|---|
| Tasks | Line-Following (`lf`), priority 4 (5 ms sampling, 20 ms aligned steering); Barcode Decode (`bc`), priority 5 (GP27 IRQ-woken, 2 ms poll while capturing, D3) |
| Pins / peripherals | `IR_R` `GP26` = `ADC0` (RIGHT, Grove 5), `IR_C` `GP28` = `ADC2` (CENTRE, or LEFT under `-DS3_LF_STRADDLE=ON`, Grove 7, VBAT jumper OPEN); barcode `IR_BC` `GP27` digital edge IRQ (never `adc_gpio_init()`); owns the ADC |
| Interfaces | sends steering (`TMO_POL`), LF events & barcode commands (5 ms) into `IF-01`; sets `LINE_FOUND` in `IF-07` `EVF_NAV`; reads `IF-08` (`motion_get_state()`); owns `IF-09` edge ring (20-bit gaps, D8); calls `tel_set_ir_raw()` & `tel_post_barcode_letter()` (`IF-12`); exposes `bc_task_set_map()` / `bc_task_get_map()` (`subsystem3.h`) |
| Modules | Edge follower (`lf_core.c/.h`, `lf_link.c/.h`), straddle follower (`lf_straddle.c/.h`, active car build uses `-DS3_LF_STRADDLE=ON -DLF_SD_SIMPLE=1 -DLF_SD_BANG=1`), ADC sanity (`drv_ir.c/.h`, `lf_sanity.c/.h`), `task_lf.c/.h`, Code 39 decode & edge capture (`drv_bc.c/.h`, `bc_edge.c/.h`, `bc_frame.c/.h`, `bc_decode.c/.h`, `task_bc.c/.h`, `BC_MIN_SPEED_MMPS = 20 mm/s`), printable cards (`testing/subsystem3/cards/`) |
| Status | **Integrated on `main`.** Line following (both edge & straddle cores) and `A–Z` Code 39 barcode decoding + runtime mapping run live on the car; all `7` S3 host tests (`test_bc_decode`, `test_bc_edge`, `test_bc_frame`, `test_lf_sanity`, `test_lf_core`, `test_lf_link`, `test_lf_straddle`) PASS. |

### Buddy 4: Vincent (2500709, Vincent6430), IMU + safety, `app_program/subsystem4/`

| | |
|---|---|
| Tasks | Safety / Lift Response (`safety`), priority 1 (`task_safety.c`); IMU Processing (`imu`), priority 8 (10 ms `app_period_wait`) |
| Pins / peripherals | IMU GY-511 on `I2C1`, SDA `GP2`, SCL `GP3` (Grove 2 stays unused); owns `I2C1` |
| Interfaces | owns `IF-06` `EVF_SAFETY` (`LIFTED\|IMU_ALIVE` with `TWF_ORW\|TWF_BITCLR`, watchdog `SAFETY_IMU_WDOG_MS = 50 ms`); brakes only through `motion_emergency_brake()` in `../subsystem2/subsystem2.h` (`IF-11`); never takes `MTX_MOTION`; reads `IF-08` (`motion_get_state()`); calls `tel_set_attitude()` & `tel_post_hump()` (`IF-04` / `IF-12`); exposes `subsystem4_get_pitch_half_deg()` (`subsystem4.h`) |
| Modules | `drv_imu.c/.h` (I2C1 `400 kHz`, auto-detects LSM303DLHC `0x19`/`0x1E` or LSM303D `0x1D`/`0x1E`, integer pitch & roll); `imu_lift.c/.h` (lift/overturn `> 30°` or `> 0.10 m/s` for `50 ms`, replaced collision on 11 Nov per `docs/REQ_CHANGE_LIFT.md`); `imu_hump.c/.h` (calibrated `IMU_HUMP_WHEELBASE_MM = 95U` mm, `3°` entry / `2.5°` exit, `120 mm` level-end, `15 mm` min travel, `9 mm` min height, `max(tilt, climb)` + turn/speed-change gating) |
| Status | **Integrated on `main`.** Lift-while-driving stop verified at `134 ms` (`S4-U23`, < `200 ms` NFR10 limit) and hump/safety tests recorded in `testing/subsystem4/TEST_LOG.md`; both host tests (`imu_lift`, `imu_hump`) PASS at `95 mm` wheelbase. Note: lifting the car latches `FAULT_LATCHED` (end of run), so place the car before power-on or send `car/cmd/reset`. |

### Buddy 5: Chay Han (2500596, longchayhan03-eng), obstacle scan + avoid, `app_program/subsystem5/`

| | |
|---|---|
| Tasks | Obstacle Scanning (`obs_scan`), priority 6 (60 ms servo step); Obstacle Avoidance (`obs_avoid`), priority 7 |
| Pins / peripherals | ultrasonic TRIG `GP4`, ECHO `GP5` (Grove 3, 3.3 V-safe echo); servo `GP15` = PWM slice 7 B (never `GP12`); owns PWM slice 7 |
| Interfaces | owns `IF-03` `MBF_OBSTACLE` (drain-then-send newest-wins), `IF-05` `SEM_ECHO`, `IF-07` `EVF_NAV` (`AVOID_ACTIVE`, `LINE_REACQUIRED`) and `EVF_SAFETY b7 STATUS_NO_ECHO`; sends bypass commands into `IF-01`; reads `IF-08`; calls `tel_set_range_cm()` & `tel_post_obstacle()` (`IF-12`); exposes `subsystem5_sensors_ok()`, `subsystem5_get_avoid_state()`, `subsystem5_avoid_is_active()` |
| Modules | `drv_servo.c/.h`, `drv_ultrasonic.c/.h`, `task_obs_scan.c/.h` (`90°` watch-ahead -> stop & standstill confirm -> smooth slew & stationary `5°` sweep across `50°–130°`, 4 pings / `12 ms` / `400 mm` cap -> 3-echo median, oblique dropout bridging, `OBS_BEAM_DEG = 40°`, `OBS_EDGE_STEP_MM = 55 mm`), `task_obs_avoid.c/.h` (clearance decision using `CAR_WIDTH_MM = 130U` mm + `50 mm` margin, encoder-closed 4-turn `45°` bypass with `OBS_OBSTACLE_DEPTH_MM = 100U` mm, directional rejoin search + immediate `LINE_FOUND` handoff) |
| Status | **Integrated on `main`.** `S5-U01..U31` and full bypass chain tested on car (`testing/subsystem5/TEST_LOG.md`). Currently `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5 cm/s`, `60 s` budget) for reliable slow line reacquisition; revert to `0` (`25 cm/s` bypass, `15 cm/s` floor) for the `NFR14` (`<= 5.0 s`) timing run. |

---

## 4. Batch B: platform work (COMPLETE — Buddy 2, helper Buddy 3)

Batch B (`stages B0..B7` in `.agents/tasks/batchB-plan.md`) is **complete on `main`**:

| Stage | What | Status |
|---|---|---|
| B0–B3 | Docs, `dbg_` ring, `APP_TEST=ON`, `ipc_init()`, stack painting, kernel port layer (`src/rtos/port/`) | **done & merged** (PRs #1, #2) |
| B4 | SDK `main()` + debug drain: **milestone M1-lite** (tasks run on car, RAM fault record) | **done & merged** (PRs #3, #4) |
| B5 | Heartbeats + USB CDC bring-up test (`SYS-01`): **milestone M1** | **done & merged** (PR #5, `testing/system/TEST_LOG.md`) |
| B6–B7 | Baseline (`docs/measurements/week6-baseline.txt`) & packed `src/common/data_repr.h` (`week8-packed.txt`): **milestone M2** | **done & merged** (PRs #6, #7) |

---

## 5. Setting up a device and checking the build

1. Install the **Raspberry Pi Pico SDK v1.5.1 Windows installer** at
   `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1` (GCC 10.3.1, CMake 3.25.2, Ninja, Python,
   Git, OpenOCD). Another path works if you change `$R` below.
2. `git clone --recursive https://github.com/leeruochen/inf-2004-project.git`, or in an
   existing clone `git submodule update --init`. `git submodule status` must show
   ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`.
3. In every new PowerShell window, from the repo root:

```powershell
$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"
$env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"
$env:PICO_SDK_PATH = "$R\pico-sdk"
$CM = "$R\cmake\bin\cmake.exe"
$common = @("-G","Ninja","-DCMAKE_BUILD_TYPE=Debug",
  "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk",
  "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools",
  "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe",
  "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")
```

4. Firmware builds, host C tests, dashboard Python tests, and rule checks (all must pass with 0 warnings and `check_rules: PASS`):

```powershell
# USB-console firmware
& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb

# Active car Wi-Fi/MQTT + straddle firmware (requires local config/secrets.h)
& $CM -S . -B build/wifi @common -DDBG_BACKEND=USB -DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON '-DCMAKE_C_FLAGS=-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1'
& $CM --build build/wifi

# On-target test firmware (add -DAPP_TEST_ONLY=N for subsystem N only)
& $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
& $CM --build build/test

# Host C unit tests (14/14) & Python dashboard tests (7 files)
& $CM -S tests -B build/host -G Ninja
& $CM --build build/host
ctest --test-dir build/host --output-on-failure
python -m unittest discover -s tools/dashboard -p "test_*.py" -v

# Standalone rule checks
& $CM -DRULES_ROOT=. -P cmake/check_rules.cmake
& $CM -P cmake/check_rules_selftest.cmake
```

`DBG_BACKEND=UART0` is removed and fails at configure (`GP0`/`GP1` are the right encoder). USB CDC
is the only console; the Debug Probe is SWD only (`docs/DEBUG.md`).

---

## 6. Rules that must not be broken

- Never commit or push for someone else, never `git rm --cached`, `reset --hard`, `clean`, or
  force-push.
- Never edit `ProjectTemplate/**`, `external/mtk3_bsp/**`, `INF 2004 Embedded Systems.md`,
  `specs.md`. `design.md` is the team's document: an agent never edits it; only team members
  do. Show the diff to the team before changing `README.md` or `.gitignore`.
- `NOTICE.md`: preserve the owner's edit (`owner: 16`); only append.
- Names and student IDs come only from the Team Information table in
  `INF 2004 Embedded Systems.md`; GitHub usernames from the owner's entries in design.md.
- A `.c` file includes kernel headers (`<tk/...>`, `<tm/...>`, `<sys/...>`, `app.h`,
  `subsystemN.h`, `test_report.h`) **or** SDK headers (`pico/`, `hardware/`), never both.
- No blocking SDK calls in tasks (`sleep_ms`, `busy_wait*`, `mutex_enter_blocking`,
  `sem_acquire*`, `pio_sm_get_blocking`, `pio_sm_put_blocking`); use `tk_dly_tsk`. Only the debug
  drain task calls stdio; `tm_printf` only in the port boot/fault files. Every ISR goes through
  the SDK IRQ API with the `knl_taskindp` wrapper. One owning task per peripheral. Core 1 unused.
- `-Wall -Wextra -Werror` on our code only. No C bit-fields (masks and shifts).
- Template files contain UTF-8 characters: never round-trip them through
  `Get-Content | Set-Content`.
- Keep every `TODO(owner-to-confirm)` marker until the decision is made and recorded.

---

## 7. Decisions already made (binding)

Full text of early platform decisions: "OWNER CORRECTIONS" at the top of
`.agents/tasks/week8-impact-report.md` and `.agents/tasks/merge-week8-brief.md` (with subsequent
design deltas `D1`–`D17` recorded in [`ARCHITECTURE.md`](ARCHITECTURE.md)).

- Template merge Option C: our build and kernel; the template's `app_program/subsystemN/`,
  `TEAM.md`, `testing/`, `videos/` layout. Deviations T1–T7 in `ARCHITECTURE.md`.
- Pin map v2 (`config/pins.h`, `ARCHITECTURE.md` D5). Servo on `GP15`. Encoders on `PIO0`, one
  state machine per wheel, API `enc_get_count()` (D7).
- USB CDC is the only console (D6). Kernel text from `tm_snd_dat` (boot banner, system messages)
  goes into a system-message ring emptied by the debug drain task (C3). Faults go to a separate
  RAM fault record in `.uninitialized_data` with magic `0x464C5452` and IEEE CRC-32 (`crc32.h`).
- NFR2: `design.md` / `specs.md` stay untouched per Section 6; updated wording is tracked in D7
  (`ARCHITECTURE.md`) pending final top-speed zero-missed-counts sign-off.
- Data representation (D8, `docs/DATA_REPRESENTATION.md`): pitch in `0.5°` steps in 8 bits;
  `motion_cmd_t` / `nav_input_t` packed payloads; `obstacle_profile_t` with left/right clearances
  and 2-bit sequence; system FSM 4 bits; `EVF_SAFETY` fault bits (D9); binary MQTT wire frames
  `1..11` (D10).
- **D2 (`TEL_WAKE_PERIOD_MS` = 20 ms wake, `TEL_STATUS_PERIOD_MS` = 200 ms publish):** **Approved
  and implemented** on `main` (`app_program/subsystem1/task_tel.c`).
- **Calibrated chassis parameters (`config/thresholds.h`):** `ENC_CPR = 616U`, `WHEEL_CIRC_MM =
  207U`, `CAR_WIDTH_MM = 130U`, and `IMU_HUMP_WHEELBASE_MM = 95U` are measured and active on `main`.
- **Batch B decisions (C1–C8, O1–O9):** **Completed and merged** (`src/common/ipc_types.h` Week 8
  types, `PLATFORM_BRINGUP` / `PLATFORM_SELFTEST` default OFF, `PORT_FAULT_HALT`, Pico SDK 1.5.1 +
  GCC 10.3.1 toolchain, per-subsystem tests).
- **Post-integration design deltas (`D14`–`D17`, `ARCHITECTURE.md`):**
  - **D14 (Line Following straddle mode):** `S3_LF_STRADDLE` defaults `ON` in `CMakeLists.txt`; active
    car tracks the black tape edge with Centre (`GP28`) + Right (`GP26`) ADC sensors and
    `MOTION_STEER_TARGETS`.
  - **D15 (Lift/overturn safety + 95 mm hump calibration):** `docs/REQ_CHANGE_LIFT.md` replaces
    collision detection with lift/overturn safety (`>30°` tilt or `>0.10 m/s` vertical velocity) plus
    the 50 ms `IMU_ALIVE` watchdog (`task_safety.c`); `IMU_HUMP_WHEELBASE_MM` updated to `95U` with
    motion hump climb boost (`tests/host/test_imu_hump.c` is updated to `WHEELBASE_MM = 95.0` and
    passes `S4-U24` in `ctest`).
  - **D16 (Obstacle 90° watch & slow bypass bring-up):** `task_obs_scan` monitors straight ahead at
    `90°` (`25 cm` threshold), halts the car, runs a fine `5°` scan, and executes an encoder-closed
    bypass; `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5/5/5 cm/s`) is temporarily active during bring-up.
  - **D17 (Dynamic Barcode A–Z & `car/cmd/bc_map`):** Barcode decodes Code 39 `A–Z` and executes
    the runtime dashboard mapping within `BC_D_CMD_CM = 60 cm`; `car/cmd/bc_map` uses framed type `11`
    (`DREP_FT_CMD_BC_MAP`) with non-retained publish, and Telemetry emits a single buffered
    state-change heartbeat (`app_program/subsystem1/MQTT.md`).

---

## 8. Open questions & remaining follow-ups

| # | Item | Current status / next action |
|---|---|---|
| 1 | Buddy 5 GitHub username in `design.md` (`"Chay Hanlongchayhan03-eng"`) | Open in `design.md` (untouched per Section 6); `TEAM.md` uses `longchayhan03-eng`. |
| 2 | `design.md` Team Role table line break on Buddy 1's cell | Open in `design.md` (untouched per Section 6). |
| 3 | `README.md` stale Phase 1 / build TODO text | `README.md` kept untouched per Section 6; update after showing diff to team. |
| 4 | `D2` (`20 ms` wake), `ENC_CPR`/`WHEEL_CIRC_MM`/`CAR_WIDTH_MM`, `95 mm` `imu_hump` host test (`S4-U24`), single state-change heartbeat, framed non-retained `car/cmd/bc_map`, Barr C masks/shifts, Batch B `C1–C8` | **Resolved** and merged on `main`. |
| 5 | `S1-M01`, `S1-M03`..`S1-M06` formal Wi-Fi soak / drop / hump summary logs | Remaining formal captures in `testing/subsystem1/logs/` (`MQTT.md` Section 9). |
| 6 | `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5/5/5 cm/s`) & wide-block ultrasonic width accuracy | Buddy 5 to tune speeds toward `NFR14` (`<= 3.0 s / 5.0 s`) and investigate `50–65 mm` reads on `150 mm` block. |
| 7 | NFR2 wording sign-off (`design.md` / `specs.md`) & `NOTICE.md` instructor attribution | Team sign-off before final submission. |
| 8 | Full-course `INT-01..INT-05` & `SYS-01..SYS-06` hardware logs & demo videos | Run final course tests and log verified results in `testing/integration/TEST_LOG.md`, `testing/system/TEST_LOG.md`, and `videos/README.md`. |

---

## 9. Reports in `.agents/tasks/`

| File | What it is |
|---|---|
| `batchB-plan.md` | **Batch B plan** (stages B0-B7, milestones M1-lite/M1/M2, decisions C1-C8, O1-O11, owner questions, workflow and gate commands); section 5 lists the stale parts of the reports below |
| `batchB-b0-review-report.md` | Buddy 2's review of B0: official gate results, decision-by-decision verdicts, the fixes and recommendations Rev 6 applies |
| `merge-week8-summary.md` | Latest summary: files changed, assumptions, full verification table, owner questions |
| `merge-week8-diffs.patch` | Diff of the merge + Week 8 work (before commit) |
| `merge-verify.md`, `week8-verify.md`, `*-review.md/.json` | Stage evidence and review gates |
| `merge-week8-plan.md`, `merge-week8-brief.md` | The plan and brief for the finished merge + Week 8 work |
| `week8-impact-report.md` | Week 8 analysis (pins, quadrature, data representation); binding OWNER CORRECTIONS at the top |
| `template-merge-report.md` | Template inventory and the Option C plan |
| `mtk3-rp2040-integration-report.md` | Kernel integration analysis: port layer, interrupts, tick, USB CDC test (Batch B reference) |
| `batchA-*.md/.json` | Batch A plan, evidence and review |
| `step0-verify.md`, `merge-samename-diffs.md` | Steps 0-1 evidence |
