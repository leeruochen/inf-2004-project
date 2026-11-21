# New Device & AI Agent Start Guide (`START_GUIDE.md`)

> **Purpose:** This file is the **single entry point** when switching to a new computer, starting a fresh AI agent session, or onboarding a teammate for integration.  
> Instead of hunting across folders, tell your AI agent on the new device:
>
> ```text
> Read START_GUIDE.md first, then read the Tier 1 and Tier 2 files listed in it.
> I am Buddy <N> (<Name>) working on <task>.
> ```
>
> **Status as of 22 Nov 2026:** All 5 subsystems (`subsystem1..5`) and the Batch B platform are integrated on `main` (`APP_TEST=OFF`, Wi-Fi/MQTT + Python web dashboard live).

---

## 1. Why Files Stay at Their Current Paths (To Prevent Merge Conflicts)

All top-level project guides (`START_GUIDE.md`, `HANDOVER.md`, `INTEGRATION_PHASES.md`, `INTEGRATION_FIXES.md`, `TEAM.md`, `ARCHITECTURE.md`, `design.md`, `specs.md`) are kept together in the **repository root** for tidiness, while low-level technical specs (`docs/DATA_REPRESENTATION.md`, `docs/DEBUG.md`) and config headers (`config/*.h`) remain at the paths referenced by source code and CMake scripts.

**Do not move or rename those existing files:**
- Source-code comments, CMake scripts (`cmake/check_rules.cmake`), and documentation link to these exact file paths.
- Moving them into a different folder would break script references and cause unnecessary Git merge conflicts.
- Instead, **this `START_GUIDE.md` acts as the Master Index** linking every critical file in one place in the exact order an agent or developer should read them.

---

## 2. First-Time Setup on a New Windows Device

When cloning or pulling this repository on a new machine:

1. **Install Pico SDK v1.5.1** (official Windows installer at default path `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\`).
2. **Pull & Initialize the Kernel Submodule (`external/mtk3_bsp`)**:
   ```powershell
   git pull origin main
   git submodule update --init
   ```
3. **Wi-Fi / MQTT Credentials (required only for `-DTEL_BACKEND_WIFI=ON`)**:
   - Copy `config/secrets.h.example` to `config/secrets.h` (git-ignored) and fill in your 2.4 GHz Wi-Fi SSID, password, and laptop broker IP. Never commit `config/secrets.h`.
4. **Configure & Run Firmware Builds, Rule Checks, Host Tests, and Dashboard Tests**:
   ```powershell
   $SDK = 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1'
   $env:PATH = "$SDK/cmake/bin;$SDK/gcc-arm-none-eabi/bin;$SDK/ninja;$SDK/python;$SDK/git/cmd;$env:PATH"
   $env:PICO_SDK_PATH = "$SDK/pico-sdk"
   $CM  = "$SDK/cmake/bin/cmake.exe"
   $common = @(
     '-G','Ninja',
     '-DCMAKE_BUILD_TYPE=Debug',
     "-DCMAKE_MAKE_PROGRAM=$SDK/ninja/ninja.exe",
     "-DPICO_SDK_PATH=$SDK/pico-sdk",
     "-Dpico-sdk-tools_DIR=$SDK/pico-sdk-tools",
     "-DPICO_TOOLCHAIN_PATH=$SDK/gcc-arm-none-eabi",
     '-DPICO_BOARD=pico_w'
   )

   # 1. USB-console production build (no Wi-Fi radio, APP_TEST=OFF)
   & $CM -S . -B build/usb @common -DDBG_BACKEND=USB
   & $CM --build build/usb --parallel 8

   # 2. Active Wi-Fi/MQTT + straddle line-following car build (requires config/secrets.h)
   & $CM -S . -B build/wifi @common -DDBG_BACKEND=USB -DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON '-DCMAKE_C_FLAGS=-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1'
   & $CM --build build/wifi --parallel 8
   # Optional: add -DTEL_STATUS_PERIOD_MS=1000 when capturing 1 s console status lines for the dashboard.

   # 3. On-target unit test build (add -DAPP_TEST_ONLY=N for a single subsystem 1..5)
   & $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
   & $CM --build build/test --parallel 8

   # 4. Release build (DBG_ENABLE=OFF)
   & $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release -DDBG_BACKEND=USB
   & $CM --build build/release --parallel 8

   # 5. Host C unit tests (14 test executables, plain C compiler, no Pico board required)
   & $CM -S tests -B build/host -G Ninja
   & $CM --build build/host
   ctest --test-dir build/host --output-on-failure

   # 6. Python dashboard unit tests (7 test files, Python 3.8+)
   python -m unittest discover -s tools/dashboard -p "test_*.py" -v

   # 7. Standalone static rule check (R1..R6; also runs automatically on firmware builds)
   & $CM -DRULES_ROOT=. -P cmake/check_rules.cmake
   ```
   All firmware builds must complete with **0 warnings** (`-Wall -Wextra -Werror`) and `check_rules: PASS`, and all 14 host tests and 7 dashboard test files must pass. To launch the browser dashboard locally: double-click `tools\dashboard\dashboard.cmd` or run `python tools/dashboard/tel_web.py` (`pip install paho-mqtt` needed for live MQTT).

---

## 3. Master Reading List for AI Agents (Ordered by Priority)

### Tier 1 — Mandatory Core Context (Read First on Every Session)

| Order | File Path | What It Contains & Why It Is Mandatory |
|---|---|---|
| **1** | [`HANDOVER.md`](HANDOVER.md) | **Master project status & rules:** Current subsystem and platform state, binding coding rules (**R1–R6**), edit restrictions (Section 6), build/test instructions (Section 5), and settled/open decisions (Sections 7–8). |
| **2** | [`TEAM.md`](TEAM.md) | **Hardware & RTOS ownership table:** Pin claims (`GP0`–`GP29`), hardware wiring checklist, peripheral owning tasks (`I2C1`, `ADC`, `PWM`, `PIO0/1`, `DMA`, `IO_IRQ_BANK0`), task priorities (`P1`–`P11`), interfaces (`IF-01`..`IF-12`), and kernel object budget. |
| **3** | [`INTEGRATION_PHASES.md`](INTEGRATION_PHASES.md) | **System integration & lab course guide:** How `usermain()` starts all 5 subsystems, exact C API/IPC contracts with Subsystem 2 (VC + Motion), speed modes, pairwise tests (`INT-01`..`05`), and full track tests (`SYS-02`..`06`). |
| **4** | [`INTEGRATION_FIXES.md`](INTEGRATION_FIXES.md) | **Resolved integration fixes note (historical reference):** Documents the resolved `S2-U25` / `FAULT_LATCHED` VC deadlock fix, nominal `25 cm/s` bypass vs temporary `5 cm/s` slow bring-up override (`OBS_BYPASS_SLOW_BRINGUP`), and `NFR15` tolerance refinement. |

---

### Tier 2 — Shared Code Contracts & Architecture (Read Before Writing/Integrating Code)

| Order | File Path | What It Contains |
|---|---|---|
| **5** | [`config/pins.h`](config/pins.h)<br>[`config/thresholds.h`](config/thresholds.h)<br>[`config/timing.h`](config/timing.h)<br>[`config/priorities.h`](config/priorities.h) | **Compile-time source of truth** for all GPIO pins, calibrated motor/encoder inversions (`MOTOR_L_DIR_INVERT=1U`, `ENC_R_DIR_INVERT=1U`), wheel/car geometry (`ENC_CPR=616U`, `WHEEL_CIRC_MM=207U`, `CAR_WIDTH_MM=130U`), IMU calibrated wheelbase (`95U` mm, `3°` hump pitch, lift thresholds), barcode (`BC_D_CMD_CM=60U`, `BC_MIN_SPEED_MMPS=20U`), speeds (`OBS_BYPASS_SLOW_BRINGUP=1U`), task periods, and priorities. |
| **6** | [`src/common/ipc.h`](src/common/ipc.h)<br>[`src/common/ipc_types.h`](src/common/ipc_types.h)<br>[`src/common/data_repr.h`](src/common/data_repr.h)<br>[`docs/DATA_REPRESENTATION.md`](docs/DATA_REPRESENTATION.md) | **IPC handles, eventflag bits & bit-packed wire protocol:** Defines `MBF_NAV_IN`, `MBF_MOTION_CMD`, `MBF_OBSTACLE`, `MBF_TELEMETRY`, `EVF_SAFETY` (`b0..b9`), `EVF_NAV`, `MTX_MOTION`, `SEM_ECHO`, all `drep_*` pack/unpack/validate inline functions, and MQTT binary frame formats (`DREP_FT_*` types `1..11`). |
| **7** | [`src/rtos/port/hal.h`](src/rtos/port/hal.h)<br>[`src/common/period.h`](src/common/period.h)<br>[`src/debug/dbg.h`](src/debug/dbg.h)<br>[`docs/DEBUG.md`](docs/DEBUG.md) | **Platform HAL, periodic timing & USB CDC console:** `app_period_init()` / `app_period_wait()` (drift-free periodic tasks), `hal_gpio_irq_register()` (`IO_IRQ_BANK0` dispatcher), `hal_isr_sig()` / `hal_isr_wake()`, RAM fault record, and non-blocking `dbg_puts()` / `dbg_printf()`. |
| **8** | [`app_program/app.h`](app_program/app.h)<br>[`app_program/app_main.c`](app_program/app_main.c)<br>[`app_program/subsystem2/subsystem2.h`](app_program/subsystem2/subsystem2.h) | **Application startup & Subsystem 2 hub header:** `usermain()` startup order (`ipc_init` -> `subsystemN_init` -> `subsystemN_start`), `app_start_task()`, the 10-state System FSM (`VC_SYS_*`), `vc_get_system_state()`, `motion_get_state()`, `motion_emergency_brake()`, and public motion wrappers (`moveForward`, `turnLeft`, `turnRight`, `stop`). |
| **9** | [`ARCHITECTURE.md`](ARCHITECTURE.md)<br>[`docs/software_block_diagram.svg`](docs/software_block_diagram.svg)<br>[`docs/REQ_CHANGE_LIFT.md`](docs/REQ_CHANGE_LIFT.md)<br>[`docs/EMBEDDED_OPTIMIZATIONS.md`](docs/EMBEDDED_OPTIMIZATIONS.md)<br>[`design.md`](design.md) / [`specs.md`](specs.md) | **System design, architecture & deltas:** Block diagram ([PNG](docs/software_block_diagram.png)), final requirements (`FR1`–`FR14`, `NFR1`–`NFR17`), design deltas (`D1`–`D17`), template deviations (`T1`–`T8`), 11 Nov lift/overturn requirement change, and ARMv6-M optimization guide (`OPT-01`..`OPT-20`). Note: `design.md` and `specs.md` are baseline reference documents (never agent-edited); active differences are recorded in `ARCHITECTURE.md`. |

---

### Tier 3 — Subsystem-Specific Files (Read Based on Which Buddy You Are)

#### Buddy 1: Damian (`2501299`) — Telemetry, Wi-Fi/MQTT & Dashboard (**Integrated on `main`**)
- [`app_program/subsystem1/MQTT.md`](app_program/subsystem1/MQTT.md) — Topics (`car/telemetry/*`, `car/status`, `car/cmd/*`), QoS, binary frame payloads (`1..11`), command validation rules, link state machine & backoff (`1..30 s`), Wi-Fi setup.
- [`app_program/subsystem1/subsystem1.h`](app_program/subsystem1/subsystem1.h), [`subsystem1.c`](app_program/subsystem1/subsystem1.c) — Public `tel_post_*()` / `tel_post_barcode_letter()` / IF-12 `tel_set_*()` / `tel_post_obstacle()` API and startup.
- [`app_program/subsystem1/task_tel.h`](app_program/subsystem1/task_tel.h), [`task_tel.c`](app_program/subsystem1/task_tel.c) — Priority 9 task (`20 ms` wake, `200 ms` publish, `1 s` heartbeat, buffered state-change heartbeats, command dispatch including framed non-retained `car/cmd/bc_map`).
- [`app_program/subsystem1/tel_link.h`](app_program/subsystem1/tel_link.h) / [`.c`](app_program/subsystem1/tel_link.c), [`tel_outbox.h`](app_program/subsystem1/tel_outbox.h) / [`.c`](app_program/subsystem1/tel_outbox.c), [`tel_wire.h`](app_program/subsystem1/tel_wire.h) / [`.c`](app_program/subsystem1/tel_wire.c) — Pure C connection FSM, 16-slot event outbox, topic/command validator (`tel_wire_cmd`, `tel_wire_bc_map`).
- [`app_program/subsystem1/drv_wifi.h`](app_program/subsystem1/drv_wifi.h), [`drv_wifi.c`](app_program/subsystem1/drv_wifi.c), [`lwipopts.h`](app_program/subsystem1/lwipopts.h), [`tel_cyw43_config.h`](app_program/subsystem1/tel_cyw43_config.h) — Poll-mode CYW43 + lwIP MQTT client (`TEL_BACKEND_WIFI=ON`, needs `config/secrets.h`).
- [`app_program/subsystem1/subsystem1_test.c`](app_program/subsystem1/subsystem1_test.c), [`tests/host/test_tel.c`](tests/host/test_tel.c), [`testing/subsystem1/TEST_LOG.md`](testing/subsystem1/TEST_LOG.md) — On-target (`S1-U01..U10`), host (`tel`), and manual (`S1-M01..M06`) tests.
- [`tools/dashboard/README.md`](tools/dashboard/README.md), [`tools/dashboard/SUBSYSTEM_TESTS.md`](tools/dashboard/SUBSYSTEM_TESTS.md), `tools/dashboard/tel_*.py`, [`tel_web.html`](tools/dashboard/tel_web.html), and `tools/dashboard/test_tel_*.py` (`7` test files) — CLI (`tel_dashboard.py`) and browser web UI (`tel_web.py`) featuring the Overview Demo Rubric (`tel_demo.py`), Barcode `A–Z` mapping tab, Recording/Data-gap tables (`tel_record.py`), Line calibration (`tel_calib.py`), Drive controls (`tel_control.py`), and Subsystem test checks (`tel_views.py`).

#### Buddy 2: Ruo Chen (`2500530`) — Motion Control & Vehicle Controller (**Integrated on `main`**)
- [`app_program/subsystem2/subsystem2_roadmap.md`](app_program/subsystem2/subsystem2_roadmap.md) — Step-by-step history (`S0`–`S8` + post-`S8` track integration updates) and decisions (`Q1`–`Q4`).
- [`app_program/subsystem2/drv_motor.h`](app_program/subsystem2/drv_motor.h), [`drv_motor.c`](app_program/subsystem2/drv_motor.c) — 20 kHz PWM driver on slices 4 & 5 (`GP8`–`GP11`).
- [`app_program/subsystem2/drv_encoder.h`](app_program/subsystem2/drv_encoder.h), [`drv_encoder.c`](app_program/subsystem2/drv_encoder.c), [`quadrature_encoder.pio`](app_program/subsystem2/quadrature_encoder.pio) — PIO0 X4 quadrature decoder (`SM0` right `GP0`/`GP1`, `SM1` left `GP16`/`GP17`).
- [`app_program/subsystem2/task_motion.h`](app_program/subsystem2/task_motion.h), [`task_motion.c`](app_program/subsystem2/task_motion.c) — Priority 3, 20 ms PID velocity + cumulative heading sync (`MOTION_EFF_TRACK_MM = 114U`), `-DMOTION_STEER_TARGETS=1` differential steering targets, and gradual IMU-pitch hump climb speed/torque assist (`MOTION_HUMP_*`).
- [`app_program/subsystem2/task_vc.h`](app_program/subsystem2/task_vc.h), [`task_vc.c`](app_program/subsystem2/task_vc.c) — Priority 2 Vehicle Controller (`MBF_NAV_IN` arbiter & 10-state System FSM).
- [`app_program/subsystem2/subsystem2_test.c`](app_program/subsystem2/subsystem2_test.c) — On-target unit, bench calibration, and floor tests (`S2-U01`..`S2-U28`).
- [`testing/subsystem2/TEST_LOG.md`](testing/subsystem2/TEST_LOG.md), [`PID_TUNING_REPORT.md`](testing/subsystem2/PID_TUNING_REPORT.md), [`MOTION_ACCURACY_EVALUATION.md`](testing/subsystem2/MOTION_ACCURACY_EVALUATION.md) — Hardware test logs and deliverable reports.

#### Buddy 3: Jin Yong (`2503319`) — Line-Following & Code 39 Barcode (**Integrated on `main`**)
- [`app_program/subsystem3/subsystem3.h`](app_program/subsystem3/subsystem3.h), [`subsystem3.c`](app_program/subsystem3/subsystem3.c) — Public barcode mapping API (`bc_task_set_map()`, `bc_task_get_map()`) and init/start of IR ADC, GP27 IRQ, LF task, and BC task.
- [`app_program/subsystem3/drv_ir.h`](app_program/subsystem3/drv_ir.h) / [`.c`](app_program/subsystem3/drv_ir.c), [`lf_sanity.h`](app_program/subsystem3/lf_sanity.h) / [`.c`](app_program/subsystem3/lf_sanity.c) — Dual-channel ADC driver (`GP28`/`ADC2` CENTRE/LEFT, `GP26`/`ADC0` RIGHT) and boot sanity checks.
- [`app_program/subsystem3/lf_core.h`](app_program/subsystem3/lf_core.h) / [`.c`](app_program/subsystem3/lf_core.c), [`lf_straddle.h`](app_program/subsystem3/lf_straddle.h) / [`.c`](app_program/subsystem3/lf_straddle.c), [`lf_link.h`](app_program/subsystem3/lf_link.h) / [`.c`](app_program/subsystem3/lf_link.c), [`task_lf.h`](app_program/subsystem3/task_lf.h) / [`.c`](app_program/subsystem3/task_lf.c) — Priority 4, 5 ms line following (`lf_core` edge mode by default, or `-DS3_LF_STRADDLE=ON` with `-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1` for active straddle mode), posts steering every `20 ms` (`lf_link_post_slot`), sets `EVF_NAV_LINE_FOUND`, and feeds `tel_set_ir_raw()`.
- [`app_program/subsystem3/drv_bc.h`](app_program/subsystem3/drv_bc.h) / [`.c`](app_program/subsystem3/drv_bc.c), [`bc_edge.h`](app_program/subsystem3/bc_edge.h) / [`.c`](app_program/subsystem3/bc_edge.c), [`bc_frame.h`](app_program/subsystem3/bc_frame.h) / [`.c`](app_program/subsystem3/bc_frame.c), [`bc_decode.h`](app_program/subsystem3/bc_decode.h) / [`.c`](app_program/subsystem3/bc_decode.c), [`task_bc.h`](app_program/subsystem3/task_bc.h) / [`.c`](app_program/subsystem3/task_bc.c) — Priority 5 GP27 Code 39 `A–Z` barcode decoder (`BC_MIN_SPEED_MMPS = 20 mm/s`), runtime 4-action mapping, and `tel_post_barcode_letter()`.
- [`app_program/subsystem3/subsystem3_test.c`](app_program/subsystem3/subsystem3_test.c), [`lf_hw_test.h`](app_program/subsystem3/lf_hw_test.h) / [`.c`](app_program/subsystem3/lf_hw_test.c), [`lf_run_test.h`](app_program/subsystem3/lf_run_test.h) / [`.c`](app_program/subsystem3/lf_run_test.c), [`lf_step_test.h`](app_program/subsystem3/lf_step_test.h) / [`.c`](app_program/subsystem3/lf_step_test.c), `tests/host/test_bc_*.c`, `tests/host/test_lf_*.c`, [`testing/subsystem3/TEST_LOG.md`](testing/subsystem3/TEST_LOG.md), `testing/subsystem3/cards/`.

#### Buddy 4: Vincent (`2500709`) — IMU Processing & Safety / Lift-Overturn (**Integrated on `main`**)
- [`docs/REQ_CHANGE_LIFT.md`](docs/REQ_CHANGE_LIFT.md) — 11 Nov 2026 requirement change replacing IMU collision detection with lift/overturn detection.
- [`app_program/subsystem4/subsystem4.h`](app_program/subsystem4/subsystem4.h), [`subsystem4.c`](app_program/subsystem4/subsystem4.c) — Public accessors (`subsystem4_get_pitch_half_deg()`, `subsystem4_get_accel_mag_mg()`, `subsystem4_imu_found()`).
- [`app_program/subsystem4/drv_imu.h`](app_program/subsystem4/drv_imu.h), [`drv_imu.c`](app_program/subsystem4/drv_imu.c) — I2C1 `400 kHz` driver auto-detecting LSM303DLHC (`0x19`/`0x1E`) or LSM303D (`0x1D`/`0x1E`), fixed-point `pitch`/`roll`.
- [`app_program/subsystem4/imu_lift.h`](app_program/subsystem4/imu_lift.h) / [`.c`](app_program/subsystem4/imu_lift.c), [`imu_hump.h`](app_program/subsystem4/imu_hump.h) / [`.c`](app_program/subsystem4/imu_hump.c) — Pure C lift/overturn detector (`> 30°` or `> 0.10 m/s` for `50 ms`) and hump height estimator (`max(tilt, climb)`, calibrated `IMU_HUMP_WHEELBASE_MM = 95U` mm, `3°` entry / `2.5°` exit).
- [`app_program/subsystem4/task_imu.h`](app_program/subsystem4/task_imu.h) / [`.c`](app_program/subsystem4/task_imu.c), [`task_safety.h`](app_program/subsystem4/task_safety.h) / [`.c`](app_program/subsystem4/task_safety.c) — Priority 8 IMU task (`10 ms` loop, `tel_set_attitude()`, `tel_post_hump()`, `EVF_SAFETY_IMU_ALIVE`/`LIFTED`) and Priority 1 Safety task (`50 ms` watchdog, `motion_emergency_brake()` via IF-11, `FAULT_LATCHED`).
- [`app_program/subsystem4/subsystem4_test.c`](app_program/subsystem4/subsystem4_test.c), [`tests/host/test_imu_lift.c`](tests/host/test_imu_lift.c), [`tests/host/test_imu_hump.c`](tests/host/test_imu_hump.c), [`testing/subsystem4/TEST_LOG.md`](testing/subsystem4/TEST_LOG.md), [`testing/subsystem4/s4_test.ps1`](testing/subsystem4/s4_test.ps1).

#### Buddy 5: Chay Han (`2500596`) — Obstacle Scanning & Avoidance (**Integrated on `main`**)
- [`app_program/subsystem5/subsystem5.h`](app_program/subsystem5/subsystem5.h), [`subsystem5.c`](app_program/subsystem5/subsystem5.c) — Public status API (`subsystem5_sensors_ok()`, `subsystem5_get_avoid_state()`, `subsystem5_avoid_is_active()`).
- [`app_program/subsystem5/drv_servo.h`](app_program/subsystem5/drv_servo.h) / [`.c`](app_program/subsystem5/drv_servo.c), [`drv_ultrasonic.h`](app_program/subsystem5/drv_ultrasonic.h) / [`.c`](app_program/subsystem5/drv_ultrasonic.c) — 50 Hz servo on `GP15` (slice 7B) and HC-SR04 trigger/echo ISR (`GP4`/`GP5`, signals `SEM_ECHO`).
- [`app_program/subsystem5/task_obs_scan.h`](app_program/subsystem5/task_obs_scan.h), [`task_obs_scan.c`](app_program/subsystem5/task_obs_scan.c) — Priority 6 (`60 ms` grid): `90°` watch-ahead -> stop & standstill confirm -> smooth slew & stationary `5°` width sweep (`50°–130°`, 4 pings / `12 ms` timeout / `400 mm` cap -> 3-echo median, oblique dropout bridging, `OBS_BEAM_DEG = 40°`), `tel_set_range_cm()`, `tel_post_obstacle()`.
- [`app_program/subsystem5/task_obs_avoid.h`](app_program/subsystem5/task_obs_avoid.h), [`task_obs_avoid.c`](app_program/subsystem5/task_obs_avoid.c) — Priority 7: clearance decision (`CAR_WIDTH_MM = 130U` mm + `50 mm` margin), encoder-closed 4-turn `45°` bypass (`OBS_OBSTACLE_DEPTH_MM = 100U` mm), directional line-rejoin search + immediate `LINE_FOUND` handoff; currently uses `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5 cm/s`, `60 s` budget; set `0` + `25 cm/s` for `NFR14`).
- [`app_program/subsystem5/subsystem5_test.c`](app_program/subsystem5/subsystem5_test.c), [`testing/subsystem5/TEST_LOG.md`](testing/subsystem5/TEST_LOG.md) — `S5-U01..U33` & `INT-S5-01..04`.

---

### Tier 4 — Integration, Lab Course Testing & Video Logs (Read During Team Integration)

| File Path | Purpose |
|---|---|
| [`testing/README.md`](testing/README.md) | Rules for defining tests, running host/dashboard tests, capturing USB CDC logs (`tools/serial_capture.ps1`), and recording results. |
| [`testing/integration/TEST_LOG.md`](testing/integration/TEST_LOG.md) | Pairwise subsystem integration test cases (`INT-01`..`INT-05`) and results table. |
| [`testing/system/TEST_LOG.md`](testing/system/TEST_LOG.md) | Full-product & lab-course test cases (`SYS-01`..`SYS-06`) and results table. |
| [`videos/README.md`](videos/README.md) | Index of unlisted YouTube video links for hardware tests and lab course demos. |
| [`docs/software_block_diagram.svg`](docs/software_block_diagram.svg) | Latest RTOS task, IPC, driver, and external-interface software block diagram (also [`docs/software_block_diagram.png`](docs/software_block_diagram.png)). |
