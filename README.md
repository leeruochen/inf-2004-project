# INF2004 Embedded Systems — Autonomous Line-Following RoboCar (Group IS16)

Uses the TRON Forum uT-Kernel 3.0 BSP (micro T-Kernel 3.0) under the T-License 2.2; see [NOTICE.md](NOTICE.md) and the [BSP T-License 2.2 PDF](external/mtk3_bsp/docs/TEF000-219-200401.pdf).

Autonomous line-following robot car built on a **Raspberry Pi Pico W (RP2040)** mounted on a **Cytron Robo Pico**, running **micro T-Kernel 3.0** (`external/mtk3_bsp`, core 0 only) with the **Pico C SDK 1.5.1** and a Python 3 live MQTT / web telemetry dashboard (`tools/dashboard/`).

---

## Examiner / Assessment Guide (For Professor & Reviewers)

- **GitHub Repository:** <https://github.com/leeruochen/inf-2004-project>
- **Team & Subsystems:** Group `IS16` — Subsystem 1 (`2501299` Damian), Subsystem 2 (`2500530` Ruo Chen), Subsystem 3 (`2503319` Jin Yong), Subsystem 4 (`2500709` Vincent), Subsystem 5 (`2500596` Chay Han). See [TEAM.md](TEAM.md).
- **Full Autonomous Course Demo Video (Unlisted YouTube):** <https://www.youtube.com/watch?v=eDlNAmzyx_U> — continuous physical-track run demonstrating Code 39 barcode **`Q` → `STRAIGHT`** runtime mapping (`car/cmd/bc_map`), two-sensor IR straddle line following, `< 25 cm` obstacle stop + stationary `3°` ultrasonic sweep (`51°–129°`) + encoder-closed 4-turn `45°` bypass & line rejoin, and `15 mm` hump pitch detection (`>= 3°`) with gradual climb assist. Full video index & supplementary `10/10` dashboard rubric screenshots ([`videos/all_pass1.jpeg`](videos/all_pass1.jpeg), [`videos/all_pass2.jpeg`](videos/all_pass2.jpeg)): [videos/README.md](videos/README.md).

### 1. Requirements & Architecture Traceability

| Document | What it covers |
|---|---|
| **[docs/trib2/requirements.md](docs/trib2/requirements.md)** | **Consolidated final requirements (`FR1`–`FR14`, `NFR1`–`NFR17`)**, verification status, and subsystem owners (`B1`–`B5`). Start here to map each requirement to code and tests. |
| **[docs/REQ_CHANGE_LIFT.md](docs/REQ_CHANGE_LIFT.md)** | Requirement change note (`11 Nov 2026`): IMU lift/overturn end-of-run braking (`FR12`) replacing raw IMU collision detection. |
| **[docs/trib2/software_block_diagram.svg](docs/trib2/software_block_diagram.svg)** | Full system software block diagram (9 application tasks, IPC objects `IF-01`–`IF-12`, peripherals, and priorities). |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Source layout, kernel/SDK isolation boundaries, enforced build rules (`R1`–`R6`), design deltas (`D1`–`D17`), and template deviations (`T1`–`T8`). |
| **[TEAM.md](TEAM.md)** | Pin map (`config/pins.h`), peripheral ownership, task priorities (`config/priorities.h`), IPC interface contracts, and kernel object budget. |
| **[design.md](design.md)** | Original Week 6 baseline architecture and design rationale. |

### 2. Working Code Map (Requirements → Source Folders)

| Subsystem / Layer | Requirements (`docs/trib2/requirements.md`) | Source Location | Key Modules |
|---|---|---|---|
| **S1 — Telemetry, Wi-Fi & MQTT** | `FR1`–`FR3`, `NFR1` | [app_program/subsystem1/](app_program/subsystem1/) & [tools/dashboard/](tools/dashboard/) | `task_tel.c`, `drv_wifi.c`, `tel_wire.c`, `tel_link.c`, `tel_outbox.c`; dashboard `tel_web.py`, `tel_dashboard.py`, `tel_codec.py` |
| **S2 — Motion Control & Vehicle Controller** | `FR4`–`FR7`, `NFR2`–`NFR6` | [app_program/subsystem2/](app_program/subsystem2/) | `task_vc.c` (arbitration & system FSM), `task_motion.c` (50 Hz PID + heading sync + hump assist), `drv_motor.c`, `drv_encoder.c` (`quadrature_encoder.pio`) |
| **S3 — Line Following & Code 39 Barcode** | `FR8`–`FR10`, `NFR7`–`NFR9` | [app_program/subsystem3/](app_program/subsystem3/) | `task_lf.c`, `lf_straddle.c` / `lf_core.c`, `lf_link.c`, `lf_sanity.c`, `drv_ir.c`, `task_bc.c`, `bc_decode.c`, `bc_frame.c`, `bc_edge.c`, `drv_bc.c` |
| **S4 — IMU Processing & Safety** | `FR11`–`FR12`, `NFR10`–`NFR13` | [app_program/subsystem4/](app_program/subsystem4/) | `task_safety.c` (P1 watchdog/lift emergency brake), `task_imu.c`, `imu_hump.c`, `imu_lift.c`, `drv_imu.c` |
| **S5 — Obstacle Scanning & Avoidance** | `FR13`–`FR14`, `NFR14`–`NFR15` | [app_program/subsystem5/](app_program/subsystem5/) | `task_obs_scan.c` (watch-ahead + stationary `3°` sweep), `task_obs_avoid.c` (encoder-closed 4-turn bypass), `drv_ultrasonic.c`, `drv_servo.c` |
| **Platform, RTOS Port & Shared Config** | `NFR16`–`NFR17` | [app_program/](app_program/), [src/](src/), [config/](config/) | `app_main.c`, `src/app/main.c`, `src/rtos/port/`, `src/common/data_repr.h`, `src/common/ipc.c`, `src/debug/dbg_*.c`, `config/*.h` |

### 3. Test Cases & Verification Evidence

Tests are organized at three levels (see [testing/README.md](testing/README.md) for conventions):

1. **Automated Host C Unit Tests (`tests/host/`, 14 CTest suites — runs on PC, no hardware needed):**
   Covers pure C modules compiled with `-Wall -Wextra -Werror`:
   - *Platform & Telemetry (`B1` / Platform):* `dbg_ring`, `crc32`, `period`, `data_repr` (273 bit-packing/unpacking assertions), `tel` (link FSM, outbox, command checks, shared golden MQTT frames `1..11`).
   - *Barcode & Line Following (`B3`):* `bc_decode` (`S3-U01`, Code 39 `A–Z` bidirectional decode), `bc_frame` (`S3-U02`), `bc_edge` (`S3-U07`, 2 ms glitch filter), `lf_sanity` (`S3-U08`), `lf_core` (`S3-U10`), `lf_straddle`, `lf_link` (`S3-U11`).
   - *IMU (`B4`):* `imu_lift` (`S4-U21`, lift/overturn thresholds and re-arming), `imu_hump` (`S4-U24`, hump detection and height estimation with track trace replay).
   - Run all 14 suites:
     ```powershell
     cmake -S tests -B build/host -G Ninja
     cmake --build build/host
     ctest --test-dir build/host --output-on-failure
     ```
2. **Automated Python Dashboard & Protocol Tests ([tools/dashboard/](tools/dashboard/), 7 test suites):**
   Verifies cross-language frame compatibility (`test_tel_decode.py` against `test_tel.c`), log replay across hardware logs (`test_tel_record.py`, `test_tel_views.py`), IR calibration (`test_tel_calib.py`), command safety (`test_tel_control.py`), live Demo Rubric state tracking (`test_tel_demo.py`), and HTTP API (`test_tel_web.py`):
   ```powershell
   python -m unittest discover -s tools/dashboard -p "test_*.py" -v
   ```
3. **On-Target Hardware Self-Tests & Subsystem/System Logs:**
   Building with `-DAPP_TEST=ON` compiles `app_program/subsystemN/subsystemN_test.c` and runs automated on-board checks (`[TEST] <ID> PASS|FAIL`) over USB CDC.

| Test Record / Evidence | Contents |
|---|---|
| [testing/README.md](testing/README.md) | Test hierarchy (`S*-U*`, `INT-*`, `SYS-*`), run instructions, and logging rules |
| [testing/subsystem1/TEST_LOG.md](testing/subsystem1/TEST_LOG.md) | S1 unit, host (`S1-H01..H08`), and MQTT/Wi-Fi test cases + saved logs (`testing/subsystem1/logs/`) |
| [testing/subsystem2/TEST_LOG.md](testing/subsystem2/TEST_LOG.md) | S2 unit, motor/encoder calibration, PID, and VC test cases (`S2-U01..U28`) + saved logs (`testing/subsystem2/logs/`) |
| [testing/subsystem3/TEST_LOG.md](testing/subsystem3/TEST_LOG.md) | S3 barcode, ADC, and line-following test cases (`S3-U01..U21`) + saved logs (`testing/subsystem3/logs/`) |
| [testing/subsystem4/TEST_LOG.md](testing/subsystem4/TEST_LOG.md) | S4 IMU driver, hump height, watchdog, and lift stop test cases (`S4-U01..U26`) + saved logs (`testing/subsystem4/logs/`) |
| [testing/subsystem5/TEST_LOG.md](testing/subsystem5/TEST_LOG.md) | S5 ultrasonic, servo sweep, width estimation, and bypass test cases (`S5-U01..U33`, `INT-S5-01..04`) + saved logs (`testing/subsystem5/logs/`) |
| [testing/integration/TEST_LOG.md](testing/integration/TEST_LOG.md) | Cross-subsystem interface integration definitions & results (`INT-01`–`INT-06`) |
| [testing/system/TEST_LOG.md](testing/system/TEST_LOG.md) | Whole-system acceptance definitions & results (`SYS-01`–`SYS-07`), demo video link, and [videos/README.md](videos/README.md) |

### 4. Detailed Engineering Reports & Evidence

- **MQTT Protocol & Telemetry Bandwidth:** [app_program/subsystem1/MQTT.md](app_program/subsystem1/MQTT.md)
- **PID Tuning & Motion Accuracy Evaluation (`S2` deliverables):** [testing/subsystem2/PID_TUNING_REPORT.md](testing/subsystem2/PID_TUNING_REPORT.md) and [testing/subsystem2/MOTION_ACCURACY_EVALUATION.md](testing/subsystem2/MOTION_ACCURACY_EVALUATION.md)
- **Minimum-Bit IPC Packing & Measurements:** [docs/DATA_REPRESENTATION.md](docs/DATA_REPRESENTATION.md), [docs/measurements/week6-baseline.txt](docs/measurements/week6-baseline.txt), [docs/measurements/week8-packed.txt](docs/measurements/week8-packed.txt)
- **Embedded Optimizations (ARMv6-M / SWAR / 32-bit HW Divider):** [docs/EMBEDDED_OPTIMIZATIONS.md](docs/EMBEDDED_OPTIMIZATIONS.md)

---

## 1. Set Up (Windows, Pico SDK 1.5.1)

1. Install the **Raspberry Pi Pico SDK 1.5.1 Windows installer** at its default path, `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1`. It installs the SDK, CMake, Ninja, Arm GCC 10.3.1, Python, git, and OpenOCD.
2. Clone the repository together with the pinned `external/mtk3_bsp` kernel submodule (`15ed232`, see [docs/SUBMODULE.md](docs/SUBMODULE.md)):
   ```powershell
   git clone --recurse-submodules https://github.com/leeruochen/inf-2004-project.git
   ```
   In an existing clone, run `git submodule update --init --recursive`.
3. In every new PowerShell window at the repository root, run this toolchain preamble:
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

## 2. Build

Every firmware build automatically runs `check_rules` ([ARCHITECTURE.md](ARCHITECTURE.md) rules `R1`–`R6`) and compiles all project sources with `-Wall -Wextra -Werror`. Outputs are written as `robocar.elf` and `robocar.uf2` inside the chosen build directory.

- **Active car build (Wi-Fi/MQTT + straddle line following + differential steer targets):**
  First copy `config/secrets.h.example` to `config/secrets.h` and fill in your 2.4 GHz Wi-Fi SSID/password and MQTT broker IP (never commit `config/secrets.h`):
  ```powershell
  $env:CFLAGS = "-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1"
  & $CM -S . -B build/wifi @common -DDBG_BACKEND=USB -DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON
  Remove-Item Env:CFLAGS -ErrorAction SilentlyContinue
  & $CM --build build/wifi
  ```
- **USB-console build (no Wi-Fi/radio needed):**
  ```powershell
  & $CM -S . -B build/usb @common -DDBG_BACKEND=USB
  & $CM --build build/usb
  ```
- **Release build (debug ring and drain task compiled out):**
  ```powershell
  & $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release -DDBG_BACKEND=USB
  & $CM --build build/release
  ```
- **On-target test build (`APP_TEST`):**
  ```powershell
  & $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
  & $CM --build build/test
  ```
  (Add `-DAPP_TEST_ONLY=<1..5>` to run only one subsystem's self-tests.)

## 3. Flash & Console

- **UF2 without a Debug Probe:** hold `BOOTSEL`, plug the Pico W into USB, release, and copy `robocar.uf2` (e.g. `build/wifi/robocar.uf2`) to the `RPI-RP2` drive. The Pico W reboots automatically.
- **SWD with the Raspberry Pi Debug Probe** (probe "D" port to Pico W SWD header: SWCLK, SWDIO, GND):
  ```powershell
  & "$R\openocd\openocd.exe" -s "$R\openocd\scripts" `
    -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" `
    -c "program build/usb/robocar.elf verify reset exit"
  ```
- **Console:** USB CDC on the Pico W's own USB port (`115200` baud in PuTTY or `python -m serial.tools.miniterm COMn 115200`). Tasks log through the non-blocking `dbg_` ring; RAM fault records persist across warm resets and print on the next boot. Full details in [docs/DEBUG.md](docs/DEBUG.md).

## 4. Dashboard & Host Tests

- **Launch web telemetry dashboard** (requires `pip install paho-mqtt` for live MQTT):
  ```powershell
  python tools/dashboard/tel_web.py
  ```
  Opens `http://127.0.0.1:8765` with Live Overview (10-item Demo Rubric), Drive controls, Barcode `A–Z` runtime mapping editor, Recording/CSV export, Data Gaps, Line Calibration, and Subsystem Checks. See [tools/dashboard/README.md](tools/dashboard/README.md).
- **Run host C tests (`14` suites) & Python dashboard tests (`7` files):**
  ```powershell
  & $CM -S tests -B build/host -G Ninja
  & $CM --build build/host
  ctest --test-dir build/host --output-on-failure
  python -m unittest discover -s tools/dashboard -p "test_*.py" -v
  ```

## 5. Build Options

Options are CMake cache values seeded on first configure; use a fresh build directory or pass the flag explicitly.

| Option | Values (Default) | What it does |
|---|---|---|
| `DBG_BACKEND` | `USB` (`USB`) | Debug console backend (USB CDC only; `UART0` conflicts with `GP0`/`GP1` right encoder) |
| `DBG_ENABLE` | `ON`, `OFF` (`OFF` for Release/MinSizeRel, `ON` otherwise) | Enables `dbg_` ring and debug-only drain task |
| `TEL_BACKEND_WIFI` | `ON`, (`OFF`) | Enables CYW43 + poll-mode lwIP MQTT client (requires git-ignored `config/secrets.h`) |
| `TEL_STATUS_PERIOD_MS` | `""` (`60000`) or integer `>= 200` | Periodic telemetry status line interval (set `1000` for dashboard stationary tests) |
| `S3_LF_STRADDLE` | `ON`, (`OFF`) | Uses straddle line-following core (`lf_straddle.c`) instead of edge-following core (`lf_core.c`) |
| `APP_TEST` | `ON`, (`OFF`) | Runs `subsystemN_test()` in priority-11 test task over `dbg_` (requires `DBG_ENABLE=ON`) |
| `APP_TEST_ONLY` | `0` (`0`), `1..5` | Limits `APP_TEST` execution to subsystem `1..5` (`0` = all) |
| `S4_MANUAL_TESTS` / `S4_HUMP_ONLY` | `ON`, (`OFF`) | Enables operator-guided IMU lift/hump tests in `APP_TEST` builds |
| `PLATFORM_SELFTEST` | `ON`, (`OFF`) | Debug-only forced-HardFault fault-record verification build |
| `PLATFORM_BRINGUP` | `ON`, (`OFF`) | 9 heartbeat stand-in tasks (`SYS-01` USB CDC bring-up verification) |

*(Additional Subsystem 3 diagnostic options in [app_program/subsystem3/sources.cmake](app_program/subsystem3/sources.cmake): `S3_STEP_DIAG`, `S3_TURN_DIAG`, `S3_CURVE_TEST`, `S3_BC_TEST`, `S3_BC_QUICK`, `S3_DRIVE_MM`, `S3_TURN`, `S3_QUICK_CAL`.)*

## 6. Architecture Rules Summary

- **Strict SDK / kernel separation (`R1`):** `drv_*.c` include Pico SDK headers only; `task_*.c`, `subsystemN.c`, and `subsystemN_test.c` include kernel headers only. A `drv_*.c` never includes `subsystemN.h`.
- **No blocking SDK calls in tasks (`R2`):** tasks use `tk_dly_tsk` or drift-free `app_period_wait()` (`src/common/period.h`).
- **Source registration:** every `.c` file must be explicitly listed in its folder's `sources.cmake`.
- **Pin safety:** pins are defined in `config/pins.h` and documented in [TEAM.md](TEAM.md). Never use `GP23`, `GP24`, `GP25`, or `GP29` (Pico W CYW43 radio).

## 7. Licence

Licences and third-party attributions are listed in [NOTICE.md](NOTICE.md). Adapted from the INF2004 course project template provided by the module instructor.
