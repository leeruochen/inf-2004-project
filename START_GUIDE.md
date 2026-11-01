# New Device & AI Agent Start Guide (`START_GUIDE.md`)

> **Purpose:** This file is the **single entry point** when switching to a new computer, starting a fresh AI agent session, or onboarding a teammate for integration.  
> Instead of hunting across folders, tell your AI agent on the new device:
>
> ```text
> Read START_GUIDE.md first, then read the Tier 1 and Tier 2 files listed in it.
> I am Buddy <N> (<Name>) working on <task>.
> ```

---

## 1. Why Files Stay at Their Current Paths (To Prevent Merge Conflicts)

All top-level project guides (`START_GUIDE.md`, `HANDOVER.md`, `INTEGRATION_PHASES.md`, `TEAM.md`, `ARCHITECTURE.md`, `design.md`, `specs.md`) are kept together in the **repository root** for tidiness, while low-level technical specs (`DATA_REPRESENTATION.md`, `DEBUG.md`) and config headers (`config/*.h`) remain at the paths referenced by source code and CMake scripts.

**Do not move or rename those existing files:**
- Over 50 source-code comments, CMake scripts (`cmake/check_rules.cmake`), and unmerged teammate branches (`buddy1`, `buddy3`, `buddy4`, `buddy5`) reference those exact file paths.
- Moving them into a different folder would cause unnecessary Git merge conflicts for your teammates when they merge into `main`.
- Instead, **this `START_GUIDE.md` acts as the Master Index** linking every critical file in one place in the exact order an agent or developer should read them.

---

## 2. First-Time Setup on a New Windows Device

When cloning or pulling this repository on a new machine:

1. **Install Pico SDK v1.5.1** (official Windows installer at default path `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\`).
2. **Clone & Initialize the Kernel Submodule (`external/mtk3_bsp`)**:
   ```powershell
   git pull origin main
   git submodule update --init
   ```
3. **Configure & Run the Sanity Build + Rule Check**:
   ```powershell
   $SDK = 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1'
   $CM  = "$SDK/cmake/bin/cmake.exe"
   $NJ  = "$SDK/ninja/ninja.exe"
   $TC  = "$SDK/gcc-arm-none-eabi/bin"
   $common = @(
     '-G','Ninja',
     "-DCMAKE_MAKE_PROGRAM=$NJ",
     "-DPICO_SDK_PATH=$SDK/pico-sdk",
     "-DPICO_TOOLCHAIN_PATH=$TC",
     '-DPICO_BOARD=pico_w'
   )

   # 1. Production build (APP_TEST=OFF)
   & $CM -S . -B build/s0b @common -DDBG_BACKEND=USB
   & $CM --build build/s0b --parallel 8

   # 2. On-target unit test build (APP_TEST=ON)
   & $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
   & $CM --build build/test --parallel 8

   # 3. Static rule checker (R1..R6)
   & $CM -P cmake/check_rules.cmake
   ```
   Both builds must complete with **0 warnings** (`-Wall -Wextra -Werror`) and `-- check_rules: PASS`.

---

## 3. Master Reading List for AI Agents (Ordered by Priority)

### Tier 1 — Mandatory Core Context (Read First on Every Session)

| Order | File Path | What It Contains & Why It Is Mandatory |
|---|---|---|
| **1** | [`HANDOVER.md`](HANDOVER.md) | **Master project status & rules:** Where each buddy stands, Batch A/B platform status, binding coding rules (**R1–R6**), build instructions (Section 5), and owner decisions (Section 7). |
| **2** | [`TEAM.md`](TEAM.md) | **Hardware & RTOS ownership table:** Pin claims (`GP0`–`GP29`), hardware wiring checklist, peripheral owning tasks, task priorities (`P1`–`P11`), interfaces (`IF-01`..`IF-11`), and kernel object budget. |
| **3** | [`INTEGRATION_PHASES.md`](INTEGRATION_PHASES.md) | **Official integration & lab course guide:** How `usermain()` starts all 5 subsystems, exact C code snippets for calling into Subsystem 2 (VC + Motion), pairwise bench tests (`INT-01`..`05`), and full lab course tests (`SYS-02`..`06`). |

---

### Tier 2 — Shared Code Contracts & Architecture (Read Before Writing/Integrating Code)

| Order | File Path | What It Contains |
|---|---|---|
| **4** | [`config/pins.h`](config/pins.h)<br>[`config/thresholds.h`](config/thresholds.h)<br>[`config/timing.h`](config/timing.h)<br>[`config/priorities.h`](config/priorities.h) | **Compile-time source of truth** for all GPIO numbers, calibrated motor/encoder inversions (`MOTOR_L_DIR_INVERT=1U`, `ENC_R_DIR_INVERT=1U`), wheel geometry (`ENC_CPR=616U`, `WHEEL_CIRC_MM=207U`), speeds (`MOTION_SPEED_*`), task periods, and priorities. |
| **5** | [`src/common/ipc.h`](src/common/ipc.h)<br>[`src/common/data_repr.h`](src/common/data_repr.h)<br>[`docs/DATA_REPRESENTATION.md`](docs/DATA_REPRESENTATION.md) | **IPC handles & bit-packed message API:** Defines `MBF_NAV_IN`, `MBF_MOTION_CMD`, `MBF_OBSTACLE`, `MBF_TELEMETRY`, `EVF_SAFETY`, `EVF_NAV`, `MTX_MOTION`, `SEM_ECHO`, and all `drep_*` pack/unpack/validate inline functions. |
| **6** | [`src/rtos/port/hal.h`](src/rtos/port/hal.h)<br>[`src/common/period.h`](src/common/period.h)<br>[`src/debug/dbg.h`](src/debug/dbg.h)<br>[`docs/DEBUG.md`](docs/DEBUG.md) | **Platform HAL, periodic timing & USB CDC console:** `app_period_init()` / `app_period_wait()` (drift-free periodic tasks), `hal_gpio_irq_register()` (shared `IO_IRQ_BANK0` dispatcher), `hal_isr_sig()` / `hal_isr_setflg()`, and non-blocking `dbg_puts()` / `dbg_printf()`. |
| **7** | [`app_program/app.h`](app_program/app.h)<br>[`app_program/app_main.c`](app_program/app_main.c)<br>[`app_program/subsystem2/subsystem2.h`](app_program/subsystem2/subsystem2.h) | **Application startup & Subsystem 2 hub header:** Shows `usermain()` startup order, `app_start_task()`, the 10-state System FSM (`VC_SYS_*`), `vc_get_system_state()`, `motion_get_state()`, `motion_emergency_brake()`, and public motion functions (`moveForward`, `turnLeft`, `turnRight`, `stop`). |
| **8** | [`ARCHITECTURE.md`](ARCHITECTURE.md)<br>[`design.md`](design.md)<br>[`specs.md`](specs.md) | **System design & requirements:** Functional/non-functional requirements (`FR1`–`FR12`, `NFR1`–`NFR7`), Week 8 design deltas (`D1`–`D11`), and statechart diagrams. |

---

### Tier 3 — Subsystem-Specific Files (Read Based on Which Buddy You Are)

#### Buddy 2: Ruo Chen (`2500530`) — Motion Control & Vehicle Controller (**COMPLETE on `main`**)
- [`app_program/subsystem2/subsystem2_roadmap.md`](app_program/subsystem2/subsystem2_roadmap.md) — Step-by-step completion history (`S0`–`S7`) and design decisions (`Q1`–`Q4`).
- [`app_program/subsystem2/drv_motor.h`](app_program/subsystem2/drv_motor.h), [`drv_motor.c`](app_program/subsystem2/drv_motor.c) — 20 kHz PWM driver on slices 4 & 5 (`GP8`–`GP11`).
- [`app_program/subsystem2/drv_encoder.h`](app_program/subsystem2/drv_encoder.h), [`drv_encoder.c`](app_program/subsystem2/drv_encoder.c), [`quadrature_encoder.pio`](app_program/subsystem2/quadrature_encoder.pio) — PIO0 X4 quadrature decoder (`SM0` right `GP0`/`GP1`, `SM1` left `GP16`/`GP17`).
- [`app_program/subsystem2/task_motion.h`](app_program/subsystem2/task_motion.h), [`task_motion.c`](app_program/subsystem2/task_motion.c) — Priority 3, 20 ms PID velocity + cumulative heading synchronizer (`MOTION_EFF_TRACK_MM = 114U`).
- [`app_program/subsystem2/task_vc.h`](app_program/subsystem2/task_vc.h), [`task_vc.c`](app_program/subsystem2/task_vc.c) — Priority 2 Vehicle Controller (`MBF_NAV_IN` arbiter & 10-state System FSM).
- [`app_program/subsystem2/subsystem2_test.c`](app_program/subsystem2/subsystem2_test.c) — All 28 on-target unit, bench calibration, and floor tests (`S2-U01`..`S2-U28`).
- [`testing/subsystem2/TEST_LOG.md`](testing/subsystem2/TEST_LOG.md), [`PID_TUNING_REPORT.md`](testing/subsystem2/PID_TUNING_REPORT.md), [`MOTION_ACCURACY_EVALUATION.md`](testing/subsystem2/MOTION_ACCURACY_EVALUATION.md) — Completed hardware test logs and deliverable reports.

#### Other Buddies (1, 3, 4, 5)
- **Buddy 1 (Damian — Telemetry):** `app_program/subsystem1/`, `testing/subsystem1/TEST_LOG.md`, `docs/DATA_REPRESENTATION.md` (Section 8 MQTT binary frames).
- **Buddy 3 (Jin Yong — Line-Following & Barcode):** `app_program/subsystem3/`, `testing/subsystem3/TEST_LOG.md`.
- **Buddy 4 (Vincent — IMU & Safety):** `app_program/subsystem4/`, `testing/subsystem4/TEST_LOG.md`.
- **Buddy 5 (Chay Han — Obstacle Scanning & Avoidance):** `app_program/subsystem5/`, `testing/subsystem5/TEST_LOG.md`.

---

### Tier 4 — Integration, Lab Course Testing & Video Logs (Read During Team Integration)

| File Path | Purpose |
|---|---|
| [`testing/README.md`](testing/README.md) | Rules for defining tests, capturing USB CDC logs (`tools/serial_capture.ps1`), and recording results. |
| [`testing/integration/TEST_LOG.md`](testing/integration/TEST_LOG.md) | Pairwise subsystem integration test cases (`INT-01`..`INT-05`) and results table. |
| [`testing/system/TEST_LOG.md`](testing/system/TEST_LOG.md) | Full-product & lab-course test cases (`SYS-01`..`SYS-06`) and results table. |
| [`videos/README.md`](videos/README.md) | Index of unlisted YouTube video links for hardware tests and lab course demos. |
