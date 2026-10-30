# HANDOVER: where the project stands and where each member continues

Read this first. Section 3 is for every team member (where to continue on your
own subsystem). Section 0 is for whoever continues on another device or with an
AI agent. The detailed evidence is in `.agents/tasks/` (index in section 9).

Last updated: 31 Oct 2026, Batch B stage B5 (bring-up test build), by Buddy 3 (Jin Yong) as
the Batch B helper. Previous update: 29 Oct 2026, Batch B stage B0, by Buddy 3.

---

## 0. Quick start for the next device or agent

1. **Repo root** = the folder that holds this file. Older reports in `.agents/tasks/` use the
   path `C:/Users/Admin/Downloads/Projects/inf-2004-project/...`; substitute your root.
2. **Set up** as in section 5 (SDK 1.5.1 at the default path, submodule at `15ed232`). If
   `git submodule status` starts with `-`, run `git submodule update --init`.
3. **Check the state:**
   - `git log --oneline -n 3`. The template merge + Week 8 work is committed on `main`
     (`559f479`, `8d083ca`; section 1, "Commit status"). If `app_program/` is missing, pull first.
   - Sanity build (section 5, step 4) must pass: 0 warnings, `check_rules: PASS`.
4. **What is next for the platform** is Batch B (section 4). It is owned by Buddy 2. Buddy 2
   has approved Buddy 3 (Jin Yong) as the Batch B helper (decision C1, section 7). Batch B
   follows `.agents/tasks/batchB-plan.md`. The helper writes each stage and runs the official
   compile gates on the same SDK 1.5.1 installer (section 5); Buddy 2 reviews and merges, and
   runs every hardware stage (flash, USB tests, measurements). **A stage passes only with
   official-toolchain evidence, and a hardware stage only with Buddy 2's run** (C7, section 7).
   SDK 1.5.1 + GCC 10.3.1 is the only toolchain (C8). Nobody else starts Batch B work.
5. **Do not write another member's subsystem code.** Each member writes and tests their own
   folder (section 3). An agent working for one member only edits that member's folder, plus
   shared files when the team has agreed.
6. **Rules in section 6 are binding.** Owner decisions in section 7 are binding.
7. When you finish a piece of work: run the checks in section 5, update section 1 and your
   own row in section 3 of this file, and record test results in `testing/`.

---

## 1. Where things stand

| Item | Status |
|---|---|
| Kernel integration analysis | Done (`.agents/tasks/mtk3-rp2040-integration-report.md`) |
| Batch A: build system, config headers, rule checks | Done, committed (`56826c7`) |
| Course template | Committed as `ProjectTemplate/` (`79bc417`). Reference only, never edited |
| Template merge (Option C): `app_program/`, `TEAM.md`, `testing/`, `videos/` | Done, verified (`merge-verify.md`) |
| Week 8 change request: pin map v2, USB-only console, docs | Done, verified (`week8-verify.md`) |
| Final verification from a clean `build/` | Done (`merge-week8-summary.md`) |
| **Batch B: platform (boot, kernel port, debug console, IPC objects)** | **Milestone M1 reached (31 Oct 2026): the platform passed the Phase 1 USB CDC bring-up test (SYS-01); members can run their own tasks** (section 3). B0-B5 merged on `main` (PRs #1-#5); every hardware stage passed on Buddy 2's board. Next: B6 (baseline measurement), then B7 (packed messages, milestone M2). Owner: Buddy 2; helper: Buddy 3 (C1). Plan: `.agents/tasks/batchB-plan.md`; section 4 |
| Subsystem code (all five buddies) | Not started. Every subsystem is a compile-checked stub |
| Hardware runs | B4a (boot, 150 s, replug) and B4b (self-test: forced fault, fault record, power cycle) passed, 30-31 Oct 2026; B5 bring-up test (SYS-01, 13 min with stress, replug) passed, 31 Oct 2026. Buddy 2's Pico W; logs on PRs #3-#5 and in `testing/system/logs/` |

**Commit status.** The merge + Week 8 work is committed and pushed on `main` (`559f479`,
`8d083ca` = `origin/main`), so every member can see `app_program/`, `TEAM.md` and the new pin
map. Batch B work goes on one branch and one pull request per stage (`batchB/bN-<name>`).

**Rough completion estimate** (judgement, not a measurement):

| Scope | Done | Basis |
|---|---|---|
| Phase 1 platform skeleton | about 60 % | Build, config, rules, layout, docs done. Batch B (boot, port, debug ring, IPC) is the remaining and riskiest part: the kernel has never booted on the board |
| Whole project (a car that meets design.md) | about 10 % | No subsystem logic, no driver, no hardware test yet; integration and tuning still ahead |

**Is more merging needed?** No. The template merge is finished. What remains shared is
Batch B (platform code in `src/`), which everyone's on-target tests depend on.

**Can everyone start on their own subsystem now?** Yes, with one limit:

- Now: write your drivers (`drv_*.c/.h`), task skeletons and test cases. Everything you
  register in your `sources.cmake` is compiled with `-Werror` and rule-checked on every build.
- After Batch B milestone M1 (stage B5, plan section 3): flash the firmware, run your tasks,
  see `[TEST]` lines on USB CDC. Until then the firmware cannot link (no `main()`, no kernel
  port), so nothing runs on the car from this repo. The message types stay provisional until
  milestone M2 (stage B7), so code that reads or writes message fields may need edits between
  M1 and M2.

---

## 2. Project in one paragraph

Autonomous line-following car on a Raspberry Pi **Pico W (RP2040)** on a Cytron Robo Pico
board, in **C** with **Pico SDK 1.5.1** and **micro T-Kernel 3.0** (official single-core
`tron-forum/mtk3_bsp`, branch `pico_rp2040`, submodule `external/mtk3_bsp` pinned at
`15ed232c08f2d89e54e514db79de150e224307e3`). CMake + Ninja on Windows, Barr C style. The SDK
owns startup, linker script and vector table; the kernel is a static library (`mtk3`) behind a
port layer. Team code is in `app_program/subsystem1..5/`; platform code goes in `src/`.
Sources of truth: `design.md` (team design), `specs.md` (brief), `INF 2004 Embedded Systems.md`
(Week 8 submission, read only), plus the deltas in `ARCHITECTURE.md`.

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
   console and debugging are in `docs/DEBUG.md`. The message structs in
   `src/common/ipc_types.h` are PROVISIONAL until milestone M2 (stage B7).

Every subsystem today has `subsystemN.c` (init/start stubs), `subsystemN.h` (its interfaces as
comments), `subsystemN_test.c` (empty test) and `sources.cmake`. Interfaces between
subsystems are IF-01..IF-11 in `TEAM.md` section 5. Data formats are in
`docs/DATA_REPRESENTATION.md`.

### Buddy 1: Damian (2501299, Damian-oliveiro), telemetry, `app_program/subsystem1/`

| | |
|---|---|
| Tasks | Telemetry, priority 9 (`TASK_PRI_TEL`, `TASK_STACK_TEL_BYTES`) |
| Pins / peripherals | none in Phase 1. Phase 2: CYW43 (GP23-25, GP29), PIO1, 2 DMA channels (reserved in TEAM.md) |
| Interfaces | owns IF-04 MBF_TELEMETRY (receives `tel_evt_t`); sends remote commands into IF-01 MBF_NAV_IN; reads IF-08 `motion_state_t` under MTX_MOTION |
| Start with | MQTT binary frame format and reject rules (`docs/DATA_REPRESENTATION.md` section 8, ARCHITECTURE.md D10); WiFi/MQTT ownership design (TODO(phase 2, buddy 1)); the template's WiFi code is reference only (`docs/TEMPLATE_NOTES.md`) |
| Decisions open for you | D2: 20 ms telemetry wake is NOT approved (`config/timing.h`); `DBG_DRAIN_PERIOD_MS`; whether Barr C allows bit-fields for private state |
| Status | not started |

### Buddy 2: Ruo Chen (2500530, leeruochen), motion + Vehicle Controller, `app_program/subsystem2/`

| | |
|---|---|
| Tasks | Vehicle Controller, priority 2; Motion Control, priority 3 (20 ms PID, `MOTION_PERIOD_MS`) |
| Pins | motors GP8/GP9 (left, PWM slice 4), GP10/GP11 (right, slice 5), 20 kHz; encoders GP0/GP1 (right, PIO0 SM0), GP16/GP17 (left, PIO0 SM1) |
| Interfaces | owns IF-01 MBF_NAV_IN, IF-02 MBF_MOTION_CMD, IF-08 MTX_MOTION + `motion_state_t`, IF-10 encoder counts, IF-11 `motion_emergency_brake()`; VC clears FAULT_LATCHED |
| Start with (motors) | 1. `drv_motor.c/.h`: PWM slices 4 and 5 at `MOTOR_PWM_FREQ_HZ`, duty capped at `MOTOR_DUTY_MAX_PCT`, `MOTOR_x_DIR_INVERT`, brake = both inputs LOW, coast = both HIGH. 2. `motion_emergency_brake()` (declared in `subsystem2.h`): all four channels LOW inside a critical section. 3. `drv_encoder.c`: copy pico-examples `quadrature_encoder.pio` (tag `sdk-1.5.1`, keep its header; notice already in `NOTICE.md`), load at offset 0 on PIO0, SM0 right / SM1 left, non-blocking `enc_get_count()` (prototype in `drv_encoder.h`), add `pico_generate_pio_header` in `CMakeLists.txt` (shared file: tell the team). 4. Motion task: 20 ms PID per wheel, `MOTION_T_HOLD_MS`. 5. VC task |
| Bring-up order | motor direction first (`MOTOR_x_DIR_INVERT`), then encoder sign (`ENC_x_DIR_INVERT`) by hand-spin and forward-drive tests |
| Values to measure | `ENC_CPR`, `WHEEL_CIRC_MM` (0 = not measured; code using them must `#error` while 0), `MOTOR_DUTY_MAX_PCT`, `POWER_GROVE_3V3_BUDGET_MA` |
| Also yours | Batch B platform work (section 4), labelled `TODO(phase 1 batch B, buddy 2)` across the repo; it unblocks everyone's hardware tests. Buddy 3 writes it as the helper (C1). You own the decisions, review and merge each stage PR, and run every hardware stage: from B4, build the `.uf2` from the PR commit, flash the Pico W and run the hardware/USB tests (C7, plan section 2). NFR2 wording test: drive a measured distance at 50 cm/s and compare counts, both wheels (ARCHITECTURE.md D7) |
| Status | not started |

### Buddy 3: Jin Yong (2503319, jyong1101), line following + barcode, `app_program/subsystem3/`

| | |
|---|---|
| Tasks | Line-Following, priority 4 (5 ms sampling); Barcode Decode, priority 5 (2 ms poll while capturing, D3) |
| Pins / peripherals | IR_R GP26 = ADC0 (car's RIGHT, Grove 5), IR_C GP28 = ADC2 (CENTRE, Grove 7, VBAT jumper must stay OPEN); barcode IR_BC GP27 digital edge IRQ (never `adc_gpio_init()`); owns the ADC |
| Interfaces | sends steering (TMO_POL) and barcode commands (5 ms) into IF-01; sets LINE_FOUND in IF-07 EVF_NAV; reads IF-08; owns IF-09 edge ring (20-bit gaps, D8) |
| Already there | `drv_ir.h/.c` with the `hal_adc_sanity_check()` stub, limits `ADC_*` in `config/thresholds.h` |
| Decisions open for you | steering sign for CENTRE + RIGHT sensors (proposal `steer_err > 0` = line to the right, D11); `BC_CAPTURE_TIMEOUT_MS`; ADC sanity limits; tape the Grove 6 GP26 lead (TEAM.md hardware checklist) |
| Also | Batch B helper for Buddy 2 (C1, section 4): B0 reviewed, fixes applied; next B1-B3. Needs the SDK 1.5.1 installer (section 5) to run the official compile gates (C7, plan O11) |
| Status | subsystem not started |

### Buddy 4: Vincent (2500709, Vincent6430), IMU + safety, `app_program/subsystem4/`

| | |
|---|---|
| Tasks | Safety, priority 1 (`task_safety.c`, in this folder); IMU Processing, priority 8 (10 ms) |
| Pins / peripherals | IMU on I2C1, SDA GP2, SCL GP3 (Grove 2 stays unused); owns I2C1 |
| Interfaces | owns IF-06 EVF_SAFETY; Safety waits on `COLLISION|IMU_ALIVE` with `TWF_ORW|TWF_BITCLR` (never `TWF_CLR`), watchdog `SAFETY_IMU_WDOG_MS`; brakes only through `motion_emergency_brake()` in `../subsystem2/subsystem2.h`; never takes MTX_MOTION |
| Start with | I2C bus scan to confirm the addresses (0x19 accel, 0x1E mag); `drv_imu.c`; pitch from accelerometer only (no gyro, D4) |
| Decisions open for you | IMU addresses; gyro or not (`IMU_HAS_GYRO`); proposed EVF_SAFETY bit table (`docs/DATA_REPRESENTATION.md` section 3) |
| Status | not started |

### Buddy 5: Chay Han (2500596, longchayhan03-eng, to confirm), obstacle scan + avoid, `app_program/subsystem5/`

| | |
|---|---|
| Tasks | Obstacle Scanning, priority 6 (60 ms servo step); Obstacle Avoidance, priority 7 |
| Pins / peripherals | ultrasonic TRIG GP4, ECHO GP5 (Grove 3, echo must be 3.3 V-safe); servo GP15 = PWM slice 7 B (never GP12); owns PWM slice 7 |
| Interfaces | owns IF-03 MBF_OBSTACLE, IF-05 SEM_ECHO, IF-07 EVF_NAV (AVOID_ACTIVE, LINE_REACQUIRED); sends bypass commands into IF-01 |
| Start with | `drv_servo.c`, `drv_ultrasonic.c`; newest-wins needs drain-then-send on MBF_OBSTACLE (`docs/DATA_REPRESENTATION.md` 2.3) |
| Decisions open for you | keep the 2-bit sequence number in `obstacle_profile_t`; the 3-bit bypass command in `nav_input_t`; `CAR_WIDTH_MM`; servo pulse range; settle time vs 60 ms step |
| Status | not started |

---

## 4. Batch B: platform work still to do (Buddy 2, helper Buddy 3)

**The plan is `.agents/tasks/batchB-plan.md`** (stages, decisions, open questions, gate
commands, stop conditions). It wins over this summary where they differ. Stage status:

| Stage | What | Status |
|---|---|---|
| B0 | records and prerequisites (docs only) | done, merged (PR #1) |
| B1 | `dbg_` ring API; `APP_TEST=ON` builds | done, merged (PR #2, with B2 and B3) |
| B2 | IPC objects, task start, stack painting (`src/common`) | done, merged (PR #2) |
| B3 | kernel port layer (`src/rtos/port`), fault record, system-message ring | done, merged (PR #2) |
| B4 | SDK `main()` + debug drain: first firmware, first flash: **milestone M1-lite** (members may add tasks) | done: B4a and B4b passed on the board, merged (PRs #3, #4). **M1-lite reached, 31 Oct 2026.** B4b found and fixed a task-exit fault (rule R6) |
| B5 | heartbeats + USB CDC bring-up test: **milestone M1** (Phase 1 acceptance), in parallel with subsystem work | done: passed on the board 31 Oct 2026 (SYS-01, `testing/system/TEST_LOG.md`; worst jitter 566 µs of the 1000 µs limit), merged (PR #5). **M1 reached** |
| B6 | baseline measurement, tag `baseline/week6-unpacked` | not started |
| B7 | B-packed types + host tests: **milestone M2** (messages final) | not started |

Since B4 (M1-lite) members can run their own tasks on the car. Planned as two
parts (owner decision C4): **B-baseline** (unpacked Week 6 types, measured, tagged
`baseline/week6-unpacked`) then **B-packed** (shared header + host tests). The toolchain does
not change: section 5 (SDK 1.5.1 + GCC 10.3.1) is the official environment for every gate.

**Workflow (C7).** For each stage the helper pushes a branch and posts the official compile
gates (SDK 1.5.1, fresh build directories) on the PR. Buddy 2 reviews and merges (since 31 Oct
2026 the helper may merge a stage PR after Buddy 2's test PASS is on it, with his OK). For hardware
stages (B4 on) he builds the `.uf2` from that commit, flashes it, runs the checks and posts the
logs. The stage is marked done here only after that. Critical path to members adding tasks:
B0 merge → B1-B3 PR → B4 flash (M1-lite). Details: plan section 2.

B-baseline contents, with file names already fixed by the `check_rules` allow-lists (full
file list per stage in the plan):

| File | What it does |
|---|---|
| `src/app/main.c` | SDK `main()`: SDK init, wait up to `DBG_USB_CONNECT_WAIT_MS` for USB CDC (line marked `RULE-ALLOW: pre-kernel`), then `hal_kernel_start()` |
| `src/rtos/port/` (`port_kernel_internal.c`, `port_boot.c`, `port_fault.c`, `port_irq.c`) | kernel pool, SHPR priorities, PendSV/SysTick vectors, `knl_lowmem_top/limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init/tm_snd_dat/tm_rcv_dat` (into the system-message ring, never stdio, C3), the RAM fault record and HardFault handler, `hal_irq_register()` / `hal_gpio_irq_register()` with the `knl_taskindp` wrapper (integration report section 16) |
| `src/debug/dbg_drain.c` (+ ring) | `dbg_` ring and the drain task (priority 10), the only stdio caller; empties the system-message ring every cycle; prints and clears the RAM fault record only if magic and CRC-32 match |
| `src/common/` | `ipc_init()`: every IPC object at its design.md 3.3 depth; message-buffer size macro; kernel version self-test; stack painting |
| `app_program/app_main.c` | replace the `ipc_init()` TODO and the `app_start_task()` stub (shared file) |
| `app_program/test_report.h` | replace the deliberate `#error` with `dbg_` output, so `APP_TEST=ON` builds |

Acceptance: boots, creates every object, nine heartbeat lines on USB CDC, the mandatory USB
CDC bring-up test in `docs/DEBUG.md` (10 minutes, no gaps, jitter within 1 ms).

Details: `.agents/tasks/batchB-plan.md` first (its section 5 lists the stale parts of the
reports below); `.agents/tasks/mtk3-rp2040-integration-report.md` sections 10, 14, 16;
`.agents/tasks/week8-impact-report.md` section D (baseline plan);
`docs/DATA_REPRESENTATION.md` sections 5-7.

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

4. Build and checks (expected today: exit 0, 0 warnings, `check_rules: PASS`,
   `firmware target skipped, app_program compile-checked`):

```powershell
& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb
& $CM -P cmake/check_rules_selftest.cmake      # expect PASS
```

`DBG_BACKEND=UART0` is removed and fails at configure (GP0/GP1 are the right encoder). USB CDC
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

Full text: "OWNER CORRECTIONS" at the top of `.agents/tasks/week8-impact-report.md`, and the
brief `.agents/tasks/merge-week8-brief.md`.

- Template merge Option C: our build and kernel; the template's `app_program/subsystemN/`,
  `TEAM.md`, `testing/`, `videos/` layout. Deviations T1-T7 in `ARCHITECTURE.md`.
- Pin map v2 (`config/pins.h`, ARCHITECTURE.md D5). Servo on GP15. Encoders on PIO0, one state
  machine per wheel, API `enc_get_count()` (D7).
- USB CDC is the only console (D6). Kernel text from `tm_snd_dat` (boot banner, system
  messages) goes into a system-message ring that the debug drain empties every cycle (C3
  below). Faults go to a separate RAM fault record in `.uninitialized_data` with a magic
  number and a CRC. Both implemented in Batch B.
- NFR2: no team document is edited; the proposed wording waits in D7 for the team, after a
  top-speed zero-missed-counts test.
- Data representation (D8, `docs/DATA_REPRESENTATION.md`): pitch ±45° at 0.5° in 8 bits;
  `motion_cmd_t` 19 bits; `obstacle_profile_t` with clearances, no timestamp, no bypass state,
  optional 2-bit sequence kept; system FSM 4 bits (10 states); EVF_SAFETY fault bits (D9);
  binary MQTT (D10).
- **D2 (20 ms telemetry wake) is NOT approved.**
- `BC_CAPTURE_TIMEOUT_MS = 1000` stays provisional with a static assert. `ENC_CPR` and
  `WHEEL_CIRC_MM` are 0 until measured.

**Batch B decisions.** These are not binding until Buddy 2 confirms them on the B0 PR; the
status of each is in `.agents/tasks/batchB-plan.md` sections 0 and 1, and the reasons are in
`batchB-b0-review-report.md`:

- **C1:** Buddy 2 approves Batch B work by the helper, Buddy 3 (Jin Yong). Given verbally,
  recorded here. Buddy 2 still owns Batch B and reviews every stage.
- **C2:** each member writes the tests for their own subsystem (`subsystemN_test.c`).
- **C3:** the kernel's own text (the `tm_printf` boot banner, system messages) is kept
  **separate** from the RAM fault record. It goes into a system-message ring that the drain
  empties every cycle, so no `tm_*` output is lost after boot either. The fault record holds
  faults only.
- C4 (B-baseline then B-packed), C5 (approach (a)) and C6 (`ipc_init()` creates every object
  first, T5) restate earlier decisions.
- **C7 (Buddy 2 in chat, 30 Oct 2026; confirm on the PR):** the helper writes each stage and
  runs the official compile gates on the section 5 toolchain; Buddy 2 reviews, merges, and runs
  every hardware stage. B1-B3 ship as one PR.
- **C8 (Buddy 2 in chat, 30 Oct 2026):** the only toolchain is Pico SDK 1.5.1 + GCC 10.3.1
  (section 5). No SDK migration.
- Answers following Buddy 2's review (plan section 1):
  - O1: temporary heartbeat tasks behind `PLATFORM_BRINGUP`, default OFF; B4 self-tests behind
    `PLATFORM_SELFTEST`, default OFF.
  - O2: platform writes `src/common/ipc_types.h`; the B2 baseline uses the Week 6 field set,
    and the Week 8 fields come in B7.
  - O4: fault record magic `0x464C5452`, CRC-32 IEEE in a pure `crc32.h`.
  - O5/O6: ring sizes and policies as proposed.
  - O7: after a HardFault, record, then `watchdog_reboot`; halt only on a RAM flag or the build
    option `PORT_FAULT_HALT`.
  - O8: stacks stay kernel-allocated and are painted by a start-up trampoline, including the
    init task; `config/priorities.h` is unchanged.
  - O9: `ipc` is added to the R1 project-header check.
  - Still open: O3 (default: out of Batch B), O10 (B7), and owner questions Q3, Q7, Q8 and Q9
    (plan section 1).

---

## 8. Open questions (with suggested answers)

| # | Question | Suggested answer |
|---|---|---|
| 1 | Buddy 5's GitHub username: design.md reads "Chay Hanlongchayhan03-eng" | Confirm `longchayhan03-eng` (used in TEAM.md, marked to confirm) |
| 2 | design.md Team Role table: Buddy 1's cell has a line break after "Damian-oliveiro", which breaks the markdown table | Join it onto one line: `2501299 (Damian/Damian-oliveiro)` |
| 3 | Instructor's build preference (make/WSL vs CMake/Ninja) | Ask before the Week 10 demo; keep the README TODO |
| 4 | D2: 20 ms telemetry wake | Buddy 1 decides; FR1 rates hold either way |
| 5 | 2-bit sequence number in `obstacle_profile_t` | Buddy 5 decides after drain-then-send is tested |
| 6 | NFR2 wording (D7) | Team approves after the top-speed count test |
| 7 | `ENC_CPR`, `WHEEL_CIRC_MM`, `CAR_WIDTH_MM`, IMU addresses, servo pulse range | Measure on the car |
| 8 | Barr C: bit-fields for private state? | Buddy 1 checks the standard; masks and shifts until then |
| 9 | Template author for NOTICE.md | The module instructor's name as given on the course page |
| 10 | Should each buddy get a small SDK-only bring-up program to test hardware before Batch B lands? | Optional; Batch B first is simpler. Needs team agreement because it adds a CMake target |

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
