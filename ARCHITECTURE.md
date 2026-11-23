# Architecture notes

All 5 subsystems (`app_program/subsystem1..5/`) and the Batch B platform
(`src/`) are integrated and running together on `main`. The baseline design
reference is `design.md` (team-owned). This file records
the source layout, kernel integration, enforced build rules, and every design
delta (`D1`–`D17`) and template deviation (`T1`–`T8`) against `design.md`.

**Software Block Diagram:** [`docs/trib2/software_block_diagram.svg`](docs/trib2/software_block_diagram.svg).

## Source layout

Two source roots: `src/` is platform code, `app_program/` is team code (the
course template's layout), and `tools/dashboard/` holds the laptop-side CLI
and web telemetry dashboard.

```
config/            pure-macro project config; config/mtk3/ = kernel config copies
src/               platform code (not team code), created in Batch B:
  common/            shared library code
  rtos/port/         kernel port layer
  debug/             dbg_ ring and the debug drain task
  app/main.c         SDK main(): SDK init, then starts the kernel
app_program/       team code:
  app.h              shared definitions, subsystemN_init/start/test prototypes
  app_main.c         usermain(), the kernel-side start (two-phase)
  test_report.h      TEST_INIT / TEST_CHECK / TEST_SUMMARY (APP_TEST builds)
  subsystem1/        Buddy 1: telemetry
  subsystem2/        Buddy 2: motion control and Vehicle Controller
  subsystem3/        Buddy 3: line following and barcode decode
  subsystem4/        Buddy 4: IMU processing and safety (task_safety.c)
  subsystem5/        Buddy 5: obstacle scanning and avoidance
TEAM.md            owners, pins, peripherals, priorities, interfaces, budget
testing/           test logs (unit, integration, system) and saved console logs
videos/            video index (unlisted YouTube links only)
cmake/             build helpers, linker script, rule checks
tools/             host-side scripts (cmake -P)
external/mtk3_bsp  micro T-Kernel 3.0 BSP submodule (pinned, never edited)
tests/             host tests and rule-check fixtures
docs/              developer docs
```

Inside a subsystem folder: `subsystemN.c` (init/start), `subsystemN.h` (its
public interface), `subsystemN_test.c`, tasks as `task_<name>.c`, drivers as
`drv_<name>.c` / `drv_<name>.h`. `drv_*.c` include SDK headers only (plus
their own kernel-free `drv_*.h`, `<stdint.h>` only). `task_*.c`,
`subsystemN.c` and `subsystemN_test.c` include kernel headers only. A
`drv_*.c` never includes `subsystemN.h`.

Each directory listed in `ROBOCAR_MODULE_DIRS` (`CMakeLists.txt`: `src/common`,
`src/rtos`, `src/debug`, `src/app`, `app_program`, `app_program/subsystem1..5`)
registers its files in its own `sources.cmake`, in `ROBOCAR_KERNEL_SOURCES`
(kernel side) or `ROBOCAR_SOURCES` (SDK side). Configure fails on any other
`sources.cmake` at the top level or anywhere under `src/` or `app_program/`.

Because `src/app/main.c` and the port layer (`src/rtos/port/`) are complete,
CMake builds `robocar_kside` (`ROBOCAR_KERNEL_SOURCES`, kernel include paths
only) and links it into `robocar.elf` / `robocar.uf2` (`ROBOCAR_SOURCES` +
`mtk3` + Pico SDK libraries, plus `pico_cyw43_arch_lwip_poll` / `pico_lwip_mqtt`
when `TEL_BACKEND_WIFI=ON`), gated by `check_rules` and `-Wall -Wextra -Werror`.

## Kernel integration

- Approach (a): the Pico SDK owns crt0, boot2, clocks, the linker script
  (`cmake/memmap_mtk3.ld`, a project copy of the SDK default) and the vector
  table. micro T-Kernel 3.0 (`external/mtk3_bsp`, pinned) is the static
  library `mtk3` behind the port layer (`src/rtos/port/`). All kernel-internal
  symbol use lives in `src/rtos/port/port_kernel_internal.c`.
- Kernel config: `config/mtk3/` (project copies, changes marked `PROJECT:`),
  checked by `config/mtk3/config_check.h`.
- Kernel priorities 1 to 9 = design P1 to P9. The init task (priority 1)
  creates every object and task, then sleeps with `tk_slp_tsk(TMO_FEVR)`.
- Core 1 stays unused. SMP is not enabled.

### Two entry points

- `src/app/main.c` (platform) is the SDK `main()`.
  It initialises the SDK, waits up to `DBG_USB_CONNECT_WAIT_MS` for the USB
  CDC host (the only allowed SDK wait, before the kernel starts), then starts
  micro T-Kernel via `hal_kernel_start()`.
- `app_program/app_main.c` is `usermain()`, the kernel-side start. The kernel
  creates its init task (priority 1, 1 KB stack) and the init task calls
  `usermain()`, which runs in this order:
  1. `ipc_init()` (`src/common/ipc.c`) creates every IPC object (`MBF_NAV_IN`,
     `MBF_MOTION_CMD`, `MBF_OBSTACLE`, `MBF_TELEMETRY`, `SEM_ECHO`,
     `EVF_SAFETY`, `EVF_NAV`, `MTX_MOTION`);
  2. `dbg_drain_start()` (priority 10, `DBG_ENABLE` builds only);
  3. every `subsystemN_init()` (N = 1..5);
  4. every `subsystemN_start()` (or `dbg_bringup_start()` when `PLATFORM_BRINGUP=ON`);
  5. `APP_TEST` builds only: start the test task (priority 11);
  6. `tk_slp_tsk(TMO_FEVR)`. Returning from `usermain()` shuts the kernel down.

### Known exception: init task stack

Every task stack is its stated size + 512 B for interrupt frames, except the
kernel init task. Its stack is `INITTASK_STKSZ`, fixed at 1 KB in vendored
`include/sys/inittask.h` and not overridable without editing vendored code.
`usermain` runs on it, with interrupts live, while it creates every object and
task. So `usermain` stays stack-light (no large locals, no stdio), and the init
task is included in the Batch B stack high-water report.

## Design deltas against design.md

| # | design.md | Implementation | Reason |
|---|---|---|---|
| D1 | Section 3: nine tasks | Nine tasks plus one debug-only task: the debug drain at kernel priority 10, built only when `DBG_ENABLE=ON` | The drain empties the `dbg_` ring and is the ONLY caller of stdio, so no control task ever blocks on console I/O. |
| D2 | Sections 3.1/3.3: Telemetry periodic 200 ms, MBF_TELEMETRY receive timeout 200 ms | **Implemented (`task_tel.c`):** Telemetry wakes every 20 ms (`app_period_wait`), polls MBF_TELEMETRY (`TMO_POL`), services poll-mode lwIP (`net_poll`), publishes motion/sensor every 10th wake (200 ms), and heartbeats every 50th wake (1 s; state-change heartbeats push straight into `s_outbox`) | Poll-mode CYW43/lwIP requires frequent servicing so TCP ACKs and keep-alives do not stall, while preserving FR1 publish/heartbeat rates. Constants in `config/timing.h`. |
| D3 | Sections 3 and 3.3: Barcode Decode is woken by the barcode ISR (`tk_wup_tsk`) | While capturing, the Barcode task polls the edge ring every `BC_POLL_PERIOD_MS` = 2 ms (`config/timing.h`) | Allows pairing each burst of edges with live Motion speed without busy-waiting or ring overflow. |
| D4 | Section 6.4: accelerometer + gyro, complementary filter | `IMU_HAS_GYRO 0` (`config/thresholds.h`): `drv_imu.c` auto-detects LSM303D (`0x1D`/`0x1E`) or LSM303DLHC (`0x19`/`0x1E`) on I2C1; pitch/roll from low-passed accelerometer; TURNING and speed-change gated from wheel encoders; hump estimation calibrated in `imu_hump.c` (see D15) | Fitted GY-511 hardware has no gyro; encoder turn/speed-change gating removes centripetal and braking pitch artefacts. |
| D5 | Section 2 pin table | Week 8 pin map v2 (`config/pins.h`, `TEAM.md` section 2): encoders quadrature GP0/GP1 (right, Grove 1) and GP16/GP17 (left, Grove 4), A on the lower pin; IR_R GP26/ADC0 (Grove 5), IR_C GP28/ADC2 (Grove 7, VBAT jumper open); barcode IR_BC GP27 digital (Grove 6); servo GP15 = PWM slice 7 B (confirmed; never GP12); spares GP6, GP7, GP12-GP14. GP26 has a second lead on Grove 6 (same pin), so `drv_ir_sanity()` / `hal_adc_sanity_check()` flags a stuck or noisy GP26 at boot | Supersedes `design.md` section 2. `INF 2004 Embedded Systems.md` (read only) still lists the old pins and is not edited. |
| D6 | Phase 1 console decision: Debug Probe UART on UART0 (GP0/GP1); kernel boot/fault output on polled UART0 | USB CDC is the only console; the Debug Probe is SWD only. `DBG_BACKEND=UART0` fails at configure, and `pins.h` `#error`s on `DBG_BACKEND_UART0`. Kernel text from `tm_snd_dat` (boot banner and system messages) goes into a 512 B system-message ring in RAM emptied every cycle by the debug drain. Faults go to a RAM fault record in `.uninitialized_data` (`HAL_FAULT_MAGIC = 0x464C5452` + CRC-32) printed and cleared on the next boot when USB CDC connects (`docs/DEBUG.md`). **Implemented (Batch B)** | GP0/GP1 are the right encoder, and the `pico_w` board puts stdio UART on GP0/GP1. USB CDC cannot run from a HardFault or with interrupts masked. |
| D7 | NFR2: encoder pulses "captured using GPIO interrupts" | Encoders counted by PIO0 (`SM0` right, `SM1` left), one shared copy of `quadrature_encoder.pio` (tag `sdk-1.5.1`, BSD-3-Clause, `NOTICE.md`) at offset 0 (24/32 insns). API `enc_get_count()` (`drv_encoder.h`), non-blocking 4-word FIFO drain (rule R2). Verified on car at `50 cm/s` (`S2-U21`: `0` missed counts, `ENC_CPR=616U`, `WHEEL_CIRC_MM=207U`, `MOTION_EFF_TRACK_MM=114U`) | Zero CPU overhead, keeps counting across critical sections, and keeps high-rate encoder edges off `IO_IRQ_BANK0`. |
| D8 | Week 6 types: natural `<stdint.h>` structs, raw 64-bit edge timestamps | Minimum-bit packed types in `src/common/data_repr.h` (`docs/DATA_REPRESENTATION.md`, Batch B stage B7): masks and shifts only, no C bit-fields. `motion_cmd_t` 19 bits; `obstacle_profile_t` 40 bits (adds 7-bit `clear_L`/`clear_R`, 2-bit `seq`, and 3-bit `width % 5` for 1 mm precision; drops timestamp and bypass state); `nav_input_t` adds 3-bit bypass cmd; System FSM 4 bits (10 states); edge ring 20-bit gaps; unpacked `int32_t` encoder counts | Standardizes IPC widths, eliminates bit-field ABI ambiguity, and shrinks MBF footprints inside the 96 KB pool (`docs/measurements/week8-packed.txt`). |
| D9 | Section 3.3: EVF_SAFETY = COLLISION (now LIFTED, 11 Nov 2026), IMU_ALIVE, FAULT_LATCHED | **Implemented (`src/common/ipc_types.h`):** 10 defined bits (`b0..b9`), one setter per bit (`docs/DATA_REPRESENTATION.md` section 3). `b0` is `LIFTED` (renamed from COLLISION on 11 Nov 2026). Safety waits on `LIFTED\|IMU_ALIVE` with `TWF_ORW\|TWF_BITCLR`, never `TWF_CLR`. `FAULT_LATCHED` is set by Safety and cleared only by VC on stationary RESET. Other tasks poll via `tk_ref_flg` (`TA_WSGL`) | `TWF_BITCLR` clears only waited bits; `TWF_CLR` would wipe other fault bits; `TA_WSGL` rejects a second waiter with `E_OBJ`. |
| D10 | Design overview: JSON/struct MQTT payloads | **Implemented (`data_repr.h`, `tel_wire.c`, `tools/dashboard/tel_codec.py`):** Versioned little-endian binary frames `ver u8 \| type u8 \| len u8 \| payload` across types `1..11` (including `DREP_FT_CMD_BC_MAP = 11` and `DREP_SENSOR_VALID_POS` validity flags). Commands are checked on version, type, length, and topic before reaching VC (`START/STOP/RESET`) or Barcode (`bc_map`). Reference JSON baseline stays in `tools/json_baseline/`. Laptop tooling implemented in Python 3 (`tools/dashboard/`) | Binary cuts steady-state MQTT wire bandwidth by ~70 % vs JSON; strict frame checking prevents malformed packets from actuating motors (FR2). |
| D11 | Section 6.1: line sensors left + centre (IR_L, IR_C) | **Implemented (`config/pins.h`, `lf_core.c`, `lf_straddle.c`):** Line sensors on `GP28` (`ADC2`, `IR_C`) + `GP26` (`ADC0`, `IR_R`), with `steer_err_q7 > 0` defined as steer right. Supports both right-edge tracking (`lf_core.c`, default) and dual-side straddle mounting (`lf_straddle.c` under `-DS3_LF_STRADDLE=ON`, where `IR_C` sits on the left and `IR_R` on the right of the 18 mm tape) | Week 8 wiring and track-curve reliability improvement (`D14`). |
| D12 | NFR15: obstacle width estimation error "within 10%" | **Implemented & extended (`D16`):** NFR15 requirement wording accepts `±10%` or `±1` fine-scan bearing slot, whichever is larger (matching NFR11's dual-threshold pattern); hardware scanning steps every `3°` across `51°–129°` when stopped (see D16) | Geometric resolution bound for narrow targets vs ultrasonic beam cone. |
| D13 | Section 3.2 / 6.1: T_lost 100 ms ("both sensors no line continuously for 100 ms" → LINE_LOST) | **Implemented on `main` (`lf_core.c`, `lf_straddle.c`):** `LINE_LOST` triggers after `LF_T_LOST_MS` (100 ms) **and** `LF_D_LOST_MM` (`20 mm`) of forward travel (capped at `LF_T_LOST_MAX_MS = 400 ms`), and an in-drive lost-line search ends after `LF_D_SEARCH_MAX_MM` (`150 mm`) if `T_search` (`2.0 s`) has not expired first (turn/start/avoidance hand-back searches use distance cap `LF_SD_SEARCH_MM = 400 mm` in straddle mode so slow `5 cm/s` runs still sweep the full arc) | Time-only limits tuned for `20–30 cm/s` caused premature line-loss trips or overly short search arcs at `5–15 cm/s` bring-up speeds. |
| D14 | Section 6.1 / 3.2: proportional right-edge line tracking + symmetric duty scaling | **Integrated track configuration (`-DS3_LF_STRADDLE=ON -DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1`):** Straddle follower uses simple symmetric left/right bang-control (`±127` Q7 outside a `100 pm` deadband) without blind deep-contact holds; `task_motion.c` (`MOTION_STEER_TARGETS=1`) converts `steer_q7` into distinct left/right closed-loop encoder velocity targets around `5 cm/s` cruise (`lf_straddle.h`, `task_motion.c`) | Eliminates blind turn-away swings on 90°/180° curves and overcomes asymmetric motor friction during slow-speed bends. |
| D15 | Sections 3.1/3.2/6.4/A4/A7/FR12: IMU collision detection (> 2.5 g) & raw `5°` hump pitch | **Implemented on `main` (`docs/REQ_CHANGE_LIFT.md`, `imu_lift.c`, `imu_hump.c`, `task_motion.c`):** (1) **11 Nov req change:** Collision detection removed; replaced by **lift/overturn** (`EVF_SAFETY_LIFTED`: tilt `> 30°` or upward speed `> 0.10 m/s` for `50 ms` against boot level ref), ending the run until `car/cmd/reset`. (2) **Calibrated hump detection:** `IMU_HUMP_PITCH_DEG = 3°` entry (`2.5°` exit), `IMU_HUMP_WHEELBASE_MM = 95U` mm, `120 mm` level-end window, `15 mm` min travel, `9 mm` min height, `max(tilt, climb)` with turn & speed-step (`> 80 mm/s` per `20 ms`) gating. (3) **Hump climb assist:** `task_motion.c` reads `subsystem4_get_pitch_half_deg()` and ramps target speed/torque (`+3/+5 mm/s` per `20 ms`, up to `+4.5 cm/s`) on `>= 3°` inclines | Prof Fauzi removed IMU collision (conflicted with obstacle avoidance); `3°` + `95 mm` wheelbase matches real `10–15 mm` track humps (~4.5°–9° tilt) while climb boost prevents stalling at `5 cm/s`. |
| D16 | Sections 3.1/3.3/6.3/NFR14/NFR15: continuous 5-bearing coarse scan + `10°` fine scan + `25 cm/s` open-loop legs | **Implemented on `main` (`task_obs_scan.c`, `task_obs_avoid.c`):** (1) Scanner watches ahead at `90°`, immediately halts (`DREP_NAV_AVOID_SLOW` + `DREP_BYPASS_STOP`), confirms standstill, then slews smoothly and runs a stationary `3°` width sweep across `51°–129°` (4 pings / `8 ms` timeout / `400 mm` cap, `OBS_SWEEP_MIN_HITS = 2` -> 3-echo median, single-slot oblique dropout bridging, `OBS_FINE3_BEAM_DEG = 28°`, `OBS_EDGE_TOL_MM = 65 mm`, `OBS_EDGE_STEP_MM = 35 mm`, 1 mm width wire resolution in 40-bit profile). (2) Avoidance runs encoder-closed 4-turn `45°` bypass (`CAR_WIDTH_MM = 130U` mm measured, `OBS_OBSTACLE_DEPTH_MM = 100U` mm) + directional rejoin search, releasing on `LINE_FOUND`. (3) Bring-up flag `OBS_BYPASS_SLOW_BRINGUP = 1U` temporarily sets bypass speed to `5 cm/s` (`60 s` budget; nominal `25 cm/s` / `5 s` NFR14) | Stopping before sweeping and stepping `3°` with 2-hit oblique filtering fixes motion skew, side-wall reflections, and specular dropouts; `5 cm/s` bypass prevents overshooting the line during reacquisition (clear flag `0` for `NFR14` run). |
| D17 | FR9 / Section 6.2 / D10: `A–D` fixed barcode actions, `BC_D_CMD_CM = 30 cm`, fixed map | **Implemented on `main` (`task_bc.c`, `tel_wire.c`, `data_repr.h`, `tel_web.py`):** Decodes full `'A'..'Z'` Code 39 (`BC_MIN_SPEED_MMPS = 20 mm/s` for `5 cm/s` cruise); `DREP_FT_EVENT_BARCODE` carries optional 5-bit letter offset + 1-bit unmapped flag in byte 6; dashboard configures `[L, R, S, U]` at runtime via non-retained binary command `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`, validated by `tel_wire_bc_map()`, calls `bc_task_set_map()`); pending junction command expires after `BC_D_CMD_CM = 60U` cm (`task_vc.c`) | Module clarification decoupled decoded letters (`A–Z`) from fixed actions; non-retained framing prevents stale mappings across sessions while `60 cm` prevents premature expiry on long approach straights. |

Task count: 12 kernel tasks in total (nine design tasks, init, debug drain,
APP_TEST test task). All are counted even when `DBG_ENABLE` or `APP_TEST` is
OFF, because `config_check.h` is also compiled inside `mtk3`, which never sees
those options. So `config_check.h` asserts `CNF_MAX_TSKID >= 12`.

## Template deviations

The INF2004 course project template is integrated using
"Option C": our build and kernel are the base, and we adopt its
`app_program/subsystemN/` layout, `TEAM.md`, `testing/` and `videos/`. Where
we differ from it, and why:

| # | Template | Ours | Reason |
|---|---|---|---|
| T1 | SMP kernel copy; tasks pinned to a core (`TP_PRC1`/`TP_PRC2`, `TA_ASSPRC`) | Single-core `mtk3_bsp` (`pico_rp2040`); core 1 unused; `app_start_task()` has no core argument | Pinned official BSP; rule R5 |
| T2 | GNU make under WSL, Pico SDK 2.2.0 (`build_make/`) | CMake and Ninja on Windows, Pico SDK 1.5.1 | Integrated Windows toolchain build system (Batch A) |
| T3 | `tm_printf` from tasks; `CHECK()` prints with `tm_printf` | Tasks log through the `dbg_` ring; `APP_CHECK()` | Rule R3b; no control task blocks on console I/O (D1) |
| T4 | `make TEST=1`: tests run in the 1 KB init task inside `usermain()` | `APP_TEST=ON`: tests run in a dedicated test task at priority 11, below the debug drain; needs `DBG_ENABLE=ON` | The init stack is 1 KB and must stay light; test output goes through `dbg_` |
| T5 | Each subsystem creates the objects it owns in `subsystemN_init()` | `ipc_init()` creates every IPC object centrally in `usermain()` step 1, before any `subsystemN_init()` | Owner decision (merge brief): every object exists before any subsystem code runs, and the depths and timeouts of design.md 3.3 sit in one place. The receiving subsystem still owns the IF |
| T6 | One `TASK_STACK_SIZE` (2 KB) for every task | Per-task stacks from `config/priorities.h` (+512 B interrupt margin) | Sized per task; fits the 96 KB kernel pool check |
| T7 | libbsp (`gpio_set_pin`, `pwm_...`) and kernel `device/` drivers | SDK `hardware_*` in `drv_*.c` only; no libbsp, no device manager | `libbsp_pwm.c` clashes with the SDK PWM API (R4); the SDK owns the peripherals |
| T8 | Periodic tasks woken by a cyclic handler (`tk_cre_cyc` → `tk_wup_tsk()`; the task loops on `tk_slp_tsk`, `tk_can_wup()`) | Each periodic task paces itself with `app_period_wait()` (`src/common/period.h`): drift-free releases on its own grid, missed periods counted. In use (3 Nov 2026): Motion (S2), Line-Following (S3), IMU (S4), Obstacle Scanning (S5), Telemetry (S1); no cyclic handler is created | One kernel object fewer per task, and no wake-up queue to discard. Adopted across S1–S5 |

## Rules enforced by the build

`cmake/check_rules.cmake` (target `check_rules`, runs before the firmware and
before both `app_program_check` targets). It scans every C/C++/asm and CMake
file in the tree except the top-level `external/`, `build*/`, `tests/`,
`docs/`, `.git/`, `.vscode/`, `testing/` and
`videos/`, so new directories (including `app_program/` and `tools/`) are
checked automatically.

- R1: a source file includes either kernel headers (`<tk/tkernel.h>`, or our
  kernel-side `app.h`, `test_report.h`, `subsystemN.h`) or SDK headers, never
  both. So a `drv_*.c` that includes `subsystemN.h` fails.
- R2: no `sleep_ms`, `sleep_us`, `busy_wait*`, `mutex_enter_blocking`,
  `sem_acquire*`, `pio_sm_get_blocking`, `pio_sm_put_blocking` in tasks. Use
  `tk_dly_tsk`. Only `src/app/main.c` may wait before the kernel starts (line
  marked `RULE-ALLOW: pre-kernel`).
- R3: stdio only in `src/debug/dbg_drain.c`. `tm_printf` and friends only for
  boot and fault messages in `src/rtos/port/port_boot.c` /
  `src/rtos/port/port_fault.c`.
- R4: `libbsp_pwm.c` is never linked or referenced.
- R5: no core 1 launch, no `pico_multicore`.
- R6: no `tk_ext_tsk` / `tk_exd_tsk`. A task ends by returning from its
  function. Every task starts through `app_start_task()`, and only
  `src/common/stack_mon.c` calls `tk_ext_tsk`, with the stack headroom the
  kernel needs. Called from a shallow stack, `tk_ext_tsk` overwrites its own
  return address and faults with `pc=0xfffffff8`. This happened in B4b, 30 Oct 2026.

Not checked by the build, but required: a periodic task waits with
`app_period_wait()` (`src/common/period.h`), not a `tk_dly_tsk(P)` loop. The
kernel wakes `tk_dly_tsk(d)` one tick late, so such a loop runs every P + 1 ms
(`docs/DEBUG.md`, "Periodic tasks").

The build also separates the two sides: kernel include paths reach only the
kernel-side OBJECT libraries (`robocar_kside` with the firmware,
`app_program_check` before Batch B), and `mtk3` never sees SDK headers.
`-Wall -Wextra -Werror` applies to project sources only, not to `mtk3` or SDK
sources.
