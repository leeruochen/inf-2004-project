# Architecture notes

Phase 1 is a skeleton: build system, kernel integration, config and stubs.
The design reference is `design.md`. This file records where the
implementation deliberately differs from it, and the rules the build enforces.

## Source layout

Two source roots: `src/` is platform code, `app_program/` is team code (the
course template's layout).

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
ProjectTemplate/   course template, reference only: never built, never scanned
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
`ProjectTemplate/` is never checked.

Until `src/app/main.c` exists (Batch B) the firmware cannot link, so configure
prints `firmware target skipped, app_program compile-checked` and builds two
compile-only OBJECT libraries in ALL, both with `-Wall -Wextra -Werror`:
`app_program_check` (every kernel-side source, kernel include paths only) and
`app_program_check_sdk` (every SDK-side source, `hardware_*` include paths;
created only when there is one).

## Kernel integration

- Approach (a): the Pico SDK owns crt0, boot2, clocks, the linker script
  (`cmake/memmap_mtk3.ld`, a project copy of the SDK default) and the vector
  table. micro T-Kernel 3.0 (`external/mtk3_bsp`, pinned) is the static
  library `mtk3` behind the port layer (`src/rtos/port/`). All kernel-internal
  symbol use lives in `src/rtos/port/port_kernel_internal.c` (Batch B).
- Kernel config: `config/mtk3/` (project copies, changes marked `PROJECT:`),
  checked by `config/mtk3/config_check.h`.
- Kernel priorities 1 to 9 = design P1 to P9. The init task (priority 1)
  creates every object and task, then sleeps with `tk_slp_tsk(TMO_FEVR)`.
- Core 1 stays unused. SMP is not enabled.

### Two entry points

- `src/app/main.c` (platform, Batch B, not created yet) is the SDK `main()`.
  It initialises the SDK, waits up to `DBG_USB_CONNECT_WAIT_MS` for the USB
  CDC host (the only allowed SDK wait, before the kernel starts), then starts
  micro T-Kernel.
- `app_program/app_main.c` is `usermain()`, the kernel-side start. The kernel
  creates its init task (priority 1, 1 KB stack) and the init task calls
  `usermain()`, which runs in this order:
  1. `ipc_init()` creates every IPC object (MBF_NAV_IN, MBF_MOTION_CMD,
     MBF_OBSTACLE, MBF_TELEMETRY, SEM_ECHO, EVF_SAFETY, EVF_NAV, MTX_MOTION).
     TODO(phase 1 batch B, buddy 2): the module is not created yet.
  2. every `subsystemN_init()` (N = 1..5);
  3. every `subsystemN_start()`;
  4. `APP_TEST` builds only: start the test task (priority 11);
  5. `tk_slp_tsk(TMO_FEVR)`. Returning from `usermain()` shuts the kernel down.

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
| D2 | Sections 3.1/3.3: Telemetry periodic 200 ms, MBF_TELEMETRY receive timeout 200 ms | PROPOSED, TODO(owner-to-confirm): Telemetry wakes every 20 ms, polls MBF_TELEMETRY (`TMO_POL`), publishes every 10th wake (200 ms) and heartbeats every 50th wake (1 s) | Keeps the FR1 publish/heartbeat rates while never blocking in the receive. Comes from the integration report's cyclic-handler table; not yet approved. Constants in `config/timing.h`. |
| D3 | Sections 3 and 3.3: Barcode Decode is woken by the barcode ISR (`tk_wup_tsk`) | While capturing, the Barcode task polls the edge ring every `BC_POLL_PERIOD_MS` = 2 ms (`config/timing.h`) | Batch A implementation choice (`timing.h` marks it "Not in design.md"); the Week 8 document flags it as undocumented, so it is recorded here |
| D4 | Section 6.4: accelerometer + gyro, complementary filter | `IMU_HAS_GYRO 0` (`config/thresholds.h`): pitch from filtered accelerometer data; TURNING from the encoder wheel-speed difference | The fitted GY-511 (LSM303DLHC, accel 0x19 + mag 0x1E, VERIFY by bus scan) has no gyro. TODO(owner-to-confirm, buddy 4) in `thresholds.h` |
| D5 | Section 2 pin table | Week 8 pin map v2 (`config/pins.h`, `TEAM.md` section 2): encoders quadrature GP0/GP1 (right, Grove 1) and GP16/GP17 (left, Grove 4), A on the lower pin is a provisional label; IR_R GP26/ADC0 (Grove 5), IR_C GP28/ADC2 (Grove 7, VBAT jumper open); barcode IR_BC GP27 digital (Grove 6); servo GP15 = PWM slice 7 B (confirmed; never GP12); spares GP6, GP7, GP12-GP14. GP26 has a second, untaped lead on Grove 6 (same pin); firmware cannot disconnect it, so `hal_adc_sanity_check()` flags a stuck or noisy GP26 at boot | Supersedes design.md section 2. The Week 8 document (`INF 2004 Embedded Systems.md`, read only) still lists the old pins and is not edited |
| D6 | Phase 1 console decision: Debug Probe UART on UART0 (GP0/GP1); kernel boot/fault output on polled UART0 | USB CDC is the only console; the Debug Probe is SWD only. `DBG_BACKEND=UART0` fails at configure, and `pins.h` `#error`s on `DBG_BACKEND_UART0`. Kernel text from `tm_snd_dat` (the `tm_printf` boot banner and system messages) goes into a system-message ring in RAM, separate from the fault record: one writer (`tm_snd_dat`, never blocks), one reader (the debug drain, which empties it every cycle), so no `tm_*` output is lost after boot either (owner decision C3, `.agents/tasks/batchB-plan.md`). Faults go to a RAM fault record in the SDK's `.uninitialized_data` (NOLOAD, not zeroed; not `.noinit`), with a magic number first and a CRC-32 over the payload; the fault record holds faults only. Ring text written during a HardFault is lost with the reset, so the fault record is the authority for crashes. The fault record is read over SWD, and on the next boot the debug drain prints and clears it over USB CDC only if both magic and CRC match; otherwise it is treated as random RAM (power-on, flashing) and silently re-initialised (`docs/DEBUG.md`). Implementation: Batch B | GP0/GP1 are now the right encoder, and the `pico_w` board puts stdio UART on GP0/GP1. USB CDC cannot run from a HardFault or with interrupts masked |
| D7 | NFR2: encoder pulses "captured using GPIO interrupts" | Encoders counted by PIO0, one state machine per wheel (SM0 right, SM1 left), one shared copy of the pico-examples `quadrature_encoder.pio` program (tag `sdk-1.5.1`, BSD-3-Clause, `NOTICE.md`) loaded at offset 0 (24 of 32 instructions). API `enc_get_count()` (`app_program/subsystem2/drv_encoder.h`); the reader is non-blocking (never `pio_sm_get_blocking`, rule R2). The Motion task is the single writer of the published count words. Proposed NFR2 wording, TODO(owner-to-confirm, team): "Wheel encoder pulses shall be captured by hardware (PIO quadrature decoding) on both wheels independently, with zero missed counts under normal operating speed (target: up to 50 cm/s)." "Zero missed counts" needs a top-speed test: drive a measured distance at 50 cm/s and compare counts against expected, both wheels | PIO decoding counts with zero CPU and keeps counting through kernel interrupt-masked sections, and it takes the encoders off the shared IO_IRQ_BANK0. No team document is edited; the team approves the wording |
| D8 | Week 6 types: natural `<stdint.h>` structs, raw 64-bit edge timestamps | Minimum-bit representation (`docs/DATA_REPRESENTATION.md`): masks and shifts only, no C bit-fields; the shared header is structured so Buddy 1 can later switch PRIVATE state to bit-fields without changing any wire format. Pitch signed ±45° at 0.5° in 8 bits, clamp + OOR. `motion_cmd_t`: type 3 + speed 6 + one 9-bit argument by type (distance / angle / signed steer) + OOR = 19 bits. `obstacle_profile_t`: adds left/right clearance (7 bits each), drops the timestamp, removes "bypass state", keeps an optional 2-bit sequence number (TODO(owner-to-confirm, buddy 5)). `nav_input_t` gains a 3-bit bypass command (stop, continue, left, right, reverse), flagged for Buddy 5. Edge ring: 20-bit gaps. Encoder words stay unpacked `int32_t`. Implementation: Batch B-packed | Week 8 data-representation decision; bit-packing barely changes RAM (message-buffer slots round up to words inside the fixed kernel pool), so only map-file figures are reported |
| D9 | Section 3.3: EVF_SAFETY = COLLISION, IMU_ALIVE, FAULT_LATCHED | EVF_SAFETY is extended with fault bits, one setter each (`docs/DATA_REPRESENTATION.md`). Safety waits on `COLLISION\|IMU_ALIVE` with `TWF_ORW\|TWF_BITCLR`, never `TWF_CLR`. FAULT_LATCHED (called "collision latch" before) latches both collisions and IMU watchdog trips; only Safety sets it, only the Vehicle Controller clears it. "No echo" is an informational status, outside every wait mask and the fault count. Readers other than Safety use `tk_ref_flg` (the flag is `TA_WSGL`) | `TWF_BITCLR` clears only the waited bits; `TWF_CLR` would zero every owner's bits; a second `tk_wai_flg` on a `TA_WSGL` flag returns `E_OBJ` (eventflag.c:249-251) |
| D10 | Design overview: JSON/struct MQTT payloads | MQTT uses binary frames: version byte first, then type and length, little-endian, masks and shifts. A command is rejected unless the version and length match; rejects are logged and never reach the Vehicle Controller. A reference JSON baseline (`tools/json_baseline/`, sizes from `tools/json_sizes.cmake`) gives the "before" sizes. The dashboard language is not decided: a language-neutral field table only, no decoder | Week 8 document decision: JSON replaced by compact binary, with version and length checks on every command |
| D11 | Section 6.1: line sensors left + centre (IR_L, IR_C) | Line sensors CENTRE (IR_C, GP28) + RIGHT (IR_R, GP26), the mirror of the design. The steering sign is redefined, not copied from design.md; proposal `steer_err > 0` = line to the car's right = steer right, TODO(owner-to-confirm, buddy 3). With right + centre only, a left branch cannot be seen | Week 8 wiring |

Task count: 12 kernel tasks in total (nine design tasks, init, debug drain,
APP_TEST test task). All are counted even when `DBG_ENABLE` or `APP_TEST` is
OFF, because `config_check.h` is also compiled inside `mtk3`, which never sees
those options. So `config_check.h` asserts `CNF_MAX_TSKID >= 12`.

## Template deviations

The course template (`ProjectTemplate/`, reference only) is merged as
"Option C": our build and kernel are the base, and we adopt its
`app_program/subsystemN/` layout, `TEAM.md`, `testing/` and `videos/`. Where
we differ from it, and why (details in [docs/TEMPLATE_NOTES.md](docs/TEMPLATE_NOTES.md)):

| # | Template | Ours | Reason |
|---|---|---|---|
| T1 | SMP kernel copy; tasks pinned to a core (`TP_PRC1`/`TP_PRC2`, `TA_ASSPRC`) | Single-core `mtk3_bsp` (`pico_rp2040`); core 1 unused; `app_start_task()` has no core argument | Pinned official BSP; rule R5 |
| T2 | GNU make under WSL, Pico SDK 2.2.0 (`build_make/`) | CMake and Ninja on Windows, Pico SDK 1.5.1 | Our build system (Batch A). TODO(owner-to-confirm): instructor's build preference |
| T3 | `tm_printf` from tasks; `CHECK()` prints with `tm_printf` | Tasks log through the `dbg_` ring; `APP_CHECK()` | Rule R3b; no control task blocks on console I/O (D1) |
| T4 | `make TEST=1`: tests run in the 1 KB init task inside `usermain()` | `APP_TEST=ON`: tests run in a dedicated test task at priority 11, below the debug drain; needs `DBG_ENABLE=ON` | The init stack is 1 KB and must stay light; test output goes through `dbg_` |
| T5 | Each subsystem creates the objects it owns in `subsystemN_init()` | `ipc_init()` creates every IPC object centrally in `usermain()` step 1, before any `subsystemN_init()` | Owner decision (merge brief): every object exists before any subsystem code runs, and the depths and timeouts of design.md 3.3 sit in one place. The receiving subsystem still owns the IF |
| T6 | One `TASK_STACK_SIZE` (2 KB) for every task | Per-task stacks from `config/priorities.h` (+512 B interrupt margin) | Sized per task; fits the 96 KB kernel pool check |
| T7 | libbsp (`gpio_set_pin`, `pwm_...`) and kernel `device/` drivers | SDK `hardware_*` in `drv_*.c` only; no libbsp, no device manager | `libbsp_pwm.c` clashes with the SDK PWM API (R4); the SDK owns the peripherals |

## Rules enforced by the build

`cmake/check_rules.cmake` (target `check_rules`, runs before the firmware and
before both `app_program_check` targets). It scans every C/C++/asm and CMake
file in the tree except the top-level `external/`, `build*/`, `tests/`,
`docs/`, `.git/`, `.agents/`, `.vscode/`, `ProjectTemplate/`, `testing/` and
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

The build also separates the two sides: kernel include paths reach only the
kernel-side OBJECT libraries (`robocar_kside` with the firmware,
`app_program_check` before Batch B), and `mtk3` never sees SDK headers.
`-Wall -Wextra -Werror` applies to project sources only, not to `mtk3` or SDK
sources.
