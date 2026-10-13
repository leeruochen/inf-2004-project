# Architecture notes

Phase 1 is a skeleton: build system, kernel integration, config and stubs.
The design reference is `design.md`. This file records where the
implementation deliberately differs from it, and the rules the build enforces.

## Source layout

```
config/            pure-macro project config; config/mtk3/ = kernel config copies
src/<module>/      firmware modules: common, rtos (port layer in src/rtos/port/),
                   debug, motion, vc, safety, line, barcode, obstacle, imu,
                   telemetry, app
cmake/             build helpers, linker script, rule checks
external/mtk3_bsp  micro T-Kernel 3.0 BSP submodule (pinned, never edited)
tests/             host tests and rule-check fixtures
docs/              developer docs
```

Each module registers its files in `src/<module>/sources.cmake`. Configure
fails on a `sources.cmake` anywhere else, or for a module not listed in
`ROBOCAR_MODULE_DIRS` (`CMakeLists.txt`).

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

Task count: 11 kernel tasks in total (nine design tasks, init, debug drain), so
`config_check.h` asserts `CNF_MAX_TSKID >= 11`.

## Rules enforced by the build

`cmake/check_rules.cmake` (target `check_rules`, runs before `robocar`). It
scans every C/C++/asm and CMake file in the tree except `external/`, `build*/`,
`tests/` and `docs/`, so new directories are checked automatically.

- R1: a source file includes either `<tk/tkernel.h>` (kernel side) or SDK
  headers, never both.
- R2: no `sleep_ms`, `sleep_us`, `busy_wait*`, `mutex_enter_blocking`,
  `sem_acquire*` in tasks. Use `tk_dly_tsk`. Only `src/app/main.c` may wait
  before the kernel starts (line marked `RULE-ALLOW: pre-kernel`).
- R3: stdio only in `src/debug/dbg_drain.c`. `tm_printf` and friends only for
  boot and fault messages in `src/rtos/port/port_boot.c` /
  `src/rtos/port/port_fault.c`.
- R4: `libbsp_pwm.c` is never linked or referenced.
- R5: no core 1 launch, no `pico_multicore`.

The build also separates the two sides: kernel include paths reach only the
kernel-side OBJECT library (`robocar_kside`), and `mtk3` never sees SDK headers.
`-Wall -Wextra -Werror` applies to project sources only, not to `mtk3` or SDK
sources.
