# Merge verification (Stage 1: template merge, MV-1..MV-8)

Repo root on this device: `C:\Users\asus\Downloads\inf-2004-project` (the plan's
`C:/Users/Admin/Downloads/Projects/inf-2004-project` paths substituted). SDK at the
default `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1`, so `$R` is unchanged.
HEAD = `67f9b00` (owner's WIP commit "workflow", contains Steps 0-1 and HANDOVER.md;
the plan's `df67420` is its parent's parent). No commit was made.

Starting state (HANDOVER section 0 step 3): `app_program/` absent; submodule folder was
empty (`-15ed232...`), fixed with `git submodule update --init`, now
` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)`.
Sanity build (`build/usb`, `-DDBG_BACKEND=USB`): configure 0, build 0, 0 warnings,
`check_rules: PASS (14 files scanned)`, self-test PASS.

Logs: `build/merge-logs/*.log` (git-ignored). Warning/error counts = lines matching
`warning:` / `error:` in each log.

## M-items

| Item | Status | Notes |
|---|---|---|
| M-01 | Done | `TASK_PRI_TEST (11U)`, `TASK_STACK_BASE_TEST_BYTES (1024U)`, `TASK_STACK_TEST_BYTES`, in total + multiple-of-8 check, `TASK_COUNT_TOTAL (12U)`, `#error` if test task not lowest; `config_check.h` `CNF_MAX_TSKID >= 12`, `CNF_MAX_TSKPRI >= TASK_PRI_TEST`. Mutation `TASK_PRI_TEST (10U)` -> `config_check` exit 1 with `priorities.h: the APP_TEST test task must be the lowest-priority task`; reverted (`git diff --stat` = 2 files, intended lines only) |
| M-02 | Done | `option(APP_TEST OFF)`, FATAL_ERROR without DBG_ENABLE, `APP_TEST=` in status line, `APP_TEST=1/0` define |
| M-03 | Done | R2 + `pio_sm_get_blocking|pio_sm_put_blocking`; R1 second kernel regex `_RE_KINC_PROJ` for `"[path/]app.h"`, `test_report.h`, `subsystemN.h`; header comment updated; allow-lists unchanged |
| M-04 | Done | `app_program/app.h` |
| M-05 | Done | `app_program/app_main.c` (usermain order 1-5, `app_start_task()` stub returns `E_NOSPT`) |
| M-06 | Done | `app_program/test_report.h` (P-4 `#error` inside `#if APP_TEST`) |
| M-07 | Done | 15 subsystem files; no demo task ported |
| M-08 | Done | 6 `sources.cmake` |
| M-09 | Done | include rule in `app.h` and every `subsystemN.h` |
| M-10 | Done | repo-relative `ROBOCAR_MODULE_DIRS`, guard over top-level + `src/**` + `app_program/**`, `app_program_check` / `app_program_check_sdk` OBJECT libs in ALL |
| M-11 | Done | five `git mv` (R100), two new fixtures, self-test expectations |
| M-12 | Done | `TEAM.md` |
| M-13 | Done | `testing/` (15 files) |
| M-14 | Done | `videos/README.md` (one line removed) |
| M-15 | Done | `docs/TEMPLATE_NOTES.md` |
| M-16 | Done | `ARCHITECTURE.md` source layout, two entry points, T1-T7, task count 12, rules |
| M-17 | Done | `NOTICE.md` appended; `docs/DEBUG.md` APP_TEST section; `cmake/mtk3.cmake` comment |

## Commands and results

Preamble from HANDOVER section 3 in every run (`build/env.ps1`, git-ignored).

| # | Command | Exit | Warnings | Errors | Result |
|---|---|---|---|---|---|
| MV-1 | `& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0` (fresh) | 0 | 0 | 0 | `robocar: DBG_ENABLE=ON DBG_BACKEND=UART0 TEL_BACKEND_WIFI=OFF APP_TEST=OFF`; `mtk3: ... pinned at 15ed232..., clean`; `robocar: src/app/main.c not present (Batch B); firmware target skipped, app_program compile-checked` |
| MV-1 | `--build build/uart0 --target mtk3` | 0 | 0 | 0 | |
| MV-1 | `--build build/uart0 --target config_check` | 0 | 0 | 0 | |
| MV-1 | `--build build/uart0 --target app_program_check` | 0 | 0 | 0 | |
| MV-1 | `--build build/uart0` (ALL) | 0 | 0 | 0 | `check_rules: PASS (38 files scanned)` |
| MV-1 | same five steps for `build/usb -DDBG_BACKEND=USB` (fresh) | 0 | 0 | 0 | `DBG_BACKEND=USB`, same status lines, `check_rules: PASS (38 files scanned)` |
| MV-1 | `build/usb/compile_commands.json`, `app_main.c` | | | | `-DAPP_TEST=0 -DDBG_BACKEND_USB=1 -DDBG_ENABLE=1 -D_PICO_RP2040_ -I.../config/mtk3 -I.../external/mtk3_bsp/include -I<root> ... -Wall -Wextra -Werror`; no `pico-sdk/src` path |
| MV-2 | `--build build/usb --target check_rules_selftest` | 0 | 0 | 0 | `check_rules_selftest: PASS` |
| MV-2 | `& $CM -P cmake/check_rules.cmake` | 0 | | | `check_rules: PASS (38 files scanned)` |
| MV-2 | `& $CM -P cmake/check_rules_selftest.cmake` | 0 | | | PASS |
| MV-2 | `& $CM -DRULES_ROOT=<root>/tests/rules_fixtures -P cmake/check_rules.cmake` | 1 | | | `check_rules: 13 violation(s)`: subsystem1/r3_app_program.c:2 R3b; subsystem2/r1_drv_subsystem_hdr.c:3 R1; subsystem2/r1_mixed.c:3 R1; subsystem2/r2_blocking.c:2, :3 R2; subsystem2/r2_pio_blocking.c:2, :3 R2; subsystem2/r4_libbsp.c:2 R4; subsystem3/r3_stdio.c:2 R3a, :4 R3b; subsystem4/r5_core1.c:2 R5; r3_root_file.c:2 R3a; tools/r2_unlisted_dir.c:2 R2. No `ProjectTemplate/`, `docs/`, `src/app/main.c` line |
| MV-3 | temp `app_program/subsystem6/sources.cmake`, configure | 1 | | | `robocar: app_program/subsystem6/sources.cmake is not registered.` |
| MV-3 | temp `app_program/subsystem2/extra/sources.cmake`, configure | 1 | | | `... app_program/subsystem2/extra/sources.cmake is not registered.` |
| MV-3 | temp `src/newmod/sources.cmake`, configure | 1 | | | `... src/newmod/sources.cmake is not registered.` |
| MV-4 | temp `app_program/subsystem2/x.c` `tm_printf("x");` | 1 | | | `x.c:1: [R3b]` |
| MV-4 | same file `sleep_ms(1);` | 1 | | | `x.c:1: [R2]` |
| MV-4 | same file `pio_sm_get_blocking(p, 0);` | 1 | | | `x.c:1: [R2]` |
| MV-4 | temp `app_program/subsystem5/drv_y.c` `#include "subsystem5.h"` + `#include "hardware/gpio.h"` | 1 | | | `drv_y.c:2: [R1]` |
| MV-4 | temp `src/foo/x.c` `sleep_ms(1);` | 1 | | | `src/foo/x.c:1: [R2]` |
| MV-3/4 | temp files and dirs deleted; reconfigure + rebuild `build/usb` | 0 | 0 | 0 | `src/` absent again; no extra dirs under `app_program/` |
| MV-5 | `& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release -DDBG_BACKEND=USB` | 0 | | | `DBG_ENABLE=OFF ... APP_TEST=OFF` |
| MV-5 | same in fresh `build/release_test` + `-DAPP_TEST=ON` | 1 | | | `APP_TEST=ON needs DBG_ENABLE=ON: test results are printed through the dbg_ ring` |
| MV-5 | `& $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON` | 0 | | | `DBG_ENABLE=ON ... APP_TEST=ON` |
| MV-5 | `& $CM --build build/test` | 1 | 0 | 5 | all 5 `error:` lines are `test_report.h:35:2: error: #error "APP_TEST=ON needs the dbg_ ring API: TODO(phase 1 batch B, buddy 2)"` (expected Batch B deferral). Dirs deleted |
| MV-6 | temp `app_program/subsystem5/drv_probe.c` (`hardware/gpio.h`) in `ROBOCAR_SOURCES`; reconfigure + build | 0 | 0 | 0 | `app_program_check_sdk` built; compile command has `hardware_gpio/include` and `-Werror`, no `mtk3_bsp/include` |
| MV-6 | unused local in the probe | 1 | | 1 | `drv_probe.c:7:9: error: unused variable 'unused_probe' [-Werror=unused-variable]` |
| MV-6 | unused local in `subsystem5.c` | 1 | | 1 | `subsystem5.c:14:13: error: unused variable 'unused_probe' [-Werror=unused-variable]` |
| MV-6 | revert, reconfigure, rebuild | 0 | 0 | 0 | file hashes of `subsystem5.c` and `subsystem5/sources.cmake` identical to before; probe deleted |
| MV-7 | `git -C external/mtk3_bsp status --porcelain` | 0 | | | empty |
| MV-7 | `git submodule status` | 0 | | | ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |
| MV-7 | `git status --porcelain -- ProjectTemplate design.md specs.md "INF 2004 Embedded Systems.md" README.md .gitignore` | 0 | | | empty (README.md/.gitignore Step 1 edits are already in commit 67f9b00, unchanged since) |
| MV-7 | `git diff --cached -M --name-status` | 0 | | | exactly five `R100` fixture renames |
| MV-7 | `git log -1 --format=%h` | 0 | | | `67f9b00` (no commit) |
| MV-7 | tabs in `app_program/` | | | | 0 |
| MV-7 | Student IDs in TEAM.md / headers, `Select-String -SimpleMatch` in the course doc | | | | all five found |

## Observations

- `check_rules` file count is 38 = 14 before + 24 new (18 C/H + 6 `sources.cmake`); the plan
  estimated 25.
- Linking `hardware_*` INTERFACE libraries into `app_program_check_sdk` also compiles their SDK
  sources (`gpio.c`, `irq.c`, `irq_handler_chain.S`, ...) inside that object library. They get no
  `-Werror` (per-source flags) and produced 0 warnings. No crt0/stdio sources are pulled in.
- The `app_program/` files were written as full rewrites of the template files rather than
  `Copy-Item` + edit: no template bytes survive in them, so the encoding risk does not apply.
  `TEAM.md` was copied with `Copy-Item`, then rewritten in full with the edit tool (kept
  template paragraphs re-typed verbatim). `testing/` and `videos/` were copied with
  `Copy-Item` and edited with .NET UTF-8 (no BOM) reads/writes, never
  `Get-Content | Set-Content`; LF kept; em dash bytes `e2 80 94` checked.
