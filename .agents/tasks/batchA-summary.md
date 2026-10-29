# Batch A summary (Phase 1: build system + config)

Iteration 2: this pass fixes every finding in `batchA-review.json` / `batchA-review.md`.
Nothing is committed. `git submodule add` staged `.gitmodules` and `external/mtk3_bsp`; every other file is untracked or modified.

## Review findings and fixes

| # | Finding | Fix |
|---|---|---|
| 1 (blocking) | Layout dropped the owner's `src/` prefix | Modules now live under `src/<module>/`, following the owner's `src/rtos/port/port_kernel_internal.c`. `config/`, `cmake/`, `tests/`, `docs/` and `external/` stay at the top level, as approved (`config/mtk3/`). Updated in `CMakeLists.txt` (`ROBOCAR_SRC_DIR`, registration from `src/<dir>/sources.cmake`), the `cmake/check_rules.cmake` allow-lists (`src/debug/dbg_drain.c`, `src/rtos/port/port_boot.c`, `src/rtos/port/port_fault.c`, `src/app/main.c`, `RULES_PORT_DIR_REGEX ^src/rtos/port/`), `ARCHITECTURE.md` (new "Source layout" section) and the `cmake/mtk3.cmake` comment. Rule fixtures moved to `tests/rules_fixtures/src/...`. |
| 2 | Rule checker failed open | `_rules_collect()` scans the whole tree for C/C++/asm and for CMake files (`CMakeLists.txt`, `*.cmake`). It skips only the top-level `external`, `build*`, `tests`, `docs`, `.git`, `.agents` and `.vscode`, and never descends into them. New fixtures: `tools/r2_unlisted_dir.c` (unlisted dir, must fire), `r3_root_file.c` (root file, must fire) and `docs/excluded.c` (excluded, must not fire). |
| 3 | Unlisted `sources.cmake` silently ignored | Configure stops with FATAL_ERROR on any `*/sources.cmake` (top level) or `src/**/sources.cmake` that is not `src/<module>/sources.cmake` for a module in `ROBOCAR_MODULE_DIRS`. |
| 4 | `DBG_ENABLE` was ON for release builds | The default now comes from `CMAKE_BUILD_TYPE`: OFF for `Release`/`MinSizeRel`, ON otherwise. It is evaluated after `pico_sdk_init()`, which sets an empty build type to Release. |
| 5 | `DBG_DEFAULT` had no effect on an existing build dir | Documented in `docs/DEBUG.md` and in the `CMakeLists.txt` comment: cache defaults are seeded only on the first configure, so use a fresh build dir or set `-DDBG_BACKEND` / `-DDBG_ENABLE` directly. Also added a Release configure example. The logic itself is unchanged. |
| 6 | Init task stack has no +512 B | Recorded as a known exception in `ARCHITECTURE.md` and `config/priorities.h`. I confirmed `INITTASK_STKSZ (1*1024)` with no `#ifndef` in vendored `include/sys/inittask.h`. `usermain` must stay stack-light, and the init task goes into the Batch B high-water report. |
| 7 | D2 read as settled | `ARCHITECTURE.md` D2 now reads "PROPOSED, TODO(owner-to-confirm)". `config/timing.h` has a `TODO(owner-to-confirm, buddy 1)`. Values are unchanged. |

## Files created / changed (cumulative)

| File | Notes |
|---|---|
| `.gitattributes` | `* text=auto eol=lf`, binaries marked |
| `.gitignore` | `build/`, `build-*/`, `config/secrets.h`, firmware outputs |
| `.gitmodules`, `external/mtk3_bsp` | Submodule on branch pico_rp2040, pinned to 15ed232c08f2d89e54e514db79de150e224307e3 (staged) |
| `CMakeLists.txt` | PICO_BOARD pico_w, gnu11, options (DBG_ENABLE default by build type, DBG_BACKEND validated, TEL_BACKEND_WIFI), check_rules, config_check, src/ module registration with an unlisted-file guard, robocar (skipped while there are no sources), tests hook |
| `pico_sdk_import.cmake` | Verbatim SDK copy |
| `cmake/mtk3.cmake` | `mtk3` STATIC (43 explicit files), `mtk3_headers` INTERFACE, pin/dirty guard |
| `cmake/warnings.cmake` | `robocar_apply_warnings()`: per-source `-Wall -Wextra -Werror`, our files only |
| `cmake/memmap_mtk3.ld` | SDK memmap_default.ld copy plus `.data_vector (NOLOAD)` and `.noinit (NOLOAD)`, with ASSERT |
| `cmake/check_rules.cmake` | R1-R5, whole-tree scan; run via `cmake -P` or the `check_rules` target (before robocar) |
| `cmake/check_rules_selftest.cmake` + `tests/rules_fixtures/**` (9 files) | Proves every rule fires, the scan reaches unlisted dirs and root files, and allowed/excluded cases are not reported |
| `cmake/config_check.c` | OBJECT lib `config_check`, includes all config headers, `-Wall -Wextra -Werror` |
| `config/pins.h`, `timing.h`, `priorities.h`, `thresholds.h` | Pure macros, Barr C style, compile-time checks |
| `config/secrets.h.example` | WIFI_SSID / WIFI_PASSWORD / MQTT_BROKER_HOST / MQTT_BROKER_PORT placeholders |
| `config/mtk3/config.h`, `config_func.h`, `config_tm.h`, `config_device.h` | BSP copies; licence headers kept; `PROJECT:` note and edits |
| `config/mtk3/config_check.h` | `_Static_assert`s on kernel object counts, tick, pool fit, bypass speed |
| `docs/SUBMODULE.md`, `docs/DEBUG.md` | Submodule pin and verification; probe wiring, serial port, SWD flash, backend switch, cache-default note |
| `ARCHITECTURE.md` | Source layout, kernel integration, init-stack exception, deltas D1 (approved) and D2 (TODO(owner-to-confirm)), enforced rules |
| `NOTICE.md`, `README.md` (line added) | T-License 2.2 credit to the TRON Forum uT-Kernel 3.0 BSP, TODO(owner-to-confirm) wording, link to `external/mtk3_bsp/docs/TEF000-219-200401.pdf` |

The kernel config edits are unchanged from iteration 1: `CNF_TIMER_PERIOD 10->1`, `CNF_MAX_CYCID 4->8`, `CNF_MAX_MBFID 4->8`, `USE_SDEV_DRV (1)->(0)` (config.h), `USE_DEVICE (1)->(0)` (config_func.h), and `#include "config_check.h"` at the end of config.h. `USE_PTMR` stays `(0)`.

## Assumptions

- The layout is `src/<module>/` for firmware modules. `config/` stays top-level because the owner approved `config/mtk3/`. Batch B files will be `src/rtos/port/port_kernel_internal.c`, `src/rtos/port/port_boot.c`, `src/rtos/port/port_fault.c`, `src/debug/dbg_drain.c` and `src/app/main.c`.
- The include root for our code is still the repo root (`#include "config/pins.h"`). Whether to add `src/` as an include root is left to Batch B.
- Build dirs are `build/uart0` and `build/usb`.
- `.noinit` is zero-filled despite its name (USE_NOINIT 0 semantics).
- Kernel `char` is signed (`-fsigned-char` on mtk3 only).

## Open items

1. Path: resolved. The owner's `src/rtos/port/` layout is adopted throughout.
2. `CNF_MAX_TSKID >= 11` is asserted, although the owner said >= 10. The count is 9 design tasks + init + debug drain = 11.
3. `config/lwipopts.h` is deferred because Phase 1 has no WiFi. `TEL_BACKEND_WIFI=ON` fails at configure with "not implemented in Phase 1".
4. D2 (20 ms telemetry wake and poll) is a proposal awaiting owner confirmation. FR1 rates hold either way.
5. The init task stack is fixed at 1 KB with no +512 B (vendored). It is a known exception, so `usermain` stays stack-light and the init task is included in the Batch B high-water report.
6. `DBG_ENABLE`, `DBG_DEFAULT` and `DBG_BACKEND` defaults apply on the first configure only (CMake cache). This is documented.
7. Provisional values are tagged TODO(owner-to-confirm) + TUNE: `BC_CAPTURE_TIMEOUT_MS 1000` (buddy 3), `DBG_DRAIN_PERIOD_MS 10` (buddy 1), `MOTOR_DUTY_MAX_PCT 80` (buddy 2), `OBS_SERVO_SETTLE_MS 50` (buddy 5), `CAR_WIDTH_MM 120` (buddy 5), IMU I2C addresses 0x19/0x1E (buddy 4), `POWER_GROVE_3V3_BUDGET_MA 300` (buddy 2). Stack base sizes and `KERNEL_POOL_MBF_RESERVE_BYTES 4096` are ASSUMPTION/TUNE.
8. Obstacle timing: settle 50 ms + 3 × 30 ms echo can exceed the 60 ms step. Even 3 × 30 ms alone exceeds it in design.md. TODO(owner-to-confirm, buddy 5).
9. design.md 3.2 cites NFR16 for the IMU watchdog; the headers cite NFR13.
10. `IMU_HAS_GYRO 0` conflicts with the design's "Accel + Gyro". TODO(owner-to-confirm, buddy 4).
11. NOTICE/README attribution wording is TODO(owner-to-confirm).
12. Rule-check limits: the checks are line-based, so they miss calls split across lines or hidden behind macros. `i2c_*_blocking` and `uart_*_blocking` are not in R2 (decision deferred to the IMU batch).
13. In USB builds, routing `tm_snd_dat` boot/fault output to polled UART0 is a Batch B port decision (recommended).
14. Batch B: MBF depths, `MBF_BUFSZ`, the send-N / N+1 timeout / drain check, kernel version self-test, stack painting.
15. `exchdr_tbl` (.data_vector) is never used for dispatch because the SDK owns VTOR. Batch B must route every ISR through the SDK irq API; `tk_def_int` registrations would never fire.

## Verification (iteration 2, exact commands, from the repo root in PowerShell)

Preamble:

```
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

Before the run I deleted `build/`, so everything was built from scratch. Each command's output went to a log, and warnings and errors were counted with `Select-String "warning:"` / `"error:"`.

| Command | Result |
|---|---|
| `& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0` | exit 0; prints `DBG_ENABLE=ON DBG_BACKEND=UART0 TEL_BACKEND_WIFI=OFF`, `mtk3: ... pinned at 15ed232..., clean`, `no module sources yet (Batch A); firmware target skipped` |
| `& $CM --build build/uart0 --target mtk3` | exit 0, 43 objects + `libmtk3.a`, **0 warnings, 0 errors** |
| `& $CM --build build/uart0 --target config_check` | exit 0, **0 warnings** (compile_commands shows `-Wall -Wextra -Werror`) |
| `& $CM --build build/uart0` (ALL) | exit 0, 0 warnings, `check_rules: PASS (14 files scanned)` |
| `& $CM --build build/uart0 --target check_rules_selftest` | exit 0, `check_rules_selftest: PASS` |
| `& $CM -S . -B build/usb @common -DDBG_BACKEND=USB` | exit 0; prints `DBG_BACKEND=USB`, same other messages |
| `& $CM --build build/usb --target mtk3` | exit 0, **0 warnings, 0 errors** |
| `& $CM --build build/usb --target config_check` | exit 0, **0 warnings** |
| `& $CM --build build/usb` (ALL) / `--target check_rules_selftest` | exit 0 / exit 0, PASS |
| `& $CM -P cmake/check_rules.cmake` | exit 0, `check_rules: PASS (14 files scanned)` (whole tree; iteration 1 scanned 12 because the dir list missed `pico_sdk_import.cmake` and so on) |
| `& $CM -P cmake/check_rules_selftest.cmake` | exit 0, `check_rules_selftest: PASS` |
| `& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release` | exit 0, `DBG_ENABLE=OFF` (dir deleted afterwards) |
| `git -C external/mtk3_bsp status --porcelain` | empty, exit 0: no vendored file modified |
| `git submodule status` | ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |

Warnings in OUR code: **0**. mtk3 library warnings: **0**, with the mtk3-only `-Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type` at `-Og`.

Negative tests for the new behaviour. Each used a temporary file that was deleted afterwards, and the dir was reconfigured clean (exit 0):

- `src/newmod/sources.cmake`, then `& $CM build/uart0`: exit 1, `robocar: src/newmod/sources.cmake is not registered. Module sources.cmake ...`
- `motion/sources.cmake` (old top-level layout), then `& $CM build/uart0`: exit 1, `robocar: motion/sources.cmake is not registered ...`
- `src/foo/x.c` with `sleep_ms(1)` and `tools/y.c` with `printf("x")` (neither dir is in any list), then `& $CM -P cmake/check_rules.cmake`: exit 1, `src/foo/x.c:1: [R2] ...`, `tools/y.c:1: [R3a] ...`, `check_rules: 2 violation(s)`
- `src/rtos/sources.cmake` (registered) containing a `message()`: configure exit 0, the message printed, so it was included.

Iteration-1 evidence still holds, because the kernel, linker script and config contents are unchanged: the temporary robocar smoke link for both backends, `.data_vector`/`.noinit` NOLOAD placement, the `libmtk3.a` nm check, and no SDK headers in mtk3 deps.

Waiting for owner approval before Batch B.
