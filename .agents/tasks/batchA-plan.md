# Implementation Plan: Phase 1, Batch A (build system and config only)

Scope: build system, vendored kernel, config headers, rule checks, submodule notes. No module
sources (common/, rtos/, debug/, motion/, ... arrive in Batches B-D). No PID, line math, decoding,
IMU filtering, scanning, bypass logic or WiFi. Work directly in the repo
`c:\Users\Admin\Downloads\Projects\inf-2004-project`. Do NOT commit. Never modify files under
`external/mtk3_bsp/`.

## 0. Facts verified during planning (do not re-derive)

Checked on disk and by trial runs in %TEMP% during planning (temp files deleted):

- Upstream `refs/heads/pico_rp2040` of `https://github.com/tron-forum/mtk3_bsp.git` is
  `15ed232c08f2d89e54e514db79de150e224307e3` (`git ls-remote`, network works).
- BSP `config/` contains exactly `config.h`, `config_func.h`, `config_tm.h`, `config_device.h`.
  - `config.h`: `CNF_TIMER_PERIOD 10`, `CNF_MAX_CYCID 4`, `CNF_MAX_MBFID 4`, `#define USE_SDEV_DRV (1)`,
    `#define USE_PTMR (0)`, `USE_NOINIT (0)`, `USE_IMALLOC (1)`, `CNF_MAX_TSKID 32`, `CNF_MAX_SEMID 16`,
    `CNF_MAX_FLGID 16`, `CNF_MAX_MTXID 4`, `CNF_MAX_TSKPRI 32`; ends with `#include "config_func.h"` and
    `#endif /* __TK_CONFIG__ */`.
  - `USE_DEVICE` lives in **`config_func.h` line 33**: `#define USE_DEVICE		(1)` (not in config.h).
  - Kernel headers include config with angle brackets: `<config.h>` (`include/sys/machine.h:23`,
    `include/sys/knldef.h:24`, `armv6m/cpudef.h:23`, `wusec_armv6m.c:24`, `fastlock.c:20`,
    `fastmlock.c:20`), `<config_tm.h>` (`lib/libtm/libtm.h:19`). So putting `config/mtk3` FIRST on
    the include path overrides them; `config.h` pulls `"config_func.h"` from its own directory (ours).
  - `kernel/sysdepend/cpu/core/armv6m/dispatch.S` includes `<sys/machine.h>` -> `<config.h>`, so
    anything reachable from `config.h` must be assembler-safe (guard C-only code with
    `#if !defined(__ASSEMBLER__)`).
- `Noinit(decl)` = `decl __attribute__((section(".noinit")))` (`include/sys/machine.h:77`), used for
  every kernel control-block table (TCB, SEMCB, ... `knl_tmp_stack`). With `USE_NOINIT 0` the BSP
  `Reset_Handler` zeroes from `__bss_start` and its linker script places `*(.noinit)` INSIDE `.bss`
  (`reset_hdl.c:73-80`, `tkernel_map.ld` `.bss` block). **So `.noinit` must be zero-filled.**
- `exchdr_tbl[N_SYSVEC + N_INTVEC]` is in section `.data_vector` (`armv6m/interrupt.c:29`). The SDK
  `.data` output section has `*(.data*)`, which would swallow `.data_vector` unless a `.data_vector`
  output section appears BEFORE `.data` in the script.
- SDK crt0 zero-fills `__bss_start__ .. __bss_end__` (`pico_standard_link/crt0.S:243-252`).
- Trial link (in %TEMP%, deleted) with the exact linker edits of item 6 produced:
  `.data_vector` NOLOAD at 0x200000c0 (before `.data`), `.noinit` NOLOAD at 0x200003c4 with
  `__bss_start__ == __noinit_start`, `.bss` following, `__bss_end__` after both. Works.
- Kernel source file inventory matches report section 11 exactly (43 files, listed in item 7).
- `include/sys/` holds `inittask.h knldef.h machine.h profile.h queue.h sysdef.h sysdepend/`.
  `INITTASK_STKSZ (1*1024)`, `INITTASK_ITSKPRI (1)` (`include/sys/inittask.h`).
  `DEFAULT_SYS_STKSZ = MIN_SYS_STACK_SIZE = 128` (`include/sys/sysdepend/cpu/core/armv6m/sysdef.h:114,120`).
- No kernel header defines macros with our prefixes (`KERNEL_ TASK_ STACK_ DBG_ TEL_ LF_ BC_ OBS_ IMU_
  SAFETY_ MOTION_ NAV_ US_ MOTOR_ SERVO_ CAR_ PINS_`), so our pure config headers may be included
  into kernel TUs.
- SDK `memmap_default.ld` has no SPDX line; its comment block starts `/* Based on GCC ARM embedded
  samples.`. SDK licence is BSD-3-Clause, "Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd."
  (`pico-sdk/LICENSE.TXT`).
- `pico_set_linker_script(TARGET LDSCRIPT)` also adds a link dependency
  (`pico_standard_link/CMakeLists.txt:37-40`).
- SDK adds no directory-scope compile flags; `-ffunction-sections -fdata-sections` come only via
  the `pico_standard_link` INTERFACE target.
- SDK INTERFACE-library sources are compiled as part of the consuming executable, so target-level
  PRIVATE `-Werror` on `robocar` would hit SDK sources. **Warnings must be applied per source file.**
- `pico_w` board: `PICO_DEFAULT_UART 0`, TX GP0, RX GP1 (`boards/pico_w.h:21-28`); no LED pin.
- `stdio_usb_connected()` exists (`pico_stdio_usb/include/pico/stdio_usb.h:137`).
- Toolchain: `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\{cmake\bin, gcc-arm-none-eabi\bin,
  ninja\ninja.exe, python, git\cmd, pico-sdk-tools, picotool, openocd}`.
  **PATH already contains `C:\Strawberry\c\bin` (another cmake/ninja/gcc).** Always call the SDK
  cmake by full path and pass `CMAKE_MAKE_PROGRAM`.
  **`pico-sdk-tools` is NOT registered** in the CMake package registry; pass
  `-Dpico-sdk-tools_DIR=...` or the SDK tries to build pioasm/elf2uf2 from source with a host compiler.
  `find_package(Python3)` runs at configure (`boot_stage2/CMakeLists.txt:63`); it found
  `C:/msys64/ucrt64/bin/python3.12.exe` in the trial, which works.

### Command preamble (environment does NOT persist between tool calls; prefix every command)

```powershell
$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"; $env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"; $CM = "$R\cmake\bin\cmake.exe"
```

### The two build directories (configure + build)

```powershell
# UART0 backend (Phase 1 default)
& $CM -S . -B build-uart0 -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk" "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools" "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe" "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi" -DDBG_BACKEND=UART0
& $CM --build build-uart0

# USB CDC backend (extra bring-up test only)
& $CM -S . -B build-usb -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk" "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools" "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe" "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi" -DDBG_BACKEND=USB
& $CM --build build-usb
```

Run from the repo root (`cwd` = repo). Use forward slashes inside `-D` values.

## Design decisions made in this plan

1. **Module source registration:** each later module dir gets a `<dir>/sources.cmake` that does
   `list(APPEND ROBOCAR_SOURCES ...)` (SDK-side files) or `list(APPEND ROBOCAR_KERNEL_SOURCES ...)`
   (files that include `<tk/tkernel.h>`). Top-level `include()`s each one that exists. `include()`
   keeps one directory scope, so source-file properties (warnings) work. No globbing.
2. **Header separation enforced by the build, not only by grep:** `mtk3` has PRIVATE include dirs
   and defines only (no usage requirements leak into `robocar`). An INTERFACE target `mtk3_headers`
   (config/mtk3 + BSP include + `_PICO_RP2040_`) is linked ONLY by the OBJECT library
   `robocar_kside` that compiles `ROBOCAR_KERNEL_SOURCES`. SDK-side sources never see kernel include
   paths. `kernel/knlinc` is not exported (Batch B's `port_kernel_internal.c` declares the few
   internal externs itself, as report section 16 says).
3. **`robocar_kside` is an OBJECT library**, so its objects always land on the link line. That
   avoids the static-archive ordering cycle (mtk3 needs `tm_snd_dat`/`usermain`/`knl_lowmem_*` from
   the port; the port calls `tk_*` in mtk3). Within the single `libmtk3.a`, ld rescans the archive.
4. **Warnings:** `cmake/warnings.cmake` sets source-file `COMPILE_OPTIONS -Wall;-Wextra;-Werror` on
   our sources only (both lists). SDK sources compiled into robocar and the `mtk3` library never
   get `-Werror`.
5. **`.noinit` linker section:** a separate `.noinit (NOLOAD)` output section placed immediately
   before `.bss`, with `__bss_start__` moved to its start, so crt0 zero-fills it (required by
   `USE_NOINIT 0`). This satisfies the approved "`.noinit (NOLOAD)` in RAM" wording and the kernel's
   zero-init expectation. `.data_vector (NOLOAD)` goes right after `.ram_vector_table` (before
   `.data`). Verified by the trial link.
6. **config_check.h is included from our `config/mtk3/config.h`** (after `config_func.h`, marked
   `PROJECT:`), so every kernel TU checks it and the Batch A `mtk3` build proves it. It includes
   `../priorities.h` and `../timing.h` for cross-checks (single source of truth). C-only content is
   guarded by `#if !defined(__ASSEMBLER__)` because `dispatch.S` reaches `config.h`.
7. **Config header compile-time checks use `#if ... #error`** (works for C, host C and any TU)
   instead of `_Static_assert`, except `config_check.h` which uses `_Static_assert` as the user asked.
8. **Submodule pin is enforced at configure time:** `cmake/mtk3.cmake` runs `git rev-parse HEAD` in
   the submodule and FATAL_ERRORs on mismatch or on a dirty vendored tree, unless
   `-DMTK3_ALLOW_UNPINNED=ON`. The runtime kernel-version self-test is Batch B.
9. **Placeholders for `[FILL]` values that later batches must compile against** are defined with a
   provisional value and a `TODO(owner-to-confirm, buddy N)` + `TUNE` tag, and all are listed in open
   items. Leaving them undefined would block Batch B (drain period) and Batch C.
10. **tests/ hook:** top-level does `add_subdirectory(tests)` only `if(EXISTS tests/CMakeLists.txt)`.
    Batch D's `tests/CMakeLists.txt` must `return()` when `CMAKE_CROSSCOMPILING` and is configured
    standalone on the host (`cmake -S tests -B build-host -G Ninja`). Documented in a comment.
11. **Rule checker self-test:** fixtures under `tests/rules_fixtures/` (not scanned by the real check,
    never compiled) plus `cmake/check_rules_selftest.cmake`, so the checker is proven to fail.
12. **Extra rule R5 (core 1 unused):** forbid `multicore_launch_core1*` and `pico_multicore`. Cheap
    enforcement of "Core 1 stays unused. Do not enable SMP."
13. **Build type:** always pass `-DCMAKE_BUILD_TYPE=Debug` (DBG_ENABLE ON is the debug config).

## Files created in Batch A (in creation order)

```
.gitattributes
.gitignore
config/secrets.h.example
.gitmodules                       (by git submodule add)
external/mtk3_bsp                 (submodule gitlink, pinned 15ed232)
docs/SUBMODULE.md
config/pins.h
config/timing.h
config/priorities.h
config/thresholds.h
config/mtk3/config.h              (copy + PROJECT: edits)
config/mtk3/config_func.h         (copy + PROJECT: edit)
config/mtk3/config_tm.h           (copy, header note only)
config/mtk3/config_device.h       (copy, header note only)
config/mtk3/config_check.h
pico_sdk_import.cmake             (verbatim copy of SDK external/pico_sdk_import.cmake)
cmake/warnings.cmake
cmake/memmap_mtk3.ld
cmake/mtk3.cmake
cmake/check_rules.cmake
cmake/check_rules_selftest.cmake
tests/rules_fixtures/...          (5-6 tiny fixture files, item 9)
CMakeLists.txt
```

---

# Implementation Plan

- [ ] 1. Repo hygiene files.
      `.gitattributes`: `* text=auto eol=lf`, plus `*.pdf *.png *.jpg *.uf2 *.bin *.elf binary`.
      `.gitignore`: `build/`, `build-*/`, `config/secrets.h`, `*.uf2`, `*.elf`, `*.bin`, `*.hex`,
      `*.map`, `*.dis` (root-level outputs only matter if someone builds in-tree), `.vscode/` is NOT
      ignored (leave it out).
      `config/secrets.h.example`: header guard `SECRETS_H`, comment "Copy to config/secrets.h (git-ignored).
      Phase 1 does not use WiFi.", then `#define WIFI_SSID "your-ssid"`, `#define WIFI_PASSWORD
      "your-password"`, `#define MQTT_BROKER_HOST "192.168.1.10"`, `#define MQTT_BROKER_PORT (1883U)`,
      each `/* TODO(owner-to-confirm, buddy 1) */`.
      Files: .gitattributes, .gitignore, config/secrets.h.example
      Verify: `git check-ignore -v config/secrets.h build-uart0/x` prints a matching rule for both;
      `git check-ignore config/secrets.h.example` prints nothing (exit 1).

- [ ] 2. Add and pin the kernel submodule, then write the submodule notes.
      Commands (repo root):
      `git submodule add -b pico_rp2040 https://github.com/tron-forum/mtk3_bsp.git external/mtk3_bsp`
      `git -C external/mtk3_bsp checkout --detach 15ed232c08f2d89e54e514db79de150e224307e3`
      `git add .gitmodules external/mtk3_bsp`   (stages only; do not commit)
      If the clone fails on network: stop, put these exact commands in the summary, and report blocked.
      Write `docs/SUBMODULE.md` (short): what is vendored (TRON Forum micro T-Kernel 3.0 BSP, branch
      pico_rp2040, T-License 2.2, licence text in `external/mtk3_bsp/docs/TEF000-219-200401.pdf`);
      the pin (`15ed232c08f2d89e54e514db79de150e224307e3`, 2023-05-24 "Update manual"); fresh clone
      `git clone --recurse-submodules https://github.com/leeruochen/inf-2004-project.git`; existing
      clone `git submodule update --init --recursive`; verify with `git submodule status` (line must
      start with a space and show the hash) and `git -C external/mtk3_bsp rev-parse HEAD`; NEVER run
      `git submodule update --remote` (it would move off the pin); vendored files are never edited
      (CMake configure fails if the submodule is dirty or off-pin, override `-DMTK3_ALLOW_UNPINNED=ON`
      for experiments only); bumping the pin needs owner approval, re-running the Batch B kernel
      self-test, and updating the hash in `cmake/mtk3.cmake` and this file. Note the BSP manual PDF
      `external/mtk3_bsp/docs/uTK3bsp_pico_man_jp.pdf` (Japanese).
      Files: .gitmodules, external/mtk3_bsp (gitlink), docs/SUBMODULE.md
      Verify: `git submodule status` shows ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`;
      `git ls-files -s external/mtk3_bsp` shows mode `160000` with that hash;
      `git -C external/mtk3_bsp status --porcelain` prints nothing.

- [ ] 3. Create the four pure config headers in `config/`.
      Rules for all four: no `#include` at all; Barr C style; header guard `<NAME>_H`; file doc
      comment; one brief doc comment per macro; every value parenthesised with `U`/`UL` suffix; units in
      names; `TUNE` tag on every tunable; `/* TODO(owner-to-confirm, buddy N): ... */` on every unknown.
      Buddy map: 1 telemetry/WiFi, 2 motion, 3 line+barcode, 4 IMU, 5 obstacle.
      Include style for later batches: repo root is on the include path, so code writes
      `#include "config/pins.h"`.

      **config/pins.h** (design.md section 2):
      `MOTOR_L_A_PIN (8U)`, `MOTOR_L_B_PIN (9U)`, `MOTOR_R_A_PIN (10U)`, `MOTOR_R_B_PIN (11U)`,
      `MOTOR_L_DIR_INVERT (0U)` TUNE, `MOTOR_R_DIR_INVERT (0U)` TUNE; comment: ROBO PICO truth table
      both inputs LOW = brake, both HIGH = coast.
      `ENC_L_PIN (6U)`, `ENC_R_PIN (7U)` (GPIO IRQ, shared IO_IRQ_BANK0).
      `IR_L_PIN (26U)`, `IR_C_PIN (27U)`, `IR_L_ADC_CH (0U)`, `IR_C_ADC_CH (1U)`.
      `IR_BC_PIN (28U)` digital GPIO IRQ; comment: VBAT-sense jumper (GP28-ADC2) must stay open.
      `IMU_I2C_PORT_NUM (1U)`, `IMU_I2C_SDA_PIN (2U)`, `IMU_I2C_SCL_PIN (3U)`; comment: Grove 2 must stay unused.
      `US_TRIG_PIN (4U)`, `US_ECHO_PIN (5U)` (echo must be 3.3 V-safe, A8).
      `SERVO_PIN (12U)`.
      `DBG_UART0_TX_PIN (0U)`, `DBG_UART0_RX_PIN (1U)`, `DBG_UART0_BAUD_BPS (115200UL)`; comment:
      Grove 1 reserved for the Raspberry Pi Debug Probe (probe TX -> GP1, probe RX -> GP0, common GND), 8N1.
      Reserved, unused in Phase 1: `NEOPIXEL_PIN (18U)` (reserved for a later visible status; not
      implemented), `BTN0_PIN (20U)`, `BTN1_PIN (21U)`, `BUZZER_PIN (22U)`.
      CYW43-owned, NEVER USE: `PIN_CYW43_WL_REG_ON (23U)`, `PIN_CYW43_WL_DATA_IRQ (24U)`,
      `PIN_CYW43_WL_CS (25U)`, `PIN_CYW43_WL_CLK_VSYS_ADC3 (29U)`.
      Spares (comment + `PINS_SPARE_MASK`): GP13-15 (servo 2-4), GP16/GP17 (Grove 4).
      Checks: `#define PIN_BIT(p) (1UL << (p))`; `PINS_ALLOCATED_MASK` = OR of PIN_BIT for all 20
      allocated pins (0,1,2,3,4,5,6,7,8,9,10,11,12,18,20,21,22,26,27,28); `PINS_ALLOCATED_SUM` = the
      same terms with `+`. `#if PINS_ALLOCATED_SUM != PINS_ALLOCATED_MASK` -> `#error "pins.h: two
      pins share a GPIO"` (sum equals OR only if all bits are distinct; `#if` arithmetic is intmax_t
      so no overflow). `PINS_CYW43_MASK` = bits 23,24,25,29; `#if (PINS_ALLOCATED_MASK &
      PINS_CYW43_MASK) != 0UL` -> `#error`. Also `#error` if `IR_L_PIN - 26U != IR_L_ADC_CH` or
      `IR_C_PIN - 26U != IR_C_ADC_CH`, and if `(PINS_ALLOCATED_MASK & PINS_SPARE_MASK) != 0`.

      **config/timing.h** (every design.md 3.2 timing constant; mapping table below):
      `KERNEL_TICK_MS (1U)`; `LF_SAMPLE_PERIOD_MS (5U)` NFR7; `LF_STEER_POST_PERIOD_MS (20U)`;
      `LF_STEER_POST_EVERY_N_SAMPLES (LF_STEER_POST_PERIOD_MS / LF_SAMPLE_PERIOD_MS)` + `#error` if not
      an exact multiple; `MOTION_PERIOD_MS (20U)` FR6; `IMU_SAMPLE_PERIOD_MS (10U)`;
      `SAFETY_IMU_WDOG_MS (50U)` NFR13; `TEL_WAKE_PERIOD_MS (20U)` (design delta: 3.1/3.3 say 200 ms
      periodic / 200 ms receive timeout; telemetry now wakes every 20 ms and polls);
      `TEL_PUBLISH_EVERY_N_WAKES (10U)`, `TEL_HEARTBEAT_EVERY_N_WAKES (50U)`, derived
      `TEL_PUBLISH_PERIOD_MS` and `TEL_HEARTBEAT_PERIOD_MS` with `#error` unless they equal 200 and
      1000 (FR1); `TEL_MBF_RECV_TMO_MS (0U)` (0 = TMO_POL each wake; design delta vs 3.3's 200 ms);
      `OBS_SERVO_STEP_PERIOD_MS (60U)`, `OBS_COARSE_BEARING_COUNT (5U)`, `OBS_COARSE_STEP_DEG (30U)`,
      `OBS_FINE_STEP_DEG (10U)` TUNE (not in design), `OBS_FINE_HALF_SPAN_DEG (30U)` TUNE (not in
      design), `OBS_SERVO_SETTLE_MS (50U)` TUNE + TODO(buddy 5, see open item 5),
      `OBS_ECHO_TIMEOUT_MS (30U)` (SEM_ECHO wait), `OBS_PINGS_PER_BEARING (3U)` NFR15;
      `LF_HYST_SAMPLES (3U)`, derived `LF_HYST_MS (LF_HYST_SAMPLES * LF_SAMPLE_PERIOD_MS)` (=15);
      `LF_T_LOST_MS (100U)`, `LF_T_TURN_MS (500U)`, `LF_T_UTURN_MS (1000U)`, `LF_T_SEARCH_MS (2000U)`;
      `BC_POLL_PERIOD_MS (2U)` TUNE (not in design; design says ISR-woken);
      `BC_CAPTURE_TIMEOUT_MS (1000U)` TUNE + `TODO(owner-to-confirm, buddy 3): provisional`;
      `BC_T_END_WIDEST_MULT (3U)`; `BC_EDGE_RING_LEN (64U)` + `#error` unless power of two;
      `OBS_BYPASS_BUDGET_MS (3000U)` NFR14 target; derived `OBS_AVOID_WORST_CASE_MS
      (OBS_BYPASS_BUDGET_MS + LF_T_SEARCH_MS)` + `#error` if > 5000 (NFR14 hard limit);
      `MOTION_T_HOLD_MS (100U)` + `#error` if < `MOTION_PERIOD_MS`; `SAFETY_T_STILL_MS (500U)`;
      `TEL_KEEPALIVE_MS (5000U)`, `TEL_BACKOFF_MIN_MS (1000U)`, `TEL_BACKOFF_MAX_MS (30000U)` +
      `#error` if min > max; `NAV_MBF_SEND_TMO_MS (5U)`; `DBG_USB_CONNECT_WAIT_MS (3000U)` (USB builds
      only, pre-kernel, the only allowed SDK wait); `DBG_DRAIN_PERIOD_MS (10U)` TUNE +
      `TODO(owner-to-confirm, buddy 1): provisional`. Add `#error` if any period above is 0.

      design.md 3.2 -> header mapping (all must appear):
      | 3.2 parameter | macro(s) | file |
      |---|---|---|
      | LF sample 5 ms | LF_SAMPLE_PERIOD_MS | timing.h |
      | Steering post 20 ms | LF_STEER_POST_PERIOD_MS | timing.h |
      | Motion period 20 ms | MOTION_PERIOD_MS | timing.h |
      | IMU sample 10 ms | IMU_SAMPLE_PERIOD_MS | timing.h |
      | IMU watchdog 50 ms | SAFETY_IMU_WDOG_MS | timing.h |
      | Telemetry publish/heartbeat 200 ms / 1 s | TEL_WAKE_PERIOD_MS, TEL_PUBLISH_EVERY_N_WAKES, TEL_HEARTBEAT_EVERY_N_WAKES | timing.h |
      | Servo step 60 ms, 5 bearings, 30 deg | OBS_SERVO_STEP_PERIOD_MS, OBS_COARSE_BEARING_COUNT, OBS_COARSE_STEP_DEG | timing.h |
      | N hysteresis 3 (15 ms) | LF_HYST_SAMPLES, LF_HYST_MS | timing.h |
      | T_lost 100 ms | LF_T_LOST_MS | timing.h |
      | T_turn 500 ms | LF_T_TURN_MS | timing.h |
      | T_uturn 1.0 s | LF_T_UTURN_MS | timing.h |
      | T_search 2.0 s | LF_T_SEARCH_MS | timing.h |
      | D_cmd 30 cm | BC_D_CMD_CM | thresholds.h |
      | T_end 3 x widest | BC_T_END_WIDEST_MULT | timing.h |
      | D_detect 25 cm | OBS_D_DETECT_CM (+ OBS_DETECT_CONFIRM_COUNT 2) | thresholds.h |
      | Side clearance margin 5 cm | OBS_SIDE_MARGIN_MM | thresholds.h |
      | Bypass budget 3.0 s | OBS_BYPASS_BUDGET_MS | timing.h |
      | T_hold 100 ms | MOTION_T_HOLD_MS | timing.h |
      | a_move 0.05 g | IMU_A_MOVE_G_X1000 | thresholds.h |
      | degree_hump 5 deg | IMU_HUMP_PITCH_DEG | thresholds.h |
      | g_collision 2.5 g | IMU_G_COLLISION_X1000 | thresholds.h |
      | T_still 500 ms | SAFETY_T_STILL_MS | timing.h |
      | T_ka 5 s / backoff 1-30 s | TEL_KEEPALIVE_MS, TEL_BACKOFF_MIN_MS, TEL_BACKOFF_MAX_MS | timing.h |
      Put this table (condensed) in the timing.h file comment.

      **config/priorities.h**: kernel priority = design priority. `TASK_PRI_SAFETY (1U)`,
      `TASK_PRI_VC (2U)`, `TASK_PRI_MOTION (3U)`, `TASK_PRI_LF (4U)`, `TASK_PRI_BC (5U)`,
      `TASK_PRI_OBS_SCAN (6U)`, `TASK_PRI_OBS_AVOID (7U)`, `TASK_PRI_IMU (8U)`, `TASK_PRI_TEL (9U)`,
      `TASK_PRI_DBG_DRAIN (10U)` (debug-only, design delta: nine tasks plus one debug-only task),
      `TASK_PRI_INIT (1U)` (kernel `INITTASK_ITSKPRI`; creates all objects then `tk_slp_tsk(TMO_FEVR)`).
      `TASK_COUNT_APP (9U)`, `TASK_COUNT_TOTAL (11U)` (9 + init + debug drain; counted even when
      DBG_ENABLE is OFF, worst case).
      Stacks: `STACK_IRQ_MARGIN_BYTES (512U)` (tasks run on MSP; ISR frames land on the task stack);
      base sizes `TASK_STACK_BASE_<X>_BYTES`: SAFETY, VC, MOTION, LF, BC, OBS_SCAN, OBS_AVOID = 1024U,
      IMU = 1536U, TEL = 4096U, DBG_DRAIN = 1024U (ASSUMPTION, TUNE); final
      `TASK_STACK_<X>_BYTES (TASK_STACK_BASE_<X>_BYTES + STACK_IRQ_MARGIN_BYTES)`. Comment: do not shrink
      until high-water marks are measured (stack painting, Batch B).
      `KERNEL_SYS_STACK_BYTES (128U)` (DEFAULT_SYS_STKSZ, added by tk_cre_tsk), 
      `KERNEL_INIT_TASK_STACK_BYTES (1024U)` (INITTASK_STKSZ), `KERNEL_POOL_BYTES (96U * 1024U)` (static
      pool, do not shrink yet), `KERNEL_POOL_ALLOC_OVERHEAD_BYTES (16U)` TUNE (per Imalloc block,
      estimate), `KERNEL_POOL_MBF_RESERVE_BYTES (4096U)` TUNE (message buffers come from the same pool;
      exact sizes from MBF_BUFSZ in Batch B).
      `TASK_STACK_TOTAL_BYTES` = sum of the ten task stacks + `TASK_COUNT_TOTAL *
      (KERNEL_SYS_STACK_BYTES + KERNEL_POOL_ALLOC_OVERHEAD_BYTES)` + `KERNEL_INIT_TASK_STACK_BYTES`.
      `#error` if `TASK_STACK_TOTAL_BYTES + KERNEL_POOL_MBF_RESERVE_BYTES > KERNEL_POOL_BYTES`; `#error`
      if any final stack size is not a multiple of 8. (Current total is about 22.6 KB of 96 KB.)

      **config/thresholds.h**: IMU: `IMU_A_MOVE_G_X1000 (50U)`, `IMU_HUMP_PITCH_DEG (5U)`,
      `IMU_G_COLLISION_X1000 (2500U)`, `IMU_ACCEL_FS_G (4U)` TUNE (`#error` if == 2; `#error` if
      `IMU_ACCEL_FS_G * 1000U <= IMU_G_COLLISION_X1000`), `IMU_ODR_HZ (400U)` TUNE, `IMU_HAS_GYRO (0U)`
      TODO(buddy 4, see open item 7), `IMU_MAG_HEADING_ENABLE (0U)`, `IMU_ACCEL_I2C_ADDR (0x19U)` and
      `IMU_MAG_I2C_ADDR (0x1EU)` with `TODO(owner-to-confirm, buddy 4): VERIFY by I2C bus scan`.
      Obstacle: `OBS_D_DETECT_CM (25U)`, `OBS_DETECT_CONFIRM_COUNT (2U)`, `OBS_SIDE_MARGIN_MM (50U)`,
      `CAR_WIDTH_MM (120U)` ASSUMPTION + TODO(buddy 5, measure), derived `OBS_BYPASS_CLEAR_MIN_MM
      (CAR_WIDTH_MM + OBS_SIDE_MARGIN_MM)` (170 today), `US_MIN_RANGE_MM (20U)`.
      Barcode: `BC_D_CMD_CM (30U)`, `BC_REPEAT_IGNORE_CM (10U)` TUNE (not in design).
      Speeds (all TUNE): nominal `MOTION_SPEED_CRUISE_NOMINAL_CMPS (30U)`, `..._SLOW_NOMINAL_CMPS (15U)`,
      `..._BYPASS_NOMINAL_CMPS (20U)`; bring-up `..._CRUISE_BRINGUP_CMPS (20U)`, `..._SLOW_BRINGUP_CMPS
      (10U)`, `..._BYPASS_BRINGUP_CMPS (15U)`; `#ifndef MOTION_BRINGUP_SPEEDS` / `#define
      MOTION_BRINGUP_SPEEDS (1U)`; selected `MOTION_SPEED_CRUISE_CMPS`, `MOTION_SPEED_SLOW_CMPS`,
      `MOTION_SPEED_BYPASS_CMPS` via `#if MOTION_BRINGUP_SPEEDS`. `MOTION_SPEED_MAX_CMPS (50U)` (NFR2
      hard cap), `MOTION_SPEED_BYPASS_MIN_CMPS (15U)`. Checks on BOTH sets: every speed <= MAX,
      bypass >= BYPASS_MIN, slow <= cruise.
      PWM: `MOTOR_PWM_FREQ_HZ (20000UL)`, `MOTOR_DUTY_MAX_PCT (80U)` TUNE + `TODO(owner-to-confirm,
      buddy 2): provisional duty cap` (`#error` if > 100), `SERVO_PWM_FREQ_HZ (50U)`,
      `SERVO_PULSE_MIN_US (1000U)`, `SERVO_PULSE_MAX_US (2000U)` TUNE (`#error` unless min < max <
      1000000/SERVO_PWM_FREQ_HZ).
      Power notes: `POWER_VIN_MIN_MV (3600U)`, `POWER_VIN_MAX_MV (6000U)`,
      `POWER_GROVE_3V3_BUDGET_MA (300U)` with `TODO(owner-to-confirm, buddy 2): verify 300 mA against
      ROBO PICO datasheet`; comment: the power bank supplies motor driver and Pico W.

      Files: config/pins.h, config/timing.h, config/priorities.h, config/thresholds.h
      Verify (preamble first): create `%TEMP%\robocar_cfg_check.c` containing the four
      `#include "config/<x>.h"` lines and `int robocar_cfg_check(void) { return (int)PINS_ALLOCATED_MASK; }`;
      run `arm-none-eabi-gcc -std=gnu11 -Wall -Wextra -Werror -fsyntax-only -I . "$env:TEMP\robocar_cfg_check.c"`
      -> exit 0, no output. If `gcc` (host) is on PATH also run `gcc -std=c11 -Wall -Wextra -Werror
      -fsyntax-only -I . ...` -> exit 0. Negative pin test: copy `config/pins.h` to
      `%TEMP%\neg\config\pins.h` with `SERVO_PIN (12U)` replaced by `SERVO_PIN (8U)`, compile the same
      TU with `-I $env:TEMP\neg` -> must fail with the "two pins share a GPIO" `#error`. Repeat with
      `NEOPIXEL_PIN (25U)` -> must fail with the CYW43 `#error`. Delete all temp files.

- [ ] 4. Copy the kernel config into `config/mtk3/` and add `config_check.h`. Depends on 2 and 3.
      `Copy-Item` the four files from `external/mtk3_bsp/config/` (keep the TRON licence header block
      byte-for-byte). Directly after each licence block add:
      `/* PROJECT: Project copy of BSP config, commit 15ed232 (tron-forum/mtk3_bsp pico_rp2040).
         Changes marked PROJECT:. Rest is BSP default. */`
      Edits (keep tab layout, append `/* PROJECT: was <old> */` on each changed line):
      - config.h: `CNF_TIMER_PERIOD 10` -> `1`; `CNF_MAX_CYCID 4` -> `8`; `CNF_MAX_MBFID 4` -> `8`;
        `USE_SDEV_DRV (1)` -> `(0)`. `USE_PTMR` stays `(0)` (verify, do not edit). After
        `#include "config_func.h"` add `#include "config_check.h"	/* PROJECT: compile-time checks */`.
      - config_func.h: `USE_DEVICE (1)` -> `(0)`.
      - config_tm.h, config_device.h: header note only.
      Before editing, confirm each of the five macro names exists in the copied file (they do per section 0).
      `config/mtk3/config_check.h`: guard `MTK3_CONFIG_CHECK_H`; whole body inside
      `#if !defined(__ASSEMBLER__)`; `#include "../priorities.h"` and `#include "../timing.h"`; then
      `_Static_assert` with a message for each: `CNF_TIMER_PERIOD == 1`, `CNF_TIMER_PERIOD ==
      KERNEL_TICK_MS`, `CNF_MAX_CYCID >= 5` (Motion 20 ms, Line 5 ms, ObsScan 60 ms, IMU 10 ms,
      Telemetry 20 ms), `CNF_MAX_MBFID >= 4` (NAV_IN, MOTION_CMD, OBSTACLE, TELEMETRY), `CNF_MAX_TSKID
      >= 11` and `>= TASK_COUNT_TOTAL` (comment: user asked >= 10; there are 11 tasks incl. init and
      debug drain), `CNF_MAX_FLGID >= 2`, `CNF_MAX_SEMID >= 1`, `CNF_MAX_MTXID >= 1`, `CNF_MAX_TSKPRI >=
      10` and `>= TASK_PRI_DBG_DRAIN`, `USE_PTMR == 0` (PWM IRQ is SDK-owned), `USE_SDEV_DRV == 0`,
      `USE_DEVICE == 0`, `USE_IMALLOC == 1` (port supplies the static pool). Keep the T-License
      header off this file (it is ours); add a one-line project header.
      Files: config/mtk3/config.h, config_func.h, config_tm.h, config_device.h, config_check.h
      Verify: `git diff --no-index --ignore-cr-at-eol external/mtk3_bsp/config/config.h config/mtk3/config.h`
      (and the same for the other three) shows only the header note, the five value changes and the
      config_check include. Full proof is the `mtk3` build in item 8.

- [ ] 5. Copy `pico_sdk_import.cmake` and write `cmake/warnings.cmake`.
      `pico_sdk_import.cmake`: verbatim copy of `<SDK>/external/pico_sdk_import.cmake` to repo root.
      `cmake/warnings.cmake`: `function(robocar_apply_warnings)` taking a list of source paths;
      `set_source_files_properties(${ARGN} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")`.
      Header comment: applies to OUR sources only, per source because SDK INTERFACE sources compile
      inside the robocar target; never used on `mtk3` or SDK. No `-fdiagnostics-plain-output` (GCC 10).
      Files: pico_sdk_import.cmake, cmake/warnings.cmake
      Verify: covered by item 8 configure (no errors) and item 10 smoke (our file gets `-Werror`,
      SDK files do not).

- [ ] 6. Create the project linker script `cmake/memmap_mtk3.ld`.
      Copy `<SDK>/src/rp2_common/pico_standard_link/memmap_default.ld`. Prepend a header comment:
      "PROJECT: copy of pico-sdk 1.5.1 src/rp2_common/pico_standard_link/memmap_default.ld.
      Copyright (c) 2020 Raspberry Pi (Trading) Ltd. SPDX-License-Identifier: BSD-3-Clause.
      Changes marked PROJECT:." Keep the original comment block and everything else unchanged.
      Addition 1, immediately after the `.ram_vector_table (NOLOAD)` block and BEFORE `.data`:
      ```
      /* PROJECT: micro T-Kernel exchdr_tbl (armv6m/interrupt.c) lives in .data_vector. It must be
         matched here, before .data, or the SDK's *(.data*) pattern pulls it into .data. The SDK
         owns VTOR (approach (a)), so this 192 B table is unused and needs no 256 B alignment. */
      .data_vector (NOLOAD) : {
          . = ALIGN(4);
          __mtk3_data_vector_start = .;
          *(.data_vector)
          __mtk3_data_vector_end = .;
      } > RAM
      ```
      Addition 2, immediately before the `.bss` block; move `__bss_start__ = .;` out of `.bss` into it:
      ```
      /* PROJECT: kernel Noinit() objects (control blocks, knl_tmp_stack). Despite the name they MUST
         be zeroed: config USE_NOINIT 0 means the kernel relies on zero-init (BSP reset_hdl.c zeroes
         from __bss_start, and its tkernel_map.ld puts .noinit inside .bss). __bss_start__ is placed
         here so SDK crt0 zero-fills .noinit together with .bss. */
      .noinit (NOLOAD) : {
          . = ALIGN(4);
          __bss_start__ = .;
          __noinit_start = .;
          *(.noinit .noinit.*)
          . = ALIGN(4);
          __noinit_end = .;
      } > RAM
      ```
      and in `.bss` keep `. = ALIGN(4);` but delete its `__bss_start__ = .;` line (comment
      `/* PROJECT: __bss_start__ moved to .noinit */`). Add near the existing ASSERTs:
      `ASSERT(__bss_start__ == __noinit_start, "PROJECT: .noinit must start the zero-filled range")`.
      Files: cmake/memmap_mtk3.ld
      Verify: item 10 smoke link (objdump/nm checks).

- [ ] 7. Create `cmake/mtk3.cmake` (static library `mtk3` and INTERFACE `mtk3_headers`).
      `set(MTK3_ROOT "${PROJECT_SOURCE_DIR}/external/mtk3_bsp")`, `set(MTK3_PINNED_COMMIT
      15ed232c08f2d89e54e514db79de150e224307e3)`.
      Guards (configure time): FATAL_ERROR with the submodule commands from item 2 if
      `${MTK3_ROOT}/kernel/tkernel/task.c` does not exist; `find_package(Git QUIET)`; if found and
      `${MTK3_ROOT}/.git` exists, `execute_process(git -C ${MTK3_ROOT} rev-parse HEAD)` and
      `status --porcelain`; FATAL_ERROR on hash mismatch or non-empty status unless
      `option(MTK3_ALLOW_UNPINNED "..." OFF)` is ON (then WARNING). Without git: WARNING only.
      Explicit source list `MTK3_SOURCES` (43 files, prefix `${MTK3_ROOT}/`):
      ```
      kernel/tkernel/cpuctl.c kernel/tkernel/device.c kernel/tkernel/deviceio.c
      kernel/tkernel/eventflag.c kernel/tkernel/int.c kernel/tkernel/klock.c
      kernel/tkernel/mailbox.c kernel/tkernel/memory.c kernel/tkernel/mempfix.c
      kernel/tkernel/mempool.c kernel/tkernel/messagebuf.c kernel/tkernel/misc_calls.c
      kernel/tkernel/mutex.c kernel/tkernel/objname.c kernel/tkernel/power.c
      kernel/tkernel/rendezvous.c kernel/tkernel/semaphore.c kernel/tkernel/task.c
      kernel/tkernel/task_manage.c kernel/tkernel/task_sync.c kernel/tkernel/timer.c
      kernel/tkernel/time_calls.c kernel/tkernel/tkinit.c kernel/tkernel/wait.c
      kernel/tstdlib/bitop.c kernel/tstdlib/string.c
      kernel/sysinit/sysinit.c
      kernel/inittask/inittask.c
      kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
      kernel/sysdepend/cpu/core/armv6m/interrupt.c kernel/sysdepend/cpu/core/armv6m/dispatch.S
      kernel/sysdepend/pico_rp2040/devinit.c kernel/sysdepend/pico_rp2040/power_save.c
      lib/libtk/fastlock.c lib/libtk/fastmlock.c lib/libtk/kmalloc.c
      lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
      lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
      lib/libtm/libtm.c lib/libtm/libtm_printf.c
      ```
      Comment block listing the EXCLUDED files and why: `kernel/sysdepend/cpu/core/armv6m/reset_hdl.c`,
      `kernel/sysdepend/cpu/rp2040/vector_tbl.c`, `kernel/sysdepend/cpu/rp2040/boot_2nd.c`,
      `kernel/sysdepend/pico_rp2040/cpu_clock.c`, `kernel/sysdepend/pico_rp2040/hw_setting.c` (SDK owns
      startup/clocks/vectors; hw_setting drives GP25 = CYW43 CS), `lib/libtm/sysdepend/pico_rp2040/tm_com.c`
      (port provides tm_com_*), `lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c` (never link; clashes with
      hardware/pwm.h) -- this comment line must end with `# RULE-ALLOW: exclusion-note`,
      `device/**`, `app_program/**`, `kernel/usermain/usermain.c` (we provide usermain).
      Self-guards: `foreach` source -> FATAL_ERROR if not EXISTS; `list(FILTER <copy> INCLUDE REGEX
      "reset_hdl|vector_tbl|boot_2nd|cpu_clock|hw_setting|tm_com|libbsp|/device/|app_program|usermain")`
      must be empty, else FATAL_ERROR.
      Target: `add_library(mtk3 STATIC ${MTK3_SOURCES})`;
      `target_include_directories(mtk3 PRIVATE "${PROJECT_SOURCE_DIR}/config/mtk3" "${MTK3_ROOT}/include" "${MTK3_ROOT}/kernel/knlinc")` (this order);
      `target_compile_definitions(mtk3 PRIVATE _PICO_RP2040_)`;
      `target_compile_options(mtk3 PRIVATE -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections
      $<$<COMPILE_LANGUAGE:C>:-ffreestanding -fsigned-char -Wall -Wextra -Wno-unused-parameter
      -Wno-sign-compare -Wno-cast-function-type>)` -- NO `-Werror`;
      `set_target_properties(mtk3 PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)`;
      `set_source_files_properties("${MTK3_ROOT}/kernel/sysinit/sysinit.c" PROPERTIES COMPILE_DEFINITIONS "main=mtk3_kernel_main")`.
      No `target_link_libraries` on mtk3 (it must never see pico_stdlib or any SDK header). No PUBLIC
      or INTERFACE properties on mtk3.
      `add_library(mtk3_headers INTERFACE)`; `target_include_directories(mtk3_headers INTERFACE
      "${PROJECT_SOURCE_DIR}/config/mtk3" "${MTK3_ROOT}/include")`;
      `target_compile_definitions(mtk3_headers INTERFACE _PICO_RP2040_)`. Comment: link ONLY from the
      kernel-side OBJECT library, never from SDK-side code.
      Header comment in the file: unresolved symbols supplied by the port in Batch B:
      `knl_lowmem_top`, `knl_lowmem_limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init`,
      `tm_snd_dat`, `tm_rcv_dat`, `usermain`; `__aeabi_*` division from SDK `pico_divider`.
      Files: cmake/mtk3.cmake
      Verify: item 8.

- [ ] 8. Create the top-level `CMakeLists.txt`, then configure and build both backends. Depends on 1-7.
      Order inside the file:
      1. `cmake_minimum_required(VERSION 3.13)`; `set(PICO_BOARD pico_w CACHE STRING "Board")`;
         `include(pico_sdk_import.cmake)`; `project(robocar C CXX ASM)`; `set(CMAKE_C_STANDARD 11)`;
         `set(CMAKE_C_STANDARD_REQUIRED ON)`; `set(CMAKE_C_EXTENSIONS ON)`; `set(CMAKE_CXX_STANDARD 17)`;
         `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)`; `pico_sdk_init()`. Comment: RP2040 only, core 1 unused,
         no SMP.
      2. Options: `option(DBG_ENABLE "Debug ring + drain task (ON debug, OFF release)" ON)`;
         `set(DBG_DEFAULT "UART0" CACHE STRING "Default debug backend")`;
         `set(DBG_BACKEND "${DBG_DEFAULT}" CACHE STRING "Debug console backend: UART0 or USB")`;
         `set_property(CACHE DBG_BACKEND PROPERTY STRINGS UART0 USB)`; FATAL_ERROR unless
         `DBG_BACKEND MATCHES "^(UART0|USB)$"`; `option(TEL_BACKEND_WIFI "WiFi/MQTT telemetry" OFF)`;
         if ON -> `message(FATAL_ERROR "TEL_BACKEND_WIFI=ON is not implemented in Phase 1")`. When OFF
         nothing links cyw43/lwIP. `message(STATUS ...)` printing DBG_ENABLE, DBG_BACKEND, TEL_BACKEND_WIFI.
      3. `include(cmake/warnings.cmake)`, `include(cmake/mtk3.cmake)`.
      4. `check_rules` target: `add_custom_target(check_rules ALL COMMAND "${CMAKE_COMMAND}"
         "-DRULES_ROOT=${PROJECT_SOURCE_DIR}" -P "${PROJECT_SOURCE_DIR}/cmake/check_rules.cmake"
         COMMENT "Project rule checks" VERBATIM)` and `add_custom_target(check_rules_selftest COMMAND
         "${CMAKE_COMMAND}" -P "${PROJECT_SOURCE_DIR}/cmake/check_rules_selftest.cmake" VERBATIM)`.
         (The scripts are created in item 9. Configure does not need them; in item 8 build only
         `--target mtk3`. The full ALL build, which runs check_rules, is verified in item 9.)
      5. `robocar_options` INTERFACE target: include dir `${PROJECT_SOURCE_DIR}` (repo root);
         definitions `DBG_ENABLE=1` or `DBG_ENABLE=0`; `DBG_BACKEND_UART0=1` or `DBG_BACKEND_USB=1`.
      6. Source registration: `set(ROBOCAR_MODULE_DIRS common rtos debug motion vc safety line barcode
         obstacle imu telemetry app)`; `set(ROBOCAR_SOURCES)`, `set(ROBOCAR_KERNEL_SOURCES)`; `foreach`
         dir: `if(EXISTS "${PROJECT_SOURCE_DIR}/${dir}/sources.cmake") include(...)`. Comment the
         contract: sources.cmake appends absolute paths (`${CMAKE_CURRENT_LIST_DIR}/x.c`); SDK-side
         files -> ROBOCAR_SOURCES; files including `<tk/...>` -> ROBOCAR_KERNEL_SOURCES.
      7. `if(NOT ROBOCAR_SOURCES)`: `message(STATUS "robocar: no module sources yet (Batch A); firmware
         target skipped")`. `else()`:
         - if ROBOCAR_KERNEL_SOURCES: `add_library(robocar_kside OBJECT ${ROBOCAR_KERNEL_SOURCES})`,
           `target_link_libraries(robocar_kside PRIVATE mtk3_headers robocar_options)`,
           `set_target_properties(... C_STANDARD 11 C_EXTENSIONS ON)`,
           `robocar_apply_warnings(${ROBOCAR_KERNEL_SOURCES})`.
         - `add_executable(robocar ${ROBOCAR_SOURCES})`; `robocar_apply_warnings(${ROBOCAR_SOURCES})`;
           `target_link_libraries(robocar PRIVATE robocar_options mtk3 pico_stdlib hardware_pwm
           hardware_adc hardware_i2c hardware_gpio hardware_irq hardware_uart)` plus `robocar_kside`
           if it exists; `pico_set_linker_script(robocar "${PROJECT_SOURCE_DIR}/cmake/memmap_mtk3.ld")`;
           UART0: `pico_enable_stdio_uart(robocar 1)` + `pico_enable_stdio_usb(robocar 0)`; USB: the
           reverse; `pico_add_extra_outputs(robocar)`; `add_dependencies(robocar check_rules)`.
      8. `if(EXISTS "${PROJECT_SOURCE_DIR}/tests/CMakeLists.txt") add_subdirectory(tests)` with comment:
         host unit tests (Batch D) are configured standalone with a host compiler
         (`cmake -S tests -B build-host -G Ninja`); tests/CMakeLists.txt must `return()` when
         `CMAKE_CROSSCOMPILING`.
      Files: CMakeLists.txt
      Verify (preamble first, repo root):
      - Run both configure commands from section 0. Each prints
        `robocar: no module sources yet (Batch A); firmware target skipped` and finishes with
        `Build files have been written`. Then `& $CM --build build-uart0 --target mtk3` and
        `& $CM --build build-usb --target mtk3` both exit 0.
      - `arm-none-eabi-ar t build-uart0/libmtk3.a` lists exactly 43 members.
      - `arm-none-eabi-nm build-uart0/libmtk3.a`: `T mtk3_kernel_main` present; no defined (`T`) `main`,
        `Reset_Handler`, `vector_tbl`, `knl_startup_hw`, `tm_snd_dat` or any `libbsp`/`pwm_set_` symbol;
        `U usermain`, `U tm_snd_dat`, `U knl_lowmem_top` present (expected, resolved in Batch B).
      - `& "$R\ninja\ninja.exe" -C build-uart0 -t deps | Select-String "pico-sdk"` prints nothing
        (no mtk3 object depends on any SDK header).
      - `build-uart0/compile_commands.json`: every mtk3 entry has `-std=gnu11`, `-D_PICO_RP2040_`, and
        `config/mtk3` before `mtk3_bsp/include`; only the sysinit.c entry has `-Dmain=mtk3_kernel_main`;
        no mtk3 entry has `-Werror`.
      - The mtk3 build log has 0 errors and no `redefined` warnings (proves our config headers do not
        clash with kernel macros); remaining kernel warnings are expected.
      - Negative configures (then delete the dirs): `-DDBG_BACKEND=SPI` into `build-neg1` -> FATAL_ERROR
        naming UART0|USB; `-DTEL_BACKEND_WIFI=ON` into `build-neg2` -> FATAL_ERROR "not implemented in
        Phase 1".

- [ ] 9. Rule checks: `cmake/check_rules.cmake`, `cmake/check_rules_selftest.cmake`, fixtures.
      `check_rules.cmake` (script mode, starts with `cmake_minimum_required(VERSION 3.13)`):
      - Root = `RULES_ROOT` if defined, else `${CMAKE_CURRENT_LIST_DIR}/..`. Scan dirs (skip missing):
        `app common config rtos debug motion vc safety line barcode obstacle imu telemetry`; file types
        `*.c *.h *.S *.s *.cpp *.hpp` via `file(GLOB_RECURSE)`. Never scan `external/`, `build*/`,
        `tests/`, `docs/`. Rule R4/R5 also scan `CMakeLists.txt`, `cmake/*.cmake`, `*/sources.cmake`,
        skipping `cmake/check_rules*.cmake`.
      - Line reading (CMake list pitfalls): `file(READ)`, strip `\r`, replace every `\`, `;`, `[`, `]`
        with `_`, then `string(REPLACE "\n" ";" lines ...)` and `foreach(line IN LISTS lines)` with a
        1-based counter (empty lines are kept, so numbers stay correct).
      - Comment stripping for R1-R3, R5: per-line state machine (`in_block` flag; drop text up to `*/`
        when inside a block; remove complete `/* ... */` with regex `/\*([^*]|\*+[^*/])*\*+/`; if a `/*`
        remains, truncate there and set in_block; then strip `//.*`). Allow markers and R4 use the RAW line.
        CMake regex has no `\b`/`\s`: use `(^|[^A-Za-z0-9_])NAME[ \t]*\(` for word boundaries.
      - R1 (per file): kernel include regex `^[ \t]*#[ \t]*include[ \t]*[<"](tk/|tm/|bsp/|config/mtk3/|sys/(inittask|knldef|machine|profile|queue|sysdef)\.h|sys/sysdepend/)`;
        SDK include regex `^[ \t]*#[ \t]*include[ \t]*[<"](pico/|pico\.h|hardware/|boards/|tusb|lwip/|cyw43)`.
        If a file has both, report the line of the later one: `[R1] file includes both kernel and SDK headers`.
      - R2: `(sleep_(ms|us|until)|busy_wait[A-Za-z0-9_]*|mutex_enter_[A-Za-z0-9_]*|sem_acquire[A-Za-z0-9_]*)`
        call. Allowed only in `app/main.c` on a line containing `RULE-ALLOW: pre-kernel`.
      - R3a stdio: `(printf|vprintf|puts|putchar|putchar_raw|puts_raw|fputs|fputc|fprintf|getchar|getchar_timeout_us|stdio_flush)`
        call (boundary excludes `snprintf`, `tm_printf`). Allowed in `debug/dbg_drain.c`, and in
        `app/main.c` lines with `RULE-ALLOW: pre-kernel`.
        R3b `(tm_printf|tm_putstring|tm_putchar|tm_sprintf)` call. Allowed only in
        `rtos/port/port_boot.c` and `rtos/port/port_fault.c`, or lines with `RULE-ALLOW: boot-console`
        inside `rtos/port/`. Put the allow-list file names in variables at the top of the script.
      - R4: raw line contains `libbsp_pwm` and not `RULE-ALLOW: exclusion-note`.
      - R5: `multicore_launch_core1[A-Za-z0-9_]*` call in sources, or `pico_multicore` in CMake files.
      - Output per violation: `<relpath>:<line>: [Rn] <message>: <trimmed raw line>`. At the end:
        violations -> `message(FATAL_ERROR "check_rules: N violation(s)")`; else
        `message(STATUS "check_rules: PASS (M files scanned)")`.
      `check_rules_selftest.cmake`: `execute_process(COMMAND "${CMAKE_COMMAND}"
      -DRULES_ROOT=${CMAKE_CURRENT_LIST_DIR}/../tests/rules_fixtures -P
      ${CMAKE_CURRENT_LIST_DIR}/check_rules.cmake RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE
      err)`; FATAL_ERROR unless rc != 0, output contains each of `motion/r1_mixed.c:`, `[R1]`,
      `motion/r2_blocking.c:`, `[R2]`, `line/r3_stdio.c:`, `[R3`, `vc/r4_libbsp.c:`, `[R4]`,
      `safety/r5_core1.c:`, `[R5]`, and does NOT contain `app/main.c` or `snprintf`. Else STATUS PASS.
      Fixtures (never compiled; tests/CMakeLists.txt in Batch D must not glob them), each 3-6 lines:
      `tests/rules_fixtures/motion/r1_mixed.c` (`#include <tk/tkernel.h>` + `#include "hardware/pwm.h"`),
      `tests/rules_fixtures/motion/r2_blocking.c` (`sleep_ms(5);` plus the same with a pre-kernel
      marker, which must STILL be flagged outside app/main.c),
      `tests/rules_fixtures/line/r3_stdio.c` (`printf("x");` and `snprintf(b, 4, "x");` -- only printf flagged; also `tm_printf("x");` -> R3),
      `tests/rules_fixtures/vc/r4_libbsp.c` (`extern void libbsp_pwm_init(void);`),
      `tests/rules_fixtures/safety/r5_core1.c` (`multicore_launch_core1(fn);`),
      `tests/rules_fixtures/app/main.c` (`sleep_ms(1); /* RULE-ALLOW: pre-kernel */` and a comment
      `/* never call sleep_ms( here */` -> neither flagged).
      Files: cmake/check_rules.cmake, cmake/check_rules_selftest.cmake, tests/rules_fixtures/** (6 files)
      Verify (preamble, repo root): `& $CM -P cmake/check_rules.cmake` -> exit 0, `check_rules: PASS`;
      `& $CM -P cmake/check_rules_selftest.cmake` -> exit 0, PASS; `& $CM --build build-uart0 --target
      check_rules_selftest` -> exit 0. Then the full builds `& $CM --build build-uart0` and
      `& $CM --build build-usb` (section 0) both exit 0 and print `check_rules: PASS`.

- [ ] 10. Temporary integration smoke of the robocar path (both backends), then remove it.
      Purpose: prove the linker script, per-source warnings, stdio backend switch, kside OBJECT lib,
      header separation and config headers vs SDK/kernel macros, before Batch B relies on them.
      Confirm `app/` and `rtos/` do not exist, then create TEMPORARY files:
      - `app/sources.cmake`: `list(APPEND ROBOCAR_SOURCES "${CMAKE_CURRENT_LIST_DIR}/main.c")`
      - `app/main.c`: includes `"pico/stdlib.h"` and the four `config/*.h`; defines
        `__attribute__((section(".noinit"))) unsigned int smoke_ni[16];`,
        `__attribute__((section(".data_vector"))) unsigned int smoke_dv[48];`, and
        `int main(void) { stdio_init_all(); smoke_dv[1] = MOTOR_L_A_PIN; return (int)(smoke_ni[3] + smoke_dv[1] + TASK_PRI_TEL + LF_SAMPLE_PERIOD_MS + MOTION_SPEED_MAX_CMPS); }` (no printf/sleep, so rules pass).
      - `rtos/sources.cmake`: `list(APPEND ROBOCAR_KERNEL_SOURCES "${CMAKE_CURRENT_LIST_DIR}/port/smoke_kside.c")`
      - `rtos/port/smoke_kside.c`: `#include <tk/tkernel.h>`, the four `config/*.h`; one external
        function `int smoke_kside(void);` + definition returning `(int)TMO_FEVR + (int)TASK_PRI_DBG_DRAIN`
        (no kernel calls, so mtk3 members are not pulled and the link has no unresolved port symbols).
      Re-run both configure + build pairs. Expect `robocar.elf` and `robocar.uf2` in both build dirs.
      Checks (build-uart0, repeat key ones on build-usb):
      - `arm-none-eabi-objdump -h build-uart0/robocar.elf`: `.data_vector` and `.noinit` exist, in RAM
        (0x2000xxxx), without CONTENTS/LOAD flags; `.data_vector` is before `.data`.
      - `arm-none-eabi-nm -n build-uart0/robocar.elf`: `__bss_start__ == __noinit_start`, `smoke_ni`
        between `__noinit_start` and `__noinit_end`, `__bss_end__` above both; `smoke_dv` between
        `__mtk3_data_vector_start/_end` and below `__data_start__`.
      - compile_commands.json: `app/main.c` and `smoke_kside.c` have `-Werror`; SDK sources in robocar
        (e.g. `pico_stdio/stdio.c`) do NOT; `app/main.c` has no `mtk3_bsp` include path;
        `smoke_kside.c` has `config/mtk3` and `mtk3_bsp/include` and no `pico-sdk/src` include path.
      - UART0 build: compile_commands for robocar sources contain `LIB_PICO_STDIO_UART=1` and not
        `LIB_PICO_STDIO_USB=1`; USB build the reverse.
      - `check_rules` ran before robocar (visible in build output).
      Then DELETE `app/` and `rtos/` entirely (only the four temp files), re-run both configures and
      confirm the "firmware target skipped" message again and both builds exit 0.
      Files: none permanent (temporary app/, rtos/ removed)
      Verify: as listed; final `git status --short` shows no `app/` or `rtos/` entries.

- [ ] 11. Final sweep and report.
      Run `& $CM -P cmake/check_rules.cmake` (PASS), both builds again (exit 0),
      `git -C external/mtk3_bsp status --porcelain` (empty), `git submodule status` (pinned hash),
      `git status --short` (only Batch A files + staged .gitmodules/external/mtk3_bsp). Remove
      `build-neg*` dirs and every %TEMP% file created. Do not commit.
      Report to the user: files created, assumptions, the open items below, and wait for approval
      before Batch B.
      Files: none
      Verify: commands above.

---

## Open items / assumptions to report (carry into the Batch A summary)

1. Path: the user wrote `src/rtos/port/port_kernel_internal.c`; the approved tree has no `src/`, so
   Batch B uses `rtos/port/port_kernel_internal.c`. TODO(owner-to-confirm).
2. `CNF_MAX_TSKID >= 11` asserted (user said >= 10; there are 11 tasks: 9 + init + debug drain).
3. `config/lwipopts.h` not created: Phase 1 has no WiFi; `TEL_BACKEND_WIFI=ON` fails at configure.
4. Provisional values, all `TODO(owner-to-confirm)` + `TUNE`: `BC_CAPTURE_TIMEOUT_MS 1000` (buddy 3),
   `DBG_DRAIN_PERIOD_MS 10` (buddy 1), `MOTOR_DUTY_MAX_PCT 80` (buddy 2), `OBS_SERVO_SETTLE_MS 50`
   (buddy 5), `CAR_WIDTH_MM 120` (buddy 5, measure), IMU I2C addresses 0x19/0x1E (buddy 4, bus scan),
   `TASK_STACK_BASE_DBG_DRAIN_BYTES 1024`, `KERNEL_POOL_MBF_RESERVE_BYTES 4096`,
   `POWER_GROVE_3V3_BUDGET_MA 300` (verify against the ROBO PICO datasheet). Values not in design.md
   but given in the brief: `OBS_FINE_STEP_DEG 10`, `OBS_FINE_HALF_SPAN_DEG 30`, `BC_POLL_PERIOD_MS 2`,
   `BC_REPEAT_IGNORE_CM 10`, bring-up speeds.
5. Obstacle timing conflict: one servo step per 60 ms, but settle 50 ms + 3 pings x up to 30 ms echo
   timeout = up to 140 ms per bearing. No compile-time check added. TODO(owner-to-confirm, buddy 5).
6. design.md 3.2 cites NFR16 for the IMU watchdog; the requirement is NFR13. Headers cite NFR13.
7. `IMU_HAS_GYRO 0` (per brief, LSM303-style accel+mag at 0x19/0x1E) conflicts with design.md
   ("IMU (Accel + Gyro)", gyro bias re-check at standstill). TODO(owner-to-confirm, buddy 4).
8. Telemetry design delta (20 ms wake, poll receive, publish every 10th wake, heartbeat every 50th) vs
   design.md 3.1/3.3 (200 ms periodic, 200 ms receive timeout): to be recorded in ARCHITECTURE.md.
9. Deferred to a later batch (not in the user's Batch A output list): `NOTICE.md`, README licence line
   (T-License 2.2 wording TODO(owner-to-confirm), link to `external/mtk3_bsp/docs/`),
   `docs/DEBUG.md`, `ARCHITECTURE.md` (debug-drain delta vs design.md section 3, header-separation
   rule, rule-check list). Suggest Batch B.
10. Rule-check allow-list fixes these Batch B file names: `debug/dbg_drain.c` (only stdio caller),
    `rtos/port/port_boot.c` and `rtos/port/port_fault.c` (only tm_* callers), `app/main.c` (pre-kernel
    marker). Batch B must use them or update the variables in `cmake/check_rules.cmake`.
11. Rule-check limits: line-based; misses calls split across lines or hidden behind macros; string
    literals containing `//` or `/*` may hide code on that line. `<sys/queue.h>` from newlib would be
    classed as a kernel header (unlikely on the SDK side).
12. `.noinit` is zero-filled despite its name (USE_NOINIT 0 semantics); documented in the linker script.
13. `mtk3` is built with `-fsigned-char`; SDK-side code uses the ARM default unsigned `char`. The port
    API must not pass plain `char` across the boundary (use the kernel's B/UB or stdint types).
14. `i2c_*_blocking` / `i2c_*_timeout_us` (spin-waits) are not forbidden by R2; the IMU batch must
    decide. TODO(owner-to-confirm, buddy 4).
15. USB backend: whether `tm_snd_dat` fault/boot output still goes to polled UART0 (GP0/GP1) in USB
    builds is a Batch B port decision (recommend yes: UART0 is always wired to the probe).
16. MBF depths (8/4/2/32), `MBF_BUFSZ` macro and the send-N / N+1 timeout / drain startup check are
    Batch B (`rtos/`), not config/.
17. The backend switch is only exercised when `robocar` exists; Batch A proves it with the temporary
    smoke in item 10, then removes it.
18. Configure picks Python from `C:/msys64/ucrt64/bin/python3.12.exe` (found by FindPython3); this
    works. The installer's bundled Python is also on PATH via the preamble.
