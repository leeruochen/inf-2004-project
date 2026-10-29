# Template merge report: ProjectTemplate vs Phase 1 Batch A

Read-only investigation. Nothing in the repo was changed except this report. Line numbers are 1-based. Diffs against the BSP were taken with
`git diff --no-index --ignore-cr-at-eol` (the template's files are LF and the submodule checkout is CRLF, so a raw hash compare reports 269 false "different" files).

## 0. Summary

1. **The template is not the official BSP, and it is not a verified equivalent of any non-negotiable.** `ProjectTemplate/` is a full copy of micro T-Kernel 3.0 BSP `tron-forum/mtk3_bsp` `pico_rp2040` (the copyright headers say "micro T-Kernel 3.00.01"). On top of that copy sits an **SMP (dual-core) fork with USB-CDC console, CYW43 WiFi, lwIP and MQTT**, built with **GNU make under a POSIX shell** (macOS, Linux or WSL), **Pico SDK 2.2.0**, and its **own startup, vector table and linker script** (`-nostartfiles`, no Pico SDK runtime). Defaults are `SMP=1 CONSOLE=usb_cdc WIFI=cyw43` (`build_make/pico_rp2040.mk:20,32,37`). Each of these breaks a constraint. Its `SMP=0` profile still compiles modified kernel sources (for example `kernel/tkernel/time_calls.c` changes `BEGIN_DISABLE_INTERRUPT` to `BEGIN_CRITICAL_SECTION` with no SMP guard), so it is **not** the official single-core kernel.
2. **The parts worth importing are process and layout conventions, not code.** These are `app_program/` (shared `app.h` / `app_main.c` / `test_report.h`, plus one folder per student `subsystem1..5/` with `subsystemN.{c,h}`, `subsystemN_test.c`, `task_*.c` and `drv_<name>.{c,h}`), `TEAM.md` (pins, peripherals, priorities, interfaces, object budget), `testing/` (TEST_LOG.md per level, plus `logs/`), `videos/README.md`, and the `.gitignore` video/OS rules.
3. **The template's example code violates our rules.** Running `cmake -P cmake/check_rules.cmake` now (HEAD 56826c7 plus the untracked `ProjectTemplate/`) **FAILS with 94 violations, all inside `ProjectTemplate/`**: R3b=81, R3a=9, R1=3, R4=1. So **while the folder sits in the repo root, the `check_rules` ALL target fails**. Batch A verification cannot be re-run green until the folder is moved out or excluded.
4. **Recommendation: Option C (hybrid, ours as base).** Keep our CMake, Ninja, SDK 1.5.1, pinned single-core submodule, port layer and rule checker. Adopt the template's `app_program/subsystemN/` layout and naming for team code (Buddy N maps to subsystem N), its two-phase `usermain()` init/start pattern, `test_report.h` (re-targeted from `tm_printf` to the `dbg_` ring), `TEAM.md`, `testing/` and `videos/`. **Do not import** the template kernel copy, `build_make/`, `tools/`, `lib/libwifi`, `lib/liblwip` or `config/*.h`. No `src/` files exist yet (`git ls-files` shows none), so no tracked module file has to move. Only the rule fixtures, CMake registration and docs change.
5. **Template coding-standard tooling: none.** There is no `.clang-format`, `.editorconfig`, cppcheck, lint script, CI (`.github`) or Barr C reference anywhere in the template. Nothing needs wiring next to `check_rules`.

---

## 1. Step 1: Template inventory

### 1.1 Tree, and whether it is a git repo

`c:\Users\Admin\Downloads\Projects\inf-2004-project\ProjectTemplate`: 643 files. **It is not a git repo**: there is no `.git`, `.gitmodules`, `.github`, `.cproject`, `.project`, `.settings`, `CMakeLists.txt` or `*.cmake` anywhere in it (checked with `Get-ChildItem -Recurse -Force -Include ...`). It is untracked in our repo (`git status --short` gives `?? ProjectTemplate/`).

| Top level | Files | Content |
|---|---|---|
| `.gitignore`, `README.md`, `TEAM.md` | 3 | see 1.6 and 1.8 |
| `app_program/` | 22 | team code (listed below) |
| `build_make/` | 76 | `makefile`, `pico_rp2040.mk`, `iote_*.mk`, `nucleo_h723.mk`, `.gitignore`, `tools/.gitkeep`, `mtkernel_3/**/subdir.mk` |
| `config/` | 5 | `config.h`, `config_device.h`, `config_func.h`, `config_tm.h`, `wifi_secrets.example.h` |
| `device/` | 86 | BSP sample drivers adc, i2c, ser, common/drvif, for 7 CPUs |
| `etc/linker/` | 10 | `tkernel_map.ld` for 9 boards (incl. `pico_rp2040/`) |
| `include/` | 124 | `tk/`, `sys/`, `tm/`, `bsp/` (multi-target) plus new `tk/smp.h` |
| `kernel/` | 237 | inittask, knlinc, sysinit, tkernel (46 + 3 SMP files), tstdlib, usermain, sysdepend for 9 boards and 7 CPUs, plus new `cpu/rp2040/smp_*` |
| `lib/` | 60 | libtk, libtm (plus new `sysdepend/pico_rp2040/usb/` TinyUSB console), libbsp, new `libwifi/sysdepend/pico_rp2040/` (cyw43), new `liblwip/sysdepend/pico_rp2040/` (lwIP port, MQTT) |
| `testing/` | 15 | `README.md`, `{subsystem1..5,integration,system}/TEST_LOG.md` + `logs/.gitkeep` |
| `tools/` | 4 | `elf2uf2/{main.cpp,elf.h,boot/uf2.h}` (RPi, BSD-3-Clause), `verify_phase11_usb.py` (port-development leftover, "Phase 11 frames") |
| `videos/` | 1 | `README.md` (unlisted-YouTube index) |

The kernel area (config, device, etc, include, kernel, lib, build_make) is a multi-target BSP. Only about 190 of its files relate to RP2040 or are common.

Every file under `app_program/`:
`app.h`, `app_main.c`, `test_report.h`;
`subsystem1/{subsystem1.c,subsystem1.h,subsystem1_test.c,task_led.c}`;
`subsystem2/{subsystem2.c,subsystem2.h,subsystem2_test.c,task_producer.c}`;
`subsystem3/{subsystem3.c,subsystem3.h,subsystem3_test.c,task_consumer.c}`;
`subsystem4/{subsystem4.c,subsystem4.h,subsystem4_test.c,task_monitor.c}`;
`subsystem5/{subsystem5.c,subsystem5.h,subsystem5_test.c}`.

### 1.2 Build system and toolchain assumptions

- **GNU make only.** `build_make/makefile:14` has `TARGET := _PICO_RP2040_` and includes `pico_rp2040.mk` (`makefile:52-54`). There is no CMake. The BSP at 15ed232 has **no** `pico_rp2040.mk` and its makefile targets `_NUCLEO_H723_`. The template adds `pico_rp2040.mk` (471 lines) and the rp2040/armv6m `subdir.mk` files. This confirms the integration report's statement that the BSP has no rp2040 make target.
- **Host: POSIX shell.** It uses `command -v`, `find | xargs -r rm`, `cat`, `rm -rf` and `mkdir -p` (`pico_rp2040.mk` "Fail early" block, "Stale-object guard", `makefile:16,63`). README says Windows means WSL Ubuntu (`README.md:9-11`). **Native Windows with Ninja is not supported.**
- **Compiler:** `arm-none-eabi-gcc` from PATH (`pico_rp2040.mk` `GCC :=`). README suggests Arm GNU Toolchain 15.3.rel1 (`README.md` 1.4). Version is not pinned.
- **SDK: Pico SDK 2.2.0** (`git clone --branch 2.2.0`, README 1.2; "2.3.0 has also been tested"). It is used **only as a source of headers plus the TinyUSB and cyw43 sources**. The comment says "the port does not link the pico-sdk runtime" (`pico_rp2040.mk`, "Console selection" block). `CONSOLE=uart WIFI=none` needs no SDK at all. App code therefore **cannot use `hardware_pwm`, `hardware_i2c`, `hardware_adc` or `pico_stdlib`**. The template gives app code only BSP `libbsp` (`gpio_set_pin`/`gpio_set_val`, `app.h:14`) and the T-Kernel device drivers.
- **Flags:** `-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -O2 -g3 -MMD -MP -mfloat-abi=soft`, `LFLAGS ... -nostartfiles`, `LNKFILE := ../etc/linker/pico_rp2040/tkernel_map.ld`. **No `-Wall`, `-Wextra` or `-Werror`** on rp2040 code (the only `-Wall` hits are in rx231 subdir.mk files).
- **Defines from make options:** `-DCNF_SMP=1` (SMP=1), `-DAPP_TEST=$(TEST)`, `-DTM_CONSOLE_UART=$(CRASH_REPORT)`, `-DBOARD_UART0_FREE=1`, `-DTM_CONSOLE_USB_CDC=1 -DNDEBUG`, `-DTM_WIFI_CYW43=1 ... -DRASPBERRYPI_PICO_W=1 -DCYW43_DEFAULT_PIN_WL_*`, `-DTM_NET_LWIP=1`, `-DTM_MQTT=1`.
- **Options** (README section 3 table): `SMP 0|1 (1)`, `CONSOLE usb_cdc|uart (usb_cdc)`, `WIFI cyw43|none (cyw43)`, `TEST 0|1 (0)`, `CRASH_REPORT 0|1 (0)`, plus `NET=lwip` and `MQTT=1`. The image name encodes the profile, for example `mtk3pico_smp1_usb_cdc_wifi.uf2`. A `.build_profile` file discards stale objects.
- **Board:** Pico W (`RASPBERRYPI_PICO_W=1`; `include/sys/sysdepend/pico_rp2040/sysdef.h` new block "Board pin assignment"). There is no `PICO_BOARD` because there is no SDK CMake.
- **UF2:** it builds the host tool `tools/elf2uf2` with `c++` (`pico_rp2040.mk` `$(E2U)` rule).

### 1.3 Kernel: what the template ships, and how it compares to `external/mtk3_bsp` at 15ed232

Submodule state: `git -C external/mtk3_bsp rev-parse HEAD` = `15ed232c08f2d89e54e514db79de150e224307e3` (tag `v1.00.00.B5-pico_rp2040`, tip of `origin/pico_rp2040`). `status --porcelain` is empty.

Comparison over the kernel area (`config device etc include kernel lib build_make`), ignoring EOL:

| Bucket | Count | Notes |
|---|---|---|
| Identical to BSP | about 224 kernel-area files (the raw hash compare showed 224 identical plus 269 differing only by CRLF; the EOL-insensitive diff shows changes in just the files listed below) | includes `vector_tbl.c`, `boot_2nd.c`, `cpu_clock.c`, `include/sys/inittask.h` (INITTASK_STKSZ 1 KB), `kernel/usermain/usermain.c`, `lib/libbsp/**` (incl. `libbsp_pwm.c`) |
| Modified | config 3, device 7, etc 1, include 4, kernel 23, lib 3, build_make 6 | see the list below |
| Only in template | 29 source files + about 13 new rp2040 `subdir.mk` + `pico_rp2040.mk` | SMP core, USB console, WiFi, lwIP, MQTT |
| Only in BSP | `.cproject`, `.project`, `.settings/`, `.gitignore`, `README.md`, `ucode.png`, `docs/*.pdf` (incl. the **T-License text `TEF000-219-200401.pdf`**), `app_program/app_main.c` (replaced) | the template **does not ship the licence PDF** |

Modified kernel files (lines changed): `kernel/knlinc/kernel.h` 17; `kernel/sysdepend/cpu/core/armv6m/{cpu_cntl.c 69, cpu_status.h 87, dispatch.S 90, exc_hdr.c 77, reset_hdl.c 14}`; `kernel/sysdepend/pico_rp2040/{devinit.c 40, hw_setting.c 141, power_save.c 4}`; `kernel/sysinit/sysinit.c 15`; `kernel/tkernel/{cpuctl.c 54, klock.c 17, misc_calls.c 11, task.c 176, task.h 53, task_manage.c 94, task_sync.c 15, time_calls.c 267, time_calls.h 22, timer.c 66, timer.h 21, wait.c 76}`; `include/sys/sysdepend/cpu/rp2040/{profile.h 1, sysdef.h 45}`, `include/sys/sysdepend/pico_rp2040/sysdef.h 51`, `include/tk/syscall.h 27`; `lib/libtk/fastlock.c 12`, `lib/libtm/libtm_printf.c 50`, `lib/libtm/sysdepend/pico_rp2040/tm_com.c 66`; `device/{adc,i2c,ser}/...rp2040...` and `device/ser/ser.{c,h}`; `etc/linker/pico_rp2040/tkernel_map.ld 6`.

New files only in the template: `include/tk/smp.h`, `kernel/knlinc/{smp.h,smp_lock.h}`, `kernel/sysdepend/cpu/rp2040/{smp_atomic.c,smp_rp2040.c,smp_rp2040.h}`, `kernel/tkernel/{ready_queue_smp.c,smp_dispatch.c,smp_lock.c}`, `lib/libtm/sysdepend/pico_rp2040/usb/{usb_console.c,usb_descriptors.c,usb_sdk_compat.c,usb_tinyusb_glue.c,tusb_config.h,pico/config_autogen.h,pico/version.h}`, `lib/libwifi/sysdepend/pico_rp2040/{cyw43_utk.c,cyw43_utk.h,cyw43_utk_compat.c,cyw43_utk_kernel.c,cyw43_configport.h,cyw43_bus_pio_spi.pio.h}`, `lib/liblwip/sysdepend/pico_rp2040/{sys_arch.c,mqtt_utk.c,mqtt_utk.h,lwipopts.h,include/arch/{cc.h,sys_arch.h}}`, `config/wifi_secrets.example.h`.

Key diffs:
- `config/config.h`: `CNF_MAX_{MBX,MTX,MBF,MPL,MPF,CYC,ALM}ID` go from 4 to 12. `CNF_TMP_STACK_SIZE` is 1024 under `CNF_SMP`, otherwise 256. `CNF_TIMER_PERIOD` stays **10**. `USE_SDEV_DRV (1)`.
- `config/config_device.h`: `DEVCNF_USE_IIC 1 -> 0`.
- `config/config_tm.h`: adds `TM_CONSOLE_USB_CDC (0)` and `TM_CONSOLE_UART (1)` fallbacks.
- `etc/linker/pico_rp2040/tkernel_map.ld`: adds `. = ALIGN(4);` before `__data_org`. This fixes a "HardFaulted every boot of the stock port" bug in the BSP's own linker script (`include/sys/sysdepend/pico_rp2040/sysdef.h` new comment). **It does not affect us**: we use the SDK linker script `cmake/memmap_mtk3.ld`.
- `kernel/sysdepend/pico_rp2040/hw_setting.c`: drops the BSP's **GP25 LED output**, because GP25 is CYW43 CS on Pico W. This independently confirms our exclusion reason in `cmake/mtk3.cmake:73`. UART0 pins are now conditional (`#if !BOARD_UART0_FREE`). It adds PIO0/PIO1/DMA reset release for WiFi and the `BOARD_DIAG_GPIO_SCAN` diagnostic.
- `kernel/sysdepend/pico_rp2040/power_save.c`: `low_pow()` gains `wfi`. The BSP version is empty, so idle busy-spins. We compile the BSP version (`cmake/mtk3.cmake:112`).
- `kernel/sysdepend/pico_rp2040/devinit.c`: launches core 1 (`knl_smp_launch_core1`), starts the USB console task and the cyw43 task.
- `lib/libtm/sysdepend/pico_rp2040/tm_com.c`: bounded UART TX spin (`UART_TX_SPIN_LIMIT`) and a USB sink. We replace `tm_com.c` in the port anyway (`cmake/mtk3.cmake:74`).
- `kernel/tkernel/time_calls.c`: changes `BEGIN_DISABLE_INTERRUPT` to `BEGIN_CRITICAL_SECTION` **unconditionally**, and adds `#if TK_SUPPORT_SMP` per-core handler queues. `timer.c` adds SMP diagnostics.
- `include/sys/sysdepend/cpu/rp2040/sysdef.h:80-86`: `TK_SUPPORT_SMP` and `TK_MAX_CORE` come from `CNF_SMP`.

Version and provenance: the kernel file headers say "micro T-Kernel 3.00.01", the same as the BSP. No `TKERNEL_PRVER`/`SPVER` define was found by grep (UNVERIFIED where the version macros live). The README footer says "This dual-core RP2040 port is derived from the official μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`)". The diff (no deletions, roughly 224 identical files) is **consistent with** a base of 15ed232 (the branch tip). The exact base commit is UNVERIFIED. The template references `docs/RELEASE.md` "safety rule 6" (`pico_rp2040.mk`, `cyw43_utk.c:14`) and `docs/evidence/phase0/` (`sysdef.h`), but **neither exists in the template**.

**Kind: an SMP fork of pico_rp2040** with a single-core `SMP=0` profile. It is not the official single-core BSP, and not a generic multi-target distribution (although the non-rp2040 board trees are still present).

**Startup and linker ownership: the kernel owns them.** It uses BSP `boot_2nd.c`, `vector_tbl.c`, `reset_hdl.c` (modified), `cpu_clock.c`, the template's `tkernel_map.ld`, and `-nostartfiles`. There is no SDK crt0, runtime or `memmap_default.ld`. This is the opposite of our approach (a).

### 1.4 Coding-standard tooling and conventions

- Tooling: **none.** There is no `.clang-format`, `.editorconfig`, cppcheck config, lint script, CI or Barr C reference. A grep for `Barr|clang-format|cppcheck|MISRA` found only unrelated hits.
- Conventions inferred from `app_program/`:
  - **File header:** a block comment with the file name, a one-line purpose, an "APIs shown:" line and notes, for example `app_program/subsystem2/task_producer.c:1-10`. Subsystem headers carry `Student ID: <student ID>` (`subsystem1.h:3`).
  - **Header guards:** `SUBSYSTEMN_H`, `APP_H`, `TEST_REPORT_H` (`subsystem1.h:10-11`, `app.h:9-10`).
  - **Style:** tabs, K&R braces, T-Kernel types (`INT`, `UW`, `ID`, `ER`, `BOOL`, `UB`), `EXPORT`/`IMPORT`/`LOCAL` storage macros, `while (1)` loops, `(void)stacd; (void)exinf;`.
  - **Names:** upper-case typedefs (`SAMPLE_MSG`, `APP_STATS`), `S<N>_` macro prefixes (`S1_PRI_LED`, `S4_SUMMARY_MAX`), object IDs as exported globals (`tid_led`, `mbx_samples`).
  - **Includes:** cross-subsystem includes go through `"../subsystemN/subsystemN.h"` (`task_monitor.c:12-14`).
  - **Barr C conflicts in the template examples:**
    - Brace-less `if` (`subsystem2_test.c:24`).
    - Upper-case typedefs with no `_t`.
    - Non-`<stdint.h>` types (NFR16 in `design.md:117` requires stdint and module prefixes).
    - Exported mutable globals.

### 1.5 Layout and naming rules, and pre-written code

From `README.md` 2.5 and `TEAM.md` sections 1 and 3:
- "Your code lives in `app_program/`. Every `.c` file there, and in its subfolders, is compiled automatically" (`build_make/mtkernel_3/app_program/subdir.mk:6`: `app_program/*.c` plus one subfolder level).
- One folder per student: `app_program/subsystem1/` to `subsystem5/`.
- `subsystemN.h` is that subsystem's public interface. `app.h` holds team-wide definitions.
- Drivers live in the owning subsystem folder as `drv_<name>.c/.h`, "never under `device/`".
- Shared files (`app_main.c`, `app.h`, `TEAM.md`) change only by team agreement. "Only the owner edits a subsystem's folders".
- Pins: use GP0-22 and GP26-28. Never GP23/24/25/29, and use an external LED (`README.md` "Pins"; `app.h:16-21`).

APIs and patterns:
- `app.h`:
  - Includes `<tk/tkernel.h>`, `<tm/tmonitor.h>` and `<bsp/libbsp.h>` (lines 12-14).
  - `TASK_STACK_SIZE 2048` ("tm_printf needs ~1 KB", line 28).
  - `app_start_task(FP, PRI, UW prc, const char*)` (line 32).
  - `subsystemN_init/start` prototypes (lines 42-52), and `subsystemN_test` under `#if APP_TEST` (lines 54-60).
  - The `CHECK(er, what)` macro prints with `tm_printf` (lines 63-69).
- `app_main.c`:
  - `app_start_task()` builds `T_CTSK` with `TA_HLNG|TA_RNG3` and `stksz=TASK_STACK_SIZE`. Under `#if TK_SUPPORT_SMP` it adds `TA_ASSPRC` and `assprc=prc`. Then it calls `tk_cre_tsk` and `tk_sta_tsk` (lines 62-93).
  - `usermain()` prints a `tk_ref_ver` banner with `tm_printf`. It runs **phase 1**, all `subsystemN_init()`, then **phase 2**, all `subsystemN_start()`. Under `APP_TEST` it runs every `subsystemN_test()` in the init task. Finally `tk_slp_tsk(TMO_FEVR)` (lines 95-142). The header comment (lines 1-54) is a tutorial on startup and the IF-01..05 interfaces.
- Examples:
  - **S1** (`subsystem1.c`, `task_led.c`): a cyclic handler (`tk_cre_cyc`, 250 ms) does `tk_wup_tsk`. `led_task` runs `tk_slp_tsk`/`tk_can_wup` and toggles **GP19** with libbsp `gpio_set_pin`/`gpio_set_val`. Priority 12, any core.
  - **S2** (`subsystem2.c`, `task_producer.c`): semaphore `sem_slots` and fixed pool `mpf_samples`. `producer_task` (priority 14, **`TP_PRC2`**) does `tk_wai_sem`, `tk_get_mpf(TMO_POL)`, then `tk_snd_mbx(mbx_samples)` every 300 ms.
  - **S3** (`subsystem3.c`, `task_consumer.c`): mailbox `mbx_samples` and mutex `mtx_stats` (`TA_INHERIT`). `consumer_task` (priority 16, **`TP_PRC1`**) does `tk_rcv_mbx`, updates `app_stats` under the mutex, `tm_sprintf`, `tk_snd_mbf(TMO_POL)` and `tk_set_flg`, then `tk_rel_mpf` and `tk_sig_sem`.
  - **S4** (`subsystem4.c`, `task_monitor.c`): event flag, message buffer and alarm. `monitor_task` (priority 18) does `tk_sta_alm`, `tk_wai_flg`, drains the MBF, `tm_printf`s reports, and `tk_sus_tsk`/`tk_rsm_tsk`/`tk_chg_pri` on `tid_led`.
  - **S5:** empty template, `S5_PRI_MAIN 20`.
- **`TP_PRC1`, `TP_PRC2` and `TA_ASSPRC` do not exist in our pinned BSP headers.** A grep of `external/mtk3_bsp/include` finds no match. `subsystem2.c:42` and `subsystem3.c:39` use them outside `#if`, so **the template examples do not compile against our kernel unchanged**.
- Nothing in the template is marked as mandatory to keep verbatim. README says "Replace the examples with your own subsystems" (2.5), and TEAM.md says "Replace those rows with your own as you replace the example code".

### 1.6 Required docs and submission rules

- `README.md` is a setup and build guide for the template's own make flow. It has no "required README sections" and no report template. Its footer credits T-License 2.2 and Ken Sakamura / TRON Forum.
- `TEAM.md` sections are the de facto required records: 1 Members and subsystems (Student ID, GitHub username, Code, Tests), 2 Pins (one owner per pin, claim before wiring), 3 Peripherals (one owning task per peripheral; WiFi is owned by the kernel `cyw43_utk_task`), 4 Task priorities (bands 12-21, kernel USB 3 and WiFi 4), 5 Interfaces (IF-NN table: provider creates the object and publishes its ID in `subsystemN.h`; each IF has at least one integration test), 6 Kernel object budget.
- `testing/README.md` rules:
  - Test IDs `S<N>-U<nn>`, `INT-<nn>`, `SYS-<nn>`; never reused.
  - Define first, then automate with `TEST_CHECK` where possible.
  - Build with `make TEST=1`.
  - Save logs as `logs/YYYY-MM-DD_<what>.txt`.
  - Record date, ID, commit (`git rev-parse --short HEAD`), tester, PASS/FAIL and log.
  - **Never edit or delete old results.**
  - **Commit before testing.**
  - Re-run unit tests before merging into `main`.
- `videos/README.md`: unlisted YouTube links only, never commit video files, an index table with Date, Subsystem, Test ID, Commit, Student ID and Link.
- There is no commit-message style, required script or file-name rule beyond the above.

### 1.7 Test setup

All tests run **on the target only**. `subsystemN_test.c` is wrapped in `#if APP_TEST` and compiled only with `make TEST=1` (`-DAPP_TEST=1`). `usermain()` calls each test from the **init task after all tasks start** (`app_main.c:126-137`). `test_report.h` provides `TEST_INIT`, `TEST_CHECK(id, cond, desc)` and `TEST_SUMMARY(name)`, which print `[TEST] <ID> PASS|FAIL ...` with **`tm_printf`** (`test_report.h:25-41`). There is no host runner. `tools/verify_phase11_usb.py` is an unrelated port-validation script. The template runs `tm_printf`-heavy tests on the init task stack, which is still 1 KB (`include/sys/inittask.h` is unchanged).

### 1.8 Template `.gitignore` (verbatim content summary)

`Debug`, `Release`, `*.elf`, `*.uf2`, `*.map`, `*.o`, `*.d`, `build_make/tools/elf2uf2`, `tools/elf2uf2/elf2uf2`, `build_make/.build_profile`, `config/wifi_secrets.h`, `*.mp4`, `*.mov`, `*.webm`, `*.avi`, `*.mkv`, `.DS_Store`. `build_make/.gitignore` has `*.elf *.uf2 *.map **/*.d **/*.o tools/elf2uf2`. There is no `.gitattributes`. The `testing/*/logs/.gitkeep` files keep the empty directories.

### 1.9 Conflicts with the non-negotiable constraints

| # | Constraint | Template | Verified equivalent? |
|---|---|---|---|
| C1 | Pico SDK 1.5.1 | SDK 2.2.0 (headers and TinyUSB/cyw43 sources only) | No |
| C2 | GCC 10.3.1 | any arm-none-eabi-gcc, suggests 15.3 | No |
| C3 | CMake + Ninja on Windows | GNU make + POSIX shell (WSL) | No |
| C4 | PICO_BOARD=pico_w | `RASPBERRYPI_PICO_W=1` define, no SDK board | partial |
| C5 | Official single-core mtk3_bsp pico_rp2040, pinned, never edited | edited copy, SMP fork; `SMP=0` still uses modified kernel files | **No** |
| C6 | SDK owns startup, linker and vector table | kernel owns them (`reset_hdl.c`, `vector_tbl.c`, `boot_2nd.c`, `tkernel_map.ld`, `-nostartfiles`) | No |
| C7 | `-Wall -Wextra`, `-Werror` on our code | no warning flags | No |
| C8 | Kernel OR SDK headers per file (R1) | `cyw43_utk.c`, `mqtt_utk.c` and `sys_arch.c` mix them (R1 fires) | No |
| C9 | No blocking SDK calls in tasks, only the drain calls stdio, `tm_printf` only in port boot/fault | `tm_printf`/`tm_sprintf` from tasks, the `CHECK` macro and `TEST_CHECK` (R3b fires 17 times in `app_program/`) | No |
| C10 | ISRs via the SDK IRQ API | `tk_def_int` (`usb_sdk_compat.c` maps SDK hooks to `tk_def_int`) | No |
| C11 | Console UART0 default, USB option | USB CDC default, UART with `CONSOLE=uart`; GP0/GP1 free unless `CRASH_REPORT=1` | Same pins, opposite default |
| C12 | DBG_ENABLE from the build type | no build types (`-O2 -g3` always) | No |
| C13 | No WiFi in Phase 1 | `WIFI=cyw43` default | No (`WIFI=none` exists) |
| C14 | Core 1 unused (R5) | `SMP=1` default launches core 1 | No (`SMP=0` exists) |
| C15 | Sample device drivers off (`USE_SDEV_DRV 0`, `USE_DEVICE 0`) | `USE_SDEV_DRV 1`, `USE_DEVICE 1`, ADC driver on | No |
| C16 | libbsp never linked (`mtk3.cmake:127`) | app uses libbsp `gpio_set_pin` (`task_led.c:34`) | No |
| C17 | GP25 / on-board LED | template avoids GP25 (radio CS) and uses GP19 | **Compatible** (agrees with our hw_setting exclusion) |

---

## 2. Step 2: Conflict and gap table

| Area | Template says | We have | Conflict? | Options | Impact on Batches B-D | Decision |
|---|---|---|---|---|---|---|
| Layout | `app_program/{app.h,app_main.c,test_report.h}`, `subsystem1..5/`; kernel tree at the root; `testing/`, `videos/`, `TEAM.md` | `src/<module>/` (none created yet), `config/`, `cmake/`, `tests/`, `docs/`, `external/` (`ARCHITECTURE.md` "Source layout") | Yes | a) keep src/<module>; b) template layout for team code and src/ for the platform; c) everything under app_program | B: usermain moves to `app_program/app_main.c`. C/D: modules land in `subsystemN/` | **MERGE**: team code in `app_program/subsystemN/` (the brief says follow the template structure); platform (port, debug, common, SDK main) stays in `src/` because the template's equivalent is the kernel tree, not app_program |
| Build | make, POSIX, wildcard source discovery, options SMP/CONSOLE/WIFI/TEST/CRASH_REPORT | CMake + Ninja, explicit `sources.cmake`, options DBG_ENABLE/DBG_BACKEND/TEL_BACKEND_WIFI | Yes | keep ours, add `APP_TEST`; or optionally a thin `build_make/makefile` shim | B: add APP_TEST option and registration of `app_program/**` | **KEEP OURS + add APP_TEST** (C1-C3 non-negotiable) |
| Kernel | edited SMP fork, multi-target copy | pinned official submodule, `mtk3` static lib (43 files) | Yes | import (breaks C5), or ignore | none if ignored | **KEEP OURS**. Optional open item: the `low_pow()` WFI idea could be supplied by the port (exclude `power_save.c`); Batch B decides |
| Startup + linker | kernel `reset_hdl`/`vector_tbl`/`boot_2nd`, `tkernel_map.ld` (with ALIGN fix) | SDK crt0/boot2/VTOR, `cmake/memmap_mtk3.ld` | Yes | none compatible | none | **KEEP OURS** (C6). The template's ALIGN fix applies only to its own .ld |
| Console | `tm_printf` everywhere; USB CDC default with UART0 mirror; `CRASH_REPORT` for panics | `dbg_` ring + drain, UART0 default, USB option; Batch B item 13 = polled UART0 for boot/fault in USB builds | Yes (who prints) | port `CHECK`/`TEST_CHECK` to `dbg_` | B: the `dbg_` API must offer a formatted line call usable by `test_report.h`; keep the item 13 recommendation (it equals the template's CRASH_REPORT idea) | **KEEP OURS**, adapt template macros |
| Coding style | T-Kernel K&R style, typedefs upper case, `S<N>_` macros, file header block, no tooling | Barr C, `-Werror`, module prefixes (NFR16) | Partial | Barr naming with the template's file and folder naming | C/D: files `subsystemN.{c,h}`, `task_<x>.c`, `drv_<x>.{c,h}`, `subsystemN_test.c`; functions `motion_`, `lf_`, `bc_`, `obs_`, `imu_`, `safety_`, `vc_`, `tel_`; entry points `subsystemN_init/start/test` | **MERGE** (template file names, Barr inside files, keep the template file-header block with Student ID) |
| Warning flags | none | `-Wall -Wextra -Werror` (ours), mtk3 `-Wall -Wextra` + 3 `-Wno-*` | Yes | — | apply `robocar_apply_warnings` to app_program sources | **KEEP OURS** |
| File naming | `subsystemN*`, `task_*.c`, `drv_<name>.c`, `TEST_LOG.md`, `logs/YYYY-MM-DD_<what>.txt` | planned `src/<module>/<module>_*.c`; port/debug names fixed by the owner | Partial | adopt the template for team code | rule allow-lists unchanged (port/debug/main paths unchanged) | **KEEP TEMPLATE** for team code, **KEEP OURS** for platform |
| Docs | README (make/WSL guide), TEAM.md, testing/README.md, videos/README.md | README (attribution + links), ARCHITECTURE.md, NOTICE.md, docs/DEBUG.md, docs/SUBMODULE.md | Partial | rewrite README with template-style sections for our flow; adopt TEAM.md, testing/, videos/ filled with our data | B: TEAM.md filled from pins.h, priorities.h, config.h | **MERGE** |
| Tests | on-target `subsystemN_test.c` + `test_report.h`, records in `testing/` | `check_rules` + selftest + fixtures; host tests planned in Batch D (`tests/CMakeLists.txt`) | No (complementary) | keep both | D: host tests stay in `tests/`; on-target tests in `subsystemN_test.c`; records in `testing/` | **MERGE** |
| Git config | `.gitignore` (Eclipse dirs, objects, wifi_secrets, videos, .DS_Store); no `.gitattributes` | `.gitignore` (build/, secrets.h, outputs, .agents/), `.gitattributes` LF | Minor | union | none | **MERGE**: add video extensions, `.DS_Store`, `*.o`, `*.d`; keep `config/secrets.h` (ours covers WiFi + MQTT); drop the build_make entries |
| Licence | README footer: T-License 2.2, derived from mtk3_bsp pico_rp2040; no licence PDF; elf2uf2 BSD-3 | NOTICE.md (T-License 2.2, pinned commit, PDF link), README link | No | keep ours; add credit for the course template docs | none | **KEEP OURS + add template credit** (author UNVERIFIED, see Q7) |
| Kernel config | `config/config*.h` at config root, objects up to 12, timer 10 ms, SDEV on, IIC off | `config/mtk3/config*.h` (timer 1 ms, MBF 8, CYC 8, SDEV/DEVICE off, `config_check.h`) | Yes | — | none | **KEEP OURS** (no path clash; template files dropped) |
| Pins / priorities | GP19 example LED, bands 12-21, kernel USB 3 / WiFi 4 | `config/pins.h` (GP0-12, 18, 20-22, 26-28; GP19 free), priorities 1-10 | No pin clash | fill TEAM.md from our headers | — | **KEEP OURS**, documented in TEAM.md |

---

## 3. Step 3: Merge plan (not executed)

### 3.1 Options

**Option A: ours as base, template docs only.** Keep `src/<module>/` and add `TEAM.md`, `testing/`, `videos/`, the `test_report.h` concept and a README rewrite. Map buddies to `subsystemN` in TEAM.md only.
- Pros: smallest change (docs only plus APP_TEST), no CMake registration change, fixtures unchanged.
- Cons: does not follow the template folder structure that the original brief required. A grader looking for `app_program/subsystemN/` will not find it.

**Option B: template as base, port our work in.** Use the template kernel and make build, then re-add config, rules and docs.
- Pros: the literal template, with ready-made USB/WiFi/MQTT services for later phases.
- Cons: breaks C1-C3, C5-C7 and C10-C16. No Windows-native build. App code loses the Pico SDK hardware libraries (NFR16 says "using the Pico C SDK"). We would edit vendored kernel code. All Batch A verification (mtk3 0 warnings, pin guard, check_rules) would be redone, and Batches B-D redesigned (no port layer, different ISR model, SMP).
- **Not viable** without dropping non-negotiables.

**Option C (RECOMMENDED): hybrid, ours as base, template app layout imported.** Keep our whole build, kernel, port and rules base. Team code goes to `app_program/` in the template's shape, with Buddy N mapped to `subsystemN`. Add TEAM.md, testing/ and videos/ with our data. Port `app.h`, `app_main.c` and `test_report.h` to our rules.
- Pros: matches the template structure students and graders expect, and keeps every non-negotiable and all Batch A evidence.
- Cons: two source roots (`src/` platform, `app_program/` team). Template examples cannot be kept verbatim (tm_printf, libbsp, `TP_PRC*`). `make` in `build_make/` does not exist (see Q1).

### 3.2 Option C detail

Files **moved or renamed** (use `git mv`; the only tracked files affected are fixtures):

| From | To |
|---|---|
| `tests/rules_fixtures/src/motion/r1_mixed.c` | `tests/rules_fixtures/app_program/subsystem2/r1_mixed.c` |
| `tests/rules_fixtures/src/motion/r2_blocking.c` | `tests/rules_fixtures/app_program/subsystem2/r2_blocking.c` |
| `tests/rules_fixtures/src/line/r3_stdio.c` | `tests/rules_fixtures/app_program/subsystem3/r3_stdio.c` |
| `tests/rules_fixtures/src/vc/r4_libbsp.c` | `tests/rules_fixtures/app_program/subsystem2/r4_libbsp.c` |
| `tests/rules_fixtures/src/safety/r5_core1.c` | `tests/rules_fixtures/app_program/subsystem4/r5_core1.c` |

Kept unchanged: `tests/rules_fixtures/src/app/main.c`, `tools/r2_unlisted_dir.c`, `r3_root_file.c` and `docs/excluded.c`.

Files **imported from the template** (copied, then edited; none exist in our repo, so nothing is overwritten):
- `TEAM.md`: keep the structure. Fill in our 5 buddies (`design.md:13-18`), pins from `config/pins.h`, peripherals from design 2.x, priorities from `config/priorities.h` (replacing the 12-21 bands), interfaces from design 3.3 (MBF_NAV_IN, MBF_MOTION_CMD, ...) and object limits from `config/mtk3/config.h`. Remove the `cyw43_utk_task` rows (Phase 1 has no WiFi; leave a TODO for Phase 2). Keep GP23/24/25/29 reserved.
- `testing/README.md`, `testing/{subsystem1..5,integration,system}/TEST_LOG.md`, `testing/*/logs/.gitkeep`: replace `make TEST=1` with our CMake `-DAPP_TEST=ON` command and the USB/UART capture commands for Windows; clear the example rows.
- `videos/README.md` (verbatim apart from the example row).
- `app_program/app.h`: keep the two-phase contract and the `subsystemN_init/start/test` prototypes. Kernel side only; drop `<bsp/libbsp.h>`. `CHECK()` goes through `dbg_`. `TASK_STACK_SIZE` is replaced by per-task stacks from `config/priorities.h`. `app_start_task()` drops `prc` (single core; `TP_PRC*` do not exist).
- `app_program/app_main.c`: becomes our `usermain()` (kernel side). Phase 1 creates all objects, phase 2 starts all tasks, then `tk_slp_tsk(TMO_FEVR)`. The banner goes through `dbg_`. Under `APP_TEST` it starts a test-runner task instead of running tests on the 1 KB init stack (see the Q6 deviation).
- `app_program/test_report.h`: same macros and same `[TEST] ID PASS|FAIL` line format, output through the `dbg_` API.
- `app_program/subsystem1..5/{subsystemN.c,subsystemN.h,subsystemN_test.c}`: stubs with TODO markers; example tasks dropped (see Q5).

Files **dropped (not imported); listed, not deleted**, because they stay in `ProjectTemplate/` or the backup: `ProjectTemplate/build_make/**` (76), `config/**` (5), `device/**` (86), `etc/**` (10), `include/**` (124), `kernel/**` (237), `lib/**` (60), `tools/**` (4), `README.md` (replaced by our merged README), `.gitignore` (merged), `app_program/subsystem1/task_led.c`, `subsystem2/task_producer.c`, `subsystem3/task_consumer.c`, `subsystem4/task_monitor.c` (examples; see Q5). After the import, the `ProjectTemplate/` folder must leave the repo tree or be excluded. Recommended: **move it to `c:\Users\Admin\Downloads\Projects\ProjectTemplate-original`** (owner approval needed, Q8).

Files **merged**: `.gitignore` (add `*.o`, `*.d`, `*.mp4`, `*.mov`, `*.webm`, `*.avi`, `*.mkv`, `.DS_Store`), `README.md`, `NOTICE.md`, `ARCHITECTURE.md`, `docs/DEBUG.md`.

**Same-name clashes** (same relative path in both trees):

| Path | Diff |
|---|---|
| `README.md` | Completely different. Ours: 5 lines (attribution with TODO(owner-to-confirm), links to DEBUG/SUBMODULE). Template: 220-line make/WSL setup guide. Merge: our attribution line and TODO stay at the top; add template-style sections "Set up" (SDK 1.5.1 installer, our preamble), "Build" (CMake commands), "Flash" (SWD/UF2), "Write your own code" (app_program layout), "Build options" (DBG_BACKEND, DBG_ENABLE, APP_TEST; template SMP/WIFI/CRASH_REPORT stated as not supported in Phase 1), "Working as a team" (TEAM.md, testing/, videos/). The licence footer is folded into NOTICE.md. |
| `.gitignore` | Ours: `build/ build-*/ config/secrets.h *.uf2 *.elf *.bin *.hex *.map *.dis .agents/`. Template only: `Debug Release *.o *.d build_make/tools/elf2uf2 tools/elf2uf2/elf2uf2 build_make/.build_profile config/wifi_secrets.h *.mp4 *.mov *.webm *.avi *.mkv .DS_Store`. Merge as above; skip `Debug`, `Release` (Eclipse), the build_make entries and `wifi_secrets.h` (ours is `secrets.h`). |
| `config/config*.h` vs `config/mtk3/config*.h` | Different paths, so no clash. Diff in 1.3 and the table above. Template files are not imported. |
| `app_program/app_main.c` vs planned `src/app/main.c` | Different roles. The template file is kernel-side `usermain()`; ours is the SDK `main()` that starts the kernel. Both are kept. |
| `config/wifi_secrets.example.h` vs `config/secrets.h.example` | Equivalent purpose. Keep ours (it also has MQTT_BROKER_*). |

### 3.3 Every path that must change (Option C)

- `CMakeLists.txt`:
  - Replace the single-root registration (`ROBOCAR_SRC_DIR`, `ROBOCAR_MODULE_DIRS common rtos debug motion vc safety line barcode obstacle imu telemetry app`) with repo-relative module dirs: `src/common src/rtos src/debug src/app app_program app_program/subsystem1 app_program/subsystem2 app_program/subsystem3 app_program/subsystem4 app_program/subsystem5`.
  - Extend the unregistered-`sources.cmake` guard to glob `app_program/**/sources.cmake` as well as `*/sources.cmake` and `src/**/sources.cmake`. Update the error text.
  - Add `option(APP_TEST "On-target subsystem self-tests" OFF)` with `APP_TEST=0/1` on `robocar_options`, and `FATAL_ERROR` when `APP_TEST=ON` with `DBG_ENABLE=OFF`. Print it in the status line.
  - Optionally add `app_program/` as an include root. The template uses `"../subsystemN/subsystemN.h"` relative includes, which work without it.
- `cmake/check_rules.cmake`:
  - `RULES_STDIO_ALLOWED_FILES`, `RULES_TM_ALLOWED_FILES`, `RULES_PREKERNEL_FILE` and `RULES_PORT_DIR_REGEX` are **unchanged** (port, debug and main paths do not move).
  - `RULES_EXCLUDE_TOP_REGEX`: add `testing|videos` (docs only; harmless). Add `ProjectTemplate` **only if** the owner keeps the folder in the repo.
  - Header comment: mention `app_program/`.
  - Optional: allow `tm_sprintf` (a pure formatter) in R3b? Recommend no; use `snprintf`-free `dbg_` formatting instead.
- `cmake/check_rules_selftest.cmake`: `expected` strings move from `src/motion/...`, `src/line/...`, `src/vc/...`, `src/safety/...` to `app_program/subsystem{2,3,2,4}/...`. `unexpected` keeps `src/app/main.c`, `docs/excluded.c` and the `r3_stdio.c:3:` line under its new path.
- `tests/rules_fixtures/**`: the 5 `git mv` above; update the comment in `r2_blocking.c:1` if it names paths.
- `cmake/mtk3.cmake`: comment line 9 is unchanged. Line 76 comment (`app_program/**` = BSP sample app) must say it refers to `external/mtk3_bsp/app_program`, not ours. The forbidden regex `app_program` (line 127) checks `MTK3_SOURCES_REL` only (BSP-relative), so it is still correct.
- `ARCHITECTURE.md`: "Source layout" gains `app_program/` (app.h, app_main.c = usermain, test_report.h, subsystem1..5 = Buddy 1..5), `TEAM.md`, `testing/` and `videos/`. "Kernel integration" says usermain lives in `app_program/app_main.c`. Add a deltas row for "template deviations" (no SMP, no make, no tm_printf, test runner task).
- `README.md`, `NOTICE.md` (add credit for the course template docs; T-License credit unchanged), `docs/DEBUG.md` (APP_TEST build and `[TEST]` log capture), `docs/SUBMODULE.md` (no change expected).
- Planned Batch B-D names, old to new:

| Old (planned) | New |
|---|---|
| `src/rtos/port/port_kernel_internal.c`, `port_boot.c`, `port_fault.c` | unchanged |
| `src/debug/dbg_drain.c` | unchanged |
| `src/app/main.c` (SDK `main()`, pre-kernel) | unchanged (see Q4) |
| `usermain()` (location not yet fixed) | `app_program/app_main.c` |
| `src/common/` | unchanged (shared library code); team-wide definitions in `app_program/app.h` |
| `src/telemetry/` (Buddy 1) | `app_program/subsystem1/` (`task_telemetry.c`, `tel_*`) |
| `src/motion/`, `src/vc/` (Buddy 2) | `app_program/subsystem2/` (`task_motion.c`, `task_vc.c`, `drv_motor.c`, `drv_encoder.c`) |
| `src/line/`, `src/barcode/` (Buddy 3) | `app_program/subsystem3/` (`task_line.c`, `task_barcode.c`, `drv_ir.c`) |
| `src/imu/`, `src/safety/` (Buddy 4, safety shared with Buddy 2) | `app_program/subsystem4/` (`task_imu.c`, `task_safety.c`, `drv_imu.c`) (see Q3) |
| `src/obstacle/` (Buddy 5) | `app_program/subsystem5/` (`task_obs_scan.c`, `task_obs_avoid.c`, `drv_ultrasonic.c`, `drv_servo.c`) |
| host tests `tests/` (Batch D) | unchanged; on-target tests in `subsystemN_test.c`; records in `testing/` |

R1 split inside a subsystem: `drv_*.c` are SDK-side (`ROBOCAR_SOURCES`) and include only `hardware/*.h` plus their own kernel-free `drv_*.h` (stdint only). `task_*.c`, `subsystemN.c` and `subsystemN_test.c` are kernel-side (`ROBOCAR_KERNEL_SOURCES`) and include `subsystemN.h` / `app.h`. `subsystemN.h` must never be included by a `drv_*.c`.

Git history: Batch A has no `src/` files and none of the imported files exist, so the only history to keep is the 5 fixture moves (use `git mv`). Commit them in a separate commit from the content edits so `git log --follow` tracks the renames.

### 3.4 Risks

1. **The grader may build with `cd build_make && make`** (template README 2.2). Our tree has no make flow. Mitigation: ask (Q1). If needed, add a POSIX `build_make/makefile` shim that calls CMake. It still needs the SDK 1.5.1 toolchain, which is UNVERIFIED on the grader's machine.
2. **The template examples fail our rules and kernel** (R3b, libbsp, `TP_PRC*`). Keeping them means rewriting them. A half-kept example could leak `tm_printf` into tasks.
3. **The `ProjectTemplate/` folder in the repo root breaks `check_rules` (94 violations) now.** It must be moved or excluded before verification. It must never be committed (643 files, a kernel copy that differs from the pin).
4. **Tests on the init task**: the template pattern runs tests with formatted output on the 1 KB `INITTASK_STKSZ` stack. That conflicts with our ARCHITECTURE "usermain stays stack-light" exception. Mitigation: a dedicated test-runner task.
5. **Release builds**: `DBG_ENABLE=OFF` would silence `TEST_CHECK`. Mitigation: configure fails when `APP_TEST=ON` with `DBG_ENABLE=OFF`.
6. **Two source roots** could confuse registration. Mitigation: the extended unregistered-`sources.cmake` guard plus a negative test.
7. **Template kernel improvements we will not get**: WFI idle (`power_save.c`), bounded UART TX (we replace `tm_com.c` anyway), and the `.data` ALIGN fix (not our linker script). These are open items for Batch B, not reasons to import.
8. **EOL**: template files are LF and some are UTF-8 with non-ASCII characters (`—`, `≥`, `Ω`). `.gitattributes` `eol=lf` is fine. Keep UTF-8 when editing.

### 3.5 Verification plan (after execution)

Use the exact preamble from `.agents/tasks/batchA-summary.md` (`$R`, `$env:PATH`, `$env:PICO_SDK_PATH`, `$CM`, `$common`), run from the repo root with `ProjectTemplate/` moved out of the tree, and delete `build/` first:

1. `& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0` should exit 0 and print `DBG_ENABLE=ON DBG_BACKEND=UART0 ... APP_TEST=OFF` and `mtk3: ... pinned at 15ed232..., clean`.
2. `& $CM --build build/uart0 --target mtk3` should give 0 warnings and 0 errors. `--target config_check` should give 0 warnings. `& $CM --build build/uart0` (ALL) should exit 0 with `check_rules: PASS`.
3. Repeat 1-2 for `build/usb` with `-DDBG_BACKEND=USB`.
4. `& $CM --build build/uart0 --target check_rules_selftest` and `& $CM -P cmake/check_rules_selftest.cmake` should both report PASS with the new fixture paths.
5. Negative tests (temporary files, deleted afterwards):
   - `app_program/subsystem6/sources.cmake` should make configure fail ("not registered").
   - `app_program/subsystem2/x.c` with `tm_printf(` should give R3b, and one with `sleep_ms(1)` should give R2. Both mean `check_rules` exits 1.
   - `src/foo/x.c` with `sleep_ms(1)` should still fire.
6. `& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release` should show `DBG_ENABLE=OFF`. Adding `-DAPP_TEST=ON` should make configure fail (FATAL_ERROR).
7. `& $CM -S . -B build/test @common -DDBG_BACKEND=UART0 -DAPP_TEST=ON` should exit 0. Its build should give 0 warnings, once stubs exist.
8. `git -C external/mtk3_bsp status --porcelain` should be empty, and `git submodule status` should show ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)`.
9. `git status` should show no `ProjectTemplate/` and no stray files. `git log --follow tests/rules_fixtures/app_program/subsystem2/r1_mixed.c` should show the old path.

## 4. Questions for the owner

1. Will the instructor build with the template's `cd build_make && make` (WSL), or only from our repo instructions? Should we add a make-to-CMake shim?
2. Is the `app_program/subsystem1..5/` layout mandatory with those exact folder names, or may we use names like `app_program/telemetry/`? Option C assumes the exact names.
3. Is mapping Buddy N to `subsystemN` acceptable (1 telemetry, 2 motion + vc, 3 line + barcode, 4 imu + safety, 5 obstacle)? Safety is shared by Buddy 4 (watchdog) and Buddy 2 (brake). Should `task_safety.c` live in subsystem4 or subsystem2?
4. Should `src/app/main.c` (SDK `main()`) keep its name next to `app_program/app_main.c` (`usermain`), or be renamed (for example `src/boot/main.c`)? A rename changes `RULES_PREKERNEL_FILE` and one fixture.
5. Should the template demo tasks (LED on GP19, producer, consumer, monitor) be kept as a ported demo under `APP_TEST` or a `DEMO` option, or dropped? Option C drops them (they need tm_printf, libbsp and `TP_PRC*`).
6. Is it OK to deviate from the template by running `APP_TEST` tests in a dedicated test task instead of the 1 KB init task?
7. Who authored the template (for the NOTICE.md credit)? Is there a licence for the template docs? UNVERIFIED: the template carries no licence of its own.
8. After the import, may `ProjectTemplate/` be moved out of the repo to `..\ProjectTemplate-original`? The backups `backup/pre-template-merge` and `..\inf-2004-project-backup-pre-template-merge` already exist. The alternative is to keep it in place and add it to `.gitignore` and `RULES_EXCLUDE_TOP_REGEX`.
9. Should TEAM.md section 4 list our priorities 1-10 instead of the template's 12-21 bands? Recommended yes, because they come from design.md.
10. For Phase 2 WiFi, should we plan on the template's `cyw43_utk`/lwIP/MQTT design ideas? The files themselves depend on the SMP kernel and break R1. Recommended: design only, re-implement on the SDK side.
