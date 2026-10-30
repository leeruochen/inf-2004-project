# Batch B, stage B0: independent review of PR fae43bf

Reviewer: read-only investigation for Ruo Chen (Buddy 2). PR author: Jin Yong (Buddy 3).
Repo: `c:\Users\Admin\Downloads\Projects\inf-2004-project`. Nothing in it was edited, checked out, stashed or committed. The only write outside this report was a scratch clone in `%TEMP%\b0-verify`, which is deleted.

Conventions: `file:line` refers to the PR branch (`fae43bf`) unless a path starts with `external/` (kernel, pinned `15ed232`) or `SDK/` (`C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk\src`). UNVERIFIED = I could not confirm it from code on disk.

---

## 0. Summary and verdict

**Verdict: merge after small fixes.** B0 is docs only and behaves exactly like the base: I ran the plan §2 gates on `fae43bf` with SDK 1.5.1 + GCC 10.3.1 in a scratch clone. Debug builds 84/84 with 0 warnings and `check_rules: PASS (42 files scanned)`. The self-test passes, `APP_TEST=ON` stops only at `test_report.h:35`, and Release builds 84/84 with 0 warnings. No protected file is touched, and the branch fast-forwards onto `main`.

Refs are as stated: `origin/batchB/b0-records` = `fae43bf`, its parent is `8d083ca` = `origin/main`, and `rev-list --left-right --count origin/main...origin/batchB/b0-records` = `0 1`. The owner's working tree here is already on `batchB/b0-records` at `fae43bf`, clean, in sync with origin. A merge into `main` cannot conflict.

Fixes before merge (details in §3, all a few lines):
1. `HANDOVER.md:190` still says B3 includes "linker overrides". The plan (Rev 5) says no linker work.
2. The Rev 5 claim that `.data_vector` sits "inside the zero-filled range" is wrong for `.data_vector`. It is true for `.noinit` (`cmake/memmap_mtk3.ld:126-137` vs `:217-231`). Harmless, but the wording must be corrected.
3. O7 as written is not implementable: Cortex-M0+ code cannot read `DHCSR.C_DEBUGEN`. Also `SYSRESETREQ` resets only the core on RP2040. Record the corrected decision before it becomes "binding".
4. Decision provenance is inconsistent. "Decisions already made (binding)" now contains C7, C8 and O1-O9 that the plan says the helper answered. Mark them "confirmed by Buddy 2" only when you confirm.
5. Stale lines: plan "not committed" (`batchB-plan.md:165`) and `CMakeLists.txt:150` (should be `:161`).

Biggest process finding (not a B0 defect): the C7 loop makes every stage wait on the owner's PC, including compile-only stages. The cheapest fix is for Jin Yong to install the same SDK 1.5.1 installer (README:22-25 already says "Do not use another SDK or compiler version"). See §2a(iv).

---

## 1. SDK facts

### This machine (read-only)
- Installed: only `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1` (`version.ini`: `PICO_SDK_VERSION=1.5.1`). `arm-none-eabi-gcc --version` = GNU Arm Embedded Toolchain 10.3-2021.10, 10.3.1 20210824. CMake 3.25.2, Ninja 1.11.1 inside the installer. `%USERPROFILE%\.pico-sdk` does not exist, so no SDK 2.x is installed here. `PICO_SDK_PATH` is not set globally.
- Caveat: this machine's user is `Admin`. HANDOVER names the owner's PC as `C:\Users\asus\...` (`HANDOVER.md:7-9`). So these facts do not prove what the owner's own PC has. UNVERIFIED for `asus`.
- To settle it on the owner's PC: `Select-String 'PICO_SDK_VERSION_(MAJOR|MINOR|REVISION) ' "$env:PICO_SDK_PATH\pico_sdk_version.cmake"` (or the configure log line "Pico SDK is located at ...") and `arm-none-eabi-gcc --version`.

### Does the committed repo assume 1.5.1 everywhere? Yes.
`CMakeLists.txt:1` (comment), `cmake/memmap_mtk3.ld:1` ("copy of pico-sdk 1.5.1 ... memmap_default.ld"), `cmake/warnings.cmake:10`, `pico_sdk_import.cmake` (copy of the SDK file, `NOTICE.md:18-19`), `README.md:9,22-25,35-43` ("Do not use another SDK or compiler version"), `HANDOVER.md` §5, `NOTICE.md:36-37` (pico-examples tag `sdk-1.5.1`), `ARCHITECTURE.md` T2, and the `.agents/tasks/*` verify files. The build actually passed on 1.5.1 here (§4). No file in the repo requires 2.x.

### Case (a): owner's PC on SDK 1.5.1 + GCC 10.3.1
C8 holds with no work. BM is moot. All gates, `.uf2` builds and B6 size numbers come from this environment. The plan's open question about GCC 15 warnings is answered by my run: with GCC 10.3.1 the Release build has 0 warnings (no `libtm_printf.c:50/277` warnings), so no per-file warning option for `mtk3` is needed.

### Case (b): owner's PC already on SDK 2.3.1
Then C8 is wrong for the owner's PC. BM would have to come back before B1, because every official gate runs there. What breaks or needs change (repo evidence; the 2.3.1 behaviour is taken from the plan §0 and is UNVERIFIED by me, since no 2.x tree exists here):

| Item | Repo line | Effect on 2.3.1 |
|---|---|---|
| HANDOVER §5 preamble | `$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"`, `-DPICO_SDK_PATH=.../Pico SDK v1.5.1/pico-sdk`, `-Dpico-sdk-tools_DIR`, `-DCMAKE_MAKE_PROGRAM`, `-DPICO_TOOLCHAIN_PATH` | Every path is the 1.5.1 installer layout. 2.x via the VS Code extension lives elsewhere, and `-Dpico-sdk-tools_DIR` is an installer concept. The preamble must be rewritten, and the README (team document, show the diff first). |
| `pico_sdk_import.cmake` | `NOTICE.md:18-19` says copied from 1.5.1 | Probably still configures (the plan's trial says it did); a 2.3.1 copy is cleaner. |
| `cmake/memmap_mtk3.ld` | `:1`, `:126-137`, `:213-231`, `:274-275` | It is a full copy of the 1.5.1 script. The plan says 2.3.1 builds its script from include files with override hooks, so this copy may fail to link or give a different layout. This is the real BM cost: B3's `.noinit` zero-fill and the `ASSERT`s must be redone as override files. |
| `CMakeLists.txt` | `:14` `PICO_BOARD pico_w`, `:26` `pico_sdk_init()`, `:217` `pico_set_linker_script`, `:221-222` stdio flags | Board name and stdio functions exist in 2.x. Only the linker script line is at risk. |
| `cmake/config_check.c`, `cmake/mtk3.cmake` | `mtk3.cmake:17-18` ("`__aeabi_*` integer division comes from the SDK") | No SDK header dependency; low risk. |
| `pico_generate_pio_header` | not yet used (HANDOVER §3 Buddy 2 row says "add `pico_generate_pio_header`") | Needs `pioasm`. In 1.5.1 the installer ships `pico-sdk-tools`; 2.x must find or build `pioasm`. UNVERIFIED. |
| `hardware_*` APIs, board header names | `CMakeLists.txt:176-185` links `hardware_gpio/adc/pwm/i2c/irq/sync/pio` | Names are stable. Not a risk. |
| GCC flags | `warnings.cmake:10` (comment only), `-Wall -Wextra -Werror` on our code | GCC 15 adds warnings in vendored `libtm_printf.c` (plan §0). Our `-Werror` is per-file on our code only (`warnings.cmake:14-18`), so vendored warnings do not fail the build. |
| pico-examples PIO program | `NOTICE.md:36-37` | Program was taken at `sdk-1.5.1`; re-check on 2.x. |

Decision rule for the owner: if the owner's PC is on 1.5.1, keep C8. If it is on 2.3.1, either (1) install the 1.5.1 installer alongside (side-by-side is supported, the installer uses its own folder) and keep C8, or (2) run BM first. Option (1) is far cheaper.

---

## 2. Decision by decision

| ID | Verdict | One-line reason (checked against code) |
|---|---|---|
| C7 workflow | **agree with changes** | Sound for B4+ hardware, but forcing owner verification on compile-only stages and writing the hardware-sensitive stages blind is the slowest design; see §2a(ii),(iv). |
| C8 toolchain 1.5.1, defer BM | **agree** | The repo is 1.5.1 throughout (§1) and B0 passes there. Only invalid if the owner's PC is on 2.3.1 (§1 case b). |
| O1 PLATFORM_BRINGUP heartbeats | **agree with changes** | Needed for the mandatory USB CDC test (`docs/DEBUG.md` "USB CDC bring-up test"), but off the critical path: do it after "firmware boots" and give it the same option as the B4 self-test hooks. |
| O2 platform writes `src/common/ipc_types.h` | **agree with changes** | Matches C4 and `DATA_REPRESENTATION.md:239-245` ("Week 6 unpacked types in one header"). But "each field from DATA_REPRESENTATION §2" mixes in Week 8 fields (clearances, bypass command, sequence; `ARCHITECTURE.md` D8). A baseline must use the Week 6 field set. State that. |
| O4 fault record | **agree with changes** | Layout fine: 1 magic + 7 payload + 1 CRC = 9 words = 36 B. CRC variant is standard CRC-32. Add SP and current task id; put CRC in a pure header so it can be host-tested. See below. |
| O5 512 B system-message ring | **agree** | Safe without a lock: `tm_putchar/tm_putstring` wrap `tm_snd_dat` in `DI/EI` (`external/mtk3_bsp/lib/libtm/libtm.c:89-94, 108-112`). Banner is about 34 B. |
| O6 2 KB dbg ring, 96 B, drop newest | **agree** | Multiple task writers are serialised by a `DI/EI` section around reserve+copy (`armv6m/syslib.h:43-46`); drop-newest keeps the root cause. Watch `vsnprintf` stack depth (UNVERIFIED). |
| O7 record then reset, halt if debugger | **agree with changes** | The "debugger attached" clause cannot be implemented (see below). Reset method should be `watchdog_reboot`, not `SYSRESETREQ`. |
| O8 TA_USERBUF arena | **agree with changes (prefer simpler option)** | Not needed for fit (stacks 23 KB of a 96 KB pool) and does not solve the init-task measurement. Paint from a task trampoline instead. See §2a(iii). |
| O9 add `ipc` to R1 | **agree** | `_RE_KINC_PROJ` (`cmake/check_rules.cmake:61`) lists only `app|test_report|subsystemN`; `ipc.h` includes `<tk/tkernel.h>`, so it must count as kernel side. |
| C3 system-message ring + faults-only record | **agree** | Consistent with the kernel: banner is `tm_printf` at `external/mtk3_bsp/kernel/inittask/inittask.c:118-120`; kernel fault handlers that also print are never installed (`exc_hdr.c:45-59`). D6/DEBUG.md text matches F-4 (`.uninitialized_data`, not `.noinit`). |

### Sanity checks per decision

**O4 magic and CRC.** `0x464C5452`: bytes 0x46 'F', 0x4C 'L', 0x54 'T', 0x52 'R' read as a number. On the little-endian RP2040 the bytes in memory are `52 54 4C 46` ("RTLF"), so `x/4cb` in gdb shows "RTLF" while `x/wx` shows `0x464c5452`. Say so in DEBUG.md so nobody thinks the magic is wrong. Layout: magic, 7 payload words (version, fault type, PC, LR, xPSR, kernel error, boot count), CRC = 9 words (36 B). Reflected polynomial `0xEDB88320`, init and xorout `0xFFFFFFFF` is standard CRC-32/IEEE (check value for ASCII "123456789" is `0xCBF43926`; standard, not computed here). Testability: `tests/CMakeLists.txt` does not exist yet (only `tests/rules_fixtures/`), and host tests are scheduled in B7. So put `crc32` in a pure header (for example `src/common/crc32.h`) and add the `0xCBF43926` check to the B4 on-target test and to the B7 host test. Two weaknesses: boot count inside a record that is cleared after printing and re-initialised on random RAM is not a real boot counter; and without SP and task id (`knl_ctxtsk->tskid`) a crash is hard to locate. Optional additions, not blockers.

**O5 SPSC on M0+.** Single writer holds: all `tm_*` output goes through `tm_putchar`/`tm_putstring`, which run `DI` around every `tm_snd_dat` call (`libtm.c:89-94`, `:108-112`), so concurrent tasks and ISRs cannot interleave. The only reader is the drain task. Aligned 32-bit loads/stores are single instructions on ARMv6-M, so `volatile uint32_t head` (writer-only) and `tail` (reader-only) are atomic. Single core, so no DMB is needed, but a compiler barrier is. Rules for the implementer: the drain reads `head` once per cycle, copies, then writes `tail`; the dropped-byte counter is written by the writer only. A stale `tail` only makes the writer think there is less room, which is safe. Size: the banner is `"\n\nmicroT-Kernel Version %x.%02x\n\n"` (`inittask.c:119`; `VER_MAJOR 3`, `VER_MINOR 0`, `include/sys/knldef.h:228-229`), about 30 B, plus 4 CR bytes added by `tm_putchar` for each LF (`libtm.c:90-92`) = about 34 B. `USE_SYSTEM_MESSAGE`/`USE_TMONITOR` are 1 (`config/mtk3/config.h:137-138`). `DBG_DRAIN_PERIOD_MS` = 10 (`config/timing.h`), but the drain cannot run until the init task (priority 1) finishes `usermain`, so the ring just holds the banner until then. 512 B is ample. Note: text can still be written in a HardFault (the handler is also under masked interrupts), but it dies with the reset, as documented.

**O6 many writers.** Mutual exclusion is the `DI/EI` critical section (plan B1). `vsnprintf` runs before the section into a stack buffer, then reserve+copy of at most 96 B runs inside it. On a 125 MHz M0+ that is a few microseconds (estimate, UNVERIFIED), far below the 1 ms jitter budget. A kernel mutex would invert priorities and cannot be used from the init task before objects exist. Drop-newest is right for diagnostics (the first error is kept, a counter reports the loss). Risk to measure in B6: `vsnprintf` from newlib on 1.5 KB task stacks (`config/priorities.h:63-86`).

**O7 details.**
- *Debugger detection:* `DHCSR.C_DEBUGEN` is a debug-port register. ARM TRM text for the Cortex-M1 says debug components are reachable only from the debug port and software accesses are reserved; an ST community answer says the same for Cortex-M0 ([ARM TRM](http://infocenter.arm.com/help/topic/com.arm.doc.ddi0413d/Bcgbfdhc.html), [ST community](https://community.st.com/t5/stm32-mcus-products/detect-debugger-connected-to-stm32l0/td-p/405688)). On disk: the SDK's `m0plus.h` defines no DHCSR register (only a comment at `SDK/rp2040/hardware_regs/include/hardware/regs/m0plus.h:786`), and the CMSIS stub `core_cm0plus.h:606-611` has only an empty "CoreDebug" group heading. Not confirmed on RP2040 silicon. A `bkpt` with no debugger attached escalates to a HardFault from inside the HardFault handler, which is LOCKUP, so it must not be executed blindly. Implementable alternative: a volatile RAM flag `port_fault_halt` that the owner sets from gdb before a debug session (or a build option `PORT_FAULT_HALT`), default 0 = reset.
- *Reset method:* on RP2040, `SYSRESETREQ` resets only the core that asserts it, not the wider system ([pico-feedback #329](https://github.com/raspberrypi/pico-feedback/issues/329)). PWM, PIO and DMA keep running until the SDK's `runtime_init` resets all blocks except QSPI, USB PLL, USBCTRL, SYSCFG and SYS PLL (`SDK/rp2_common/pico_runtime/runtime.c:72-81`). `watchdog_reboot(0,0,delay_ms)` (`SDK/rp2_common/hardware_watchdog/watchdog.c:77-97`, prototype `watchdog.h:45`) resets everything through PSM except the oscillators (`watchdog.c:40`) and works from a fault handler (register writes only). Recommend it. Pick a small non-zero delay; zero-delay behaviour was not checked (UNVERIFIED).
- *USB across the reset:* either way the host sees a disconnect and re-enumeration (TinyUSB resets USBCTRL at init, `SDK/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c:68-69`). Terminals such as PuTTY do not reopen the port, so the next-boot fault print goes unseen. The RAM record survives a warm reset (`.uninitialized_data` is NOLOAD and not zeroed, `memmap_mtk3.ld:191-194`; that the bootrom keeps SRAM on a watchdog reset was not checked here).
- *Design hole for B4:* `stdio_usb_out_chars` silently discards data when no host is connected (`SDK/rp2_common/pico_stdio_usb/stdio_usb.c:96, 116-119`). The plan's drain prints and then clears the record at its start, so a record printed before the host reopens the port is lost. Fix: clear only after a confirmed connected print. `dbg_drain.c` is kernel side and `pico/stdio_usb.h` is an SDK header (R1, `check_rules.cmake:62`), so this needs a `hal_` function (for example `hal_console_connected()`), and the plan's `hal.h` list (§B3) lacks it.
- *Stack-overflow case:* tasks run on MSP (`config/priorities.h:54`), so an overflowing task makes exception stacking fault and the core may go to LOCKUP, where no handler runs. Only a watchdog recovers it. Worth one line in the plan (known gap, not a B0 blocker).

**O1.** DEBUG.md still requires nine heartbeat lines for Phase 1 acceptance, and `DATA_REPRESENTATION.md:239-241` also lists nine stub tasks in B-baseline, so something must supply them. Keep the option, default OFF. Fold the B4 self-test hooks (late `tm_printf`, forced fault) into the same option family (see §2a(ii)).

**O2 vs Week 8.** The OWNER CORRECTIONS block (`.agents/tasks/week8-impact-report.md`, item 3) keeps the optional 2-bit sequence number until the owner decides and requires `TODO(owner-to-confirm, buddy 5)`. That is a B7 (packed) matter and compatible with unpacked now. Conflict only in the baseline field set (see table).

**O9.** Adding `ipc` means `#include "ipc.h"` (any relative path) counts as a kernel-side include. Then any file that also includes `pico/`, `hardware/`, `boards/`, `tusb`, `lwip/` or `cyw43` fails R1 (`check_rules.cmake:177-184`). Consequence: `src/common/ipc.h` is kernel side only. SDK-side port files and `drv_*.c` reach IDs through `hal_*`, which is already the plan's design. `src/common/ipc_types.h` and `mbf_size.h` stay pure and need no entry. Also add a fixture under `tests/rules_fixtures/` (existing `app_program/subsystem2/r1_drv_subsystem_hdr.c` is the pattern) and update `cmake/check_rules_selftest.cmake`.

---

## 2a. Deep checks

### (i) Linker script and exchdr_tbl: "B3 needs no linker work" is correct, with one wording error
- `.data_vector (NOLOAD)`: `memmap_mtk3.ld:126-137`, placed after `.ram_vector_table` and **before** `.data` (`:139`). It is **not** inside `__bss_start__..__bss_end__` and is not zeroed. The Rev 5 sentence ("`.data_vector` (NOLOAD) and `.noinit` inside the zero-filled range") is false for `.data_vector`.
- `.noinit (NOLOAD)`: `:217-224`; `__bss_start__` is defined inside it (`:219`), `__bss_end__` at `:231`; `ASSERT(__bss_start__ == __noinit_start)` at `:275`. crt0 zero-fills exactly that range (`SDK/rp2_common/pico_standard_link/crt0.S:244-245`). So `.noinit` is zeroed. Correct.
- Kernel use: `Noinit(decl)` = `section(".noinit")` (`external/mtk3_bsp/include/sys/machine.h:77`); about 30 control-block tables use it (`task.c:30-44`, `semaphore.c:26`, `messagebuf.c:27` and so on). `USE_NOINIT` is 0 (`external/mtk3_bsp/config/config.h:91`), which the BSP's own reset code treats as "zero from `__bss_start`" (`reset_hdl.c:73-77`). So zero-fill of `.noinit` is required and provided.
- `exchdr_tbl` (`armv6m/interrupt.c:29`, section `.data_vector`) is read or written only by `knl_define_inthdr()` (`:73-89`, the `tk_def_int` path) and by the excluded `reset_hdl.c:54-61`. With the SDK owning VTOR the table is dead storage (48 words = 192 B: `N_SYSVEC` 16 + `N_INTVEC` 32, `sysdef.h:58-59`). Dispatch does not use it: PendSV and SysTick are entries 14/15 of the **SDK's** RAM vector table (`SDK/rp2_common/pico_runtime/runtime.c:40, 146-148`, 48 words copied from flash), which the port overwrites with `knl_dispatch_entry` and `knl_systim_inthdr` (plan B3 `port_boot.c`; symbols exist at `armv6m/dispatch.S:58-60` and `interrupt.c:60`). The kernel's SVC handler is a weak stub (`exc_hdr.c:62-66`). This confirms integration report item 15 as far as I can check: the kernel does not need `exchdr_tbl` when `tk_def_int` is never used (plan: "Never `tk_def_int`").
- F-4 ("use `.uninitialized_data` via `__uninitialized_ram`, not `.noinit`") still holds in Rev 5: plan B3 `port_fault.c` row, `ARCHITECTURE.md` D6, `docs/DEBUG.md` all say so. The macro exists: `SDK/rp2_common/pico_platform/include/pico/platform.h:235`. `.uninitialized_data` is at `memmap_mtk3.ld:191-194`, before `.noinit`, outside the zero range. No linker work for the fault record either.
- Thus nothing in B3 needs linker edits. Only fix the wording and delete "linker overrides" from `HANDOVER.md:190`.

### (ii) Gaps in the C7 workflow
1. **Temporary debug code for B4.** Plan B4 pushes debug-only code as a separate commit, the owner flashes it, the helper removes it before merge. Result: the merged commit is never the one that ran on hardware, and the round trip doubles (push, flash, report, remove, re-verify). Better: one CMake option (extend O1's `PLATFORM_BRINGUP`, default OFF, or a sibling `PLATFORM_SELFTEST`) that compiles in the late-`tm_printf` check (R3b allows `port_boot.c`) and the `PORT_FAULT_SELFTEST` forced fault. The owner builds with `-DPLATFORM_SELFTEST=ON`; nothing needs removing and the tested commit is the merged commit. This moves the shared `CMakeLists.txt` change from B5 to B4 (tell the team in the PR).
2. **Who commits logs, and where.** The plan says "Buddy 2 commits his own logs" (§2) into `testing/system/logs/`. If that targets the helper's branch it creates push-rights and rebase conflicts. Recommend: B4 logs go in a PR comment; B5 log is committed by the owner on `main` after the stage merges, and the helper cites its hash.
3. **Turnaround.** A hardware failure needs: owner posts log, helper fixes blind (no board, other compiler), pushes, owner reruns. Each loop costs two schedules. For B4 (first flash) and B3 (HardFault handler, SHPR, VTOR writes) allow the owner to push fixup commits to the stage branch or work in a live session.
4. **Bottleneck.** "The next stage starts from a passed one" (HANDOVER §0 item 4, plan §2 item 4) serialises everything, including B1-B3, which cannot run on hardware anyway. Allow stacked branches (B2 on top of B1) and verify in order; or merge B1-B3 (below).
5. **Branch and merge flow.** One branch and one PR per stage is fine. But "after each stage passes ... update HANDOVER §1 and §4" and "helper copies the summary into the stage's result block" happen after the owner's run, so each stage needs a docs-only follow-up commit. State the rule: docs-only deltas after verification do not need a re-run, and are included in the same PR before merge. Keep HANDOVER.md edits to one person (the helper in stage PRs) to avoid conflicts; `.agents/tasks` is now tracked, so the plan file is edited by every stage PR.
6. **Owner's own Batch B work.** HANDOVER §0 item 4 says nobody else starts Batch B, and C1 is "given verbally" (UNVERIFIED by any record). If the owner or an agent also writes Batch B code on another branch, `src/` collides (the directory does not exist yet: `git ls-tree` of `src/` is empty). Say explicitly that the owner will not write Batch B code while Jin Yong does, or split the stages.
7. **Better split (recommendation).** Put the hardware-sensitive stages (B3 port layer, B4 first flash) with the person who has the board, or pair on them. Give B1, B2 (pure kernel side, compile-only, no board needed) and the host-test work to Jin Yong. Note Jin Yong also owns subsystem 3 (`HANDOVER.md` §3), so every week as helper delays that work.

### (iii) O8: TA_USERBUF arena versus a simpler option
Facts:
- Kernel stack allocation: `stack = knl_Imalloc(stksz + DEFAULT_SYS_STKSZ)` rounded to 8 (`kernel/tkernel/task_manage.c:60-65`); `DEFAULT_SYS_STKSZ` = `MIN_SYS_STACK_SIZE` = 128 on armv6m (`include/sys/sysdepend/cpu/core/armv6m/sysdef.h:114,120`). With `TA_USERBUF` the caller's buffer is the whole stack, minimum 128 B (`task_manage.c:55-59`). There is a single stack per task: initial frame at `tcb->isstack = stack + sstksz` (`task_manage.c:93`, `armv6m/cpu_task.h:48-59`).
- Budget: `config/priorities.h:131-139` totals 20,480 B of task stacks (base+512 margin each) + 12 x (128+16) + 1,024 init = 23,232 B, plus 4,096 B MBF reserve = 27,328 B, against `KERNEL_POOL_BYTES` = 98,304 B (`:124`; check at `:145`). So the pool is about 28% used. Moving stacks out of the pool is not needed for fit, and it forces edits to a shared file (`priorities.h`) and its `#error` check.
- The init task stack (1 KB, `include/sys/inittask.h:26-27`) is always kernel-allocated, so option (a) leaves it unmeasurable (the plan admits this: "Still open in either case").

Simplest alternative (c), no `TA_USERBUF`, no `priorities.h` change: `app_start_task()` creates the task normally and passes the real entry through a small trampoline. At entry the trampoline takes `__builtin_frame_address(0)` as the approximate stack top (SP at first dispatch equals `isstack`, because the initial frame is fully popped), computes the bottom as top minus (`stksz` + 128, rounded to 8, same formula as `task_manage.c:63-64`), paints from just below its own frame down to the bottom, records top/bottom in a static table, then calls the real entry. B6 scans the table. `usermain()` does the same for the init task using 1,024 B, which resolves the open measurement question. Caveats: the top is approximate by a few dozen bytes (fine for a high-water margin), and it is UNVERIFIED on target until B4/B6; add a one-time check in B4. ISR frames landing on the task stack during painting are harmless (they sit below SP and are dead after return). Painting happens once at task start, so it costs microseconds.

Verdict: O8(a) works but costs more and covers less. Choose (c), or keep (a) only if the team wants static stacks for another reason.

### (iv) Shortened path
Already after M1 in the plan: B6 and B7. They cannot be on the critical path for "subsystems can add tasks". Other cuts:

| Plan item | Recommendation |
|---|---|
| B0 gate | No full owner gate needed: docs-only (no file under `check_rules` scan scope: `.agents/`, `docs/` are excluded at `check_rules.cmake:47`, and `HANDOVER.md`/`ARCHITECTURE.md` are not source). I ran it (§4). |
| B1 + B2 + B3 | Merge into one branch/PR (`batchB/b1-platform-code`), three commits. All three are compile-only and use the same gate. One owner round trip instead of three. Keep per-commit `check_rules` clean. |
| B4 | Keep. Minimum content: SDK `main()`, `hal_kernel_start()`, system-message ring + `dbg_` ring drained by the drain task, `ipc_init()`, `app_start_task()`. Success = "pre-kernel ok", kernel banner, "ipc_init OK" on USB CDC. |
| M1-lite | Declare "firmware links and boots; members may add tasks" after B4 passes. Subsystem owners can then start. |
| B5 heartbeats and 10-minute test | Keep as the Phase 1 acceptance, but run it after M1-lite, in parallel with subsystem work (use PLATFORM_BRINGUP, or simply members' real tasks once they exist). Skipping the 9-task heartbeat build is only possible if the team accepts changing the mandatory test in DEBUG.md; that is a team decision, not mine. |
| O2 types in B2 | Needed for `ipc_init()` (message sizes), keep, minimal Week 6 fields. |
| `MBF` depth self-check in `ipc_init` | Keep: cheap, catches size-macro errors. |
| B6 | After M1-lite; can overlap subsystem work. |
| B7 packing | After B6; host tests do not need the owner's PC or the SDK, so Jin Yong can run them locally and the owner only verifies the firmware rebuild. |
| Toolchain for the helper | Install the SDK 1.5.1 installer on Jin Yong's PC. Then B1-B3 and B7 are verified with the official toolchain locally and posted as evidence; the owner needs the PC only for B4 flashing and hardware tests. This removes most of the C7 bottleneck. |

Minimum path to "firmware links and boots, subsystem teams add tasks": B0 merge, then one platform-code PR (B1+B2+B3), then B4 flash. Everything else follows in parallel.

---

## 3. B0 diff review findings

`git diff 8d083ca fae43bf`: 4 files, 509 insertions, 43 deletions. `git diff --check` clean (exit 0).

| # | Severity | Finding |
|---|---|---|
| 1 | should-fix | `HANDOVER.md:190`: B3 row says "...system-message ring, **linker overrides**". Plan Rev 5 and the plan's B3 linker row say no linker work. HANDOVER §4 says the plan wins, but a future agent reading the table will plan linker work. Remove "linker overrides". |
| 2 | should-fix | `batchB-plan.md:20` (Rev 5 bullet) and the B3 table row: "`.data_vector (NOLOAD)` and `.noinit` inside the zero-filled `__bss_start__..__bss_end__` range" is false for `.data_vector` (it sits before `.data`, `memmap_mtk3.ld:126-137`; the zero range is `:219-231`). Reword: "`.noinit` is inside the zero-filled range; `.data_vector` holds the unused `exchdr_tbl` (SDK owns VTOR) and needs no zeroing". Also the cited range `:129-136` should be `:126-137`. |
| 3 | should-fix | O7 (plan §1 row `batchB-plan.md:104`, HANDOVER §7 summary): "halt if a debugger is attached (`DHCSR.C_DEBUGEN`)" is not implementable on Cortex-M0+, and `SYSRESETREQ` resets only the core on RP2040. Replace with: record, then `watchdog_reboot`; halt only when a RAM flag or build option says so. See §2. |
| 4 | should-fix | Provenance. `batchB-plan.md:90` says O1/O2/O4/O7/O8/O9 were accepted because "the helper answered for the team"; line 168 says "owner approved the plan's decisions (29 Oct 2026)"; HANDOVER §7 puts C7, C8 and the O answers under "Decisions already made (binding)" while its own C7/C8 bullets say "Buddy 2 confirms in the B0 review". Pick one truth. Suggest: keep them labelled "proposed by helper" until the owner's PR reply, then record "confirmed by Buddy 2, <date>, PR link". C1 is "verbal" with no record (UNVERIFIED). |
| 5 | should-fix | `batchB-plan.md:165`: "branch `batchB/b0-records`, **not committed**" is stale; it is committed as `fae43bf`. |
| 6 | nit | `batchB-plan.md:74`: firmware switch cited as `CMakeLists.txt:150`; the `if(NOT EXISTS .../src/app/main.c)` is at `CMakeLists.txt:161`. |
| 7 | should-fix | O2 text: "each field from DATA_REPRESENTATION §2" mixes Week 8 fields into the "Week 6 baseline" (D8 adds clearances, the bypass command and a sequence number, drops the timestamp). Specify the Week 6 set for B2 and add the Week 8 fields in B7. |
| 8 | should-fix (for B4, not B0) | Add to B4's stop conditions: the drain clears the fault record only after a print that was delivered (host connected), `stdio_usb.c:116-119`; needs a `hal_` connected query (R1). |
| 9 | nit | `docs/DEBUG.md:88-89` still says the drain prints the record "through the `dbg_` ring", while the plan (B4) prints directly from the drain. Align wording. |
| 10 | nit | O4: add a note that `0x464C5452` reads "RTLF" as bytes in memory; mention optional SP and task id. |
| 11 | nit | Plan keeps Rev 4 bullets (2.3.1 move "by Buddy 2", stage BM text) "for history". They are marked WITHDRAWN and BM says "DEFERRED", so the risk of a future agent following them is low. The Rev 4 phrase "Buddy 2 moved the project" is the one a careless reader could act on; it is already countered by the bold WITHDRAWN. Acceptable. |

Checked and fine:
- **Protected files:** `git diff --name-only` = `.agents/tasks/batchB-plan.md`, `ARCHITECTURE.md`, `HANDOVER.md`, `docs/DEBUG.md`. Nothing under `ProjectTemplate/`, `external/mtk3_bsp/`, `INF 2004 Embedded Systems.md`, `design.md`, `specs.md`, `README.md`, `.gitignore`, `NOTICE.md`, `TEAM.md`, `CMakeLists.txt`, `cmake/`, `config/`, `src/`, `app_program/` (`git diff --stat` on those paths returns nothing). The same-name-file gate (README/.gitignore need team review) is respected.
- **OWNER CORRECTIONS preserved:** D2 is not touched by the diff (still `TODO(owner-to-confirm, buddy 1)`); D6 still says magic first and CRC-32 over the payload, prints and clears only when both match, `.uninitialized_data` not `.noinit` (`ARCHITECTURE.md:108`); D7 NFR2 wording and the "zero missed counts needs a top-speed test" note are unchanged; D8 keeps the optional 2-bit sequence number. Names: only "Buddy 2 (Ruo Chen)" and "Buddy 3 (Jin Yong)" appear, matching HANDOVER §3 headings; the commit author field (`PhaJinYong`) is git metadata.
- **HANDOVER §6 rules** (never commit for someone else, never edit protected files, show README/.gitignore diffs first, keep `TODO(owner-to-confirm)` markers) are unchanged. C7 does not weaken them; the owner commits his own logs.
- **Commit status text** (`559f479`, `8d083ca` = `origin/main`) is true: `git log` shows `8d083ca` (main) over `559f479`.
- **Stage table vs plan:** rows B0, BM, B1, B2, B4-B7 match the plan; B3 mismatch is finding 1. Milestone wording M1/M2 matches plan §3.
- **Kernel/SDK citations spot-checked:** `kernel.h:215-216` (`knl_shutdown_hw`, `knl_restart_hw`), `sysinit.c:26`, `reset_hdl.c:61,100-102`, `exc_hdr.c:28,56,83`, `inittask.c:119`, `cpu_cntl.c:30`, `task_manage.c:55-60`, `profile.h:47`, `libtm.h:25-28`, `mtk3.cmake:15-18`, `drv_encoder.h` TODO: all match. Only `CMakeLists.txt:150` is off.
- **D6/DEBUG.md technical text:** "USB CDC cannot run from a HardFault or with interrupts masked (TinyUSB needs its IRQ and a 1 ms alarm)" matches `stdio_usb.c:51,79` and `PICO_STDIO_USB_TASK_INTERVAL_US 1000`. `tm_snd_dat` is the BSP hook (`libtm.h:27`); `tm_printf` goes through `tm_putstring` -> `tm_putchar` -> `tm_snd_dat` (`libtm_printf.c:284,292`, `libtm.c:93,110`). 512 B cannot overflow at boot (about 34 B). Nothing in the docs is overstated beyond findings 2 and 3.
- **Merge conflicts:** the branch is one commit ahead of `origin/main` and zero behind, so merging into `main` is a fast-forward. I did not run `git merge-tree --write-tree` because it writes objects into the owner's `.git`; `git merge-base --is-ancestor origin/main origin/batchB/b0-records` returned 0 instead. The owner's current tree (here) is on `batchB/b0-records`, HEAD `fae43bf`, `git status --porcelain` empty, so nothing uncommitted can collide. Caveat: that is this machine's clone, not necessarily the owner's `asus` PC (UNVERIFIED there; the plan mentions an untracked `session.md` on the owner's device).

---

## 4. Verification steps for the owner's PC

### ACTUAL results (scratch clone on this machine, SDK 1.5.1 + GCC 10.3.1, commit `fae43bf`, fresh dirs)
| Step | Result |
|---|---|
| Clone `--no-hardlinks`, `git submodule update --init` | OK; ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |
| G1 configure `build/b0` | exit 0. Lines: `robocar: DBG_ENABLE=ON DBG_BACKEND=USB TEL_BACKEND_WIFI=OFF APP_TEST=OFF`, `mtk3: external/mtk3_bsp pinned at 15ed232..., clean`, `robocar: src/app/main.c not present (Batch B); firmware target skipped, app_program compile-checked` |
| G2 build `build/b0` | exit 0, `[84/84]`, 0 warnings, `check_rules: PASS (42 files scanned)` |
| G3 `check_rules_selftest.cmake` | exit 0, `check_rules_selftest: PASS` |
| G4/G5 `build/b0-test` (`APP_TEST=ON`) | configure exit 0; build exit 1, 5 targets FAILED (`subsystem1..5_test.c`), each only `test_report.h:35:2: error: #error "APP_TEST=ON needs the dbg_ ring API: TODO(phase 1 batch B, buddy 2)"` |
| G6/G7 `build/b0-rel` (Release) | configure exit 0 (`DBG_ENABLE=OFF`, same "firmware target skipped" line); build exit 0, `[84/84]`, 0 warnings, `check_rules: PASS (42 files scanned)` |
| `git status --short` in scratch | empty (no tracked changes) |
| Protected-path `git diff --stat 8d083ca fae43bf` | empty |
Scratch clone and temp scripts deleted afterwards; the owner repo was never modified (`git status -sb` still `## batchB/b0-records...origin/batchB/b0-records`).

### Steps (copy into PowerShell on the owner's PC, from the repo root)
Gate expectation for a docs-only stage: identical to the base commit `8d083ca`. The only gate that cannot run until code exists is the firmware link (reported as "firmware target skipped").

1. Inspect without touching local work:
```powershell
git status --short                      # expect empty (an untracked session.md is fine); stop if anything is modified
git fetch origin
git log --oneline -3 origin/batchB/b0-records     # fae43bf, 8d083ca, 559f479
git rev-list --left-right --count origin/main...origin/batchB/b0-records   # expect: 0   1
git diff --name-only origin/main origin/batchB/b0-records   # exactly 4 files: .agents/tasks/batchB-plan.md, ARCHITECTURE.md, HANDOVER.md, docs/DEBUG.md
git diff --stat origin/main origin/batchB/b0-records -- ProjectTemplate external design.md specs.md "INF 2004 Embedded Systems.md" README.md .gitignore CMakeLists.txt cmake config src app_program   # expect no output
git diff --check origin/main origin/batchB/b0-records ; $LASTEXITCODE            # expect 0
```
2. Get the code without disturbing your branch (separate worktree, or a throwaway clone):
```powershell
git worktree add ..\inf-b0 origin/batchB/b0-records --detach
Set-Location ..\inf-b0
git submodule update --init
git submodule status        # expect:  15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)   (leading space, not - or +)
```
   If you prefer switching: with a clean tree, `git switch batchB/b0-records` (it tracks origin). Clean up later with `git worktree remove ..\inf-b0`.
3. Environment (HANDOVER §5 step 3, exactly):
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
& "$R\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe" --version | Select-Object -First 1   # expect 10.3.1 (this also settles the SDK-version question)
```
4. Debug configure and build (fresh `build/b0`):
```powershell
& $CM -S . -B build/b0 @common -DDBG_BACKEND=USB     # exit 0; mtk3 ... pinned at 15ed232..., clean; "firmware target skipped, app_program compile-checked"
& $CM --build build/b0                                # exit 0; [84/84]; no "warning"; check_rules: PASS (42 files scanned)
```
5. Rule-check self-test:
```powershell
& $CM -P cmake/check_rules_selftest.cmake            # exit 0; check_rules_selftest: PASS
```
6. APP_TEST build (expected to fail until B1):
```powershell
& $CM -S . -B build/b0-test @common -DDBG_BACKEND=USB -DAPP_TEST=ON   # exit 0
& $CM --build build/b0-test                                            # exit 1 by design: 5 FAILED (subsystem1..5_test.c), each only test_report.h:35 #error. Any other error is a defect.
```
7. Release (DBG_ENABLE=OFF):
```powershell
& $CM -S . -B build/b0-rel @common -DCMAKE_BUILD_TYPE=Release   # exit 0; DBG_ENABLE=OFF
& $CM --build build/b0-rel                                       # exit 0; [84/84]; 0 warnings; check_rules: PASS (42 files scanned)
```
8. Worktree state and cleanup:
```powershell
git status --short          # expect empty (build/ is ignored)
Set-Location <your repo>; git worktree remove ..\inf-b0 --force   # only after saving any logs
```
9. Merge (after the small fixes are pushed, or as is): `git switch main; git pull --ff-only; git merge --ff-only origin/batchB/b0-records` (fast-forward is possible; verified `0 1`). Do not push until you are ready.

Predicted-versus-actual: all outputs above were obtained on a SDK 1.5.1 machine; the only difference expected on the owner's PC is the path prefix. Build step counts (84) are the same as the base (`batchB-plan.md` §0 baseline: `build/fresh` 84/84). The plan's 97/97 figures come from the helper's 2.3.1 + GCC 15 setup and must not be expected on the official one.

---

## 5. Verdict and PR reply

**Merge after small fixes.** Fix list (all docs; Jin Yong can push to the same branch):
1. `HANDOVER.md:190`: delete ", linker overrides" from the B3 row.
2. `.agents/tasks/batchB-plan.md:20` and the B3 `linker script` table row: reword the `.data_vector` claim as in §3 finding 2; cite `memmap_mtk3.ld:126-137, 217-231, 275`.
3. `.agents/tasks/batchB-plan.md` §1 O7 row and the HANDOVER §7 O7 text: replace "halt if a debugger is attached (`DHCSR.C_DEBUGEN`)" with "record, then `watchdog_reboot`; halt only if a RAM flag or build option is set".
4. `.agents/tasks/batchB-plan.md:90`, `:168` and HANDOVER §7 heading: make the provenance of C1, C7, C8 and O1-O9 consistent; mark "confirmed by Buddy 2" with the date and PR link only after the owner replies.
5. `.agents/tasks/batchB-plan.md:165` ("not committed") and `:74` (`CMakeLists.txt:150` to `:161`).
6. Plan O2 row: baseline = Week 6 field set; Week 8 fields arrive in B7.
7. (Recommended, can go in the B1 PR instead) Plan B4: clear the fault record only after a delivered print, via a `hal_` connected query; replace the "debug-only commit" with a default-OFF CMake option; adopt the simpler stack painting (§2a iii) or keep O8(a) knowingly.

Reply to post on the PR (plain text):

Hi Jin Yong, thanks for B0. I ran the plan section 2 gates on fae43bf in a fresh clone with the official SDK 1.5.1 and GCC 10.3.1. Debug builds 84/84 with 0 warnings and check_rules passes on 42 files. The self-test passes. APP_TEST=ON stops only at the test_report.h:35 #error, as expected. Release builds with 0 warnings, so the GCC 15 warnings you saw in libtm_printf.c do not appear with our compiler. The diff touches only the four docs files and the branch fast-forwards onto main, so I can merge it once a few lines are fixed.

I agree with keeping SDK 1.5.1 and deferring BM, and with C3 and the system-message ring. Please fix these before I merge: (1) HANDOVER.md line 190 still lists "linker overrides" for B3, but Rev 5 says no linker work. (2) In the plan, the Rev 5 sentence says .data_vector is inside the zero-filled range. It is not (memmap_mtk3.ld lines 126-137, before .data). Only .noinit is zeroed. Please say that .data_vector is unused because the SDK owns VTOR. (3) O7 cannot be done as written. Cortex-M0+ code cannot read DHCSR, and SYSRESETREQ resets only the core on RP2040. Please change it to record, then watchdog_reboot, with a RAM flag or build option for halting under the debugger. (4) Please mark C1, C7, C8 and the O answers as proposed until I confirm them here; I confirm C7, C8 and O1, O4, O5, O6, O9 with the notes below. (5) Remove the stale "not committed" line and change CMakeLists.txt:150 to :161. (6) For O2, please base the first ipc_types.h on the Week 6 field set, and add the Week 8 fields in B7.

For the next PRs: please install the SDK 1.5.1 installer on your PC too, so you can run the official gates yourself and I only need to run the hardware stages. I would like B1, B2 and B3 in one PR, since none of them can run on the board. For O8 please try a simpler way: keep kernel-allocated stacks, paint the stack from a start-up trampoline, and do the same in usermain for the init task. For B4, please use a default-OFF CMake option for the forced fault and the late tm_printf instead of a temporary commit, and clear the fault record only after it was really printed to a connected host. Thanks, Ruo Chen

---

## 6. Questions for the owner (with recommended answers)

1. **Which SDK/GCC is on your own PC (the `asus` machine)?** Recommended: confirm 1.5.1 + 10.3.1 with the two commands in §1. If it is 2.3.1, install the 1.5.1 installer side by side instead of running BM.
2. **Do you confirm C1 (Jin Yong as helper), C7 and C8 in the PR?** Recommended: yes for C1 and C8; yes for C7 with the changes in §2a(ii): helper installs SDK 1.5.1, B1-B3 merged into one PR, owner may push fixups to B3/B4 branches.
3. **Will you write any Batch B code yourself while Jin Yong does?** Recommended: no; instead, you or you plus an agent take B3/B4 (hardware-sensitive), Jin Yong takes B1/B2 and later B7 host tests.
4. **O7 reset path:** watchdog reboot with a RAM flag for halting? Recommended: yes.
5. **O8:** trampoline painting (no `priorities.h` change) or the arena? Recommended: trampoline.
6. **O2 baseline fields:** Week 6 set now, Week 8 fields in B7? Recommended: yes.
7. **The mandatory 10-minute USB CDC test:** run it after "firmware boots" (M1-lite) in parallel with subsystem work? Recommended: yes; it changes the timing of the acceptance, not the test, so the team should be told.
8. **Fault-record content:** add SP and current task id (and keep boot count or drop it)? Recommended: add SP and task id, drop boot count unless it is stored outside the cleared record.
9. **Does the team accept a watchdog enabled in B5 or later to cover LOCKUP from stack overflow?** Recommended: yes, as a follow-up item, not in the critical path.
