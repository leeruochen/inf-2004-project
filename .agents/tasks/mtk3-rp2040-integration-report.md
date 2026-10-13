# KERNEL INTEGRATION REPORT: micro T-Kernel 3.0 (mtk3_bsp, pico_rp2040) in a CMake/Ninja Pico SDK 1.5.1 project

Source investigated: `github.com/tron-forum/mtk3_bsp`, branch `pico_rp2040`, commit
**`15ed232c08f2d89e54e514db79de150e224307e3`** (2023-05-24, "Update manual"). This hash was confirmed with
`git ls-remote` and is the value to pin the submodule to. The SDK was read on disk at
`C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk` (`pico_sdk_version.cmake` = 1.5.1). The installer
bundles GCC `10.3-2021.10 (10.3.1)`, CMake 3.25.2 and Ninja 1.11.1.

All BSP paths below are relative to the BSP root. SDK paths are relative to `pico-sdk/`.
Trial compiles were run in `%TEMP%` (deleted afterwards). Nothing in the project was modified except this file.

---

## 0. Summary answer

| Question | Answer |
|---|---|
| Does the BSP use the Pico SDK? | **No.** It is fully bare-metal: its own boot2 blob, vector table, reset handler, clock/PLL setup, linker script and register macros. No SDK header is included and `PICO_SDK_PATH` is not needed. |
| Who owns startup in the BSP? | The kernel. `Reset_Handler` (`kernel/sysdepend/cpu/core/armv6m/reset_hdl.c`) calls the kernel's own `main()` in `kernel/sysinit/sysinit.c`. |
| Tick | **SysTick**. The configured value is **`CNF_TIMER_PERIOD 10` (10 ms)** in `config/config.h:41`. The allowed range is 1 to 50 ms (`include/sys/sysdepend/cpu/rp2040/sysdef.h:51-52`). **1 ms is supported.** We must set it to 1. |
| Console | `tm_printf` goes to **UART0 polled, GP0/GP1, 115200 8N1**. **Interrupts are masked while each character, and each whole `tm_putstring`, is sent.** |
| Interrupts | `tk_def_int()` writes into a kernel-owned RAM vector table (`exchdr_tbl`). Critical sections use **PRIMASK** (all IRQs masked). There is no BASEPRI on M0+. NVIC levels are 0 to 3. SysTick and SVC are at 0, PendSV at 3. |
| Fits our object counts by default? | **No.** `CNF_MAX_CYCID 4` but we need 5 cyclic handlers. `CNF_MAX_MBFID 4` and we need exactly 4, which leaves no headroom. Both must be raised. |
| Msg-buffer overhead | 4-byte `INT` header per message, and the payload is rounded up to 4. `bufsz = N × (4 + ROUNDUP4(sizeof(T)))`. |
| Builds under GCC 10.3 / gnu11 / -Wall -Wextra? | **Yes.** 58 files gave 0 errors and 70 warnings. So `-Werror` must stay off the kernel. |
| Pico W clash | The BSP drives **GP25 as an LED output**. On a Pico W, GP25 is **CYW43 SPI CS**. The BSP also muxes GP8/9 to I2C0 (our motor pins) and GP28 to ADC (our barcode input). |
| **Recommendation** | **(a) The SDK owns crt0, boot2, clocks, linker and vector table.** The kernel is built as a static library with its startup and board files removed, and is bound through a thin port layer. **Alternative (b):** the BSP owns startup and we link selected `hardware_*` only. (b) breaks `clock_get_hz()`, USB CDC and any future CYW43. |
| USB CDC stdio | Probably workable for **output funnelled through one task**. It needs a hardware bring-up test (procedure in §14). The fallback is UART0 on GP0/GP1. |
| WiFi (report only) | `pico_cyw43_arch_lwip_poll` exists. `lib/lwip/src/apps/mqtt/mqtt.c` exists, with CMake target `pico_lwip_mqtt`. |

---

## 1. Folder layout of the branch

| Path | Contents |
|---|---|
| `kernel/tkernel/` | The portable kernel: `task*.c`, `semaphore.c`, `eventflag.c`, `mutex.c`, `messagebuf.c`, `mailbox.c`, `mempool.c`, `mempfix.c`, `timer.c`, `time_calls.c`, `int.c`, `device*.c`, `memory.c`, `tkinit.c`, `wait.c`, `power.c`, `objname.c`, `rendezvous.c`, `misc_calls.c`, `klock.c` and headers. |
| `kernel/sysinit/sysinit.c` | The kernel's `main()`, which is the boot sequence. |
| `kernel/inittask/inittask.c` | The initial task. It calls `usermain()`. |
| `kernel/usermain/usermain.c` | A weak default `usermain()`. |
| `kernel/tstdlib/` | `bitop.c`, `string.c`. |
| `kernel/knlinc/` | Kernel-internal headers (`kernel.h`, `tstdlib.h`, ...). |
| `kernel/sysdepend/cpu/core/armv6m/` | Cortex-M0+ port: `dispatch.S`, `reset_hdl.c`, `interrupt.c`, `cpu_cntl.c`, `exc_hdr.c`, `sys_timer.h`, `cpu_status.h`, `cpu_task.h`, `offset.h`. |
| `kernel/sysdepend/cpu/rp2040/` | `vector_tbl.c`, `boot_2nd.c` (a 256-byte boot2 blob), and thin headers. |
| `kernel/sysdepend/pico_rp2040/` | Board files: `cpu_clock.c`, `hw_setting.c`, `devinit.c`, `power_save.c`. |
| `kernel/sysdepend/{iote_*,nucleo_*,rsk_*,rtb_*}`, `cpu/{stm32*,rx*,...}` | Other targets. Not used. |
| `include/tk/` | Public API: `tkernel.h`, `syscall.h`, `typedef.h`, `errno.h`, `syslib.h`, plus `sysdepend/cpu/rp2040`, `sysdepend/cpu/core/armv6m`. |
| `include/sys/` | `machine.h` (selects the target by `_PICO_RP2040_`), `sysdef.h`, `knldef.h` (config checks), `inittask.h`, `queue.h`. |
| `include/tm/tmonitor.h` | The `tm_printf` API. |
| `include/bsp/` | `libbsp.h`. GPIO/PWM helpers for rp2040. |
| `config/` | `config.h`, `config_func.h`, `config_tm.h`, `config_device.h`. User configuration. |
| `lib/libtk/` | `kmalloc.c`, `fastlock.c`, `fastmlock.c`, plus `sysdepend/cpu/rp2040/{int_rp2040.c,ptimer_rp2040.c}` and `sysdepend/cpu/core/armv6m/{int_armv6m.c,wusec_armv6m.c}`. |
| `lib/libtm/` | `libtm.c`, `libtm_printf.c`, `sysdepend/pico_rp2040/tm_com.c` (UART0 console). |
| `lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c` | GPIO/PWM helpers. |
| `device/{ser,adc,i2c}/` (+ `sysdepend/rp2040/`), `device/common/drvif/`, `device/include/` | Sample device drivers that sit on the T-Kernel device manager. |
| `etc/linker/pico_rp2040/tkernel_map.ld` | The linker script. |
| `app_program/app_main.c` | Sample app: blinks GP25. |
| `.cproject`, `.project`, `.settings/` | Eclipse CDT managed build. |
| `build_make/` | GNU make files. **There is no rp2040 .mk**: `build_make/` has only `iote_*`, `nucleo_h723.mk`. So the Eclipse `.cproject` is the only build description for rp2040. |
| `docs/` | `uTK3bsp_pico_man_jp.pdf` (Japanese BSP manual, not read: UNVERIFIED content), `TEF000-219-200401.pdf` (the T-License 2.2 text, per tron.org), other boards' manuals. |

**Licence.** There is **no LICENSE/COPYING file** in the repo (`git ls-files` showed none). The licence is stated in every source header ("distributed under the T-License 2.2", e.g. `reset_hdl.c:5-6`), and the full text is `docs/TEF000-219-200401.pdf`. Vendoring the whole branch as a submodule keeps both. T-License 2.2 Art. 3(6) requires displaying that the source is used. tron.org publishes the method (information_id=91). The exact wording to put in README is UNVERIFIED and should be checked on tron.org.

## 2. Startup code and vector table

- **Vector table:** `kernel/sysdepend/cpu/rp2040/vector_tbl.c` holds `vector_tbl[]` in section `.vector`. It has 16 system entries plus 32 IRQs. Entry 0 = `INITIAL_SP` (= `INTERNAL_RAM_END` = 0x20040000, `cpu/rp2040/sysdef.h:43`). 1 = `Reset_Handler`. 11 = `Svcall_Handler`. 14 (PendSV) = `knl_dispatch_entry`. 15 (SysTick) = `knl_systim_inthdr`. Every IRQ = `Default_Handler`.
- **Entry symbol:** `ENTRY(Reset_Handler)` (`tkernel_map.ld:1`).
- **`Reset_Handler`** (`armv6m/reset_hdl.c:43-107`) runs in this order:
  1. `knl_startup_hw()` (clocks, resets, pin mux).
  2. If `!USE_STATIC_IVT`, it copies the ROM table into the RAM `exchdr_tbl` and sets `SCB_VTOR` (l.51-62).
  3. `.data` copy and `.bss` zero.
  4. Sets the Imalloc area `knl_lowmem_top = max(SYSTEMAREA_TOP, __bss_end)` and `knl_lowmem_limit = INTERNAL_RAM_END - EXC_STACK_SIZE` (l.82-98).
  5. Writes the SHPR2/SHPR3 priorities: SVC = 0, SysTick = 0, PendSV = 3 (l.100-102).
  6. Calls `main()` (the kernel's, `sysinit.c:26`).
- **The kernel owns the vector table** (both the ROM table and the RAM copy `exchdr_tbl[48]` in section `.data_vector`, `armv6m/interrupt.c:29`). There is no SDK crt0.
- **boot2:** `kernel/sysdepend/cpu/rp2040/boot_2nd.c` is a precompiled 256-byte array in `.boot2`. The linker asserts that it is 256 bytes (`tkernel_map.ld:14`). Which flash chip variant it was built for is UNVERIFIED (it is only a byte blob). The SDK uses `PICO_BOOT_STAGE2_CHOOSE_W25Q080` for both pico and pico_w (`boards/pico_w.h:65`).
- **Clock init:** `kernel/sysdepend/pico_rp2040/cpu_clock.c` `startup_clock()` sets XOSC 12 MHz, PLL_SYS 1500/6/2 = **125 MHz**, PLL_USB 480/5/2 = 48 MHz, clk_peri = clk_sys, clk_usb/adc = 48 MHz, clk_rtc = 46875 Hz (`pico_rp2040/sysdef.h:39-58`). This is the same as the SDK default (`hardware_clocks/include/hardware/clocks.h:122-130`).
- **`knl_startup_hw()`** (`pico_rp2040/hw_setting.c`):
  - Unresets IO_BANK0, PADS_BANK0, UART0. It unresets PWM if `USE_PTMR`. It unresets ADC and I2C0 if `USE_SDEV_DRV`.
  - Pin mux: **GP25 is set to SIO output (LED)**, GP0/1 to UART0. If `USE_SDEV_DRV` (default **1**), it also sets **GP26/27/28 to ADC** and **GP8/9 to I2C0 with pull-ups**.

## 3. Linker script

`etc/linker/pico_rp2040/tkernel_map.ld`:

- `ROM 0x10000000, 2048K`. `RAM 0x20000000, 256K` (l.3-6). There are **no SCRATCH_X/Y**, so the top 8 KB of SRAM (0x20040000 to 0x20042000) is unused.
- Section order:
  - `boot2` (256 B).
  - `.text`: `.vector` first, then all `.text*` and `.rodata*`.
  - `.ARM.extab`, `.ARM.exidx`.
  - `vector 0x20000000 (NOLOAD)` = `.data_vector` (the RAM vector table at the bottom of RAM).
  - `.data AT>ROM`.
  - `.bss` including `.noinit` (l.55-66).
- Symbols: `__data_org/start/end`, `__bss_start/end`, `__noinit_*`, `__end`.
- The stack is at the top of RAM (`INITIAL_SP`). The exception stack is the top `CNF_EXC_STACK_SIZE` = 2048 B. The Imalloc heap runs from `__bss_end` to `RAM_END - 2048`.

Differences from the SDK `src/rp2_common/pico_standard_link/memmap_default.ld`:

| SDK feature | BSP script |
|---|---|
| `ENTRY(_entry_point)`, `.binary_info` (picotool) | missing |
| `.ram_vector_table`, `.time_critical*` placed in RAM (l.120-128) | missing |
| `__preinit/__init/__fini_array`, `__mutex_array` (needed by `runtime_init`, `runtime.c:98-176`) | missing |
| SCRATCH_X/Y, `__StackTop/__StackLimit/__HeapLimit`, `end` (newlib `_sbrk`, `runtime.c:192-209`) | missing |
| `.data_vector`, `.noinit` | **SDK has neither**. They would be orphan sections. |

**Flash size:** Pico and Pico W are both 2 MB (`boards/pico.h:73-74`, `boards/pico_w.h:72`), so the BSP's 2048K matches.

## 4. Use of the Pico SDK

- No SDK headers or libraries are used. All registers are hand-defined in `include/sys/sysdepend/cpu/rp2040/sysdef.h` (CLOCKS, RESETS, IO_BANK0, PADS, XOSC, PLL, SIO, TIMER, PWM) and the driver `*_sysdep.h` files. `PICO_SDK_PATH` is not referenced. No SDK files are copied.
- **Name clashes if the kernel and SDK headers meet in one translation unit.** A trial scan found 16 macro clashes:
  - `CLOCKS_BASE`, `IO_BANK0_BASE`, `IO_QSPI_BASE`, `PADS_BANK0_BASE`, `PLL_SYS_BASE`, `PLL_USB_BASE`, `PWM_BASE`, `RESETS_BASE`, `SIO_BASE`, `TIMER_BASE`, `XOSC_BASE` (vs `hardware/regs/addressmap.h`)
  - `GPIO_IN`, `GPIO_OUT` (vs `hardware/gpio.h`)
  - `PICO_RP2040` (vs `pico/platform.h`)
  - `XOSC_MHz`, `XOSC_KHz` (vs `platform_defs.h`)
  - Function clashes: `libbsp` `pwm_set_wrap` and `pwm_set_enabled` vs `hardware/pwm.h:254,488`.
  - **Rule:** a translation unit includes either `<tk/tkernel.h>` or SDK headers, never both. Do not link `libbsp_pwm.c`.
- **Newlib/libgcc:** the 219 kernel global symbols have **no clash with `libc.a`** (checked with `nm`). The kernel needs `__aeabi_idiv/uidiv/idivmod/uidivmod`. With the SDK, `pico_divider` provides these, and its divider state is saved lazily per call, which is safe under preemption (`pico_divider/divider.S:97-101`).

## 5. Interrupt registration

- **API:** `ER tk_def_int(UINT intno, CONST T_DINT *pk_dint)` with `T_DINT { ATR intatr; FP inthdr; }` (`include/tk/syscall.h:352`). `intno` is the NVIC IRQ number 0 to 31 (`CHECK_PAR(intno < N_INTVEC)`, `kernel/tkernel/int.c:35`). Attributes are `TA_HLNG` or `TA_ASM` (`int.c:37`).
  - With `TA_HLNG`, `knl_define_inthdr()` stores the handler in `knl_inthdr_tbl[]` and puts `knl_hll_inthdr` in `exchdr_tbl[16+intno]` (`armv6m/interrupt.c:73-89`).
  - `knl_hll_inthdr` does `ENTER_TASK_INDEPENDENT` (`knl_taskindp++`), then `handler(intno)`, then `LEAVE_TASK_INDEPENDENT` (l.40-53). The handler signature is `void h(UINT intno)`.
- **Enable and priority:** `EnableInt(intno, level)`, `DisableInt`, `ClearInt`, `CheckInt` (`lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c`). The priority is written as `level << 6` (`INTPRI_GROUP`, `armv6m/syslib.h:55`; `INTPRI_BITWIDTH 2`). Levels are 0 to 3.
  - `INTPRI_MAX_EXTINT_PRI 1`, SVC 0, SysTick 0, PendSV 3 (`cpu/rp2040/sysdef.h:71-74`).
  - The BSP drivers use level 2 (`DEVCNF_*_INTPRI 2`).
- **Critical sections:** `BEGIN_CRITICAL_SECTION` = `disint()` (sets PRIMASK). `END_CRITICAL_SECTION` restores PRIMASK and, if a reschedule is needed, pends PendSV via `knl_dispatch()` (`armv6m/cpu_status.h:29-35`; `cpu_cntl.c:136-139`). `ENABLE_INTERRUPT_UPTO` is empty ("ARMv6M does not support CPU interrupt mask levels", l.49-54). **So every kernel critical section masks all IRQs.** No priority is "reserved" from the kernel, but nothing preempts a kernel critical section.
- **The SysTick handler runs the whole timer queue, including every cyclic handler, inside a critical section** (`kernel/tkernel/timer.c:146-182`; `time_calls.c:185-193`). Cyclic handlers therefore run with IRQs masked. This fits the "handler only wakes the task" rule, and handlers must stay minimal.
- **ISR-safe calls** (from the context checks):
  - `tk_wup_tsk`: only `CHECK_TSKID` and `CHECK_NONSELF` (`task_sync.c:208`). OK.
  - `tk_sig_sem`: OK (`semaphore.c:138`).
  - `tk_set_flg`: OK (`eventflag.c:133`).
  - `tk_snd_mbf`: OK only with `TMO_POL` (`CHECK_DISPATCH_POL`, `messagebuf.c:282`).
  - `tk_wai_flg`: **E_CTX in an ISR** (`CHECK_DISPATCH`).
- **Gotcha:** `tk_clr_flg(id, clrptn)` **ANDs** with `clrptn` (`eventflag.c:203`: `flgptn &= clrptn`). To clear IMU_ALIVE, call `tk_clr_flg(flg, ~IMU_ALIVE)`.
- **Coexistence with the SDK:** the SDK `irq_set_exclusive_handler` and `irq_add_shared_handler` write into whatever table `scb_hw->vtor` points at (`hardware_irq/irq.c:30-44`). `tk_def_int` writes only `exchdr_tbl`. They agree only if VTOR = `exchdr_tbl`.
  - An SDK-installed handler does **not** increment `knl_taskindp`. If it calls a kernel service, the kernel thinks it is in a task (e.g. `CHECK_NONSELF` compares against `knl_ctxtsk`), which gives wrong context checks.
  - **So every ISR that calls the kernel must run inside a wrapper that does exactly what `knl_hll_inthdr` does.** Dispatch is deferred to PendSV, which is the lowest priority, so the wrapper does not need anything more.
  - **IO_IRQ_BANK0 (13) is one IRQ for all GPIOs** (encoders GP6/GP7, echo GP5, barcode GP28, and later CYW43 on GP24). It must stay owned by the SDK's shared-handler chain (`gpio.c:190-215`). Never register it with `tk_def_int`.

## 6. Tick source and period

- Source: **SysTick, core clock** (`armv6m/sys_timer.h:26-43`): `SYST_RVR = TIMER_PERIOD * TMCLK_KHz - 1`, with `TMCLK_MHz 125` (`pico_rp2040/sysdef.h:40`).
- **The actual configured value is 10 ms** (`config/config.h:41 CNF_TIMER_PERIOD 10`). The range check is in `include/sys/knldef.h:28-30`.
- **1 ms is supported.** RVR = 124 999 fits in 24 bits. The SDK does not use SysTick (a grep of `src/rp2_common` and `src/common` found only the weak `isr_systick` in `crt0.S:101`).
- The kernel adds `TIMER_PERIOD` to every timeout (`timer.c:107,119`). With a 1 ms tick, a 50 ms wait lasts 50 to 51 ms.

## 7. Console driver

- `tm_printf`/`tm_putstring` → `lib/libtm/libtm.c` → `tm_snd_dat()` in `lib/libtm/sysdepend/pico_rp2040/tm_com.c`.
- Raw **UART0** at 0x40034000, polled. `IBRD 67, FBRD 52` at clk_peri 125 MHz gives **115200 baud**. `LCR_H 0x70` = 8N1 with FIFO (l.59-65). The pins GP0/GP1 are muxed in `hw_setting.c` (not in tm_com).
- **Interrupts are disabled per character in `tm_putchar` and for the entire string in `tm_putstring`** (`libtm.c:80-112`). A 60-character line at 115200 is about 5 ms with all IRQs masked. **Do not use `tm_printf` at runtime.** Use it only for kernel boot and fault messages.
- Redirect: `config_tm.h` `TM_COM_SERIAL_DEV 1 / TM_COM_NO_DEV 0`. To redirect, replace `tm_com.c` (provide `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`). Our `dbg_` backend should be separate (buffered and drained by a task).

## 8. Memory and stacks

- Defaults in `config/config.h`:
  - `CNF_MAX_TSKID 32`, `SEMID 16`, `FLGID 16`, `MBXID 4`, `MTXID 4`, **`MBFID 4`**, `MPLID 4`, `MPFID 4`, **`CYCID 4`**, `ALMID 4`, `CNF_MAX_TSKPRI 32`.
  - `CNF_EXC_STACK_SIZE 2048`, `CNF_TMP_STACK_SIZE 256`.
  - `USE_IMALLOC 1`, `USE_STATIC_IVT 0`, `USE_SDEV_DRV 1`, `USE_PTMR 0`, `USE_TMONITOR 1` (l.39-156).
- **Task stacks:** without `TA_USERBUF`, the kernel calls `knl_Imalloc(stksz + DEFAULT_SYS_STKSZ(128))`, rounded to 8 (`task_manage.c:55-67`). With `TA_USERBUF`, `bufptr` is used and must be at least 128 (`MIN_SYS_STACK_SIZE`, `armv6m/sysdef.h:114`). Message buffers follow the same pattern (`messagebuf.c:143-165`).
- **Tasks run in Thread mode on MSP** (initial `EXC_RETURN 0xFFFFFFF9`, `armv6m/cpu_task.h` `knl_setup_context`; the dispatcher swaps `sp`, `dispatch.S:119-141`). So **ISR and SysTick frames, and nested IRQs, are stacked on the running task's stack.** Every task stack must budget for worst-case ISR nesting. A suggested floor is task need + 256 B.
- The initial task has priority 1 and a 1 KB stack (`include/sys/inittask.h:25-33`).
- **Our needs vs defaults:**

| Object | Needed (design.md 3/3.3) | Default | Verdict |
|---|---|---|---|
| Tasks | 9 + init task | 32 | OK |
| Cyclic handlers | 5 (Motion 20 ms, Line 5 ms, ObsScan, IMU 10 ms, Telemetry 20 ms) | **4** | **Too few. Set 8.** |
| Message buffers | 4 (NAV_IN, MOTION_CMD, OBSTACLE, TELEMETRY) | **4** | No headroom. Set 8. |
| Eventflags | 2 | 16 | OK |
| Semaphores | 1 | 16 | OK |
| Mutex (TA_INHERIT, `syscall.h:77`) | 1 | 4 | OK |

- Footprint with the proposed config (core kernel only, 43 objects): text ≈ 25.8 KB, data 192 B, bss ≈ 7.3 KB (trial `arm-none-eabi-size`).

## 9. Message buffer per-message overhead

From `kernel/tkernel/messagebuf.h:58-71` and `messagebuf.c:57-86,134-165`:

- Header = `typedef INT HEADER` → **4 bytes per message**.
- The payload is stored rounded up: `ROUNDSZ(sz) = (sz+3) & ~3`. Each message consumes `HEADERSZ + ROUNDSZ(msgsz)` (l.65).
- `bufsz` is rounded up to 4. With `TA_USERBUF`, a non-multiple of 4 returns `E_PAR` (l.146).
- The admission check is `HEADERSZ + msgsz <= frbufsz` (`knl_mbf_free`).

**Formula:** `MBF_BUFSZ(N, T) = N * (4 + ((sizeof(T) + 3) & ~3))`, with `maxmsz = sizeof(T)`.

For example, if `sizeof(nav_input_t) = 8`, then 8 messages need 96 B, not 64.

**Startup assert:** after `tk_cre_mbf`:
1. `tk_ref_mbf` must report `frbufsz == bufsz`.
2. Send N dummy messages with `TMO_POL`. All must return `E_OK`.
3. Message N+1 must return `E_TMOUT`.
4. Drain with `tk_rcv_mbf(TMO_POL)`.

Plus a compile-time `_Static_assert` on the macro.

## 10. User entry point

The boot chain is:

`Reset_Handler` → `main()` (`sysinit.c:26`: `DISABLE_INTERRUPT`, `libtm_init`, `knl_init_Imalloc`, `knl_init_device`, `knl_init_interrupt`, `knl_init_object`, `knl_timer_startup` (starts SysTick), `tk_cre_tsk/tk_sta_tsk(knl_init_ctsk)`, `knl_force_dispatch`) → `init_task_main` (`inittask.c:110`: `knl_initialize_devmgr`, `knl_start_device`, banner) → **`usermain()`**.

- The default is `WEAK_FUNC usermain` in `kernel/usermain/usermain.c`. The sample override is `app_program/app_main.c`, which creates a task and then `tk_slp_tsk(TMO_FEVR)`.
- **If `usermain` returns, `shutdown_system()` runs and the system stops** (`inittask.c:126-138`). Our `usermain` must create all objects and tasks, then sleep forever.
- Because the init task has priority 1, the tasks it starts (P1 Safety ties with it, the rest are lower) do not run until it sleeps. So all objects exist before any task runs.

## 11. Building it as a CMake static library

**Eclipse settings** (`.cproject`):
- `-D_PICO_RP2040_` only (l.51, 62).
- `-I config -I include -I kernel/knlinc` (l.54-56, 65-67).
- `-std=gnu99` (l.69), `-mcpu=cortex-m0plus -mthumb`, `-ffreestanding`, `-fsigned-char`, `-Wall`, Debug `-O0`.
- `-nostartfiles -T etc/linker/pico_rp2040/tkernel_map.ld` (l.75-78).
- The assembler uses the preprocessor (for `dispatch.S`).
- The `.cproject` has no source-exclusion entries, and every file is guarded by `#ifdef CPU_RP2040 / CPU_CORE_ARMV6M / PICO_RP2040`, so compiling the other targets' files is harmless. Even so, list sources explicitly.

**Sources for the `mtk3` library under approach (a)** (trial-compiled: 43 objects, 0 errors):
- `kernel/tkernel/*.c`, `kernel/tstdlib/*.c`
- `kernel/sysinit/sysinit.c` **compiled with `-Dmain=mtk3_kernel_main`**. This was verified: the symbol is defined and `main` is no longer defined, so there is no clash with SDK `main`.
- `kernel/inittask/inittask.c`
- `kernel/sysdepend/cpu/core/armv6m/{cpu_cntl.c,exc_hdr.c,interrupt.c,dispatch.S}`
- `kernel/sysdepend/pico_rp2040/{devinit.c,power_save.c}`
- `lib/libtk/*.c`, `lib/libtk/sysdepend/cpu/core/armv6m/{int_armv6m.c,wusec_armv6m.c}`, `lib/libtk/sysdepend/cpu/rp2040/{int_rp2040.c,ptimer_rp2040.c}`
- `lib/libtm/{libtm.c,libtm_printf.c}`

**Excluded:** `reset_hdl.c`, `vector_tbl.c`, `boot_2nd.c`, `cpu_clock.c`, `hw_setting.c`, `tm_com.c`, `libbsp_pwm.c`, `device/**`, `app_program/**`, `kernel/usermain/usermain.c` (we provide `usermain`).

**Include order:** put **our `config/mtk3/` first** (our copies of `config.h`, `config_func.h`, `config_tm.h`, `config_device.h`), then `external/mtk3_bsp/include` and `external/mtk3_bsp/kernel/knlinc`. Config overrides that were trial-compiled successfully:
- `CNF_TIMER_PERIOD 1`
- `CNF_MAX_CYCID 8`
- `CNF_MAX_MBFID 8`
- `USE_SDEV_DRV 0`
- `USE_DEVICE 0`

**Symbols the port must provide** (from `nm` on the trial library):
- `knl_lowmem_top`, `knl_lowmem_limit`
- `knl_shutdown_hw`, `knl_restart_hw`
- `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`
- `usermain`
- `__aeabi_*` division helpers, supplied by the SDK.

**Warnings under `-std=gnu11 -O2 -Wall -Wextra` (GCC 10.3.1):**
- Full set (58 files): 70 warnings. 39 `-Wunused-parameter`, 16 `-Wcast-function-type`, 10 `-Wsign-compare`, 4 `-Wold-style-declaration`, 1 `-Wignored-qualifiers`.
- Core set: 13 unique warnings in `device.h:88`, `memory.c`, `mempool.c`, `eventflag.c`, `mailbox.c`, `mempfix.c`, `wait.c`, `cpu_task.h:79`, `int_rp2040.c:65`, `libtm.c:37`.
- **Keep `-Werror` off this target.** Optionally add `-Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type` on `mtk3` only.
- Note: `-fdiagnostics-plain-output` is not supported by GCC 10.

## 12. Pico vs Pico W differences that matter

| Item | Pico | Pico W | BSP behaviour / clash |
|---|---|---|---|
| Onboard LED | GP25 (`boards/pico.h:33`) | CYW43 `WL_GPIO0` (`pico_w.h:99`), reachable only through the CYW43 driver | `hw_setting.c` makes **GP25 an SIO output**. `app_main.c` toggles it. **On a Pico W, GP25 = CYW43 SPI CS** (`pico_cyw43_driver/cyw43_bus_pio_spi.c:29`). Drop this in our build (approach (a) does not run `hw_setting.c`). Phase 1 has no onboard LED. |
| GP23 | free | `CYW43_PIN_WL_REG_ON` (`pico_w.h:91`) | The BSP does not touch it. |
| GP24 | free | CYW43 data in/out and host-wake IRQ (`pico_w.h:87`, `cyw43_bus_pio_spi.c:24-25`) | The BSP does not touch it. |
| GP29 | VSYS/3 on ADC3 | CYW43 SPI CLK **and** VSYS ADC3 (`pico_w.h:113-119`, `cyw43_bus_pio_spi.c:28`) | The BSP ADC driver has 5 channels (`adc_sysdep.h:26`) but does not mux GP29 by default. Do not read VSYS while WiFi is active. |
| Flash | 2 MB | 2 MB (`pico_w.h:72`) | The linker's 2048K matches. |
| boot2 | W25Q080 | W25Q080 (`pico_w.h:65`) | The BSP blob variant is UNVERIFIED. Approach (a) uses the SDK boot2. |

**Other BSP pin clashes with our pinout** (only if `USE_SDEV_DRV 1` or the BSP startup is used):
- I2C0 on **GP8/GP9** (our motor pins) in `hw_setting.c` and `i2c_rp2040.c:297-301`.
- The I2C1 driver hard-codes **GP6/GP7** (our encoders; we need GP2/GP3), `i2c_rp2040.c:312-316`.
- **GP28 is muxed to ADC2** with input disabled (our barcode digital input).

## 13. Peripheral ownership (one owner each)

| Peripheral | Owner | Justification |
|---|---|---|
| Clocks, resets, boot2, vector table | **SDK** (`pico_runtime`, `pico_standard_link`) | Approach (a). `clock_get_hz()` is only valid after `clocks_init` (`hardware_clocks/clocks.c:115`). |
| PWM (GP8-11 at 20 kHz, GP12 servo at 50 Hz) | **SDK `hardware_pwm`** | The BSP has only raw `libbsp_pwm.c`, which has name clashes. Keep `USE_PTMR 0` so the kernel never claims the PWM IRQ (`INTNO_PTIM`, `cpu/rp2040/sysdef.h:291-313`). |
| ADC0/1 (GP26/27) | **SDK `hardware_adc`** | The BSP ADC is a device-manager driver (open/read via `tk_*dev`, IRQ 22). Polled SDK reads are simpler at 5 ms. `USE_SDEV_DRV 0`. |
| GPIO + GPIO IRQ (GP5/6/7/28) | **SDK `hardware_gpio` + `hardware_irq`** (shared handler on IO_IRQ_BANK0), wrapped by the port | It must share IRQ 13 with CYW43 later. The BSP has no GPIO IRQ support. |
| I2C1 (GP2/3) | **SDK `hardware_i2c`** | The BSP I2C1 driver forces GP6/7. |
| UART0 (GP0/1) | **SDK `hardware_uart` / `pico_stdio_uart`** | The port's `tm_snd_dat` writes polled UART0 for fault messages only. |
| USB | **SDK `pico_stdio_usb` / TinyUSB** | The BSP has no USB. |
| Hardware timer / alarms | **SDK `pico_time`** (default alarm pool on HW alarm 3, `pico/time.h:330`) | The kernel uses SysTick only and never touches TIMER. |
| DMA | **SDK `hardware_dma`** | The BSP does not use it. CYW43 claims 2 channels (`cyw43_bus_pio_spi.c:148-149`). |
| PIO | **SDK `hardware_pio`** | The BSP does not use it. CYW43 prefers PIO1 (`cyw43_bus_pio_spi.c:72`). |
| SysTick, PendSV, SVC | **Kernel** | Bound into the SDK RAM vector table by the port. |

## 14. USB CDC stdio feasibility (analysis; needs a hardware test)

What `pico_stdio_usb` uses (`src/rp2_common/pico_stdio_usb/stdio_usb.c`):
- `USBCTRL_IRQ` as a shared handler (l.203-205; TinyUSB `dcd_rp2040.c:399`).
- A **claimed unused user IRQ** as a low-priority worker that runs `tud_task()` (l.195-201).
- A **repeating 1 ms alarm** on the default alarm pool (`add_alarm_in_us`, l.208; `PICO_STDIO_USB_TASK_INTERVAL_US 1000`).
- A `mutex_t stdio_usb_mutex`.

Conflicts and assessment:
1. **Tick and IRQs:** no shared hardware with the kernel (SysTick vs TIMER alarm 3). These handlers make no kernel calls, so they do not need the `taskindp` wrapper. OK.
2. **Critical sections:** SDK `spin_lock_blocking` saves and restores PRIMASK, the same mechanism as the kernel, so they are compatible. Kernel PRIMASK sections (the SysTick handler with cyclic handlers, `tm_putstring`) add latency to USB servicing. TinyUSB tolerates millisecond latency. Avoid `tm_printf`.
3. **Mutex semantics are the real risk.** SDK mutex owner = core number (`pico_sync/include/pico/lock_core.h:95`), so every kernel task looks like the same owner. If task A is preempted while holding `stdio_usb_mutex` or the stdio `print_mutex`, task B's `mutex_try_enter_block_until` sees owner == caller and returns false immediately (`pico_sync/mutex.c:88`). The worker IRQ falls back to retrying via an alarm (`stdio_usb.c:57-79`). The likely result is dropped or garbled output, not a deadlock. The exact drop-vs-pass behaviour in `stdio.c:43-50` is UNVERIFIED.
   **Mitigation:** only **one** kernel task (the dbg drain) ever calls stdio. Other tasks write to an `dbg_` ring buffer.
4. Blocking SDK waits (`sleep_ms`, `mutex_enter_blocking`, `sem_acquire_*`) spin or WFE instead of yielding. Forbid them in tasks and use `tk_dly_tsk`.
5. Clocks: clk_usb 48 MHz is set by SDK `clocks_init` under approach (a). OK.

**Bring-up test** (Phase 1, `DBG_BACKEND=USB`, `pico_enable_stdio_usb(<t> 1)`, `pico_enable_stdio_uart(<t> 0)`):
1. Boot, then open the COM port. Expect the kernel banner (via `tm_*` → port) and 9 heartbeat lines per period, each with a task ID and a sequence number.
2. Run for 10 minutes. Pass if no sequence gaps, no HardFault, and the cyclic period jitter (each task logs `time_us_64()` deltas, max and min) is within 1 ms of nominal.
3. Stress: every task logs at 10× its normal rate through the dbg ring. Check that overflow counters increment, nothing hangs, and the control-task periods are unaffected.
4. Unplug and replug USB, and open and close the terminal. The system must keep running.
5. Repeat with `DBG_BACKEND=UART0` as the baseline comparison.

**Fallback:** `DBG_BACKEND=UART0` on GP0/GP1 at 115200, with Grove 1 reserved (as instructed).

## 15. WiFi (report only, for later phases)

- **`pico_cyw43_arch_lwip_poll` exists:** `src/rp2_common/pico_cyw43_arch/CMakeLists.txt:32-36` (with `pico_cyw43_arch_poll` at l.24, plus threadsafe_background and freertos variants).
- **The lwIP MQTT client exists:** `lib/lwip/src/apps/mqtt/mqtt.c`. The SDK target is **`pico_lwip_mqtt`** (`src/rp2_common/pico_lwip/CMakeLists.txt:245-248`).
- Possible clashes with the kernel and our port:
  1. **IO_IRQ_BANK0 sharing:** CYW43 installs `gpio_add_raw_irq_handler_with_order_priority(GP24, ..., 0x40)` and enables IRQ 13 (`pico_cyw43_driver/cyw43_driver.c:55-57`). Our GPIO ISRs must also use the SDK shared chain. Never `tk_def_int(13)`, never `irq_set_exclusive_handler(13)`. (`gpio_set_irq_callback` is also a shared handler, `gpio.c:199`.)
  2. **PIO + DMA:** one PIO1 state machine and program, plus 2 DMA channels claimed dynamically. Our code must use the `*_claim_unused_*` APIs or avoid PIO1.
  3. **async_context:** the poll variant waits with `sem_acquire_block_until` (`pico_async_context/async_context_poll.c:45`), which is a spin or WFE inside the calling task. It has a core-number check (`cyw43_driver.c:53`) and the same core-owner lock semantics as §14. **Only the Telemetry task may touch cyw43 or lwIP.** threadsafe_background runs lwIP in a low-priority IRQ plus an alarm pool (`async_context_threadsafe_background.c:14-63`); its callbacks would then be in IRQ context unknown to the kernel. Prefer poll.
  4. **Hardware alarm / time:** CYW43 sleep checks use async_context at-time workers (`CYW43_SLEEP_CHECK_MS 50`) on the default alarm pool. No kernel conflict.
  5. **GP29 / VSYS:** shared with the WiFi clock (§12).
  6. Long kernel PRIMASK sections delay the CYW43 GPIO IRQ. The DMA/PIO transfer itself is unaffected.

## 16. Recommendation

### (a) RECOMMENDED: the SDK owns startup; the kernel is a library behind a port

1. Link with the SDK `pico_standard_link` and a **project copy of `memmap_default.ld`** (set with `pico_set_linker_script`), adding `.data_vector (NOLOAD)` and `.noinit (NOLOAD)` in RAM. Without them, the kernel's `exchdr_tbl` and `Noinit()` objects become orphan sections, and their placement is UNVERIFIED.
2. SDK `main()`: board, stdio and hardware init, then `hal_kernel_start()`. That function:
   - Sets `knl_lowmem_top/limit` to the bounds of a **static, 8-byte-aligned kernel pool array** (e.g. 96 KB), so it never fights newlib `_sbrk`.
   - Writes SHPR2/SHPR3 as `reset_hdl.c:101-102` does (SVC 0, SysTick 0x00, PendSV 0xC0).
   - Writes `((uint32_t*)scb_hw->vtor)[14] = knl_dispatch_entry` and `[15] = knl_systim_inthdr`. VTOR is the SDK RAM table after `runtime_init`, `runtime.c:146-148`.
   - Calls `mtk3_kernel_main()`, which does not return.
3. The port provides `knl_shutdown_hw`, `knl_restart_hw` (return `E_NOSPT`) and `tm_com_init/tm_snd_dat/tm_rcv_dat` (polled UART0, or a no-op for USB).
4. Kernel-calling ISRs are registered through `hal_irq_register()`, which uses the SDK APIs and a wrapper equivalent to `knl_hll_inthdr`. `tk_def_int` is not used.

Rationale: this keeps the SDK fully functional (`clock_get_hz`, USB, alarms, picotool binary_info, the future CYW43), and changes no vendored file.

Risks:
- The port depends on kernel-internal symbols: `knl_taskindp` (`cpu_cntl.c:30`), `knl_lowmem_*`, `knl_dispatch_entry`, `knl_systim_inthdr`, and the `main` rename.
- This combination is not TRON-qualified.
- Macro clashes force strict header separation.
- The 192-byte `exchdr_tbl` is unused.

### (b) ALTERNATIVE: the BSP owns startup; link selected SDK `hardware_*` libraries

Use `tkernel_map.ld`, `-nostartfiles`, the BSP `Reset_Handler`/boot2/clocks, `USE_SDEV_DRV 0`, a patched `hw_setting` (GP25 removed), and link `hardware_gpio/pwm/adc/i2c/uart` only.

Pros: this is the BSP's qualified configuration, and `tk_def_int` works natively.

Risks:
- Without SDK `runtime_init`, `clock_get_hz()` returns 0, so `uart_init` and `i2c_init` miscompute baud (`uart.c:42,108`; `i2c.c:65`).
- There are no `__mutex_array`, `init_array`, spin-lock reset, alarm pool or binary_info.
- **USB CDC and CYW43 become effectively impossible.**
- The BSP startup needs a patch, which goes against "do not modify vendored code".

### Proposed port layer

`src/rtos/port/`. Only these files include kernel headers; SDK-side files declare the few externs themselves.

| Port API | Maps to |
|---|---|
| `void hal_kernel_start(void)` (no return) | pool setup, SHPR writes, vtable[14/15] writes, `mtk3_kernel_main()` |
| `int hal_irq_register(uint irq, void (*fn)(void), uint8_t level)` | `irq_set_exclusive_handler(irq, wrapper)` + `irq_set_priority(irq, level<<6)` + `irq_set_enabled`. The wrapper does `knl_taskindp++; fn(); knl_taskindp--;` |
| `int hal_gpio_irq_register(uint gpio, uint32_t events, void (*fn)(uint gpio, uint32_t ev))` | `gpio_set_irq_enabled` + `gpio_add_raw_irq_handler_masked` with the same wrapper (shared IO_IRQ_BANK0) |
| `uint32_t hal_crit_enter(void)` / `hal_crit_exit(uint32_t)` | `disint()` / `set_primask()` (identical to SDK `save_and_disable_interrupts`) |
| `hal_isr_wake(tid)`, `hal_isr_sig(sem)`, `hal_isr_setflg(flg, ptn)` | `tk_wup_tsk`, `tk_sig_sem`, `tk_set_flg` (the only calls allowed in ISRs) |
| `ipc_task_create/start`, `ipc_cyc_create` | `tk_cre_tsk`/`tk_sta_tsk` (TA_HLNG\|TA_RNG0), `tk_cre_cyc` (TA_HLNG\|TA_STA, `cycphs` for staggering) |
| `ipc_task_sleep`, `ipc_missed_wakeups` | `tk_slp_tsk`, `tk_can_wup` (a return > 0 means an overrun) |
| `ipc_mbf_create/send/recv/ref` | `tk_cre_mbf`/`tk_snd_mbf`/`tk_rcv_mbf`/`tk_ref_mbf`, sized with `MBF_BUFSZ` (§9) |
| `ipc_flg_create/set/clear/wait` | `tk_cre_flg`/`tk_set_flg`/`tk_clr_flg(id, ~bits)`/`tk_wai_flg(TWF_ORW, tmo)` |
| `ipc_sem_*`, `ipc_mtx_*` | `tk_cre_sem`/`tk_sig_sem`/`tk_wai_sem`, `tk_cre_mtx(TA_INHERIT)`/`tk_loc_mtx`/`tk_unl_mtx` |
| `ipc_delay_ms`, `ipc_time_ms` | `tk_dly_tsk`, `tk_get_otm` |

## 17. Questions and blockers for the user

1. **Approve approach (a)?** It drops the BSP's own startup (`reset_hdl.c`, `vector_tbl.c`, `boot_2nd.c`, `cpu_clock.c`, `hw_setting.c`, `tm_com.c`). The "qualified baseline" is then the kernel core, not the BSP startup.
2. **Is it acceptable for the port to use kernel-internal symbols** (`knl_taskindp`, `knl_lowmem_top/limit`, `knl_dispatch_entry`, `knl_systim_inthdr`) and to rename the kernel `main` via `-Dmain=mtk3_kernel_main` on `sysinit.c`? The only alternative is editing vendored files.
3. **Kernel config lives in our tree** (`config/mtk3/*.h`, copied from the BSP with the T-License headers kept): `CNF_TIMER_PERIOD 1`, `CNF_MAX_CYCID 8`, `CNF_MAX_MBFID 8`, `USE_SDEV_DRV 0`, `USE_DEVICE 0`, everything else at default. OK?
4. **No onboard LED in Phase 1.** The Pico W LED needs CYW43, which is excluded. Are heartbeats on the console enough?
5. **Task-to-kernel priority mapping:** use kernel priorities 1 to 9 = P1 to P9. The init task (priority 1) creates everything and then sleeps forever. OK?
6. **Task stack budget** must include ISR frames, because tasks run on MSP. The proposal is each task's need + 256 B, with a measured high-water mark (stack painting) in Phase 1. OK?
7. **Licence notice:** add the T-License 2.2 attribution text to README. The exact wording per tron.org is UNVERIFIED; please confirm or provide it.
8. The kernel Imalloc pool size: proposed 96 KB static. Confirm once task stack sizes are known.
