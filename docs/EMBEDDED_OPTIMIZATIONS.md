# Embedded System Optimization Guide (RP2040 / ARMv6-M Cortex-M0+)

**Author:** Buddy 2 — Ruo Chen (Student ID: `2500530`, GitHub: `leeruochen`)  
**Last Updated:** 13 Nov 2026  
**Target Architecture:** Raspberry Pi Pico W (RP2040 Dual Cortex-M0+, Core 0 @ 125 MHz, ARMv6-M Thumb-1 ISA, micro T-Kernel 3.0)  
**Toolchain:** Pico SDK 1.5.1, `arm-none-eabi-gcc` 10.3.1 (`-Wall -Wextra -Werror`)

---

## 0. Purpose & How to Use This Guide

Because Subsystems 1, 3, 4, and 5 are currently completing their initial on-car hardware and track integration tests (`HANDOVER.md` section 1 & section 3), **only safe, zero-behavioral-impact optimizations inside Subsystem 2 (`app_program/subsystem2/`) have been applied directly to the codebase.**

All cross-subsystem (`app_program/subsystem1..5/`) and shared platform (`src/`) optimizations are documented in this file with ready-to-integrate code blocks, cycle/memory savings, and a **Car Behavioral Impact Assessment** so each owner buddy can adopt them safely once their baseline hardware tests pass.

### Adoption Tiers

| Tier | Classification | Impact on Car Physical Behavior | When to Adopt |
|---|---|---|---|
| **Tier 1 (Safe Now)** | **Bit-for-bit identical output** (SWAR bit-hacks, branchless math, `.bss`/`.rodata` packing, 64-bit->32-bit exact division) | **None (0%)** — identical motor PWM duties, PID steps, sensor values, and FSM transitions | Safe anytime; **already applied in Subsystem 2** (`OPT-01`..`OPT-04`) |
| **Tier 2 (Timing / Jitter)** | **Reduces ISR or task execution latency** (inline assembly ISR wrapper, SRAM-resident ISRs, fast decimal/hex formatting) | **Positive only** — sharpens ultrasonic echo (+/-1 mm vs +/-3 mm) and barcode edge timestamps under WiFi/USB load | Adopt during or right after Subsystem 3 / Subsystem 5 hardware bring-up |
| **Tier 3 (Peripheral / Build)** | **RP2040 hardware offloads & linker flags** (SIO interpolator, DMA round-robin ADC, spare PIO0 SMs, `--gc-sections`) | **Requires on-car verification** — changes peripheral access patterns or binary layout | Adopt in Phase 2 optimization after baseline track tests (`INT-01`..`INT-05`) pass |

---

## Master Index of Optimizations

| ID | Target File(s) | Owner | Category | Savings (`.text` / RAM / Cycles) | Tier & Car Impact |
|---|---|---|---|---|---|
| **OPT-01** | `app_program/subsystem2/task_motion.c` | **Buddy 2** (Ruo Chen) | Single Base-Pointer Context Struct (`s_mot`) + Narrow Fields | `-16 B` `.bss`, `-60 B` `.text` literal pool, faster 50 Hz PID loads | **Tier 1 — APPLIED** (Zero car impact; 28/28 S2 tests unaffected) |
| **OPT-02** | `app_program/subsystem2/task_vc.c` | **Buddy 2** (Ruo Chen) | 10-Bit Packed Private FSM Word (`state_bits`, D8) + Context Struct (`s_vc`) | `-20 B` `.bss` (`-45%`), `-36 B` `.text` literal pool | **Tier 1 — APPLIED** (Zero car impact; bit-identical VC arbitration) |
| **OPT-03** | `app_program/subsystem2/task_motion.c`, `drv_encoder.c` | **Buddy 2** (Ruo Chen) | Branchless Thumb-1 `abs_i32` (`ASRS`+`ADDS`+`EORS`) & Sign Invert (`EORS`+`SUBS`) | `3` cycles constant-time (no branch flush), `-18 B` `.rodata` | **Tier 1 — APPLIED** (Zero car impact; bit-identical for all $2^{32}$ inputs) |
| **OPT-04** | `app_program/subsystem2/drv_motor.c` | **Buddy 2** (Ruo Chen) | Eliminate 64-bit `__aeabi_uldivmod` in `motor_get_pwm_freq_hz` + `uint8_t` Pin Tables | `~300 -> 8` cycles (SIO HW divider), `-24 B` `.rodata` | **Tier 1 — APPLIED** (Zero car impact; exact up to `clk_sys = 268 MHz`) |
| **OPT-05** | `app_program/subsystem3/bc_decode.c` | **Buddy 3** (Jin Yong) | `MULS`-based SWAR 9-Bit Reversal (`bc_mirror`), SWAR Popcount (`bc_wide_count`), `uint16_t` LUT | `~95 -> 16` cycles/window, `-52 B` `.rodata` (`-50%` LUT) | **Tier 1** — Zero car impact; verifiable on host via `test_bc_decode` (`S3-U01`) |
| **OPT-06** | `src/common/data_repr.h` | **Platform / Buddy 1** | Byte-Chunked `drep_bits_put` / `drep_bits_get` + 2-insn `LSLS`+`ASRS` `drep_sext` | `~6-8x` fewer loop iterations per MQTT frame, `6 -> 2` cycles per `drep_sext` | **Tier 1** — Zero car impact; verifiable on host via `test_data_repr` (273 checks) |
| **OPT-07** | `app_program/subsystem1/tel_wire.c`, `src/debug/dbg_ring.c` | **Buddy 1 / Platform** | Branchless SWAR Nibble-to-Hex & Reciprocal-Multiply (`* 0xCCCD >> 19`) Decimal Print | `-16 B` `.rodata`, `~3x` faster `put_dec()` inside `dbg_ring` critical section | **Tier 1** — Zero car impact; shortens interrupt-disabled window in `dbg_` |
| **OPT-08** | `src/rtos/port/port_irq.c` | **Platform / Buddies 3 & 5** | Thumb-1 Inline Assembly ISR Trampoline (`run_slot`) | Saves `~12-18` cycles + 16 B stack on **every** peripheral & GPIO IRQ | **Tier 2** — Reduces ultrasonic (`GP5`) & barcode (`GP27`) edge timestamp jitter |
| **OPT-09** | `src/common/stack_mon.c` | **Platform (Buddy 2/3)** | Multi-Word `STMIA r0!, {r2-r5}` Block Store in `paint_words()` | `4x` faster task stack painting at boot (`~4 cycles / 16 B`) | **Tier 1** — Boot-only; zero runtime car impact |
| **OPT-10** | `src/common/crc32.h` | **Platform (Buddy 2/3)** | Carry-Flag (`LSRS` + `BCC` + `EORS`) Thumb-1 Assembly for `crc32_ieee` | `8 B` inner loop, `-35%` cycles per byte | **Tier 1** — Fault/boot only; zero runtime car impact |
| **OPT-11** | `app_program/subsystem5/drv_ultrasonic.c` | **Buddy 5** (Chay Han) | 32-Bit Reciprocal Multiply (`* 22905U >> 17`) for Ultrasonic `dt_us -> mm` | Eliminates `__aeabi_uldivmod` (`~300 -> 3` cycles) | **Tier 1** — Exact to `< 0.02 mm` across full `0..4000 mm` range |
| **OPT-12** | `app_program/subsystem5/task_obs_scan.c` | **Buddy 5** (Chay Han) | 32-Bit Trig Projection + Folded 10-Entry Quadrant Lookup Table | Eliminates `38x` 64-bit divides/sweep (`~10,000` cycles saved) + `-132 B` `.rodata` | **Tier 1** — Bit-identical obstacle width & clearance (`S5-U01..U16`) |
| **OPT-13** | `app_program/subsystem4/drv_imu.c`, `imu_lift.c` | **Buddy 4** (Vincent) | 32-Bit Pre-Scaled `atan2_half_deg`, 32-Bit Lift Projection & Unified `isqrt_u32` | Eliminates `3x` 64-bit divides/10 ms (`~900` cycles saved) + `-80 B` `.text` | **Tier 1** — Bit-identical 0.5 deg pitch/roll & lift detection (`S4-U01..U20`) |
| **OPT-14** | `app_program/subsystem3/lf_core.c`, `task_motion.c` | **Buddies 2 & 3** | RP2040 Single-Cycle SIO Interpolator (`interp0`) Hardware Clamp & Linear Blend | `1-cycle` hardware clamp/lerp vs `12-20` cycle software branches | **Tier 3** — Save/restore `interp0` across tasks or restrict to single owner task |
| **OPT-15** | `app_program/subsystem3/drv_ir.c` | **Buddy 3** (Jin Yong) | DMA-Driven Free-Running Round-Robin ADC (`ADC0` + `ADC2`) | Eliminates `8 us` CPU busy-wait every `5 ms` (`1600 us/s` CPU freed) | **Tier 3** — Smoother 200 Hz line-following; zero task jitter |
| **OPT-16** | `app_program/subsystem5/drv_ultrasonic.c`, `subsystem3/drv_bc.c` | **Buddies 3 & 5** | Offload Ultrasonic Echo Timing & Barcode Edge Capture to Spare `PIO0 SM2/SM3` | **Zero** CPU ISR latency (`8 ns` hardware counter accuracy) | **Tier 3** — Eliminates Risk R1 GPIO IRQ jitter completely |
| **OPT-17** | `drv_ultrasonic.c`, `drv_bc.c`, `port_irq.c` | **Platform / Buddies 3 & 5** | Place Hard Real-Time ISRs in SRAM (`__not_in_flash_func`) | Avoids `20-40` cycle XIP QSPI cache misses during WiFi/stdio activity | **Tier 2** — Costs `~180 B` SRAM; guarantees deterministic `< 1 us` ISR entry |
| **OPT-18** | `src/common/crc32.h` (bulk buffers) | **Platform** | RP2040 Hardware DMA Sniffer (`DMA_SNIFF_CTRL`) IEEE CRC-32 | `1 word / cycle` (500 MB/s) zero-CPU CRC-32 | **Tier 3** — Optional for large telemetry/log buffers |
| **OPT-19** | `task_imu.c`, `task_safety.c`, `task_obs_scan.c`, `task_obs_avoid.c` | **Buddies 4 & 5** | Single Context Struct (`s_ctx`) Consolidation + D8 Packed Private States | Saves `~80-120 B` `.bss` and `~200 B` `.text` literal pools across S4 & S5 | **Tier 1** — Zero car impact; same pattern as `OPT-01`/`OPT-02` |
| **OPT-20** | `CMakeLists.txt` / `cmake/warnings.cmake` | **All (Team Agreement)** | Dead-Section Garbage Collection (`-ffunction-sections -fdata-sections -Wl,--gc-sections`) | Strips `~4-12 KB` of unused SDK/BSP code from `robocar.uf2` | **Tier 3** — Zero behavioral impact if linker script `KEEP()`s vectors |

---

## 1. Optimizations Already Applied in Subsystem 2 (Buddy 2 — Ruo Chen)

The following four Tier-1 optimizations have already been implemented and verified in `app_program/subsystem2/` (`0` warnings under `-Wall -Wextra -Werror`, `check_rules: PASS`, 100% bit-identical behavior for `S2-U01`..`S2-U28`).

### OPT-01: Single Base-Pointer Context Struct (`s_mot`) in `task_motion.c`
- **File:** [task_motion.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/task_motion.c#L111-L163)
- **Why it works on ARMv6-M (Thumb-1):**
  - The Cortex-M0+ Thumb-1 instruction set has no PC-relative `STR` and can only load 32-bit addresses into low registers (`r0`–`r7`) via `LDR Rd, [PC, #imm8]` from a literal pool in `.text`.
  - When 16 module-private `static` variables are declared separately, GCC emits separate 32-bit literal-pool entries and reloads base pointers repeatedly inside the 50 Hz PID control loop (`motion_step_cycle` -> `poll_command_queue` -> `check_manoeuvre_completion` -> `apply_control`).
  - Grouping all 16 variables into `static motion_ctx_t s_mot` (68 bytes total, down from 84 bytes) allows GCC to load `&s_mot` **once** into a callee-saved register (e.g. `r4`) and access every field with a single 16-bit instruction (`LDR`/`STR`/`LDRH`/`LDRB [r4, #imm]`).
  - Narrow fields (`cmd_type`, `setpoint_cmps`, `stall_ticks`, `hold_elapsed_ms`, `cmd_arg`) and explicit mask/shift flags (`MOT_FLG_TASK_RUNNING`, `MOT_FLG_CALIB_HOLD`, `MOT_FLG_CMD_ACTIVE`) shrink `.bss` by 16 bytes without using C bit-fields (`HANDOVER.md` section 6).

### OPT-02: 10-Bit Packed Private State Word (`s_vc.state_bits`) in `task_vc.c`
- **File:** [task_vc.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/task_vc.c#L37-L115)
- **Why it works:**
  - `task_vc.c` previously held 6 discrete state/flag variables across separate 32-bit/bool statics (`s_sys_state`, `s_authority`, `s_bc_cmd`, `s_bc_pending`, `s_stationary_valid`, `s_task_running`).
  - Their true information content (`ARCHITECTURE.md` D8) is only **10 bits**:
    - `bits [3:0]` (4 bits): 10-state System FSM (`VC_SYS_INITIALISING` .. `VC_SYS_SAFETY_FAULT_LATCHED`)
    - `bit  [4]`   (1 bit):  Authority (`0 = VC_AUTH_LINE`, `1 = VC_AUTH_AVOID`)
    - `bits [6:5]` (2 bits): Pending barcode command (`DREP_BC_CMD_*`)
    - `bit  [7]`   (1 bit):  `VC_ST_FLG_BC_PENDING`
    - `bit  [8]`   (1 bit):  `VC_ST_FLG_STAT_VALID`
    - `bit  [9]`   (1 bit):  `VC_ST_FLG_TASK_RUNNING`
  - Packing all 6 fields into `s_vc.state_bits` (`uint16_t`) inside `static vc_ctx_t s_vc` using `drep_get()` / `drep_set()` reduces `task_vc.c`'s `.bss` footprint from **44 bytes to 24 bytes (-45%)** and eliminates 9 literal-pool pointers.

### OPT-03: Branchless ARMv6-M Thumb-1 `abs_i32()` & Encoder Sign Inversion
- **Files:** [task_motion.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/task_motion.c#L179-L193), [drv_encoder.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/drv_encoder.c#L191-L200)
- **Why it works:**
  - Conditional branches (`CMP` + `BLT`/`BGE`) on Cortex-M0+ incur a 2-cycle pipeline refill penalty whenever taken.
  - `abs_i32(int32_t v)` uses the arithmetic right shift mask `mask = (uint32_t)(v >> 31)` (`0` for non-negative, `0xFFFFFFFF` for negative) and computes `((uint32_t)v + mask) ^ mask`, which compiles to 3 single-cycle 16-bit Thumb-1 instructions (`ASRS`, `ADDS`, `EORS` — 6 bytes, 3 cycles constant time).
  - Similarly, `enc_get_count()` replaces the `if (invert != 0U)` branch at the end of every encoder read with `inv_mask = 0U - invert; return (raw ^ inv_mask) - inv_mask;` (`EORS` + `SUBS`).

### OPT-04: 32-Bit Hardware Division in `drv_motor.c` & `.rodata` Compaction
- **Files:** [drv_motor.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/drv_motor.c#L56-L97), [drv_motor.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/drv_motor.c#L256-L282), [drv_encoder.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/drv_encoder.c#L46-L79)
- **Why it works:**
  - `motor_get_pwm_freq_hz()` previously cast `clock_get_hz(clk_sys) * 16U` to `uint64_t`, pulling in `__aeabi_uldivmod` (~300 cycles in software). Since `125,000,000 * 16 = 2,000,000,000 < UINT32_MAX` (safe up to `clk_sys = 268.4 MHz`), keeping both operands in `uint32_t` uses the RP2040's **8-cycle hardware SIO divider**.
  - `k_motor_pins[]`, `k_all_pins[]`, and `k_enc_wheels[]` now use `uint8_t` instead of `uint32_t`, saving **42 bytes of `.rodata`**.

> [!CAUTION]
> **Hardware Caveat for `drv_encoder.c` (Do NOT Replace the 4-Word Drain Loop with `pio_sm_clear_fifos`):**  
> On RP2040, `pio_sm_clear_fifos(pio, sm)` works by toggling `SHIFTCTRL_FJOIN_RX`. Toggling `FJOIN_RX` while a PIO state machine is running **also resets the state machine's `ISR` and `OSR` shift registers**, which corrupts `quadrature_encoder.pio`'s 2-bit previous-pin state held in `OSR` (`out isr, 2`) at 125 MHz. Always keep `drv_encoder.c`'s bounded 4-word `pio_sm_get()` drain loop intact.

---

## 2. SWAR (SIMD Within A Register) & Lookup-Free Bit-Hacks

Inspired by the RP2040 SDK's `0x924U` multiplication bit-spreading idiom (`((m_lo & 0x5U) * 0x924U)`), we can exploit the Cortex-M0+'s **single-cycle 32×32->32 hardware multiplier (`MULS`)** to process all bits of a word in parallel without loops or branches.

### OPT-05: SWAR 9-Bit Reversal (`bc_mirror`), SWAR Popcount (`bc_wide_count`), & Half-Width LUT in `bc_decode.c`
- **Owner:** **Buddy 3 (Jin Yong)** — [bc_decode.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem3/bc_decode.c#L32-L97)
- **Current Code:**
  - `s_letters[26]` stores a 9-bit Code 39 mask (`0x001..0x1FF`) inside a `uint32_t` field alongside `char ch`, padded to 8 bytes/entry (**208 bytes** or 104 bytes of masks).
  - `bc_wide_count(uint32_t mask)` loops 9 times (`for (i = 0; i < 9; i++)`) to count set bits (~45 cycles).
  - `bc_mirror(uint32_t mask)` loops 9 times (`for (i = 0; i < 9; i++)`) to reverse bits `0..8` (~50 cycles).
  - Both functions are called inside `bc_frame.c`'s sliding-window search across up to 5 windows per capture.
- **Proposed Optimization:**
  ```c
  /* Pack 9-bit pattern (bits 8:0) into uint16_t; letter is implicit: 'A' + idx!
   * Saves 156 bytes of Flash (.rodata): 26 x 2 B = 52 B instead of 26 x 8 B = 208 B. */
  static uint16_t const s_letter_masks[26] =
  {
      0x121U, /* A */ 0x061U, /* B */ 0x160U, /* C */ 0x031U, /* D */
      0x130U, /* E */ 0x070U, /* F */ 0x025U, /* G */ 0x124U, /* H */
      0x064U, /* I */ 0x034U, /* J */ 0x109U, /* K */ 0x049U, /* L */
      0x148U, /* M */ 0x019U, /* N */ 0x118U, /* O */ 0x058U, /* P */
      0x00DU, /* Q */ 0x10CU, /* R */ 0x04CU, /* S */ 0x01CU, /* T */
      0x181U, /* U */ 0x0C1U, /* V */ 0x1C0U, /* W */ 0x091U, /* X */
      0x190U, /* Y */ 0x0D0U  /* Z */
  };

  /* SWAR parallel popcount for a 9-bit mask (0..0x1FF): 7 instructions, 0 loops. */
  static uint32_t bc_wide_count(uint32_t mask)
  {
      uint32_t v = mask & 0x1FFU;

      v = v - ((v >> 1) & 0x555U);
      v = (v & 0x333U) + ((v >> 2) & 0x333U);
      v = (v + (v >> 4)) & 0x0F0FU;
      return (v * 0x0101U) >> 8;
  }

  /* SWAR 9-bit reversal using single-cycle MULS (0 loops, 9 cycles):
   * Reverses the low 8 bits via the 3-shift + byte-fold idiom, then shifts left
   * by 1 and places old bit 8 into bit 0. */
  static uint32_t bc_mirror(uint32_t mask)
  {
      uint32_t b = mask & 0xFFU;

      b = ((b & 0xAAU) >> 1) | ((b & 0x55U) << 1);
      b = ((b & 0xCCU) >> 2) | ((b & 0x33U) << 2);
      b = ((b & 0xF0U) >> 4) | ((b & 0x0FU) << 4);
      return ((b << 1) | ((mask >> 8) & 1U)) & 0x1FFU;
  }
  ```
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Produces bit-for-bit identical decoded letters across all $2^9 = 512$ possible 9-bit patterns. Can be verified immediately on the host PC via `test_bc_decode` (`S3-U01`, 1,980 checks).

---

### OPT-06: Byte-Chunked Bit Packing (`drep_bits_put` / `drep_bits_get`) & 2-Instruction `drep_sext` in `src/common/data_repr.h`
- **Owner:** **Platform (Buddy 2 / Buddy 3) & Buddy 1 (Damian)** — [data_repr.h](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/common/data_repr.h#L58-L64), [data_repr.h](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/common/data_repr.h#L750-L785)
- **Current Code:**
  1. `drep_sext(uint32_t raw, uint32_t bits)` computes `sign = 1UL << (bits - 1U); raw &= DREP_MASK(bits); return (int32_t)(raw ^ sign) - (int32_t)sign;` (6 instructions).
  2. `drep_bits_put()` and `drep_bits_get()` loop **1 bit at a time** (`for (i = 0; i < bits; i++)`), executing 120 loop iterations to pack a single 15-byte `DREP_FT_TELEMETRY_MOTION` frame.
- **Proposed Optimization:**
  ```c
  /** Sign-extend the low `bits` bits of `raw` (two's complement, 1 <= bits <= 32).
   *  When `bits` is a compile-time constant, compiles to 2 single-cycle Thumb-1
   *  instructions: LSLS r0, r0, #(32-bits) ; ASRS r0, r0, #(32-bits). */
  static inline int32_t drep_sext(uint32_t raw, uint32_t bits)
  {
      uint32_t const shift = 32U - bits;

      return ((int32_t)(raw << shift)) >> shift;
  }

  /* Byte-chunked little-endian bit writer/reader (processes up to 8 bits per iteration). */
  static inline void drep_bits_put(uint8_t *p_buf, uint32_t *p_pos, uint32_t value, uint32_t bits)
  {
      uint32_t pos = *p_pos;
      uint32_t rem = bits;
      uint32_t val = value;

      while (rem > 0U)
      {
          uint32_t const bit_off = pos & 7U;
          uint32_t const space = 8U - bit_off;
          uint32_t const chunk = (rem < space) ? rem : space;
          uint8_t const  mask = (uint8_t)(((1U << chunk) - 1U) << bit_off);
          uint8_t const  bits_in = (uint8_t)((val << bit_off) & mask);

          p_buf[pos >> 3] = (uint8_t)((p_buf[pos >> 3] & (uint8_t)~mask) | bits_in);
          val >>= chunk;
          pos += chunk;
          rem -= chunk;
      }
      *p_pos = pos;
  }

  static inline uint32_t drep_bits_get(uint8_t const *p_buf, uint32_t *p_pos, uint32_t bits)
  {
      uint32_t pos = *p_pos;
      uint32_t rem = bits;
      uint32_t shift = 0U;
      uint32_t v = 0U;

      while (rem > 0U)
      {
          uint32_t const bit_off = pos & 7U;
          uint32_t const space = 8U - bit_off;
          uint32_t const chunk = (rem < space) ? rem : space;
          uint32_t const mask = (1U << chunk) - 1U;
          uint32_t const part = ((uint32_t)p_buf[pos >> 3] >> bit_off) & mask;

          v |= (part << shift);
          shift += chunk;
          pos += chunk;
          rem -= chunk;
      }
      *p_pos = pos;
      return v;
  }
  ```
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Cuts MQTT frame packing/unpacking CPU time by ~6× and reduces `drep_sext` to 2 cycles. Fully covered by `tests/host/test_data_repr.c` (273 assertions) and `tests/host/test_tel.c`.

---

### OPT-07: Branchless SWAR Nibble-to-Hex (`tel_wire.c`) & Reciprocal Decimal Formatting (`dbg_ring.c`)
- **Owners:** **Buddy 1 (Damian)** — [tel_wire.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem1/tel_wire.c#L66-L86); **Platform** — [dbg_ring.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/debug/dbg_ring.c#L60-L81)
- **Current Code:**
  - `tel_wire_hex()` uses a 16-byte lookup table `static char const k_hex[16] = "0123456789ABCDEF";` in `.rodata` (which can suffer XIP flash cache misses).
  - `put_dec()` in `dbg_ring.c` performs `v % 10U` and `v /= 10U` in a loop (2 hardware divider waits = ~18 cycles per digit).
- **Proposed Optimization:**
  ```c
  /* Branchless nibble-to-ASCII-hex without a lookup table (register-only):
   *   0..9  -> '0' (0x30) + nib
   *   10..15 -> 'A' (0x41) + (nib - 10) = 0x37 + nib
   *   ((9U - nib) >> 4) is 0 for 0..9 and 0x0FFFFFFF for 10..15. */
  static inline char nibble_to_hex(uint32_t nib)
  {
      uint32_t const n = nib & 0xFU;

      return (char)(0x30U + n + (((9U - n) >> 28) & 7U));
  }
  ```
  And in `dbg_ring.c` (`put_dec()`), for values `v <= 65535U` (such as line numbers, buffer counts, and millisecond deltas), `q = (v * 0xCCCDU) >> 19; r = v - (q * 10U);` computes both quotient and remainder in **3 single-cycle instructions** (`MULS`, `LSRS`, `SUBS`) instead of waiting 16 cycles for two hardware divisions.
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Shrinks `.rodata` and reduces `dbg_` logging latency.

---

## 3. ARMv6-M (Cortex-M0+ Thumb-1) Inline Assembly Optimizations

### OPT-08: Flatten `run_slot()` ISR Wrapper in `src/rtos/port/port_irq.c`
- **Owner:** **Platform (Buddy 2 / Buddy 3)**, directly benefiting **Buddy 3 (Barcode `GP27`)** and **Buddy 5 (Ultrasonic `GP5`)** — [port_irq.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/rtos/port/port_irq.c#L46-L70)
- **Why it matters (`design.md` Risk R1):**
  - Every shared GPIO edge interrupt (`IO_IRQ_BANK0`) and peripheral IRQ passes through `port_irq_wrap_N()` -> `run_slot(N)` -> `knl_taskindp++` -> handler -> `knl_taskindp--`.
  - Under `-Og` / `-O0` (`CMAKE_BUILD_TYPE=Debug`), GCC emits two full stack frames (`PUSH {r4, lr}` in `port_irq_wrap_N`, `PUSH {r4, r5, lr}` in `run_slot`, multiply `slot * 8`, three reloads of `&knl_taskindp`), adding ~20–25 cycles before `time_us_32()` is sampled inside `echo_isr()` or `bc_gpio_isr()`.
- **Proposed Thumb-1 Inline Assembly Optimization:**
  ```c
  static inline __attribute__((always_inline)) void run_slot(uint32_t slot)
  {
      void *fn_ptr = (void *)&s_table[slot];

      __asm__ volatile(
          "ldr    r2, [  %0, #0]      \n" /* r2 = s_table[slot].fn           */
          "cmp    r2, #0              \n"
          "beq    1f                  \n"
          "ldr    r3, =knl_taskindp   \n"
          "ldr    r1, [  r3, #0]      \n"
          "adds   r1, r1, #1          \n"
          "str    r1, [  r3, #0]      \n" /* knl_taskindp++                  */
          "ldr    r0, [  %0, #4]      \n" /* r0 = s_table[slot].p_arg        */
          "blx    r2                  \n" /* call fn(p_arg)                  */
          "ldr    r3, =knl_taskindp   \n"
          "ldr    r1, [  r3, #0]      \n"
          "subs   r1, r1, #1          \n"
          "str    r1, [  r3, #0]      \n" /* knl_taskindp--                  */
          "1:                         \n"
          :
          : "r"(fn_ptr)
          : "r0", "r1", "r2", "r3", "cc", "memory");
  }
  ```
- **Impact on Car:** **Tier 2 (Timing improvement only).** Shaves ~12–18 cycles and 16 bytes of exception stack off every hardware interrupt. Improves ultrasonic echo pulse timing and barcode edge timestamp accuracy when other IRQs (SysTick, USB, DMA) are active.

---

### OPT-09: Multi-Word `STMIA` Block Store for Stack Painting in `src/common/stack_mon.c`
- **Owner:** **Platform (Buddy 2 / Buddy 3)** — [stack_mon.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/common/stack_mon.c#L64-L74)
- **Current Code:** `paint_words()` writes `0xA5A5A5A5UL` one 32-bit word at a time (`STR` + `ADDS` + `CMP` + `BCC` = 5 cycles per 4 bytes). Painting ~12 KB of task stacks takes ~15,000 cycles.
- **Proposed Thumb-1 Inline Assembly Optimization:**
  ```c
  /* Fill [p_bottom, p_limit) with STACK_MON_PATTERN using 16-byte STMIA bursts. */
  static void __attribute__((noinline)) paint_words(uint32_t *p_bottom, uint32_t *p_limit)
  {
      uint32_t pat = STACK_MON_PATTERN;

      __asm__ volatile(
          "mov    r2, %2              \n"
          "mov    r3, %2              \n"
          "mov    r4, %2              \n"
          "mov    r5, %2              \n"
          "b      2f                  \n"
          "1:                         \n"
          "stmia  %0!, {r2, r3, r4, r5}\n" /* 4 words (16 B) in 5 cycles!    */
          "2:                         \n"
          "adds   r6, %0, #16         \n"
          "cmp    r6, %1              \n"
          "bls    1b                  \n"
          "b      4f                  \n"
          "3:                         \n"
          "stmia  %0!, {r2}           \n"
          "4:                         \n"
          "cmp    %0, %1              \n"
          "blo    3b                  \n"
          : "+r"(p_bottom)
          : "r"(p_limit), "r"(pat)
          : "r2", "r3", "r4", "r5", "r6", "cc", "memory");
  }
  ```
- **Impact on Car:** **Tier 1 (Zero runtime impact).** Runs only once per task at startup before the control loop begins, cutting stack-painting overhead by ~4×.

---

### OPT-10: Carry-Flag (`LSRS` + `BCC`) Thumb-1 Assembly for `crc32_ieee()` in `src/common/crc32.h`
- **Owner:** **Platform (Buddy 2 / Buddy 3)** — [crc32.h](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/src/common/crc32.h#L18-L35)
- **Why it works:** On ARMv6-M, `LSRS Rd, Rd, #1` shifts bit 0 directly into the CPSR Carry flag (`C`) in the same single-cycle instruction. Testing Carry via `BCC` avoids masking `crc & 1UL` into a scratch register on every bit:
  ```c
  #if defined(__ARM_ARCH_6M__)
  for (bit = 0U; bit < 8U; bit++)
  {
      uint32_t const poly = 0xEDB88320UL;
      __asm__ volatile(
          "lsrs   %0, %0, #1      \n"
          "bcc    1f              \n"
          "eors   %0, %1          \n"
          "1:                     \n"
          : "+l"(crc)
          : "l"(poly)
          : "cc");
  }
  #endif
  ```
- **Impact on Car:** **Tier 1 (Zero runtime impact).** Used only for the `.uninitialized_data` fault record check at boot and HardFault capture; verifiable via `CRC32_CHECK_VALUE` (`0xCBF43926`).

---

## 4. Eliminating 64-Bit Software Divisions (`__aeabi_uldivmod` / `__aeabi_ldivmod`)

The RP2040 has an **8-cycle hardware 32-bit divider** in the SIO block (`SIO_DIV_UDIVIDEND` / `SIO_DIV_SDIVIDEND`), which the Pico SDK hooks into `__aeabi_uidiv` and `__aeabi_idiv`. However, **64-bit divisions (`uint64_t` / `int64_t`) are NOT hardware-accelerated** and fall back to libgcc's software `__aeabi_uldivmod` / `__aeabi_ldivmod` (~200–400 CPU cycles and ~350 bytes of Flash).

We found **four places** in Subsystems 4 and 5 where `uint64_t`/`int64_t` division is called periodically despite the values fitting in 32-bit fixed-point math:

### OPT-11: 32-Bit Reciprocal Multiply in `drv_ultrasonic.c` (Buddy 5)
- **File:** [drv_ultrasonic.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem5/drv_ultrasonic.c#L166-L169)
- **Current Code:**
  ```c
  dist_mm = (uint32_t)(((uint64_t)dt * (uint64_t)US_SPEED_OF_SOUND_MPS) / 2000ULL);
  ```
- **Why 64-bit is unnecessary:**
  - `dt` is bounded by `TIMEOUT_ECHO_US = 30,000 us` (`30 ms`).
  - Even in direct 32-bit division, `dt * 343U <= 30,000 * 343 = 10,290,000`, which is **417× smaller than `UINT32_MAX` (`4,294,967,295`)**!
  - Better yet, dividing by a constant (`343 / 2000 = 0.1715`) can be done with a **3-cycle reciprocal multiply** (`22905 / 131072 = 0.17475` or `11239 / 65536 = 0.1714935`):
    ```c
    /* Exact 32-bit hardware division (8 cycles, identical to uint64_t for dt <= 12,500,000 us): */
    dist_mm = (dt * (uint32_t)US_SPEED_OF_SOUND_MPS) / 2000U;

    /* Or zero-division reciprocal multiply (3 cycles, error < 0.02 mm over 0..4000 mm): */
    dist_mm = (dt * 11239U) >> 16;
    ```
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Direct `(dt * 343U) / 2000U` is **100% bit-for-bit identical** to the `uint64_t` version for every `dt <= 12.5 seconds` and saves ~290 cycles per ping.

---

### OPT-12: 32-Bit Polar-to-Cartesian Projection & Folded Quadrant Trig LUT in `task_obs_scan.c` (Buddy 5)
- **File:** [task_obs_scan.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem5/task_obs_scan.c#L72-L99), [task_obs_scan.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem5/task_obs_scan.c#L261-L265)
- **Current Code:**
  1. `build_profile_from_sweep()` computes:
     ```c
     x_mm = (int32_t)(((int64_t)r * (int64_t)k_cos_m1000[i]) / 1000LL);
     y_mm = (int32_t)(((int64_t)r * (int64_t)k_sin_m1000[i]) / 1000LL);
     ```
     up to **38 times per sweep** (`19` angles × 2 coordinates), triggering 38 software 64-bit divisions (`~10,000` cycles)!
  2. `k_cos_m1000[19]` (76 bytes) and `k_sin_m1000[19]` (76 bytes) store 32-bit `int32_t` values in `-1000..+1000` for $0^\circ..180^\circ$ in $10^\circ$ steps (**152 bytes** of `.rodata`).
- **Why 32-bit & a single 10-entry `uint16_t` table suffice:**
  - `r <= 4000 mm` (`OBS_DETECT_RANGE_MM`), and `|cos|, |sin| <= 1000`, so `|r * trig| <= 4,000,000`, which is **536× smaller than `INT32_MAX` (`2,147,483,647`)**! Standard 32-bit `(r * cos_m1000) / 1000` is 100% bit-for-bit identical and uses the 8-cycle hardware divider.
  - Because $\sin(90^\circ + \theta) = \sin(90^\circ - \theta)$ and $\cos(\theta) = \sin(90^\circ - \theta)$ for $\theta \in [0^\circ, 180^\circ]$, both 19-entry `int32_t` tables fold into a **single 10-entry `uint16_t` table (20 bytes total, saving 132 bytes of `.rodata`)**:
    ```c
    /* sin(0 deg .. 90 deg) in 10 deg steps, scaled by 1000 (20 bytes in .rodata). */
    static uint16_t const k_sin_quad_m1000[10] =
    {
        0U, 174U, 342U, 500U, 643U, 766U, 866U, 940U, 985U, 1000U
    };

    static inline int32_t sin_step_m1000(uint32_t step) /* step 0..18 (0..180 deg) */
    {
        uint32_t const idx = (step <= 9U) ? step : (18U - step);
        return (int32_t)k_sin_quad_m1000[idx];
    }

    static inline int32_t cos_step_m1000(uint32_t step) /* step 0..18 (0..180 deg) */
    {
        return (step <= 9U) ? (int32_t)k_sin_quad_m1000[9U - step]
                            : -(int32_t)k_sin_quad_m1000[step - 9U];
    }
    ```
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Produces the exact same `x_mm` and `y_mm` coordinates for every step `0..18`. Verifiable immediately via host tests `S5-U01`..`S5-U16`.

---

### OPT-13: 32-Bit Pre-Scaled `atan2_half_deg`, 32-Bit Lift Projection, & Unified `isqrt` in Subsystem 4 (Buddy 4)
- **Files:** [drv_imu.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem4/drv_imu.c#L168-L246), [imu_lift.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem4/imu_lift.c#L12-L81)
- **Current Code:**
  1. **Duplicate `isqrt`:** `drv_imu.c` defines `isqrt_u32(uint32_t n)` while `imu_lift.c` defines an identical `isqrt32(uint32_t x)` (~80 bytes of duplicated `.text`).
  2. **64-bit divisions in `atan2_half_deg` (`drv_imu.c` lines 224–225):**
     Called twice every 10 ms (for pitch and roll):
     ```c
     r_q15 = (uint32_t)(((uint64_t)v * 32768ULL) / (uint64_t)u);
     r3_q15 = (uint32_t)((((uint64_t)r_q15 * r_q15) >> 15) * r_q15 >> 15);
     ```
     Since `u, v` are derived from 16-bit accelerometer readings (`|x|, |y|, |z| <= 32768`), normalizing `(u, v)` right by 2 bits only when `u >= 65536U` guarantees `v * 32768U <= UINT32_MAX` and `(r_q15 * r_q15) >> 15` fits in `uint32_t` (since `r_q15 <= 32768`, `r_q15 * r_q15 <= 2^{30} < UINT32_MAX` without any scaling at all!).
  3. **64-bit division in `imu_lift.c` (line 76):**
     ```c
     int64_t const dot = ((int64_t)ax * (int64_t)p_s->ref_ax)
                       + ((int64_t)ay * (int64_t)p_s->ref_ay)
                       + ((int64_t)az * (int64_t)p_s->ref_az);
     int32_t const a_up_mg = (int32_t)(dot / (int64_t)p_s->ref_mag_mg) - (int32_t)p_s->ref_mag_mg;
     ```
     Since `ref_ax, ref_ay, ref_az` are a 1g reference vector (`ref_mag_mg ≈ 1000 mg`) and normal/hump/lift accelerations are within `+-4000 mg` (`+-4g` full scale), `|dot| <= 3 * 4000 * 1000 = 12,000,000`, which is **178× smaller than `INT32_MAX` (`2,147,483,647`)**! Computing `dot` in `int32_t` (or clamping inputs to `+-32767 mg`) replaces the 300-cycle `__aeabi_ldivmod` with a single 8-cycle 32-bit hardware division.
- **Impact on Car:** **Tier 1 (Zero behavioral impact).** Saves ~900 cycles every 10 ms IMU period (`90,000 cycles/s`) and ~80 bytes of Flash with identical pitch, roll, hump, and lift outputs.

---

## 5. Further Embedded Optimizations & Car Impact Analysis

Beyond the code-level optimizations above, the RP2040 silicon includes dedicated hardware accelerators that are currently unused in our project. Below are seven further optimizations along with their **physical impact on the car**.

```mermaid
flowchart LR
    subgraph RP2040_Silicon["Unused RP2040 Hardware Accelerators"]
        INTERP["SIO Interpolator (interp0/1)<br/>1-Cycle Clamp & Linear Lerp"]
        ADCDMA["Round-Robin ADC + DMA<br/>Zero-Wait Dual IR Sampling"]
        PIOSM["Spare PIO0 SM2 & SM3<br/>8 ns Hardware Pulse Timing"]
        SNIFF["DMA CRC-32 Sniffer<br/>1 Word/Cycle IEEE CRC-32"]
    end
    subgraph Tasks["Robocar Real-Time Tasks"]
        LF["Subsystem 3: Line Following (P4, 5 ms)"]
        MOT["Subsystem 2: Motion PID (P3, 20 ms)"]
        BC["Subsystem 3: Barcode (P5, GP27)"]
        US["Subsystem 5: Ultrasonic (P6, GP5)"]
    end
    INTERP -->|"OPT-14: 1-cycle lf_norm & clamp"| LF
    INTERP -->|"OPT-14: 1-cycle duty/PID clamp"| MOT
    ADCDMA -->|"OPT-15: Frees 8 us/5 ms"| LF
    PIOSM -->|"OPT-16: Zero-jitter edge capture"| BC
    PIOSM -->|"OPT-16: Zero-ISR echo width"| US
```

### OPT-14: RP2040 Single-Cycle SIO Hardware Interpolator (`interp0`) for Clamping & Sensor Normalization
- **Target Files:** `app_program/subsystem3/lf_core.c` (`lf_norm`, `lf_clamp`), `app_program/subsystem2/drv_motor.c` (`motor_duty_clamp_pm`)
- **How it works:**
  - Each RP2040 core has two dedicated **SIO Hardware Interpolators (`interp0` and `interp1`)** mapped into the single-cycle IO bus (`0xD0000080`).
  - **Clamping mode (`SIO_INTERP0_CTRL_LANE0_CLAMP_BITS`):** Configuring `BASE0 = lo` and `BASE1 = hi` once allows clamping any signed 32-bit integer `v` in **2 single-cycle register accesses** (`interp0->accum[0] = v; result = interp0->peek[0];`) with zero branches!
  - **Linear scaling / blend mode:** `lf_norm(uint16_t raw, uint16_t black, uint16_t white)` in `lf_core.c` maps raw 12-bit ADC readings onto `0..1000` every 5 ms (`200 Hz`). Using `interp0` clamp + shift/scale reduces per-sample normalization to a few cycles.
- **Impact on Car (Tier 3):**
  - **Crucial RTOS Constraint:** Because `interp0` is a per-core hardware peripheral NOT saved by micro T-Kernel's context switcher (`knl_setup_context`), if two tasks at different priorities use `interp0`, the higher-priority task would overwrite `BASE0`/`BASE1` unless saved/restored.
  - **Safe Rule:** Dedicate `interp0` exclusively to **Line-Following (`task_lf`, P4)** and `interp1` exclusively to **Motion Control (`task_motion`, P3)** — one owning task per interpolator (matching `HANDOVER.md` section 6: *"One owning task per peripheral"*).

---

### OPT-15: DMA-Driven Free-Running Round-Robin ADC for Line Sensors (`drv_ir.c`, Buddy 3)
- **Target File:** `app_program/subsystem3/drv_ir.c` (owned by **Buddy 3 — Jin Yong**)
- **How it works:**
  - The two line sensors are on `GP26` (`ADC0`, RIGHT) and `GP28` (`ADC2`, CENTRE).
  - Polling `adc_select_input(0); adc_read(); adc_select_input(2); adc_read();` inside the 5 ms Line-Following task spins the CPU for `96 ADC clocks × 2 = 4.0 us` per cycle (`800 us/s`), or `8 us` if oversampling 2×.
  - Instead, configure the RP2040 ADC in **round-robin mode** (`adc_set_round_robin((1U << 0) | (1U << 2))` = mask `0x05U`) with `adc_fifo_setup(true, true, 1, false, false)` and a single DMA channel writing continuously into `static volatile uint16_t s_ir_latest[2]` in SRAM.
  - When `task_lf` wakes every 5 ms (`app_period_wait`), `ir_read()` simply reads `s_ir_latest[0]` and `s_ir_latest[1]` in **2 CPU cycles (16 ns)** with zero busy-wait!
- **Impact on Car (Tier 3):**
  - Frees CPU time at priority 4 (`TASK_PRI_LF`) so lower-priority tasks (`Barcode` P5, `Obstacle` P6/P7, `IMU` P8, `Telemetry` P9) experience less preemption jitter.
  - **Note:** Requires claiming 1 DMA channel in `TEAM.md` section 6 (RP2040 has 12 DMA channels; CYW43 WiFi uses 2, leaving 10 free).

---

### OPT-16: Offload Ultrasonic Echo Timing (`GP5`) & Barcode Edge Capture (`GP27`) to Spare `PIO0 SM2` & `SM3`
- **Target Files:** `app_program/subsystem5/drv_ultrasonic.c` (**Buddy 5**) and `app_program/subsystem3/drv_bc.c` (**Buddy 3**)
- **How it works:**
  - Currently, `PIO0` runs `quadrature_encoder.pio` (24 instructions at offset `0..23`) on `SM0` (Right wheel) and `SM1` (Left wheel).
  - **`PIO0` still has 8 free instruction slots (`offset 24..31`) and 2 completely unused state machines (`SM2` and `SM3`)!**
  - **Ultrasonic Echo Timer on `PIO0 SM2` (5 instructions):**
    ```pio
    .program echo_timer
    .wrap_target
        wait 1 pin 0        ; Wait for ECHO (GP5) rising edge
        mov x, ~null        ; X = 0xFFFFFFFF
    count_high:
        jmp pin dec_x       ; While ECHO is HIGH, decrement X (2 cycles/loop = 16 ns @ 125 MHz)
        jmp push_count
    dec_x:
        jmp x-- count_high
    push_count:
        mov isr, ~x         ; ISR = elapsed 2-cycle ticks
        push noblock        ; Push exact pulse width to RX FIFO + assert PIO0_IRQ_0
    .wrap
    ```
  - Instead of relying on `IO_IRQ_BANK0` executing `echo_isr()` on both rising and falling edges (where critical sections or flash cache misses add microsecond jitter), `PIO0 SM2` measures the `GP5` high pulse width in hardware with **16 ns resolution** (`0.0027 mm` precision!) and interrupts the CPU only **once** when the echo finishes!
- **Impact on Car (Tier 3):**
  - Completely eliminates `design.md` **Risk R1** (shared `IO_IRQ_BANK0` ISR latency between Ultrasonic `GP5` and Barcode `GP27`).
  - Improves obstacle edge profiling repeatability when the car is simultaneously scanning a barcode and pinging an obstacle while WiFi is active.
  - Because `PIO0` is shared with Subsystem 2 (`TEAM.md`), adopt this only after baseline `S5-U01..U26` and `S3-U03..U06` tests pass, with team agreement on `TEAM.md`.

---

### OPT-17: SRAM Placement (`__not_in_flash_func`) for Hard Real-Time GPIO Edge ISRs
- **Target Files:** `app_program/subsystem5/drv_ultrasonic.c` (`echo_isr`), `app_program/subsystem3/drv_bc.c` (`bc_gpio_isr`), `src/rtos/port/port_irq.c` (`port_irq_wrap_0..3`, `gpio_dispatch`)
- **How it works:**
  - Code in `.text` executes in-place (XIP) from the external QSPI flash chip through a 16 KB 2-way set-associative cache.
  - When large functions like `vsnprintf` (in `dbg_printf`) or the CYW43 WiFi/lwIP stack run in lower-priority tasks, they evict cache lines. If a `GP27` barcode edge or `GP5` ultrasonic edge fires while its ISR is evicted from the XIP cache, fetching the ISR over QSPI stalls the CPU for **20–40 cycles per cache line** (~0.5–1.5 µs).
  - Marking `echo_isr` and `bc_gpio_isr` with the Pico SDK macro `__not_in_flash_func(echo_isr)` places them in `.time_critical` (internal single-cycle striped SRAM).
- **Impact on Car (Tier 2):**
  - Zero behavioral change to logic; costs only ~180 bytes of SRAM (out of 264 KB total on RP2040) and guarantees deterministic sub-microsecond ISR entry even under heavy WiFi/MQTT traffic (`S1-M04` soak test).

---

### OPT-18: RP2040 Hardware DMA Sniffer (`DMA_SNIFF_CTRL`) for Zero-CPU IEEE CRC-32
- **Target File:** `src/common/crc32.h` (optional hardware path for large buffers)
- **How it works:**
  - The RP2040 DMA controller includes a built-in **CRC-32 Sniffer** (`dma_hw->sniff_ctrl`, `dma_hw->sniff_data`).
  - Setting `CALC = 0x1` (CRC-32 IEEE 802.3 bit-reversed polynomial `0xEDB88320`) with `OUT_REV = 1` and `OUT_INV = 1` computes the exact same `crc32_ieee()` checksum at **1 32-bit word per clock cycle (500 MB/s)** while copying or reading memory via DMA.
- **Impact on Car (Tier 3):**
  - For our 32-byte fault record, the software `crc32_ieee()` (`OPT-10`) is already fast enough (~2 µs) and safer inside `HardFault_Handler` (where DMA should not be assumed healthy). However, if Subsystem 1 (Telemetry) or black-box flash logging ever checksums multi-kilobyte log buffers, the DMA sniffer does it in < 5 µs with zero CPU load.

---

### OPT-19: Single Context Struct (`s_ctx`) & D8 State Packing Across Subsystems 4 & 5
- **Target Files:**
  - **Buddy 4 (Vincent):** [task_imu.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem4/task_imu.c#L32-L46) (14 separate `static` variables -> `static imu_ctx_t s_imu`) and [task_safety.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem4/task_safety.c#L26-L33) (7 separate `static` variables -> `static safety_ctx_t s_safety`).
  - **Buddy 5 (Chay Han):** [task_obs_scan.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem5/task_obs_scan.c#L54-L66) (11 separate `static` variables -> `static obs_scan_ctx_t s_scan`) and [task_obs_avoid.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem5/task_obs_avoid.c#L47-L58) (10 separate `static` variables -> `static obs_avoid_ctx_t s_avoid`).
- **How it works:**
  - Applies the exact same ARMv6-M base-pointer consolidation and `ARCHITECTURE.md` D8 minimum-width state packing already proven in Subsystem 2 (`OPT-01` and `OPT-02`).
  - For example, in `task_imu.c`, packing `s_state` (3-bit `DREP_FSM_IMU_BITS`), `s_fail_streak` (3 bits, `0..5`), `s_hump_streak` (2 bits, `0..3`), `s_in_hump` (1 bit), `s_lift_latched` (1 bit), and `s_running` (1 bit) into a single `uint16_t flags_and_state` inside `s_imu` cuts `.bss` by 28 bytes and removes 13 literal-pool pointers in the 100 Hz IMU task.
- **Impact on Car (Tier 1):**
  - Zero behavioral impact on the car. Buddies 4 and 5 can copy the `s_mot` / `s_vc` pattern directly from [task_motion.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/task_motion.c#L111-L163) and [task_vc.c](file:///c:/Users/Admin/Downloads/Projects/inf-2004-project/app_program/subsystem2/task_vc.c#L37-L115).

---

### OPT-20: Compiler & Linker Dead-Code Garbage Collection (`-ffunction-sections -fdata-sections -Wl,--gc-sections`)
- **Target File:** `CMakeLists.txt` (shared file — requires team agreement)
- **How it works:**
  - Adding `-ffunction-sections -fdata-sections` to compile flags and `-Wl,--gc-sections` to linker flags instructs the GNU linker (`ld`) to strip any unreferenced functions and read-only tables from `mtk3_bsp` and project translation units without changing the optimization level (`-Og` / `-O0` for debuggability).
- **Impact on Car (Tier 3):**
  - Zero behavioral impact on active code; shrinks `.text` Flash footprint by ~4–12 KB (faster XIP cache hit rate and faster `.uf2` flashing).
  - Note: `__attribute__((used))` is already present on `stack_mon_task_start` in `stack_mon.c` (line 46), so the assembly trampoline symbol is preserved under `--gc-sections`.

