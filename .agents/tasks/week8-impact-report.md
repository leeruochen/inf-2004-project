# Week 8 change request: impact report (READ-ONLY investigation)

> ## OWNER CORRECTIONS (binding; these OVERRIDE this report and any earlier brief or suggestion)
> 0. **Names and student IDs** (TEAM.md and every file-header block) come ONLY from the Team Information table in 'INF 2004 Embedded Systems.md'. Copy them exactly as written there. Never take them from design.md and never invent them. GitHub usernames stay `TODO(owner-to-confirm)`. The same-name diffs for README.md and .gitignore are APPROVED.
> 1. **D2 (20 ms telemetry wake) is NOT approved.** It stays `TODO(owner-to-confirm, buddy 1)` in timing.h, ARCHITECTURE.md, TEAM.md and every doc. Never mark it settled, approved or final anywhere.
> 2. **RAM fault record must carry a magic number AND a CRC** (e.g. CRC-32 over the payload, magic first). Boot code prints and clears the record ONLY when both magic and CRC match; otherwise it treats RAM as random (power-on or after a flash) and silently re-initialises it. Document this in ARCHITECTURE.md D6, docs/DEBUG.md and DATA_REPRESENTATION.md. Implementation is Batch B; docs/plan only now.
> 3. **obstacle_profile_t staleness:** check what design.md 3.3 says about MBF_OBSTACLE ("Obstacle Scanning sends with TMO_POL (newest profile wins)"). A message-buffer send with TMO_POL FAILS (E_TMOUT) on a full buffer; it does NOT overwrite (verify in external/mtk3_bsp kernel/tkernel/messagebuf.c and cite). If newest-wins is wanted, document the drain-then-send pattern (sender drains with tk_rcv_mbf TMO_POL until empty, then sends; note the race if the receiver is concurrently waiting, and that a 2-slot buffer only needs the drain when full). **KEEP the optional 2-bit sequence number** in obstacle_profile_t until the owner decides; do not drop it. Mark it `TODO(owner-to-confirm, buddy 5)`.
> 4. **NFR2 wording: do NOT edit any team document** (design.md, specs.md, 'INF 2004 Embedded Systems.md'). Record the proposed wording and its reason in the deltas (ARCHITECTURE.md D7) for the team to approve: proposed "Wheel encoder pulses shall be captured by hardware (PIO quadrature decoding) on both wheels independently, with zero missed counts under normal operating speed (target: up to 50 cm/s)." Reason: PIO decoding counts with zero CPU and keeps counting through kernel interrupt-masked sections. Note that "zero missed counts" needs a top-speed test (e.g. drive a measured distance at 50 cm/s and compare counts vs expected, both wheels). Mark `TODO(owner-to-confirm, team)`.

**Status header.** The template merge (Option C, `.agents/tasks/template-merge-report.md` section 3) has NOT been executed: `app_program/` does not exist in the repo root. Under the owner's prerequisite order, **no file may be edited for this task yet**. This report is the only file written. Nothing was edited, staged or committed.

**Repo state differs from the brief.** HEAD is `79bc417 "project template given by prof"` (= origin/main), not `56826c7`. `56826c7` is now `backup/pre-template-merge`. `ProjectTemplate/` is **committed and pushed** in 79bc417, not untracked. The merge report says it "must never be committed" (template-merge-report.md risk 3.4.3). Running `cmake -P cmake/check_rules.cmake` today prints `check_rules: 94 violation(s)`, all inside `ProjectTemplate/`, so every ALL build fails. Working tree: `M NOTICE.md` (owner's edit, not touched) and `?? "INF 2004 Embedded Systems.md"`.

Sources: "the document" = `INF 2004 Embedded Systems.md` (Week 8 Tribunal 1, the source of truth). Line numbers are cited as `doc:N`. Kernel = `external/mtk3_bsp` @ 15ed232 (verified with `git rev-parse`). SDK = `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk` (cited as `SDK/...`). pico-examples is **not on disk**. The PIO program was read from GitHub tag `sdk-1.5.1` (`raspberrypi/pico-examples/pio/quadrature_encoder/`). The design.md statecharts are PNG images (design.md:446-458). I extracted them to `%TEMP%` (outside the repo) and counted the states by eye from low-resolution images (330 to 660 px wide). Treat the exact state names as UNVERIFIED wording.

---

## 0. Summary answer

1. **Pins.** Every old-pin reference is in 4 repo files: `config/pins.h`, `CMakeLists.txt`, `docs/DEBUG.md` and the document. design.md has more, report only. The new wiring kills the UART0 console: GP0/GP1 become ENC_R A/B, and `pico_w.h:25,28` makes GP0/GP1 the default stdio UART pins. USB CDC becomes the only console. The kernel's boot/fault output (`tm_snd_dat`) loses the destination planned for it (polled UART0, batchA-summary item 13). There is no PWM clash. Motors use slices 4/5. Servo GP15 = slice 7 B, GP12 = slice 6 A, so neither shares a slice with the 20 kHz motors. `pins.h` will `#error` as soon as GP16/GP17 are allocated, because `PINS_SPARE_MASK` still lists them (pins.h:167-169).
2. **Quadrature.** I recommend **PIO0, one SM per wheel, sharing one copy of the pico-examples program.** It is 24 of 32 instructions and must sit at offset 0 (`.origin 0`, computed `MOV PC, ISR`). The pin base is per SM (`sm_config_set_in_pins`), so GP0 and GP16 work with one copy. It costs zero CPU, adds no IO_IRQ_BANK0 load, and keeps counting during kernel PRIMASK sections, which an interrupt handler cannot do. This removes the encoders from risk R1. The example's reader uses `pio_sm_get_blocking` (SDK pio.h:1126-1130, a spin). Our rule R2 does not catch it, so we must use a non-blocking read. PWM B-input counting (option 3) is **not viable**: GP1 and GP17 are both PWM **slice 0** channel B, and GP0/GP16 are channel A. A PIO design also changes NFR2's wording ("captured using GPIO interrupts", design.md:93), so it needs a delta.
3. **Data representation.** Every width the document states is sufficient. Several fields are MISSING: `motion_cmd_t` distance and steering, obstacle clearances, all of `nav_input_t`, `motion_state_t` and `tel_evt_t`, the IMU, WiFi and VC state widths, and the pitch sign. The **flattened system statechart has 10 software states, more than 3 bits allow**. Use a 3-bit top level plus a 3-bit navigation sub-state, or 4 bits. LF (5), barcode (5) and obstacle (8 leaves + 1 composite = 9) fit their stated widths. Kernel facts: an eventflag pattern is `UINT` = 32 bits. `TWF_BITCLR` clears **all bits of `waiptn`** on release, not just the bits that matched, and the returned pattern is captured **before** the clear (eventflag.c:165-171, 257-262). So Safety waiting on `COLLISION|IMU_ALIVE` with `TWF_ORW|TWF_BITCLR` is correct and never touches other owners' bits. `TWF_CLR` zeroes the whole word. A second `tk_wai_flg` (even `TMO_POL`) on a `TA_WSGL` flag while Safety waits returns **E_OBJ** (eventflag.c:249-251), so other readers must use `tk_ref_flg`.
4. **RAM.** Message-buffer slots round to 4 B plus a 4 B header (messagebuf.h:58-62, messagebuf.c:65). So packing `nav_input_t` (already 4 B) saves **0 B**. Message buffers and stacks live inside the fixed 96 KB static kernel pool (priorities.h:124), so **map-file RAM does not change when message-buffer sizes shrink**. Only static arrays move in the map, such as the edge ring (−352 B). Message-buffer savings must be reported from `MBF_BUFSZ` arithmetic and pool use. The total realistic saving is about 0.6 KB.
5. **Baseline.** No firmware target exists yet (`CMakeLists.txt:134-135` skips it). So the "first working Week 6 design build" must be produced by **Batch B-baseline itself**. B-packed then converts the types. Host tests (planned for Batch D) must be pulled forward into B-packed.

---

## A. Pins

### A.1 Every reference to the old pins and console (excluding external/, build/, ProjectTemplate/)

| File:line | Content | Impact |
|---|---|---|
| config/pins.h:5-6 | header comment: "UART0 on GP0/GP1, Grove 1 reserved" | rewrite |
| config/pins.h:38-41 | `ENC_L_PIN (6U)` "Grove 5", `ENC_R_PIN (7U)` "Grove 7" | replace with ENC_R_A/B = 0/1, ENC_L_A/B = 16/17 + `ENC_*_DIR_INVERT` |
| config/pins.h:44-54 | `IR_L_PIN (26U)`, `IR_C_PIN (27U)`, `IR_L_ADC_CH (0U)`, `IR_C_ADC_CH (1U)`, "Grove 6" | rename to IR_R (26/ADC0) and IR_C (28/ADC2) |
| config/pins.h:56-62 | `IR_BC_PIN (28U)`, VBAT jumper comment | IR_BC = 27. The jumper comment moves to IR_C (it is still needed, now for the ADC) |
| config/pins.h:66 | "Grove 2 must stay unused" | unchanged (IMU still on GP2/GP3) |
| config/pins.h:87-91 | `SERVO_PIN (12U)` "Servo port 1" | 15 [FILL: confirm] |
| config/pins.h:93-104 | `DBG_UART0_TX_PIN (0U)`, `DBG_UART0_RX_PIN (1U)`, `DBG_UART0_BAUD_BPS` | delete. The pins become encoder pins |
| config/pins.h:30-33 | `MOTOR_L_DIR_INVERT`, `MOTOR_R_DIR_INVERT` | **already exist** (TUNE, 0U). Move or keep them next to the new `ENC_*_DIR_INVERT` |
| config/pins.h:139-161 | `PINS_ALLOCATED_MASK/SUM` include DBG_UART0_*, ENC_L/R, IR_L/IR_C/IR_BC | update the terms |
| config/pins.h:167-169 | `PINS_SPARE_MASK` = GP13-15, GP16, GP17 | must become GP6, GP7, GP12, GP13, GP14. Otherwise `#error "a spare pin is allocated"` (pins.h:179-181) |
| config/pins.h:183-189 | `(IR_L_PIN-26) != IR_L_ADC_CH` checks | rename. The formula still holds for GP28→ADC2 |
| CMakeLists.txt:8-9 | configure examples `build/uart0 ... -DDBG_BACKEND=UART0` | change to USB |
| CMakeLists.txt:42-47 | `DBG_DEFAULT "UART0"`, `DBG_BACKEND` STRINGS UART0 USB, validation regex | default USB; reject UART0 (see E.4) |
| CMakeLists.txt:80-84 | `DBG_BACKEND_UART0=1` / `DBG_BACKEND_USB=1` | keep the USB define |
| CMakeLists.txt:153 | `hardware_uart` linked | harmless; may stay. **`hardware_pio` is not linked** and must be added for the PIO design |
| CMakeLists.txt:160-166 | `pico_enable_stdio_uart(robocar 1)` when UART0 | must never be enabled: stdio UART takes GP0/GP1 (`SDK/src/boards/include/boards/pico_w.h:22-28`) |
| docs/DEBUG.md:3-18, 29, 40, 44, 55-56, 77-78, 96-97, 103 | probe UART wiring to Grove 1, `build/uart0`, `DBG_DEFAULT UART0`, "Phase 1 acceptance verified on UART0. USB CDC is an extra bring-up test" | rewrite: USB CDC console mandatory, Debug Probe SWD only |
| tests/rules_fixtures/** | no pin or console references (grep: no match) | none |
| cmake/check_rules.cmake, config/timing.h | no pin references. `timing.h:186-187` `DBG_USB_CONNECT_WAIT_MS 3000` already supports USB | none |
| .agents/tasks/batchA-plan.md:209-217, batchA-summary item 13, integration report:21,28,80,153-155 | old pin plan; "fallback is UART0 on GP0/GP1"; tm_com on UART0 | notes only, now stale |
| design.md:130-134, 136, 141, 385-386, 396, 429; 4 (text, "GP6/GP7", "GP28") | old pin table and rationale | report only (never edited) |
| doc:40, 41, 43, 53, 68, 72, 83, 84, 86 | GP6/GP7 encoders, GP28 barcode, GP12 servo, IR_L/IR_C ADC0/ADC1, Grove 7 split | report only. The document is READ ONLY |

### A.2 New pin map

PWM slice and channel come from `pwm_gpio_to_slice_num = (gpio>>1)&7` and `pwm_gpio_to_channel = gpio&1` (`SDK/src/rp2_common/hardware_pwm/include/hardware/pwm.h:78-93`).

| Pin | Grove/port | Signal | Function | Owner buddy | Peripheral | SDK owner (one per peripheral) | PWM slice/ch (info) |
|---|---|---|---|---|---|---|---|
| GP0 | Grove 1 | ENC_R_A | right encoder phase A (provisional, not physically verified) | 2 | PIO0 SM0 (recommended) or GPIO IRQ | Motion Control (`drv_encoder`) | 0A |
| GP1 | Grove 1 | ENC_R_B | right encoder phase B (provisional) | 2 | PIO0 SM0 | Motion Control | 0B |
| GP2 / GP3 | Grove 2 / Maker | IMU_SDA / IMU_SCL | GY-511 LSM303DLHC (no gyro), accel 0x19, mag 0x1E (VERIFY by bus scan; thresholds.h:39-44) | 4 | I2C1 | IMU Processing | 1A/1B |
| GP4 | Grove 3 | US_TRIG | HC-SR04 trigger | 5 | SIO GPIO out | Obstacle Scanning | 2A |
| GP5 | Grove 3 | US_ECHO | echo pulse (IO_IRQ_BANK0); must be 3.3 V-safe (A8) | 5 | GPIO IRQ | Obstacle Scanning | 2B |
| GP6, GP7 | Grove 5 / Grove 7 second lead | spare | see A.3 risk 5 | — | — | — | 3A/3B |
| GP8 / GP9 | onboard | MOTOR_L_A / B | left motor PWM 20 kHz | 2 | PWM slice 4 | Motion Control | 4A/4B |
| GP10 / GP11 | onboard | MOTOR_R_A / B | right motor PWM 20 kHz | 2 | PWM slice 5 | Motion Control | 5A/5B |
| GP12, GP13, GP14 | servo headers | spare | — | — | — | — | 6A/6B/7A |
| GP15 | servo header | SERVO_SIG | scanning servo 50 Hz [FILL: confirm GP15 or revert to GP12] | 5 | PWM slice 7 ch B | Obstacle Scanning | 7B |
| GP16 | Grove 4 | ENC_L_A | left encoder phase A (provisional) | 2 | PIO0 SM1 | Motion Control | 0A |
| GP17 | Grove 4 | ENC_L_B | left encoder phase B (provisional) | 2 | PIO0 SM1 | Motion Control | 0B |
| GP18 | onboard | NEOPIXEL | reserved (WS2812 via PIO) | — | (PIO0 spare SM, later) | — | 1A |
| GP20 / GP21 | onboard | BTN0 / BTN1 | reserved | — | — | — | 2A/2B |
| GP22 | onboard | BUZZER | reserved | — | — | — | 3A |
| GP23-25, GP29 | internal | CYW43 | radio (later): PIO1 + 2 DMA channels | 1 | PIO1 | Telemetry (Phase 2) | — |
| GP26 | Grove 5 (shared lead with Grove 6) | IR_R | car's RIGHT line sensor AO | 3 | ADC ch 0 | Line-Following | 5A |
| GP27 | Grove 6 | IR_BC | barcode sensor DO, digital edge IRQ | 3 | GPIO IRQ (IO_IRQ_BANK0) | Barcode (ISR) | 5B |
| GP28 | Grove 7 | IR_C | CENTER line sensor AO; GP28 VBAT-sense jumper must stay OPEN | 3 | ADC ch 2 | Line-Following | 6A |

The Grove pairs are Grove 5 = GP6 + GP26, Grove 6 = GP26 + GP27, Grove 7 = GP7 + GP28. This is inferred from design.md:130-134 and 429 and from the owner's text. Which physical lead is which signal is UNVERIFIED: the ROBO PICO datasheet is not on disk.

### A.3 Conflicts and risks

1. **GP0/GP1 encoder vs the UART0 console.** This is a hard conflict. The `pico_w` board defaults stdio UART to UART0 on GP0/GP1 (`pico_w.h:22-28`), and `CMakeLists.txt:160-162` enables it for `DBG_BACKEND=UART0`. With the new wiring, a UART0 build would drive GP0 as UART TX against the right encoder's A output. The guard is in E.4.
2. **Boot and fault console.** `tm_printf` goes to `tm_snd_dat`, which the port must supply (cmake/mtk3.cmake:16-18). Batch A planned polled UART0 for it (batchA-summary item 13). That is now impossible. USB CDC cannot be polled from a HardFault or with PRIMASK set, because TinyUSB needs its IRQ and alarm (integration report:283-300). Recommendation: `tm_snd_dat` writes into a RAM fault record (in `.noinit`) that the probe reads over SWD/GDB, and the dbg drain prints it after the next boot. It must never write to stdio. Owner decision needed (F, Q16).
3. **PWM slices: no clash.** Motors use slice 4 (GP8 A, GP9 B) and slice 5 (GP10 A, GP11 B), both 20 kHz (thresholds.h:115). Servo GP15 = slice 7 ch B, and GP12 = slice 6 ch A. Neither shares a slice with the motors, so 50 Hz and 20 kHz never meet. Consequence of GP15: slice 7's other output GP14 (spare) can only be 50 Hz PWM or plain GPIO. With GP12, the same applies to GP13. GP26/GP27 are slice 5 outputs, but they are used as ADC/GPIO, so there is no conflict. Buzzer GP22 = slice 3 A, the same channel as spare GP6. Never use both as PWM.
4. **ADC channels.** (ADC0, ADC1) become (ADC0, ADC2). The `(pin−26)==channel` checks (pins.h:183-189) still hold. The ADC peripheral has one owner (Line-Following). Round-robin mask becomes channels 0 and 2.
5. **GP27/GP28 swap.** GP28 is now an ADC input, so the VBAT-sense jumper must still stay OPEN (design.md:134 gave the old reason). If it is closed, ADC2 reads the battery divider and the centre sensor is corrupted. GP27 is ADC-capable but is now a digital edge input. `adc_gpio_init()` must **never** be called on GP27, because it disables the digital receiver (`SDK/.../hardware_adc/include/hardware/adc.h:69-76`: `gpio_set_input_enabled(gpio,false)`, pulls off). On the IR_R/IR_C pins, by contrast, `adc_gpio_init()` is required: it disables the input buffer and pulls so the analogue reading is clean.
6. **Second lead on each Grove cable (hardware risk).** A TCRT5000/LM393 module has both AO and DO. On Grove 5 (GP6 + GP26) and Grove 7 (GP7 + GP28), the module's DO may land on GP6/GP7. Then GP6/GP7 are "spare" only as **inputs** and must never be configured as outputs. On Grove 6 (GP26 + GP27), if the barcode module's AO is wired to the GP26 lead, two analogue outputs drive GP26. The owner says that lead "stays unconnected". This needs a physical check (photo plus the doc's own pin test, doc:72).
7. **Power.** Each device draws 3V3 from its own Grove cable. Total 3.3 V load must stay within `POWER_GROVE_3V3_BUDGET_MA 300` (thresholds.h:134-136, itself TODO verify). Encoders must take 3V3 (not VIN, not the servo header V pins). The 6-pin motor/encoder connector gives no phase A/B labels, so the A/B order is set in firmware by `ENC_*_DIR_INVERT`.

### A.4 Line module and calibration changes (Buddy 3)

- Rename IR_L→IR_R. design.md 6.1 (line 244 text) uses "IR_L and IR_C", and LINE_LOST "records which side the line was last seen on". With RIGHT + CENTER, the car can only see the line under the right or the centre sensor. A line drifting left is inferred when both sensors read light and the centre was last dark. The steering error sign must be **redefined**, not copied. Proposed convention for Buddy 3 to confirm: `steer_err > 0` = line is to the car's right = steer right (right wheel slower). Record it in DATA_REPRESENTATION.md and test it on the car.
- Calibration: one baseline (min/max) per sensor on ADC0 and ADC2. NFR7 (5 ms sampling) and A1 are unchanged.
- The junction rule ("both sensors dark for N samples") is unchanged in form. The doc's own concern applies even more now: with only right + centre, a left branch cannot be seen (doc:41 "Two line sensors cannot tell a left branch from a right branch").

---

## B. Quadrature decoding

Symbols: CPR = encoder cycles per wheel revolution per channel (gearbox included; unknown). X = decoding multiplier (1, 2 or 4). C = wheel circumference in cm (unknown). v_max = `MOTION_SPEED_MAX_CMPS` = 50 cm/s (thresholds.h:110). f_sys = 125 MHz (integration report:78). c_isr = cycles per encoder interrupt, including entry, exit and SDK shared-chain dispatch.

- Edge rate per wheel: **e = CPR · X · (v/C)**. Both wheels: **E = 2 · CPR · X · v_max / C** edges/s.
- CPU load (IRQ designs): **U = E · c_isr / f_sys**. Example per microsecond of ISR: U = E · 1 µs.
- Barcode timing error from one encoder ISR in the way: added latency up to **t_j = c_isr/f_sys** (plus any kernel PRIMASK section). Relative bar-width error = **t_j · v / w_n** (w_n = narrow bar width). Echo range error = **t_j · 343 m/s / 2 ≈ 0.17 mm per µs**, which is negligible.

### B.1 Option 1: GPIO IRQ on IO_IRQ_BANK0

- Mechanism: `gpio_add_raw_irq_handler_masked(mask, handler)` (`SDK/.../hardware_gpio/include/hardware/gpio.h:578`; the order-priority variant is at :516), wrapped by our planned `hal_gpio_irq_register()`. That wrapper **does not exist yet**: there is no `src/`. It must do what `knl_hll_inthdr` does (`knl_taskindp++/--`, integration report:125, 139-141) if the handler makes a kernel call. A pure counter handler makes no kernel call, so it would not strictly need the wrapper. Keep the rule anyway, for uniformity.
- **X2:** IRQ on both edges of A; direction from the level of B. e = 2·CPR·v/C per wheel.
- **X4:** IRQ on both edges of A and B; 4-bit `(prev<<2)|curr` table lookup → −1/0/+1. e = 4·CPR·v/C per wheel. That is twice the X2 rate, but it rejects bounce.
- c_isr estimate: about 60-120 cycles (0.5-1 µs). That covers M0+ entry/exit, the SDK chain walk, `gpio_acknowledge_irq`, reading `sio_hw->gpio_in`, the table and the store. **UNVERIFIED; measure** with a GPIO toggle and a logic analyser (doc test plan, doc:154).
- Kernel interaction: kernel critical sections mask all IRQs (PRIMASK; integration report:23). An edge pair arriving during a long PRIMASK section collapses into one pending IRQ, which miscounts in X4. That is the A2 "missed counts" risk.
- Shared IRQ: encoders (4 pins) share IO_IRQ_BANK0 with echo GP5 and barcode GP27 (and later CYW43 GP24, integration report:141). The doc's risk R1 (doc:151) and the shared-resource row (doc:68) apply in full. Give barcode and echo a higher order priority so their timestamps are taken first. A handler that is already running still delays them by t_j.
- Count word: `static volatile int32_t s_enc_count[2]`. Each element is 4-byte aligned. The ISR is the only writer, and IRQs of one vector do not nest, so the ISR's own read-modify-write is safe. A task read is one LDR (atomic). Use **free-running counts and deltas**. Do not "read and clear" as design.md 3.3 (line 199) says, because clearing makes the task a second writer.

### B.2 Option 2 (recommended): PIO0, one SM per wheel

Verified against `pico-examples` tag `sdk-1.5.1`, `pio/quadrature_encoder/quadrature_encoder.pio` and `.c`:

- **Size: 24 instructions.** The jump table has 4+4+4+2 entries, plus the `decrement` JMP (15), the main loop (`MOV ISR,Y; PUSH noblock; OUT ISR,2; IN PINS,2; MOV OSR,ISR; MOV PC,ISR`, 6), and `increment` (3). The PIO block holds 32 instructions, leaving 8.
- **It must be loaded at offset 0** (`.origin 0`, "uses computed jumps", `MOV PC, ISR`). One copy serves both SMs. Load it first on PIO0 with `pio_add_program(pio0, ...)` (the example) or `pio_add_program_at_offset(pio0, prog, 0)` (`SDK/.../hardware_pio/include/hardware/pio.h:502`).
- **The pin base is per SM, not fixed.** `quadrature_encoder_program_init(pio, sm, pin, max_step_rate)` calls `sm_config_set_in_pins(&c, pin)` and `sm_config_set_jmp_pin(&c, pin)` (pio.h:164, 277). It reads `IN PINS, 2` = pin and pin+1, with A on the lower pin. The program never uses `JMP PIN`. So SM0 base GP0 and SM1 base GP16 work with the same program. The wiring constraint "A on the lower pin" can be absorbed by `ENC_*_DIR_INVERT` (swapping A/B only flips the sign).
- The sdk-1.5.1 init does **not** call `pio_gpio_init` (input only) and enables pull-ups on both pins. The current master adds `.pio_version 0`, which the SDK 1.5.1 pioasm would not accept: **copy the sdk-1.5.1 variant**. `pioasm.exe` is in `pico-sdk-tools/`, and `pico_generate_pio_header` is defined at `SDK/tools/CMakeLists.txt:16`. Copying it means a BSD-3-Clause notice in NOTICE.md. The owner is editing that file now, so this is the owner's change.
- Maximum step rate is sysclk/10 = 12.5 Msteps/s at clkdiv 1, many orders above our need. This is X4 decoding with direction. Invalid double transitions are ignored silently.
- **Reading.** The program pushes the count without blocking (`PUSH noblock`), so the 4-deep RX FIFO holds stale values. The example reader drains `level+1` entries with `pio_sm_get_blocking` (`pio.h:1126-1130`: `while (pio_sm_is_rx_fifo_empty) tight_loop_contents();`). That is a blocking spin, banned in tasks by the no-blocking rule, and R2's regex does not list it (check_rules.cmake:58). Use a non-blocking reader: drain with `pio_sm_get` while `!pio_sm_is_rx_fifo_empty` (pio.h:1018, 1044), then poll again a bounded number of times (a fresh push arrives within about 10 SM cycles), else keep the last value. Add `pio_sm_get_blocking` / `pio_sm_put_blocking` to R2.
- Kernel interaction: **no ISR and no taskindp.** Motion Control (P3) polls both SMs every 20 ms tick (MOTION_PERIOD_MS, timing.h:83) and computes deltas (32-bit two's-complement wrap is safe). Counting continues during kernel PRIMASK sections.
- Count word: the PIO's Y register is the real counter. The **Motion task** is then the single writer of the aligned 32-bit `enc_count` word that it publishes in `motion_state_t` under MTX_MOTION. This changes the doc's justification "the ISR (the only writer)" (doc:115) to "the Motion task (the only writer)". Record it.
- Ownership: PIO0 is owned by Motion Control (claim SM0/SM1 with `pio_sm_claim`, pio.h:1265). CYW43 prefers PIO1 (`CYW43_SPI_PIO_PREFERRED_PIO 1`, `SDK/.../pico_cyw43_driver/cyw43_bus_pio_spi.c:71-72`). It falls back to PIO0 only if PIO1 cannot take its program (lines 94-99), so the CYW43 plan (PIO1 + 2 DMA channels) is unaffected. A later NeoPixel on PIO0 would need the remaining 8 instructions and an SM. The ws2812 program size is UNVERIFIED, and the use needs a TEAM.md peripheral row because PIO0 would have two owners at SM granularity.
- Encoders leave IO_IRQ_BANK0, which then carries only echo GP5 and barcode GP27. R1 for encoders disappears.

### B.3 Option 3: PWM slice B-input edge counting (X1). Rejected

`PWM_DIV_B_RISING` counts rising edges on the slice's **channel B pin** only (pwm.h:52-54). Channel B pins are odd GPIOs: GP1 → slice 0 B and **GP17 → slice 0 B** (`(17>>1)&7 = 0`). Both wheels' B channels land on the same slice, so only one wheel could be counted. GP0 and GP16 are channel A and cannot count. The option also gives X1 only and no direction (`pwm_get_counter`, 16-bit, pwm.h:334).

### B.4 Recommendation

**Option 2 (PIO0).** It costs zero CPU, removes encoder jitter from barcode and echo timestamps, gives direction (needed for reverse moves and U-turn pivots, NFR4), keeps working through PRIMASK, and leaves the encoder count word with a single writer. The costs: link `hardware_pio`, add a pioasm step, add a BSD-3 notice, use 24/32 PIO0 instructions, and record a delta against NFR2 ("captured using GPIO interrupts", design.md:93). If the owner rejects the NFR2 deviation, fall back to Option 1 X2 (half the IRQ rate of X4) and measure c_isr.

---

## C. Data representation

Baseline sizes assume the field list from the owner's earlier proposal with `<stdint.h>` types (NFR16, design.md:107) and natural ARM EABI alignment. B-baseline must **measure** them (D.2). Message-buffer slot = `4 (HEADER) + ROUNDSZ4(sizeof T)` (`kernel/tkernel/messagebuf.h:58-62`, `messagebuf.c:65`). The free-space check uses the unrounded size (messagebuf.h:68-71), but consumption uses the rounded one.

### C.1 Messages

| Item | Week 6 baseline (unpacked) | Document decision | MISSING / issue | Proposed packing (bits) |
|---|---|---|---|---|
| `nav_input_t` (MBF_NAV_IN, 8 slots) | `source u8, type u8, union{steering_err_q8 i16; nav_cmd u8; bypass_cmd u8; remote_cmd u8}` = **4 B**; slot 8 B; **64 B** | "message buffers ... nav_input_t ... masks and shifts" (doc:128); no widths | every width | src 2 (LINE, BARCODE, AVOID, REMOTE) · type 3 (per source: STEER/JUNCTION/TRACK_LOST; CMD; SLOW/BYPASS/RELEASE/BLOCKED; START/STOP/RESET) · payload 8 (steer int8 Q7 = ±1.0 normalised, or nav_cmd 2, or bypass step 3, or remote 2) · OOR 1 = **14 bits → u16**. Slot still 4+4 = **8 B → saving 0 B** |
| `motion_cmd_t` (MBF_MOTION_CMD, 4) | `type u8, distance_cm u16, angle_deg i16, speed_cmps u8, steer_q8 i16` = 10 B; slot 16 B; **64 B** | type 3, speed 0-50 → 6, angle 0-180 → 8 (doc:118, 172) | **no distance field** (FR4 `moveForward(distance)`, NFR4 ±3 %) and **no steering field** (line-following correction VC→Motion) | type 3 (STOP, FWD, BACK, TURN_L, TURN_R, UTURN, STEER, BRAKE) · speed 6 · **arg 9** shared by type: angle 0-180 (8 bits used), distance 0-511 cm (0 = continuous), steer signed 9-bit · OOR 1 = **19 bits → u32**; slot 8 B; **32 B** |
| `obstacle_profile_t` (MBF_OBSTACLE, 2) | `bearing_deg i16, width_mm u16, closest_mm u16, clear_left_mm u16, clear_right_mm u16, valid u8, timestamp_ms u32` = 16 B; slot 20 B; **40 B** | distance 2-400 cm @1 cm = 9; bearing index 0-12 = 4; width 0-400 mm @5 mm = 7; bypass state 4 (doc:119, 175) | **clearances L/R missing** (specs.md "Left-side clearance / Right-side clearance"; design.md:157 "bearing, width, clearance"); `valid`, timestamp dropped; "bypass state" is Avoidance's state inside a Scanning→Avoidance message (ownership question, F) | as doc + clear_L 7 + clear_R 7 (0-635 mm @5 mm, saturating "≥635 = clear") + OOR 1 = **39 bits → 5 B**; slot 12 B; **24 B**. Distance stored raw 0-511, with **0 = no echo** as the sentinel instead of `valid` |
| `motion_state_t` (MTX_MOTION; not a queue) | `speed_cmps_x10 u16, distance_cm_x10 u32, enc_count[2] i32, tick_ms u32` = **20 B** (2 B pad) | "motion_state_t ... packed" (doc:115); encoder words unpacked | widths unspecified; the doc calls it a "message" but it is mutex-shared data | word0: speed signed 10 bits (±51.1 cm/s @0.1) + distance 16 bits @1 cm (wraps at 655 m; assumes a run < 655 m) + 6 spare/OOR; enc_count[2] 2×32 **unpacked** (decision); tick 32 → **16 B** |
| `tel_evt_t` (MBF_TELEMETRY, 32) | `type u8, timestamp_ms u32, union{small payload}`. Payload size UNVERIFIED; with 8 B payload = 16 B; slot 20 B; **640 B** | JSON → compact binary (doc:111, 126, 171) | layout unspecified | w0: type 5 · timestamp 24 bits ms (4.66 h, wrap handled by the dashboard) · flags 3; w1: 32-bit per-type payload = **8 B**; slot 12 B; **384 B** |
| Hump report (in tel_evt payload) | none defined | 0-100 mm @1 mm = 7 bits (doc:174) | — | height 7 (101 values ≤ 128, OK) + OOR 1 |
| Pitch | none defined | "pitch 45 at 0.5 = 8 bits" (doc:174) | **range and sign not stated.** 0..45 @0.5 = 91 values = 7 bits; ±45 = 181 values = 8 bits, so the doc's "8 bits" implies **signed ±45°**. 8-bit signed @0.5° covers −64.0..+63.5° | int8 Q1 (0.5°), clamp ±45° + OOR; sign-extend on unpack |
| Barcode command | `nav_cmd u8` | A-D = 2 bits (doc:173, 196) | invalid reads are never posted (design.md:248), so 2 bits suffice | 2 |
| MQTT wire frames | JSON (design.md:14 "JSON/struct payloads"); nothing implemented | version byte first, little-endian, masks/shifts; commands rejected unless version and length match; rejects logged and never reach the VC (doc:111, 186) | no JSON reference format exists to measure "before" sizes (F) | frame = `ver u8 · type u8 · len u8 · payload[len]` LE. Command accepted iff `ver == TEL_WIRE_VER`, `type` known, `len == expected_len[type]`, MQTT payload length == 3+len; else `dbg_` log + counter, never posted to MBF_NAV_IN |

Width checks of the document's figures: speed 0-50 → 51 values ≤ 64 (6 bits) OK. Angle 0-180 → 181 ≤ 256 (8) OK. Obstacle distance 2-400 → 399 values, and storing raw 0-511 in 9 bits also fits, OK. Bearing 0-12 → 13 ≤ 16 (4) OK; this assumes bearings 30°+10°·i over 30-150° (specs.md coarse 30-150; `OBS_FINE_STEP_DEG 10`, timing.h:128). But `OBS_FINE_HALF_SPAN_DEG 30` (timing.h:130) around 30° or 150° reaches 0°/180°, outside index 0-12: clamp or confirm. Width 0-400 @5 → 81 ≤ 128 (7) OK. Hump 0-100 → 101 ≤ 128 (7) OK. Pitch: see the table. Barcode 4 values (2) OK.

### C.2 Barcode edge ring

- Baseline 64 × `uint64_t` = **512 B** (doc:117). Decision: 20-bit gaps, 64 × 20 = 1280 bits = **160 B**. The bound holds because `BC_CAPTURE_TIMEOUT_MS 1000` (timing.h:157-159, still "TODO provisional") × 1000 = 1,000,000 µs < 2^20 = 1,048,576, a 4.6 % margin. Add `_Static_assert(BC_CAPTURE_TIMEOUT_MS*1000UL < (1UL<<BC_GAP_BITS))` so a timeout increase fails the build.
- Packing options compared:
  - (a) Bitstream of 40 × u32: entry i at bit 20i. Entries cross a word when `(20i mod 32) > 12`, which costs two read-modify-writes in the ISR (about 30-40 cycles).
  - (b) **lo16[64] (128 B) + hi-nibble[32] (32 B) = 160 B.** The ISR does one STRH plus one byte read-modify-write (about 10 instructions). There is no cross-word shift.
  - (c) 3 B/entry = 192 B. Simple, but misses 160 B.
  - (d) 4 entries per 80-bit group (16 groups) = 160 B, with more shifting than (b).
  - **Recommend (b).** The nibble byte is shared by entries 2k and 2k+1. The ISR is the only writer and the task only reads, so the ISR's own byte read-modify-write is safe.
- ISR: `now = time_us_32(); gap = now − s_last; s_last = now; if gap > 0xFFFFF gap = 0xFFFFF` (saturated = "start of capture / timeout"). `time_us_32` is SDK-side, fine in a `drv_` file. Unsigned subtraction is wrap-safe.
- Index ownership: `head` (u8, ISR is the only writer) and `tail` (u8, Barcode task is the only writer), both free-running mod 256 and masked `&63`. Full when `(u8)(head−tail) == 64`. Overflow: the ISR increments `ovf_count` (u8, single writer). The task compares it with its snapshot and enters INVALID_READ_RESET. No PRIMASK section is needed.
- R1: the extra cost over a 64-bit store is a few cycles. Measure it against the baseline (doc:155).

### C.3 Eventflags (kernel-verified)

- Pattern width: `UINT flgptn` (eventflag.h:30; `tk_set_flg(ID, UINT)` syscall.h:752). `UINT` = `unsigned int` (tk/typedef.h:58) = **32 bits** on arm-none-eabi.
- `TWF_ANDW 0x00`, `TWF_ORW 0x01`, `TWF_CLR 0x10`, `TWF_BITCLR 0x20` (include/tk/syscall.h:89-92).
- `TWF_BITCLR` does `flgptn &= ~waiptn`: it clears **every bit in the wait pattern**, not only the bits that matched (eventflag.c:170-171 on release in `tk_set_flg`; 260-261 on an immediate match). `TWF_CLR` sets `flgptn = 0` (eventflag.c:175-177, 263-264).
- The released task's `*p_flgptn` is copied **before** the clear (eventflag.c:165, 257). Safety therefore always sees COLLISION even when BITCLR removes it in the same call.
- `tk_clr_flg` ANDs: `flgptn &= clrptn` (eventflag.c:204). To clear bit B, call `tk_clr_flg(id, ~B)`.
- Both `tk_set_flg` and `tk_clr_flg` run inside `BEGIN_CRITICAL_SECTION` (eventflag.c:146-181, 200-206), so the doc's "atomic" claim (doc:119) holds. From our SDK-installed ISRs they are legal only inside the taskindp wrapper (integration report:139-141).
- `TA_WSGL` (the default): if a task is already waiting, any other `tk_wai_flg`, **including TMO_POL**, returns `E_OBJ` (eventflag.c:249-251). Readers of EVF_SAFETY other than Safety (VC, Motion reading FAULT_LATCHED) must use `tk_ref_flg` (enabled: `USE_FUNC_TK_REF_FLG`, config/mtk3/config_func.h:81), or the flag must be created with `TA_WMUL` (syscall.h:85).

| Bit (proposed, one eventflag = EVF_SAFETY extended) | Setter | Clearer | In Safety's wait pattern? |
|---|---|---|---|
| b0 COLLISION | IMU Processing | Safety (via TWF_BITCLR) | **yes** |
| b1 IMU_ALIVE | IMU Processing (each sample) | Safety (via TWF_BITCLR) | **yes** |
| b2 FAULT_LATCHED ("collision latch") | Safety only | **Vehicle Controller only** (reset while stationary) | no |
| b3 FAULT_STALL | Motion | Motion | no |
| b4 FAULT_ENCODER | Motion | Motion | no |
| b5 FAULT_IMU (I2C error / watchdog) | IMU (I2C) or Safety (watchdog): two causes need two bits if "one setter" is strict (b5a/b5b) | its setter | no |
| b6 FAULT_NO_ECHO | Obstacle Scanning | Obstacle Scanning | **no** |
| b7 FAULT_WIFI_LOSS | Telemetry | Telemetry | **no** |

Safety call: `tk_wai_flg(EVF_SAFETY, COLLISION|IMU_ALIVE, TWF_ORW|TWF_BITCLR, &p, SAFETY_IMU_WDOG_MS)`. This is correct with this kernel: it clears only b0/b1, it never wakes on WiFi or no-echo, and `TWF_CLR` must never be used. About 9-10 of 32 bits are used. EVF_NAV (AVOID_ACTIVE, LINE_FOUND, LINE_REACQUIRED; design.md:197) is unchanged at 3 bits, with polling readers.

Issues:
- design.md 3.3 (line 196) says "no echo = nothing in range", which is normal operation, so FAULT_NO_ECHO as a *fault* conflicts with the design (F).
- The doc's "one setter per bit" leaves the **clearer** unstated. I propose setter = clearer, except FAULT_LATCHED.

### C.4 State encodings (counted from design.md statechart images)

| FSM | States counted | Doc width | Verdict |
|---|---|---|---|
| System (§5, image2) | Top: POWER_OFF, INITIALISING, READY_WAITING_FOR_START, AUTONOMOUS_NAVIGATION, OBSTACLE_AVOIDANCE/BYPASS, SAFETY_COLLISION_LATCHED_FAULT (6; 5 in software). AUTONOMOUS sub-states: LINE_TRACKING, HUMP_SENSING, TURN_LEFT, GO_STRAIGHT, U_TURN, TURN_RIGHT (6), plus a choice pseudo-state | 3 bits | **Flattened = 4 + 6 = 10 software states > 8: 3 bits do not fit.** Use hierarchical encoding (top 3 bits + nav sub-state 3 bits = 6 bits) or a 4-bit flat code |
| Line-following (6.1, image3) | ON_LINE, JUNCTION_DETECTED, LINE_LOST, REACQUIRING_TRACK, SUSPENDED_BY_AVOIDANCE (5), plus final TRACK_LOST | 3 | fits (≤ 6 with a terminal state) |
| Barcode (6.2, image4) | IDLE, EDGE_CAPTURE_READING, VALIDATING_CHECKSUM, COMMAND_READY, INVALID_READ_RESET (5) | 3 | fits |
| Obstacle (6.3, image5) | SWEEPING_COARSE, FINE_SCAN_LOCALIZATION, BYPASS_DECISION, EXECUTING_BYPASS{LEAVE_LINE, PASS_OBSTACLE, RETURN_TO_LINE}, LINE_REACQUIRED_HANDOFF, BLOCKED_STOP = 8 leaves + 1 composite = **9** | 4 (≤ 9 states) | fits (16 codes). It spans **two tasks** (Scanning P6, Avoidance P7). One shared state word with two writers breaks single-writer. Keep each task's state private and let Avoidance compose the 4-bit reported state |
| IMU/terrain (6.4, image6) | STATIONARY, ACCELERATING, CRUISING, TURNING, HUMP_DETECTED, COLLISION_FAULT (6) | **not stated** | propose 3 |
| WiFi/MQTT (6.5, image7) | DISCONNECTED, CONNECTING, CONNECTED, DEGRADED_BUFFERING, RECONNECTING (5) | **not stated** | propose 3 |
| VC authority | LINE, AVOID | not stated | 1 |
| Avoidance plan (specs.md Stage 4) | stop, continue, turn left, turn right, reverse (5) | not stated | 3 |

### C.5 Rules (from the doc) and where they apply

- Clamp before packing and set the per-message OOR bit; sign-extend on unpack (doc:185).
- Interrupts off around ISR-shared updates: `save_and_disable_interrupts()/restore_interrupts()` on the SDK side (`SDK/.../hardware_sync/include/hardware/sync.h:214,228`), `DI()/EI()` on the kernel side (`include/tk/sysdepend/cpu/core/armv6m/syslib.h:45-46`, PRIMASK).
- Mutex for task-shared data. No C11 atomics: ARMv6-M has no LDREX/STREX (architecture fact; not verifiable from the repo).

---

## D. Baseline plan

1. **"First working Week 6 design build"** cannot be any existing commit: there are no module sources, so the firmware target is skipped (`CMakeLists.txt:134-135`). **Batch B-baseline must produce it.** That means the port layer, `main.c`, `usermain`, the dbg ring and drain on **USB CDC**, all 9 design tasks as stubs at design priorities and periods, every kernel object created at its Week 6 depth, Week 6 **unpacked** types in one header, and the 64 × u64 edge ring as a static array. "Working" = boots, creates every object, prints heartbeats on USB CDC for 10 minutes (integration report:296-300), and `check_rules` passes. That last condition requires `ProjectTemplate/` to be moved or excluded first.
2. **sizeof per message, measured on the target ABI.** Add `cmake/size_report.c`, an OBJECT library like `config_check` (CMakeLists.txt:87-91), that is never linked. It defines one array per type, for example `const uint8_t g_sz_nav_input_t[sizeof(nav_input_t)];`. Then `arm-none-eabi-nm -S --size-sort size_report.c.obj` prints each sizeof as a symbol size, with no execution and no host-ABI difference. Pin the values with `_Static_assert(sizeof(T) == N)`.
3. **Map-file RAM.** `pico_add_extra_outputs` → `pico_standard_link` calls `pico_add_map_output` (`SDK/src/rp2_common/pico_standard_link/CMakeLists.txt:10`), which writes `robocar.elf.map` (`SDK/src/CMakeLists.txt:58-65`). Record `arm-none-eabi-size -A build/usb/robocar.elf` (.data, .bss, .noinit, .data_vector) and `arm-none-eabi-nm -S --size-sort` for the edge ring, `motion_state_t` and the kernel pool.
4. **Message-buffer RAM is inside the fixed pool.** `KERNEL_POOL_BYTES` = 96 KB (priorities.h:124) is one static array, so shrinking message buffers does **not** change the map. Report message-buffer bytes as `MBF_BUFSZ(N,T) = N × (4 + ROUNDUP4(sizeof T))`. Example: MBF_TELEMETRY = 32 × (4 + ROUNDUP4(sizeof tel_evt_t)) = 640 B at 16 B. Also report pool use via `tk_ref_mbf` free size and an Imalloc high-water. Note Imalloc rounds blocks to 8 B (`kernel/tkernel/memory.h:53-54`). Expected saving: MOTION −32, OBSTACLE −16, TELEMETRY −256, NAV 0, edge ring −352 (map-visible) ≈ **−0.65 KB** of 264 KB.
5. Commit the measurements (`docs/DATA_REPRESENTATION.md` baseline columns plus `docs/measurements/week6-baseline.txt`). Create an annotated tag **`baseline/week6-unpacked`** locally; the owner pushes it.
6. **B-packed:** add the shared header and its pack/unpack functions, switch every type, add host tests (pull forward the Batch D `tests/CMakeLists.txt` host project, CMakeLists.txt:178-185), re-measure with the same commands, and fill the "packed" columns and deltas. Timing evidence (doc:155, 193): ISR duration for barcode and echo before/after (GPIO toggle + logic analyser), the 20/20 barcode run, and the staged-bump stop ≤ 200 ms (NFR10).

---

## E. Deliverables

### E.1 Shared packed header

The header must be **pure** (`<stdint.h>` only, like `config/*.h`), because it is included from four places:
- SDK-side `drv_*.c` (the barcode ISR packs gaps)
- kernel-side tasks
- host tests
- the dashboard mirror

`app_program/app.h` is kernel-side only (includes `<tk/tkernel.h>`; template-merge-report:122, 242), so it cannot be the home. The merge maps `src/common/` to "shared library code" (template-merge-report:286). Proposal: **`src/common/data_repr.h`** (every `*_BITS/_SHIFT/_MASK`, `TEL_WIRE_VER`, `static inline` pack/unpack with prefix `drep_`, `_Static_assert`s on ranges and totals), plus `src/common/data_repr.c` only if out-of-line functions are wanted. `app.h` includes it. Risk R3 (doc:153) also needs the dashboard to consume the **same** definitions. The dashboard language is unknown (F).

### E.2 docs/DATA_REPRESENTATION.md columns

`Item | Field | Range | Resolution | Signed | Bits | Baseline bits/bytes | Packed bytes (wire / MBF slot) | Justification (sensor noise or bound) | Clamp/OOR | Writer (single) | Reader(s) | Owner buddy`. Plus rows for the encoder-word exception (justified by "longest run", doc:115, which needs CPR and the maximum run length), each FSM (state count vs bits), each eventflag bit (setter, clearer, in Safety wait), and a RAM summary (map vs pool).

### E.3 Host tests

- For every field: round-trip at min, max, −limit, +limit, min−1, max+1 (clamped, OOR set), and zero.
- Sign extension: steer, pitch, motion speed.
- `motion_cmd_t` with each `type` and its `arg` interpretation.
- `nav_input_t` for each source/type.
- Obstacle: 0 = no-echo sentinel, saturation of width and clearances.
- Edge ring: 64 entries, wrap, nibble-neighbour preservation, 0xFFFFF saturation, full and overflow detection.
- Telemetry frame: little-endian byte order, version mismatch rejected, length short/long rejected, unknown type rejected, rejected command never produces a `nav_input_t`.
- Compile-time checks: fault masks distinct, Safety wait mask == COLLISION|IMU_ALIVE, each FSM's state count ≤ 2^bits, packed totals ≤ container width.

### E.4 ARCHITECTURE.md deltas (current: D1, D2; ARCHITECTURE.md:48-51)

- **D3** Barcode task polls every `BC_POLL_PERIOD_MS` = 2 ms while capturing (timing.h:152-153). Design says it is woken by the GP28 ISR (design.md:157 task table "Event-driven (woken by GP28 ISR)", 3.3 line 195 `tk_wup_tsk`). The document raises this as undocumented (doc:41, 54).
- **D4** `IMU_HAS_GYRO = 0` (thresholds.h:30-33). Pitch from filtered accelerometer data, TURNING from the encoder wheel-speed difference (doc:42, 55, 99). Design 6.4 assumes a complementary filter and a gyro.
- **D5** Pin map v2: encoders quadrature on GP0/1 and GP16/17; IR_R GP26/ADC0, IR_C GP28/ADC2; IR_BC GP27; servo GP15 [FILL]. Supersedes design §2.
- **D6** Console is USB CDC only; UART0 removed; `tm_snd_dat` goes to a RAM fault record. Supersedes the Phase 1 probe-UART decision and DEBUG.md.
- **D7** Encoders counted by PIO0 (if approved). NFR2 says GPIO interrupts.
- **D8** Minimum-bit representation: packed messages, 20-bit edge gaps, FSM widths, encoder-word exception.
- **D9** Fault bits in EVF_SAFETY: single setter, clearer table, Safety `TWF_ORW|TWF_BITCLR`, other readers `tk_ref_flg`.
- **D10** MQTT binary frames replace JSON (design.md:14).

**CMake guard (DBG_BACKEND=UART0 while GP0/GP1 are encoder pins).** Two candidates:
- A CMake regex parse of `config/pins.h` is fragile: macro formatting, derived macros, and comments would all break it.
- **A C check keyed on `DBG_BACKEND_UART0`** uses the compiler's own macro evaluation over the same masks that already exist. It is the preferred form. In `pins.h`: `#if defined(DBG_BACKEND_UART0) && ((PINS_ALLOCATED_MASK & (PIN_BIT(0U)|PIN_BIT(1U))) != 0UL)` → `#error "UART0 console needs GP0/GP1, which are encoder pins"`. It runs in the `config_check` object (built in ALL with the project defines, CMakeLists.txt:87-91), so it fails at **build** time.

Configure time cannot evaluate C macros without parsing. So also make configure fail directly: drop `UART0` from the allowed `DBG_BACKEND` values and default `DBG_DEFAULT` to `USB` (CMakeLists.txt:42-47), with `FATAL_ERROR "UART0 backend removed: GP0/GP1 are ENC_R (config/pins.h)"`. The C `#error` stays as the backstop in case someone re-adds UART0.

---

## F. Differences, stale rows, questions

### F.1 Document vs owner's summary (the document wins)

1. Pitch: the doc gives a range of "45" (doc:174), which implies signed ±45° for 8 bits. The summary gives no range.
2. Wiring: the doc keeps the "pin layout and hardware ... no change" (doc:126) and rates wiring Green with "requirements unchanged" (doc:178). The summary's new final wiring contradicts this. The doc also lists GP6/GP7 encoders, GP28 barcode, GP12 servo and IR_L/IR_C ADC0/ADC1 (doc:40-43, 53, 68, 72, 83-86).
3. Debug logs: the doc says "over the UART debug drain" (doc:111). The summary says USB CDC only.
4. Bit-fields: the doc *uses* bit-fields for private state (doc:139, 199: "The design uses bit-fields for private state") pending Buddy 1's Barr C check. The summary says no bit-fields at all until confirmed. They are consistent in effect; the summary is stricter.
5. Fault list: the doc names stall, encoder, IMU, no echo, WiFi loss, collision (doc:114). The summary gives no list but adds "WiFi-loss and no-echo not in Safety's wait pattern", which the doc does not say.
6. Encoder word: the doc says "the ISR (the only writer)" and justifies the width by the longest run (doc:115). The summary omits the justification. The PIO option changes the writer.
7. `motion_state_t`: the doc says it is packed (doc:115). The summary lists it only in the baseline.
8. Baseline owner: the doc says Buddy 2 (doc:197). Timing evidence the doc requires (barcode 20/20, ISR not worse, staged bump ≤ 200 ms; doc:155, 193) is not in the summary.
9. The doc's tk_set_flg/tk_clr_flg "usable from interrupt handlers" (doc:119) holds here only through the taskindp wrapper.

### F.2 Document vs design.md

- JSON/struct payloads (design.md:14) vs binary.
- Raw timestamps in the ring (design.md:195) vs 20-bit gaps.
- EVF_SAFETY 3 bits (design.md:196) vs a fault bitmask.
- Complementary filter and gyro (design.md 6.4) vs IMU_HAS_GYRO=0.
- ISR-woken barcode vs the 2 ms poll.
- "Obstacle profile (bearing, width, clearance)" (design.md:157) vs no clearance in the doc.
- `enc_count` "Motion Control reads and clears" (design.md:199) vs the doc's "ISR only writer".

### F.3 Document vs repo

- `pins.h`, `CMakeLists.txt` and `DEBUG.md` still hold the old pins and UART0.
- There is no packed header.
- ARCHITECTURE.md lacks the BC_POLL and IMU-gyro deltas.
- D2 is still "PROPOSED" (ARCHITECTURE.md:51), consistent with doc:39.

### F.4 Stale rows noticed (report only)

- design.md:141 lists GP0/GP1 as spare.
- design.md:396 rationale "keeps all 3 ADC channels free for IR" (ADC1 is now unused).
- design.md:183 cites NFR16 for the IMU watchdog; it should be NFR13 (batchA open item 9).
- design.md 6.3 text: "median of three pings (NFR13)"; it should be NFR15.
- doc:192 says risk R4 was added, but the register (doc:151-153) stops at R3. doc:193 adds a staged-bump test that is not in the test plan (doc:154-157).
- doc readiness "Hardware wiring known: Green" (doc:178).
- batchA-summary item 13 and integration report:28 ("fallback is UART0").
- `pins.h:66` comment is still right.
- The template-merge report's "never commit ProjectTemplate" has been overtaken by 79bc417.

### F.5 Questions for the owner

1. Servo: GP15 (slice 7 B) or revert to GP12 (slice 6 A)? Neither clashes with the motors.
2. Barr C: are bit-fields allowed for private state? The standard is not on disk, so this is UNVERIFIED. Until answered: masks and shifts everywhere.
3. Pitch: confirm signed ±45° at 0.5° (8 bits).
4. `motion_cmd_t`: accept the 9-bit `arg` shared by angle, distance and steer (selected by type) to cover the missing distance and steering fields?
5. `obstacle_profile_t`: add clear_L/clear_R (7+7 bits)? Drop the timestamp? What exactly is the "bypass state 4 bits" field? It is Avoidance's state, but the message comes from Scanning.
6. Grove pin pairs (Grove 5 = GP6+GP26, Grove 7 = GP7+GP28, Grove 6 = GP26+GP27): do the line sensors' DO leads land on GP6/GP7? Is Grove 6's GP26 lead physically isolated? Which Grove numbering scheme is authoritative?
7. Execute the template merge first (required by your order)? Note that HEAD 79bc417 committed `ProjectTemplate/` and `check_rules` now fails with 94 violations. Move it out, or exclude it?
8. After the merge, should `TEAM.md` section 2 be the pin ledger? Recommended: yes, filled from `pins.h`, which stays the compile-time source of truth. The template's TEAM.md marks GP0/GP1 for its `CRASH_REPORT`, which does not apply to us.
9. Approve PIO0 encoders (a deviation from NFR2's "GPIO interrupts"), including the BSD-3 notice in NOTICE.md (your file)?
10. Faults: extend EVF_SAFETY or create a new eventflag? Does "collision latch" mean FAULT_LATCHED? Is "no echo" really a fault, given design.md:196 calls it normal?
11. System FSM: accept the hierarchical 3+3-bit encoding, since flat 10 states exceed 3 bits?
12. Dashboard: what language, and how does it consume the shared C header (risk R3)?
13. Message-size "before" for MQTT: no JSON format exists in the repo. Define a reference JSON, or compare binary vs struct only?
14. Lock `BC_CAPTURE_TIMEOUT_MS = 1000`? The 20-bit gap depends on it (4.6 % margin).
15. Encoder CPR and wheel circumference, needed for every rate in B and for the "longest run" justification.
16. Where should `tm_snd_dat` (boot/fault output) go now that UART0 is gone? Proposed: a RAM fault record read over SWD and printed on the next boot.
17. D2 (telemetry 20 ms wake) is still unapproved.
