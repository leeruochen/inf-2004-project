# Debug console and Debug Probe

USB CDC on the Pico W's own USB port is the **only** console. The Raspberry
Pi Debug Probe is used for **SWD only** (flashing and GDB); its "U" (UART)
port is not connected. UART0 was removed in the Week 8 pin map v2: GP0/GP1
are now the right wheel encoder (`ENC_R_A`/`ENC_R_B`, `config/pins.h`;
ARCHITECTURE.md D5, D6).

## Wiring

SWD (probe "D" port to the Pico W SWD header): SWCLK, SWDIO, GND.

The console needs no wiring: plug the Pico W into the PC with a USB cable.

## Opening the USB CDC console

The firmware enumerates as a USB serial device (a COM port on Windows).

- Find the port: Device Manager > Ports (COM & LPT) > "USB Serial Device (COMn)".
  The port appears only after the firmware has booted, and it disappears and
  re-enumerates after every reset or reflash.
- Open it with any terminal, for example PuTTY (Connection type Serial, COMn,
  speed 115200) or `python -m serial.tools.miniterm COMn 115200`. USB CDC
  ignores the baud rate; 115200 is only what the terminal shows.
- Before the kernel starts, the firmware waits up to `DBG_USB_CONNECT_WAIT_MS`
  (3 s, `config/timing.h`) for `stdio_usb_connected()`, then continues either
  way. This is the only allowed SDK wait, and only before `hal_kernel_start()`
  (`src/app/main.c`, line marked `RULE-ALLOW: pre-kernel`). Open the terminal
  within 3 s of a reset to see the start of the log.

Tasks never print. They write to the `dbg_` ring, and the debug-only drain task
(priority 10, `DBG_ENABLE` builds only) is the only caller of stdio
(ARCHITECTURE.md D1).

## Flashing over SWD

With the OpenOCD bundled by the Pico SDK 1.5.1 installer:

```
& "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\openocd\openocd.exe" `
  -s "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\openocd\scripts" `
  -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" `
  -c "program build/usb/robocar.elf verify reset exit"
```

For debugging, start OpenOCD without the `program` command and attach
`arm-none-eabi-gdb build/usb/robocar.elf` with `target extended-remote :3333`.

Alternative without the probe: hold BOOTSEL, plug in USB, copy `robocar.uf2`
to the RPI-RP2 drive.

## Kernel text and faults: system-message ring and RAM fault record

USB CDC cannot be used from a HardFault or with interrupts masked (TinyUSB
needs its IRQ and a 1 ms alarm), and there is no UART any more. So the kernel
side never writes to stdio. Its output goes to two separate places in RAM
(ARCHITECTURE.md D6, owner decision C3 in `.agents/tasks/batchB-plan.md`).

### System-message ring (kernel text)

- The kernel's own text (the `tm_printf` boot banner, printed on every boot,
  and any later system message) reaches the port's `tm_snd_dat`, which writes
  it into a system-message ring in RAM. `tm_snd_dat` never blocks and never
  writes to stdio.
- The debug drain empties this ring every cycle and prints it over USB CDC,
  so kernel text is not lost after boot either. If the ring is full, new
  bytes are dropped and counted; the drain prints the counter when it changes.
- Text written during a HardFault is lost with the reset. The fault record
  below is the authority for crashes.
- Planned defaults (512 B, single writer and single reader, no lock):
  `.agents/tasks/batchB-plan.md` O5.

### RAM fault record (faults only)

- The record holds faults only, never boot text.
- The record lives in the SDK's `.uninitialized_data` section, declared with
  the SDK macro `__uninitialized_ram(...)` (`pico/platform.h`). That section is
  NOLOAD and `crt0` never zeroes it, so its contents are kept across a warm
  reset (not across power-off). It is **not**
  `.noinit`, which our linker script zero-fills (`cmake/memmap_mtk3.ld`). No
  linker-script change is needed.
- Layout: a `uint32_t` magic number first, then the payload, then a CRC-32 over
  the payload (docs/DATA_REPRESENTATION.md). The magic is `0x464C5452`; in
  memory its bytes read "RTLF" (little-endian), so `x/wx` shows `0x464c5452`.
- The port's HardFault handler writes the record (`src/rtos/port/port_fault.c`).
  It never writes to stdio. It then resets the whole chip with the watchdog,
  unless the RAM flag `port_fault_halt` (set from gdb) or the build option
  `PORT_FAULT_HALT` asks it to halt for the debugger.
- Read it over SWD at any time: in `arm-none-eabi-gdb`, `print <record symbol>`
  or `x/16wx &<record symbol>`; or OpenOCD `mdw <address> 16`.
- On the next boot the debug drain prints it directly over USB CDC (not through
  the `dbg_` ring), **only if both the magic number and the CRC match**, and
  clears it only after printing while a terminal is connected. USB CDC drops
  output when no host is connected, so the record waits until you open the
  terminal. If the magic or CRC does not match, RAM holds random data
  (power-on, or after flashing), and the record is silently re-initialised.
- TODO(phase 1 batch B, buddy 2): implementation of the ring and the record
  (port layer and drain).

## Build options

The backend is a configure-time option. Only `USB` is accepted.

| Option | Values | Default |
|---|---|---|
| `DBG_BACKEND` | `USB` (`UART0` fails at configure) | `${DBG_DEFAULT}` |
| `DBG_DEFAULT` | `USB` | `USB` |
| `DBG_ENABLE` | `ON` (debug), `OFF` (release) | `OFF` for `Release`/`MinSizeRel`, `ON` otherwise |
| `APP_TEST` | `ON`, `OFF` | `OFF` |

`-DDBG_BACKEND=UART0` stops configure with "DBG_BACKEND=UART0 was removed
(Week 8 pin map v2): GP0/GP1 are ENC_R_A/ENC_R_B in config/pins.h". As a
backstop, `config/pins.h` `#error`s if code is compiled with
`DBG_BACKEND_UART0` defined while GP0/GP1 are allocated. The build defines
`DBG_BACKEND_USB=1`, `pico_enable_stdio_usb(robocar 1)` and
`pico_enable_stdio_uart(robocar 0)`. stdio UART must never be enabled: the
`pico_w` board puts it on GP0/GP1.

These defaults are CMake cache values: they are seeded on the first configure
of a build directory and kept after that. Changing `CMAKE_BUILD_TYPE` or
`DBG_DEFAULT` on an existing build directory does NOT change `DBG_ENABLE` or
`DBG_BACKEND`. Use a fresh build directory, or set the option itself
(`-DDBG_ENABLE=OFF`).

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

& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb
```

A release build (debug ring and drain task compiled out):

```
& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release   # DBG_ENABLE=OFF
```

(`$common` sets `-DCMAKE_BUILD_TYPE=Debug`; the later `-D` wins.)

## USB CDC bring-up test (mandatory)

USB CDC is the only console, so this test is required for Phase 1 acceptance
(integration report section 14). Build with `DBG_BACKEND=USB` (the only
backend). Record it in `testing/system/TEST_LOG.md`.

1. Boot, then open the COM port. Expect the kernel boot banner (through the
   system-message ring, not the `dbg_` ring) and nine heartbeat lines per
   period, each with a task ID and a sequence number.
2. Run for 10 minutes. Pass if there are no sequence gaps, no HardFault, and
   the cyclic period jitter (each task logs `time_us_64()` deltas, max and min)
   is within 1 ms of nominal.
3. Stress: every task logs at 10x its normal rate through the `dbg_` ring.
   Check that the overflow counters increment, nothing hangs, and the
   control-task periods are unaffected.
4. Unplug and replug USB, and open and close the terminal. The system must
   keep running.

The integration report's step 5 (repeat on UART0 as a baseline) no longer
applies: there is no UART0 console.

## On-target tests (APP_TEST)

`APP_TEST=ON` builds every `app_program/subsystemN/subsystemN_test.c` and
runs the tests in a dedicated test task at priority 11, below the debug drain
(10). Use a fresh build directory:

```
& $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
& $CM --build build/test
```

- Each check prints one line through the `dbg_` ring:
  `[TEST] <ID> PASS  <desc>`, `[TEST] <ID> FAIL  <desc>   (<file>:<line>)`,
  then `[TEST] <name> SUMMARY <p> passed, <f> failed`
  (`app_program/test_report.h`).
- Save the console output to `testing/subsystemN/logs/` and record the result
  in that folder's `TEST_LOG.md` (`testing/README.md`).
- `APP_TEST=ON` needs `DBG_ENABLE=ON`. A Release build (`DBG_ENABLE=OFF`) with
  `APP_TEST=ON` fails at configure.
- Deferred to Batch B: the `dbg_` ring does not exist yet, so an `APP_TEST=ON`
  build configures but stops at the deliberate `#error` in
  `app_program/test_report.h` (TODO(phase 1 batch B, buddy 2)).
