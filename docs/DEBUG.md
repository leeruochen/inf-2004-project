# Debug console and Debug Probe

Phase 1 uses a Raspberry Pi Debug Probe for both the serial console (UART0)
and flashing/debugging (SWD).

## Wiring

UART (probe "U" port to Grove 1 on the ROBO PICO, GP0/GP1):

| Debug Probe | Pico W |
|---|---|
| TX (probe output) | GP1 (UART0 RX) |
| RX (probe input) | GP0 (UART0 TX) |
| GND | GND (common ground) |

SWD (probe "D" port to the Pico W SWD header): SWCLK, SWDIO, GND.

GP0/GP1 and Grove 1 are reserved for the probe (`config/pins.h`).

## Opening the serial port

The probe enumerates as a USB serial device (a COM port on Windows). Settings:
115200 baud, 8 data bits, no parity, 1 stop bit (8N1), no flow control.

- Find the port: Device Manager > Ports (COM & LPT) > "USB Serial Device (COMn)".
- Open it with any terminal, for example PuTTY (Connection type Serial, COMn,
  speed 115200) or `python -m serial.tools.miniterm COMn 115200`.

In UART0 builds the kernel boot banner, fault messages (polled UART0) and the
runtime log (dbg_ ring drained by the debug-only task) all appear here.

## Flashing over SWD

With the OpenOCD bundled by the Pico SDK 1.5.1 installer:

```
& "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\openocd\openocd.exe" `
  -s "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\openocd\scripts" `
  -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" `
  -c "program build/uart0/robocar.elf verify reset exit"
```

For debugging, start OpenOCD without the `program` command and attach
`arm-none-eabi-gdb build/uart0/robocar.elf` with `target extended-remote :3333`.

Alternative without the probe: hold BOOTSEL, plug in USB, copy `robocar.uf2`
to the RPI-RP2 drive.

## Switching backends

The backend is a configure-time option. Both backends build.

| Option | Values | Default |
|---|---|---|
| `DBG_BACKEND` | `UART0`, `USB` | `${DBG_DEFAULT}` |
| `DBG_DEFAULT` | `UART0`, `USB` | `UART0` (Phase 1) |
| `DBG_ENABLE` | `ON` (debug), `OFF` (release) | `OFF` for `Release`/`MinSizeRel`, `ON` otherwise |

These defaults are CMake cache values: they are seeded on the first configure
of a build directory and kept after that. Changing `CMAKE_BUILD_TYPE` or
`DBG_DEFAULT` on an existing build directory does NOT change `DBG_ENABLE` or
`DBG_BACKEND`. Use a fresh build directory, or set the option itself
(`-DDBG_ENABLE=OFF`, `-DDBG_BACKEND=USB`).

Use one build directory per backend:

```
$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"
$env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"
$CM = "$R\cmake\bin\cmake.exe"
$common = @("-G","Ninja","-DCMAKE_BUILD_TYPE=Debug",
  "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk",
  "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools",
  "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe",
  "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")

& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0
& $CM --build build/uart0

& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb
```

To make USB CDC the default instead, configure a FRESH build directory with
`-DDBG_DEFAULT=USB`. On an existing build directory, set `-DDBG_BACKEND=USB`
directly.

A release build (debug ring and drain task compiled out):

```
& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release   # DBG_ENABLE=OFF
```

(`$common` sets `-DCMAKE_BUILD_TYPE=Debug`; the later `-D` wins.)

- `UART0`: `pico_enable_stdio_uart(robocar 1)`, `pico_enable_stdio_usb(robocar 0)`,
  define `DBG_BACKEND_UART0=1`.
- `USB`: the reverse, define `DBG_BACKEND_USB=1`. Before `hal_kernel_start()`
  the firmware waits up to `DBG_USB_CONNECT_WAIT_MS` (3 s, `config/timing.h`)
  for `stdio_usb_connected()`, then continues either way. This is the only
  allowed SDK wait and only before the kernel starts.

Phase 1 acceptance (nine heartbeats) is verified on UART0. USB CDC is an extra
bring-up test, reported but not required.
