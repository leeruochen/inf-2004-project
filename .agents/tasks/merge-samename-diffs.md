# Same-name file gate: proposed merges (awaiting owner approval)

Owner requirement: "Show every same-name file diff before applying it".
Status: **APPROVED AND APPLIED** (see section 6). Nothing is committed.

The proposed full files are in `.agents/tasks/proposed/README.proposed.md` and `.agents/tasks/proposed/gitignore.proposed`. The diffs below were generated from them with `git diff --no-index` against the current root files.

## 1. Same-name inventory

Method: every file under `ProjectTemplate/` (recursive, including dot files), checked for a file at the same relative path under the repo root. `build/`, `external/` and `.git/` are excluded. Run on HEAD `df67420`.

| Relative path | In template | At root | Treatment |
|---|---|---|---|
| `README.md` | 220-line make/WSL setup guide | 5 lines (attribution with TODO, links) | **Template content merged** into ours (section 3 below) |
| `.gitignore` | Eclipse dirs, objects, elf2uf2, wifi_secrets, videos, `.DS_Store` | build trees, `config/secrets.h`, firmware outputs, `.agents/` | **Template content merged** into ours (section 4 below) |

No other file clashes. Notes:
- The only shared directory is `config/`, with no shared files: the template has `config/config*.h` and `config/wifi_secrets.example.h`, we have `config/mtk3/config*.h`, `config/pins.h` and so on, and `config/secrets.h.example`. Nothing is imported from the template's `config/`.
- The template has no `CMakeLists.txt` or `*.cmake`, so our root `CMakeLists.txt` and `cmake/*` have no template counterpart. Our own build edits to them appear in the final patch, not here.
- `ProjectTemplate/build_make/.gitignore` has no root counterpart (we have no `build_make/`).
- `TEAM.md`, `testing/`, `videos/` and `app_program/` do not exist at the root yet, so they are new files (section 2), not clashes.

## 2. New root files ported from the template (later merge step, no diffs)

| New root file | Template source | How it will differ from the template |
|---|---|---|
| `TEAM.md` | `ProjectTemplate/TEAM.md` | Same sections. Example rows cleared. Members filled with names and IDs only from ProjectTemplate/TEAM.md or the design.md Team table (see question Q1). Our pins from `config/pins.h` (servo GP15, IR_C GP28, IR_R GP26, barcode DO GP27, GP6/7 and GP12-14 spare, GP23/24/25/29 reserved). Our peripherals. Our priorities 1 to 10 (11 with APP_TEST) instead of the 12-21 bands. Our IF rows from design.md 3.3. Object limits from `config/mtk3/config.h`. `cyw43_utk_task` rows and the "WiFi and MQTT are special" text removed, with a Phase 2 TODO. No `TP_PRC*` core column. |
| `testing/README.md` | same path | `make TEST=1` replaced by our CMake `-DAPP_TEST=ON` command and Windows USB CDC capture. Test ID scheme, logging rules, "never edit old results" kept. |
| `testing/{subsystem1..5,integration,system}/TEST_LOG.md` | same paths | Example rows cleared; table headers kept. |
| `testing/*/logs/.gitkeep` | same paths | Identical (empty). |
| `videos/README.md` | same path | Verbatim except the example row of the index table, which is removed. |
| `app_program/app.h` | same path | Barr C names and `<stdint.h>` types. Kernel headers only, no `<bsp/libbsp.h>`. `CHECK()` through `dbg_`. Per-task stacks from `config/priorities.h` instead of `TASK_STACK_SIZE`. `app_start_task()` without the core argument. Template file header block. |
| `app_program/app_main.c` | same path | Our kernel-side `usermain()`: create all IPC objects, every `subsystemN_init()`, every `subsystemN_start()`, then sleep forever. Banner through `dbg_`. Under APP_TEST it starts the lowest-priority test task. Header explains its role next to `src/app/main.c`. |
| `app_program/test_report.h` | same path | Same macros and `[TEST] ID PASS\|FAIL` format, output through the `dbg_` ring instead of `tm_printf`. |
| `app_program/subsystemN/{subsystemN.c,subsystemN.h,subsystemN_test.c}` (N = 1..5) | same paths | Barr C stubs with `TODO(phase N, buddy X)` markers. Our tasks, pins, priorities and IF objects (1 telemetry, 2 motion + Vehicle Controller, 3 line + barcode, 4 IMU + safety including `task_safety.c`, 5 obstacle scan + avoid). Template demo tasks (`task_led.c`, `task_producer.c`, `task_consumer.c`, `task_monitor.c`) not ported. Student IDs only from the sources in Q1. |

## 3. README.md

### 3.1 Proposed diff (current root file to proposed)

```diff
diff --git a/README.md b/.agents/tasks/proposed/README.proposed.md
index b52c100..a48778a 100644
--- a/README.md
+++ b/.agents/tasks/proposed/README.proposed.md
@@ -3,3 +3,170 @@
 Uses the TRON Forum uT-Kernel 3.0 BSP (micro T-Kernel 3.0) under the T-License 2.2; see [NOTICE.md](NOTICE.md) and the [BSP docs folder PDF](external/mtk3_bsp/docs/TEF000-219-200401.pdf). <!-- TODO(owner-to-confirm): attribution wording -->
 
 Build and debug console: [docs/DEBUG.md](docs/DEBUG.md). Kernel submodule: [docs/SUBMODULE.md](docs/SUBMODULE.md).
+
+Robot car firmware for the Raspberry Pi Pico W. micro T-Kernel 3.0 runs on
+core 0 only, from the official BSP pinned as a git submodule. The code is
+built with the Pico C SDK 1.5.1, CMake and Ninja on Windows. The design is in
+[design.md](design.md); the source layout, kernel integration and coding rules
+are in [ARCHITECTURE.md](ARCHITECTURE.md).
+
+`ProjectTemplate/` is the INF2004 course project template provided by the
+module instructor. It stays in the repo unchanged, as reference only. Nothing
+in it is built or rule-checked. The patterns we kept from it are listed in
+[docs/TEMPLATE_NOTES.md](docs/TEMPLATE_NOTES.md).
+
+---
+
+## 1. Set up
+
+1. Install the **Raspberry Pi Pico SDK 1.5.1 Windows installer** at its
+   default path, `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1`. It installs
+   the SDK, CMake, Ninja, the Arm GCC 10.3.1 compiler, Python, git and
+   OpenOCD. Do not use another SDK or compiler version.
+2. Clone the repo together with the kernel submodule:
+   ```powershell
+   git clone --recurse-submodules https://github.com/leeruochen/inf-2004-project.git
+   ```
+   In an existing clone, run `git submodule update --init`. The submodule
+   `external/mtk3_bsp` must stay at commit `15ed232` and must never be edited
+   ([docs/SUBMODULE.md](docs/SUBMODULE.md)).
+3. In every new PowerShell window, from the repo root, run this preamble:
+   ```powershell
+   $R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"
+   $env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"
+   $env:PICO_SDK_PATH = "$R\pico-sdk"
+   $CM = "$R\cmake\bin\cmake.exe"
+   $common = @("-G","Ninja","-DCMAKE_BUILD_TYPE=Debug",
+     "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk",
+     "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools",
+     "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe",
+     "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")
+   ```
+
+## 2. Build
+
+This project builds with **CMake and Ninja on Windows**. The template's
+`build_make/` flow (GNU make under WSL with Pico SDK 2.2.0) is not used and
+does not build this code.
+
+TODO(owner-to-confirm): instructor's build preference (our CMake and Ninja flow on Windows, or the template's make/WSL flow).
+
+```powershell
+& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
+& $CM --build build/usb
+```
+
+The default (ALL) build also runs `check_rules`, which enforces the coding
+rules in [ARCHITECTURE.md](ARCHITECTURE.md). A rule violation fails the
+build. The firmware is `build/usb/robocar.elf`, with `robocar.uf2` next to
+it. Until the first firmware sources exist, configure prints
+`firmware target skipped` and only the kernel library and the checks build.
+
+For a release build (debug ring and drain task compiled out), use a fresh
+build directory:
+
+```powershell
+& $CM -S . -B build/release @common -DCMAKE_BUILD_TYPE=Release -DDBG_BACKEND=USB
+& $CM --build build/release
+```
+
+## 3. Flash
+
+**SWD with the Raspberry Pi Debug Probe** (probe "D" port to the Pico W SWD
+header: SWCLK, SWDIO, GND):
+
+```powershell
+& "$R\openocd\openocd.exe" -s "$R\openocd\scripts" `
+  -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" `
+  -c "program build/usb/robocar.elf verify reset exit"
+```
+
+**UF2 without the probe:** hold BOOTSEL, plug the Pico W into USB, release,
+then copy `build/usb/robocar.uf2` to the `RPI-RP2` drive. The Pico reboots by
+itself.
+
+Debugging over SWD is described in [docs/DEBUG.md](docs/DEBUG.md).
+
+## 4. Console
+
+The only console is **USB CDC** on the Pico W's own USB port. The Debug Probe
+is used for SWD only; its UART port is not connected. Open the Pico W's COM
+port (Device Manager > Ports (COM & LPT)) with PuTTY or
+`python -m serial.tools.miniterm COMn 115200`.
+
+Tasks never print directly. They write to the `dbg_` ring, and a debug-only
+drain task sends it to USB CDC. After a fault, the fault record is kept in
+RAM, can be read over SWD, and is printed and cleared on the next boot.
+Details: [docs/DEBUG.md](docs/DEBUG.md).
+
+## 5. Write your own code
+
+Team code lives in **`app_program/`**, in the template's layout:
+
+| Path | What it is |
+|---|---|
+| `app_program/app.h` | Definitions shared by the whole team, and the `subsystemN_init/start/test` prototypes |
+| `app_program/app_main.c` | `usermain()`, the kernel-side start. It creates all IPC objects, then calls every `subsystemN_init()`, then every `subsystemN_start()`, then the init task sleeps forever |
+| `app_program/test_report.h` | `TEST_INIT`, `TEST_CHECK`, `TEST_SUMMARY` for on-target tests |
+| `app_program/subsystem1/` | Buddy 1: telemetry |
+| `app_program/subsystem2/` | Buddy 2: motion and Vehicle Controller |
+| `app_program/subsystem3/` | Buddy 3: line following and barcode |
+| `app_program/subsystem4/` | Buddy 4: IMU and safety |
+| `app_program/subsystem5/` | Buddy 5: obstacle scan and avoid |
+
+Each subsystem folder holds `subsystemN.c`, `subsystemN.h` (its public
+interface), `subsystemN_test.c`, its tasks as `task_<name>.c` and its drivers
+as `drv_<name>.c` / `drv_<name>.h`. Only the owner edits a subsystem folder.
+Shared files change only by team agreement.
+
+- `drv_*.c` files include Pico SDK headers only. `task_*.c`, `subsystemN.c`
+  and `subsystemN_test.c` include kernel headers only. A `drv_*.c` never
+  includes `subsystemN.h`.
+- Source files are not discovered automatically. List every new `.c` file in
+  that folder's `sources.cmake`. Configure fails on a `sources.cmake` that is
+  not registered.
+- Pins come from `config/pins.h` and are claimed in `TEAM.md` before wiring.
+  Never use GP23, GP24, GP25 or GP29 (Pico W radio).
+
+The platform code lives in **`src/`** and is not team code: `src/app/main.c`
+(the SDK `main()` that starts the kernel), `src/rtos/port/` (kernel port),
+`src/debug/` (debug ring and drain task) and `src/common/`. Configuration
+headers are in `config/`.
+
+## 6. Build options
+
+Options are CMake cache values. They are seeded on the first configure of a
+build directory, so use a fresh build directory or pass the option itself.
+
+| Option | Values (default) | What it does |
+|---|---|---|
+| `DBG_BACKEND` | `USB` | Debug console backend. USB CDC is the only supported backend |
+| `DBG_ENABLE` | `ON`, `OFF` (`OFF` for Release/MinSizeRel, `ON` otherwise) | Debug ring and debug-only drain task |
+| `APP_TEST` | `ON`, (`OFF`) | Builds every `subsystemN_test.c` and runs them in a dedicated lowest-priority test task, with `[TEST] <ID> PASS\|FAIL` lines through the `dbg_` ring. Needs `DBG_ENABLE=ON`; configure fails otherwise |
+| `TEL_BACKEND_WIFI` | `ON`, (`OFF`) | WiFi/MQTT telemetry. Not implemented in Phase 1; `ON` fails at configure |
+
+Test build example:
+
+```powershell
+& $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
+& $CM --build build/test
+```
+
+The template's make options `SMP`, `CONSOLE`, `WIFI`, `TEST` and
+`CRASH_REPORT` are not supported. The kernel runs on one core, USB CDC is the
+only console, Phase 1 has no WiFi, and `APP_TEST` replaces `TEST`.
+
+## 7. Working as a team
+
+- **`TEAM.md`**: owners, pins, peripherals, task priorities (ours, 1 to 10;
+  11 with `APP_TEST`), interfaces between subsystems and the kernel object
+  budget.
+- **`testing/`**: one `TEST_LOG.md` per subsystem, plus integration and
+  system logs. Rules are in `testing/README.md`.
+- **`videos/`**: test and demo videos, shared as unlisted YouTube links only.
+  Never commit video files.
+
+## 8. Licence
+
+Licences and credits are in [NOTICE.md](NOTICE.md). INF2004 course project
+template provided by the module instructor.
```

### 3.2 Notes on the README content

- The attribution line and its `TODO(owner-to-confirm)` stay at the top, unchanged. The existing links line stays as line 5.
- Section 1 states the build preamble verbatim from `.agents/tasks/batchA-summary.md`.
- Section 2 states that the template's make/WSL flow is unused and carries a visible `TODO(owner-to-confirm): instructor's build preference` line.
- Section 4 states that USB CDC is the only console and the Debug Probe is SWD only.
- Sections 2, 3 and 6 describe the state after this change request (USB-only console, `APP_TEST`, `app_program/` registration). The CMake side (UART0 guard, `APP_TEST`, `app_program` registration) lands in the same patch. `docs/DEBUG.md` still describes UART0 today and is updated in that patch too.
- The template's licence footer is not copied; section 8 points to NOTICE.md and carries the approved credit line.
- It links `docs/TEMPLATE_NOTES.md`, which is written in a later step of this request (merge answer 6).

### 3.3 Template version, for reference (`ProjectTemplate/README.md`, unchanged)

````markdown
# micro T-Kernel 3.0 project for Raspberry Pi Pico W

A starting point for your project: micro T-Kernel 3.0 running on **both cores**
of the Pico W's RP2040, with a USB serial console and the WiFi driver built in.

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the task reports. Connect an LED with a 330 Ω resistor from
**GP19** to GND to see subsystem 1's LED blink.

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks). Read its comments first.
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`.
  Subsystems 1–4 hold a working example, one task each (LED, producer,
  consumer, monitor). Together they use the main micro T-Kernel APIs and
  pass data between subsystems. Subsystem 5 is an empty template.
  Replace the examples with your own subsystems.
- `subsystemN.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` builds a test image that runs all
the subsystem tests at power-up. Short videos of tests and demos are shared
as unlisted YouTube links, listed in **`videos/`**.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`). This guide does not cover them.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
````

## 4. .gitignore

### 4.1 Proposed diff (current root file to proposed)

```diff
diff --git a/.gitignore b/.agents/tasks/proposed/gitignore.proposed
index a304e14..383a4ac 100644
--- a/.gitignore
+++ b/.agents/tasks/proposed/gitignore.proposed
@@ -13,5 +13,19 @@ config/secrets.h
 *.map
 *.dis
 
+# Object and dependency files (only if someone builds in-tree)
+*.o
+*.d
+
+# Videos are shared as unlisted YouTube links (videos/README.md), never committed
+*.mp4
+*.mov
+*.webm
+*.avi
+*.mkv
+
+# macOS
+.DS_Store
+
 # Agent working files (plans, reviews, reports)
 .agents/
```

Added: `*.o`, `*.d`, `*.mp4`, `*.mov`, `*.webm`, `*.avi`, `*.mkv`, `.DS_Store`. Kept: `config/secrets.h`, `.agents/`. Skipped from the template: `Debug`, `Release` (Eclipse), `build_make/tools/elf2uf2`, `tools/elf2uf2/elf2uf2`, `build_make/.build_profile`, `config/wifi_secrets.h` (ours is `config/secrets.h`).

Pre-check (before approval, nothing changed): `git ls-files -- ProjectTemplate config | git -c core.excludesFile=.agents/tasks/proposed/gitignore.proposed check-ignore -v --no-index --stdin` printed nothing, exit 1. The proposed rules match no tracked ProjectTemplate file and no tracked config/ file.

### 4.2 Template version, for reference (`ProjectTemplate/.gitignore`, unchanged)

```gitignore
Debug
Release

# Build artifacts
*.elf
*.uf2
*.map
*.o
*.d
build_make/tools/elf2uf2
tools/elf2uf2/elf2uf2
build_make/.build_profile

# Real WiFi credentials -- never commit these. Copy config/wifi_secrets.example.h
# to config/wifi_secrets.h and fill in your own network's SSID/password.
config/wifi_secrets.h

# Videos are shared as unlisted YouTube links (videos/README.md), never committed
*.mp4
*.mov
*.webm
*.avi
*.mkv

# macOS
.DS_Store
```

## 5. Questions for the owner

- **Q1 (TEAM.md names and IDs).** You said you filled in `ProjectTemplate/TEAM.md`, but the file on disk is identical to commit 79bc417 (`git diff HEAD -- ProjectTemplate` is empty) and every Student ID and GitHub username cell is blank. The design.md Team table (lines 11-18) has the five buddies with IDs: Buddy 1 2501299 (Damian), Buddy 2 2500530 (Ruo Chen), Buddy 3 2503319 (Jin Yong), Buddy 4 2500709 (Vincent), Buddy 5 2500596 (Chay Han). **Suggestion:** use the design.md table for names and IDs (your rule allows "TEAM.md or the Team Information table") and leave GitHub usernames as `TODO(owner-to-confirm)`. Alternatively, save your filled-in TEAM.md and tell me.
- **Q2 (README build TODO).** The build-preference TODO is a visible line, not an HTML comment like the attribution TODO, so a reader sees it. **Suggestion:** keep it visible.

## 6. Owner decision and application

Owner answer: **APPROVE both diffs (README.md and .gitignore) as written; keep the build TODO visible.**

Q1 ruling (supersedes the Q1 suggestion above and the "design.md Team table" wording in section 2): names and Student IDs come **only** from the Team Information table in `INF 2004 Embedded Systems.md` (repo root, read only, never edited), copied exactly as written there. design.md's team table is **not** used. GitHub usernames stay `TODO(owner-to-confirm)`. The same rule applies to every template file-header block (Student ID / author). **This rule must be carried into the later merge steps** (TEAM.md, app_program file headers).

Applied: `README.md` and `.gitignore` were replaced by `.agents/tasks/proposed/README.proposed.md` and `.agents/tasks/proposed/gitignore.proposed` exactly (`git diff --no-index --quiet` exit 0 for both). Both are LF, matching `.gitattributes`.

Checks after applying:

| Command | Result |
|---|---|
| `git -C C:/Users/Admin/Downloads/Projects/inf-2004-project status --porcelain -- ProjectTemplate` | empty, exit 0 |
| `git ls-files -- ProjectTemplate config \| git check-ignore -v --no-index --stdin` (643 ProjectTemplate + 10 config tracked files) | no output, exit 1: nothing ignored |
| `git check-ignore -v config/secrets.h` | `.gitignore:6:config/secrets.hconfig/secrets.h`, exit 0: only secrets.h is ignored under config/ |