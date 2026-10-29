# Notes on the course project template

`ProjectTemplate/` is the INF2004 course project template provided by the
module instructor. It stays in the repo unchanged, as reference only: it is
never built and never scanned by `check_rules`. This page lists what we took
from it, what we did not, and why. The full list of deviations is the
"Template deviations" table (T1-T7) in [ARCHITECTURE.md](../ARCHITECTURE.md).

## Patterns we kept

| Pattern | Template source | Where it lives in our tree |
|---|---|---|
| Two-phase start: every `subsystemN_init()` creates objects, then every `subsystemN_start()` starts tasks, then the init task sleeps forever with `tk_slp_tsk(TMO_FEVR)` | `app_program/app_main.c`, `app_program/app.h` | `app_program/app_main.c` (`usermain()`), `app_program/app.h`. We add a step 0: `ipc_init()` creates every IPC object centrally before any `_init()` (Batch B) |
| Cyclic-handler wake: `tk_cre_cyc` (created stopped, started in `_start()` once the task exists) calls `tk_wup_tsk()`; the task loops on `tk_slp_tsk(TMO_FEVR)` and discards queued wake-ups with `tk_can_wup()` | `app_program/subsystem1/subsystem1.c`, `app_program/subsystem1/task_led.c` | every periodic task (Motion 20 ms, Line 5 ms, IMU 10 ms, ObsScan 60 ms, Telemetry: period TODO(owner-to-confirm, buddy 1)) in Phase 2; one cyclic handler per subsystem in `TEAM.md` section 6 |
| IF-NN interface IDs, listed in the owning subsystem's `subsystemN.h` and in `TEAM.md` section 5 | `app_program/subsystemN/subsystemN.h`, `TEAM.md` | `app_program/subsystemN/subsystemN.h`, `TEAM.md`. Deviation: objects are created centrally by `ipc_init()`, so the headers list their IFs in comments and declare no object IDs yet |
| Test IDs `S<N>-U<nn>`, `INT-<nn>`, `SYS-<nn>`, never reused; one `[TEST] <ID> PASS\|FAIL` console line per check; `TEST_INIT` / `TEST_CHECK` / `TEST_SUMMARY` | `app_program/test_report.h`, `testing/README.md` | `app_program/test_report.h` (output through the `dbg_` ring), `testing/` |
| Team tables: members, pins, peripherals (one owning task each), priorities, interfaces, object budget | `TEAM.md` | `TEAM.md` (our data) |
| Test and video records | `testing/`, `videos/README.md` | `testing/`, `videos/README.md` (example rows cleared; Windows USB CDC capture) |

## Demo code we did not port, and why

| Template file | What it shows | Why it is not ported |
|---|---|---|
| `app_program/subsystem1/task_led.c` | LED on GP19 paced by a cyclic handler | Drives the pin with libbsp `gpio_set_pin` / `gpio_set_val`. We do not use libbsp (`libbsp_pwm.c` clashes with the SDK PWM API, rule R4); drivers use the SDK `hardware_*` API in `drv_*.c`. GP19 is unclaimed in our pin map. The wake pattern itself is kept (above) |
| `app_program/subsystem2/task_producer.c` | Producer task pinned to core 1 | Uses `TP_PRC2` / `TA_ASSPRC` core affinity from the SMP kernel copy. Our kernel is the single-core `mtk3_bsp` (`pico_rp2040`); core 1 stays unused (rule R5) |
| `app_program/subsystem3/task_consumer.c` | Consumer task, statistics, mailbox and memory pool | Formats text with `tm_sprintf` and relies on `TP_PRC1`. Tasks never use T-Monitor calls (rule R3b) |
| `app_program/subsystem4/task_monitor.c` | Monitor task: alarm, eventflag, message buffer, mutex, task control | Prints with `tm_printf` from a task (rule R3b: only the port's boot/fault files may), and suspends / re-prioritises another subsystem's task. Our tasks talk only through the IPC objects in `TEAM.md` section 5 |
| `app_program/app_main.c` banner and `CHECK()` | `tm_printf` boot banner and error print | Replaced by `APP_CHECK()` (a `dbg_` log line, Batch B). `usermain()` runs on the 1 KB init stack and does no console I/O |

## Phase 2 WiFi: reference only

The template's CYW43 / lwIP / MQTT stack (`lib/libwifi`, `lib/liblwip`, the
built-in `cyw43_utk_task` and `mqtt_utk.c`) is reference only. It is part of
the template's SMP kernel copy, and its sources mix kernel and SDK headers in
one file (rule R1). Telemetry (subsystem 1) designs its own WiFi/MQTT
ownership in Phase 2: TODO(phase 2, buddy 1).

## Not used at all

- `build_make/`: the GNU make flow under WSL with Pico SDK 2.2.0. We build
  with CMake and Ninja on Windows with Pico SDK 1.5.1 (`README.md`).
- The template's own kernel copy (`kernel/`, `include/`, `lib/`, `config/`,
  `device/`, `etc/`). Our kernel is the pinned submodule `external/mtk3_bsp`.
- The `CRASH_REPORT=1` UART0 crash console on GP0/GP1. GP0/GP1 are the right
  wheel encoder in our pin map, and USB CDC is the only console.
- `TASK_STACK_SIZE` (one stack size for every task). Our stacks are per task
  in `config/priorities.h`.
