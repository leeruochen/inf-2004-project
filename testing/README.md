# Testing

All test records for the project live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, photos, scope captures
├── ...
├── subsystem5/
├── integration/           tests across two or more subsystems (shared)
└── system/                tests of the whole product against its requirements (shared)
```

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished product against a requirement | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Automate it if you can.** Add a `TEST_CHECK` with the same ID to
   `app_program/subsystemN/subsystemN_test.c` (see `app_program/test_report.h`).
   Results go through the `dbg_` ring, never `tm_printf` or `printf`.
   A test that needs eyes or instruments (an LED blinks, a motor turns, a
   scope shows 1 kHz) is a *manual* test. Write its steps in the table instead.
3. **Build the test image and flash it.** Run the PowerShell preamble from
   the root `README.md` section 1 first, then:
   ```powershell
   & $CM -S . -B build/test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
   & $CM --build build/test
   ```
   Test builds compile from Batch B on (TODO(phase 1 batch B, buddy 2): dbg_
   ring). `APP_TEST=ON` needs `DBG_ENABLE=ON`, so a Release build with
   `APP_TEST=ON` fails at configure. The test image runs every subsystem's
   tests in a dedicated lowest-priority test task (priority 11) and prints one
   `[TEST] <ID> PASS|FAIL` line per check on the USB CDC console. (A normal
   build leaves the tests out.) Flash it as in `README.md` section 3.
4. **Save the console output** to `logs/`, named `YYYY-MM-DD_<what>.txt`.
   The console is USB CDC on the Pico W's own USB port. Its COM port (Device
   Manager > Ports (COM & LPT)) appears only after the firmware boots, and it
   re-enumerates after every reset. The firmware waits up to
   `DBG_USB_CONNECT_WAIT_MS` (3 s, `config/timing.h`) for the host before the
   kernel starts, so open the terminal within 3 s of a reset to see the start
   of the log.
   - PuTTY: Connection type Serial, COMn, speed 115200, then
     *Session → Logging → All session output* to
     `testing/subsystemN/logs/YYYY-MM-DD_<what>.txt`.
   - PowerShell alternative (Ctrl-C to stop):
     ```powershell
     $p = New-Object System.IO.Ports.SerialPort COMn,115200; $p.Open(); while ($true) { $p.ReadLine() | Tee-Object -FilePath testing/subsystem1/logs/2026-10-14_unit.txt -Append }
     ```
   For manual tests, save photos or scope screenshots in `logs/` the same way
   (keep each file under a few MB). Videos go in `videos/` instead (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Commit the log and the table together.

## Videos of tests and demos

A manual test (an LED, a motor, a sensor reacting) is best shown with a short
video. Record 15–60 s showing the test ID and the hardware, share it as an
**unlisted YouTube** link (the only way videos are shared), list it in
**`videos/README.md`**, and put the link in the *Log / evidence* column of
the test's result row. `videos/README.md` explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
