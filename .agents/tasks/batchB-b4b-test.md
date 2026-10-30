# Batch B stage B4b: self-test hardware test (for Buddy 2 and his AI agent)

Written 30 Oct 2026 by the Batch B helper (Buddy 3). Plan: `.agents/tasks/batchB-plan.md`, section B4.
**Run this only after B4a passed** (`.agents/tasks/batchB-b4a-test.md`).

## What this is

B4b adds the CMake option `PLATFORM_SELFTEST` (default OFF; Debug builds only). A self-test task then runs once per power-up, in three stages:
1. Checks:
   - no fault record at power-up;
   - the CRC-32 check value;
   - the stack painting bounds;
   - late kernel text through `tm_printf`.

   Then it **forces a HardFault on purpose**.
2. After the automatic reset: the fault record is printed once with the right PC. Then a plain reset.
3. After the plain reset: the fault record must NOT be printed again.

The stage is kept in a watchdog scratch register, which a reset keeps and unplugging clears. So the board resets twice by itself; this is expected. Then it stays up. Unplugging and replugging runs the whole sequence again.

What it proves, if it passes:
- the HardFault handler works;
- the RAM fault record survives a reset;
- the watchdog reset works;
- the CRC rejects random RAM;
- the record is printed only once;
- kernel text written after boot still reaches the console;
- the stack monitor's bounds are right.

After B4a and B4b pass, teammates can flash and run their own tasks (milestone M1-lite).

## Rules for the agent

Same as B4a:
- Never edit, commit or push.
- Steps marked **PERSON** need the person.
- Run commands from the repo root in PowerShell.
- On a failure, stop and report.

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session.md is fine)
git fetch origin
git switch batchB/b4b-selftest
git pull --ff-only
git log --oneline -3            # note the top hash
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build the self-test firmware (agent)

HANDOVER.md section 5 step 3 environment block first. Then:

```powershell
if (Test-Path build/b4b) { Remove-Item -Recurse -Force build/b4b }
& $CM -S . -B build/b4b @common -DDBG_BACKEND=USB -DPLATFORM_SELFTEST=ON 2>&1 | Tee-Object build/b4b-configure.txt
& $CM --build build/b4b 2>&1 | Tee-Object build/b4b-build.txt
Select-String -Path build/b4b-configure.txt -Pattern 'PLATFORM_SELFTEST=ON'                                        # expect one match
Select-String -Path build/b4b-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/b4b-build.txt -Pattern 'check_rules: PASS'                                               # expect one match
Test-Path build/b4b/robocar.uf2                                                                                    # expect True
& "$R\gcc-arm-none-eabi\bin\arm-none-eabi-nm.exe" build/b4b/robocar.elf | Select-String 'hal_selftest_force_fault'  # note the address
```

### 3. Flash (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold BOOTSEL, plug in, release. `RPI-RP2` appears. Say "done". (Unplugging also clears the self-test stage, so the test starts at stage 1.)
2. **Agent:** copy the firmware, then start step 4 at once:

```powershell
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
if (-not $v) { throw "RPI-RP2 drive not found" }
Copy-Item build/b4b/robocar.uf2 "$($v.DriveLetter):\"
```

### 4. Capture 90 s, through both automatic resets (agent)

```powershell
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 90 -Out build/b4b-console.txt
```

The script reconnects after each reset, so all three stages land in one log, separated by `[capture] port lost` / `[capture] reconnected` lines (2 of each are expected here).

### 5. Check the log (agent)

Expected, in this order (boot lines of B4a repeat after every reset and are not listed):

| # | Expected | Pass rule |
|---|---|---|
| 1 | `[SELFTEST] stage 1 of 3: checks, then a forced HardFault` | present |
| 2 | `[SELFTEST] PASS no fault record at power-up (random RAM rejected)` | PASS |
| 3 | `[SELFTEST] PASS CRC-32 check value 0xCBF43926` | PASS |
| 4 | `[SELFTEST] stack <name>: <n> of <m> B never used (...)` | one line each for `dbg_drain`, `selftest`, `init (usermain window)`; every `<n>` > 0 |
| 5 | `[SELFTEST] PASS own stack pointer inside its painted bounds, ...` and `[SELFTEST] PASS every started stack ...` | both PASS |
| 6 | `[SELFTEST] late kernel text via tm_printf` | present (kernel text after boot reaches the console) |
| 7 | `[SELFTEST] summary: <p> passed, 0 failed` | 0 failed |
| 8 | `[SELFTEST] expected [FAULT] pc: 0x<a>..0x<b> (hal_selftest_force_fault)` | note a and b |
| 9 | `[capture] port lost`, then `[capture] reconnected` | the forced reset |
| 10 | `[FAULT] HardFault before the last reset: pc=0x<p> ... resets=1` | exactly ONE such line in the whole log; a <= p < b; `resets=1` |
| 11 | `[SELFTEST] stage 2 of 3: after the forced fault` and `[SELFTEST] PASS fault record survived the reset ...` | PASS |
| 12 | `[capture] port lost`, then `[capture] reconnected` | the plain reset |
| 13 | `[SELFTEST] stage 3 of 3: after the plain reset` and `[SELFTEST] PASS printed fault record was cleared (not shown again)` | PASS, and still only one `[FAULT]` line in the whole log |
| 14 | `[SELFTEST] all stages done: ... 0 failed in stage 3` | 0 failed |
| 15 | `[DBG] alive` lines after stage 3 | the board stays up; no third reset |
| 16 | no `FAIL`, no `[ERR]` | none |

Helpful:

```powershell
Select-String build/b4b-console.txt -Pattern '\[SELFTEST\]|\[FAULT\]|\[ERR\]|port lost|reconnected'
(Select-String build/b4b-console.txt -Pattern '\[FAULT\]').Count        # expect 1
& "$R\gcc-arm-none-eabi\bin\arm-none-eabi-addr2line.exe" -f -e build/b4b/robocar.elf 0x<p>   # expect hal_selftest_force_fault, port_fault.c
```

### 6. Power cycle (PERSON + agent)

1. Agent: `powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 90 -Out build/b4b-powercycle.txt`
2. **PERSON** (while it runs): unplug USB, wait 10 s, plug in again.
3. Pass: the log shows `stage 1 of 3` again with `PASS no fault record at power-up`. So the old record does not survive a power cycle as a valid record, and the CRC rejects whatever RAM holds. Then the same sequence as step 5 follows.

## If it fails: what to look at

| Symptom | Most likely place | What to collect |
|---|---|---|
| No `[SELFTEST]` lines at all | configure did not set `PLATFORM_SELFTEST=ON`, or no terminal (the task waits for one) | `build/b4b-configure.txt` |
| Board resets but no `[FAULT]` line after it | fault record not kept (`.uninitialized_data`), or the drain did not print it | gdb: `print s_record`, `print/x watchdog_hw->scratch[0]` |
| `[FAULT]` pc outside the expected range | the HardFault entry read the wrong stack | the full `[FAULT]` line, and `addr2line` of the pc |
| More than one `[FAULT]` line | the record is not cleared after printing | the whole log |
| Keeps resetting (more than 2 resets) | a fault loop, not the self-test | the whole log; then flash the B4a firmware again to recover |
| Stage 1 shows `FAIL no fault record at power-up` | random RAM passed the CRC (very unlikely) or the record survived a power cycle | the whole log |
| A `stack ... 0 of` line | the stack was used to the bottom, or painting missed it | the line |

## Report (post on the B4b PR, and give it to the helper)

```text
B4b self-test, <date>, by <name>
Commit: <hash>
Build: warnings=<n>, check_rules=<PASS/FAIL>, PLATFORM_SELFTEST=ON confirmed
Expected lines 1-16: <PASS/FAIL each>
[FAULT] pc=0x<p>, expected range 0x<a>..0x<b>, addr2line: <function>
Stack lines: <name: unused/size, ...>
Power-cycle test: PASS/FAIL
Verdict: PASS / FAIL
Log: (attach build/b4b-console.txt and build/b4b-powercycle.txt)
```

If B4a and B4b both pass: milestone **M1-lite**, and teammates can start running their tasks on the car.
