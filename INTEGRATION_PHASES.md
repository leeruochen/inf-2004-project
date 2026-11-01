# System Integration & Lab Course Testing Guide (`INTEGRATION_PHASES.md`)

**Project:** INF2004 Autonomous Line-Following RoboCar (Pico W + Cytron Robo Pico, micro T-Kernel 3.0)  
**Maintainer:** Buddy 2 (Ruo Chen, `2500530` — Vehicle Controller & Motion Control Hub)  
**Related Docs:** `HANDOVER.md`, `TEAM.md`, `docs/DATA_REPRESENTATION.md`, `docs/DEBUG.md`, `testing/README.md`, `videos/README.md`

---

## 1. How All 5 Subsystems Wire Together (`app_program/app_main.c`)

No custom "glue file" is required when merging subsystem branches into `main`. The platform two-phase startup in `usermain()` (`app_program/app_main.c`) already initializes and starts all five subsystems in deterministic order:

1. **`ipc_init()` (`src/common/ipc.c`)**: Creates all 8 shared kernel IPC objects (`MBF_NAV_IN`, `MBF_MOTION_CMD`, `MBF_OBSTACLE`, `MBF_TELEMETRY`, `SEM_ECHO`, `EVF_SAFETY`, `EVF_NAV`, `MTX_MOTION`) before any subsystem code runs.
2. **`subsystem1_init()` .. `subsystem5_init()`**: Each subsystem initializes its hardware drivers in a safe state (for example, `subsystem2_init()` calls `motion_emergency_brake()` so all four motor channels `GP8`–`GP11` are LOW before tasks start).
3. **`subsystem1_start()` .. `subsystem5_start()`**: Each subsystem spawns its real-time tasks via `app_start_task()` at the fixed priorities in `config/priorities.h`:

| Priority | Task Name | Subsystem | Owner | Period / Trigger |
|---|---|---|---|---|
| **1 (Highest)** | `safety` | Subsystem 4 | Buddy 4 (Vincent) | Event-driven (`tk_wai_flg` on `EVF_SAFETY` with `SAFETY_IMU_WDOG_MS = 50 ms` timeout) |
| **2** | `vc` | Subsystem 2 | Buddy 2 (Ruo Chen) | Message-driven (`tk_rcv_mbf` on `MBF_NAV_IN`, `20 ms` timeout poll) — **DONE on `main`** |
| **3** | `motion` | Subsystem 2 | Buddy 2 (Ruo Chen) | Periodic **`20 ms`** (`app_period_wait`, `MOTION_PERIOD_MS`) — **DONE on `main`** |
| **4** | `line` | Subsystem 3 | Buddy 3 (Jin Yong) | Periodic **`5 ms`** (`app_period_wait`, `LINE_PERIOD_MS`) |
| **5** | `barcode` | Subsystem 3 | Buddy 3 (Jin Yong) | Periodic **`2 ms`** while capturing (`BC_POLL_PERIOD_MS`, D3) |
| **6** | `obs_scan` | Subsystem 5 | Buddy 5 (Chay Han) | Periodic **`60 ms`** (`OBS_SCAN_PERIOD_MS`) + `SEM_ECHO` (`30 ms` timeout) |
| **7** | `obs_avoid` | Subsystem 5 | Buddy 5 (Chay Han) | Message-driven (`MBF_OBSTACLE`) / periodic during bypass |
| **8** | `imu` | Subsystem 4 | Buddy 4 (Vincent) | Periodic **`10 ms`** (`IMU_PERIOD_MS`) |
| **9 (Lowest app)** | `telemetry` | Subsystem 1 | Buddy 1 (Damian) | Periodic **`200 ms`** (`TEL_PERIOD_MS`) + `MBF_TELEMETRY` drain |
| **10** | `dbg_drain` | Platform | Platform (Batch B) | Periodic `20 ms` USB CDC non-blocking drain (`DBG_ENABLE` builds) |
| **11** | `test` | Platform | Team (`APP_TEST=ON`) | Runs `subsystem1_test()..subsystem5_test()` once (`APP_TEST` builds only) |

---

## 2. Exact Code Contracts for Integrating with Subsystem 2

Subsystem 2 (`app_program/subsystem2/`) is already merged and calibrated on `main`. Below are the exact APIs and IPC calls that Subsystems 1, 3, 4, and 5 use to interact with Subsystem 2.

### 2.1 Subsystem 1 (Buddy 1 — Telemetry, WiFi & MQTT)
Include `#include "../subsystem2/subsystem2.h"` and `#include "src/common/data_repr.h"`:

1. **Sending Remote Commands (`START`, `STOP`, `RESET`) into `IF-01` (`MBF_NAV_IN`)**:
   - On boot, Vehicle Controller (`vc`) transitions from `VC_SYS_INITIALISING` to **`VC_SYS_READY_WAITING`** and keeps the motors braked until it receives `DREP_NAV_REMOTE_START`.
   ```c
   nav_input_t start_msg = drep_nav_remote(DREP_NAV_REMOTE_START);
   (void)tk_snd_mbf(MBF_NAV_IN, &start_msg, (INT)sizeof(start_msg), TMO_POL);

   nav_input_t stop_msg  = drep_nav_remote(DREP_NAV_REMOTE_STOP);
   (void)tk_snd_mbf(MBF_NAV_IN, &stop_msg, (INT)sizeof(stop_msg), TMO_POL);

   nav_input_t reset_msg = drep_nav_remote(DREP_NAV_REMOTE_RESET);
   (void)tk_snd_mbf(MBF_NAV_IN, &reset_msg, (INT)sizeof(reset_msg), TMO_POL);
   ```
2. **Reading Motion State (`IF-08`, `MTX_MOTION`) and System FSM State**:
   ```c
   motion_state_t st;
   if (motion_get_state(&st) == E_OK)
   {
       int32_t  speed_x10 = drep_ms_speed_x10(st.w);      /* 0.1 cm/s */
       uint32_t dist_cm   = drep_ms_distance_cm(st.w);    /* cumulative cm */
       int32_t  enc_l     = st.enc_count[ENC_WHEEL_LEFT];
       int32_t  enc_r     = st.enc_count[ENC_WHEEL_RIGHT];
   }
   uint32_t sys_fsm = vc_get_system_state(); /* 4-bit state: VC_SYS_INITIALISING (0) .. VC_SYS_NAV_TURN_RIGHT (9) */
   ```
3. **Receiving State-Transition Events on `IF-04` (`MBF_TELEMETRY`)**:
   - Every time the 10-state System FSM changes state, `task_vc` posts `drep_tel_state(now_ms, new_state)` (`DREP_TEL_STATE = 4U`) to `MBF_TELEMETRY` with `TMO_POL`.

---

### 2.2 Subsystem 3 (Buddy 3 — Line-Following & Barcode Decode)
Include `#include "../subsystem2/subsystem2.h"` and `#include "src/common/data_repr.h"`:

1. **Line-Following Steering (`IF-01`, `MBF_NAV_IN`, every `5 ms`)**:
   - **Hold-Window Rule (`MOTION_T_HOLD_MS = 100 ms`)**: Motion Control holds a continuous `DREP_MC_STEER` command for at most `100 ms` (`5` motion periods). Because `task_line` runs every `5 ms`, it refreshes the steering command `20` times per `100 ms` window. If `task_line` stops sending commands for `> 100 ms`, the car automatically stops.
   - **Steering Sign (`D11`)**: `steer_err_q7` is in Q0.7 (`-128 .. +127`, representing `-1.0 .. +0.992`). In `task_motion.c`, **positive `steer_err_q7 > 0` steers RIGHT** (`left_duty += steer_pm`, `right_duty -= steer_pm`).
   ```c
   /* Continuous steering while on the line: */
   nav_input_t steer_msg = drep_nav_steer(steer_err_q7);
   (void)tk_snd_mbf(MBF_NAV_IN, &steer_msg, (INT)sizeof(steer_msg), TMO_POL);

   /* If the line is completely lost after recovery timeout: */
   nav_input_t lost_msg = drep_nav_line_Simple(DREP_NAV_LINE_TRACK_LOST);
   (void)tk_snd_mbf(MBF_NAV_IN, &lost_msg, (INT)sizeof(lost_msg), (TMO)NAV_CRITICAL_SEND_TMO_MS);
   ```
2. **Barcode Commands (`IF-01`, `MBF_NAV_IN`, `5 ms` send timeout) & Distance Reading (`IF-08`)**:
   - Use `motion_get_state(&st)` and `drep_ms_distance_cm(st.w)` to check `BC_D_CMD_CM` (`30 cm` expiry) and `BC_REPEAT_IGNORE_CM` (`10 cm` de-dup).
   - When triggering a barcode action (`DREP_BC_STOP = 0`, `DREP_BC_LEFT = 1`, `DREP_BC_RIGHT = 2`, `DREP_BC_UTURN = 3`):
   ```c
   nav_input_t bc_msg = drep_nav_barcode(DREP_BC_LEFT, false);
   (void)tk_snd_mbf(MBF_NAV_IN, &bc_msg, (INT)sizeof(bc_msg), (TMO)NAV_CRITICAL_SEND_TMO_MS);
   ```

---

### 2.3 Subsystem 4 (Buddy 4 — IMU Processing & Safety Task)
Include `#include "../subsystem2/subsystem2.h"`:

1. **Emergency Brake (`IF-11`) & Fault Latch (`IF-06`, `EVF_SAFETY`)**:
   - When `task_safety` (Priority 1) detects `EVF_SAFETY_COLLISION` or a `SAFETY_IMU_WDOG_MS` (`50 ms`) timeout on `EVF_SAFETY_IMU_ALIVE`:
   ```c
   /* 1. Drive all 4 motor PWM channels LOW immediately in hardware (< 10 us): */
   motion_emergency_brake();

   /* 2. Latch the fault in EVF_SAFETY so VC (P2) and Motion (P3) stay locked out: */
   (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED);
   ```
   - **Latch Clearing Rule (`IF-06`)**: `task_safety` **never** clears `EVF_SAFETY_FAULT_LATCHED`. Only `task_vc` clears `EVF_SAFETY_FAULT_LATCHED` when a remote `RESET` arrives **and** the car has been stationary (`speed == 0` and encoder counts unchanged) for at least `SAFETY_T_STILL_MS` (`500 ms`).

---

### 2.4 Subsystem 5 (Buddy 5 — Obstacle Scanning & Avoidance)
Include `#include "src/common/ipc.h"` and `#include "src/common/data_repr.h"`:

1. **Taking Avoidance Authority (`IF-07` `EVF_NAV` + `IF-01` `MBF_NAV_IN`)**:
   - When an obstacle is confirmed within `OBS_D_DETECT_CM` (`25 cm`), set `EVF_NAV_AVOID_ACTIVE` and send bypass commands (`DREP_BYPASS_STOP = 0`, `DREP_BYPASS_LEFT = 1`, `DREP_BYPASS_RIGHT = 2`, `DREP_BYPASS_STRAIGHT = 3`, `DREP_BYPASS_RELEASE = 4`) to `MBF_NAV_IN`:
   ```c
   (void)tk_set_flg(EVF_NAV, EVF_NAV_AVOID_ACTIVE);

   nav_input_t avoid_msg = drep_nav_avoid(DREP_BYPASS_LEFT, seq_2bit, false);
   (void)tk_snd_mbf(MBF_NAV_IN, &avoid_msg, (INT)sizeof(avoid_msg), (TMO)NAV_CRITICAL_SEND_TMO_MS);
   ```
2. **Releasing Authority Back to Line-Following (`DREP_BYPASS_RELEASE`)**:
   - Once the line is reacquired during return-to-line:
   ```c
   (void)tk_clr_flg(EVF_NAV, ~EVF_NAV_AVOID_ACTIVE);
   (void)tk_set_flg(EVF_NAV, EVF_NAV_LINE_REACQUIRED);

   nav_input_t rel_msg = drep_nav_avoid(DREP_BYPASS_RELEASE, seq_2bit, false);
   (void)tk_snd_mbf(MBF_NAV_IN, &rel_msg, (INT)sizeof(rel_msg), (TMO)NAV_CRITICAL_SEND_TMO_MS);
   ```

---

## 3. Pre-Integration Hardware & Build Checklist

Before running pairwise or lab-course tests after teammates merge their branches:

1. **Pull `main` and Run the Compile + Rule Gate**:
   ```powershell
   git pull origin main
   cmake -P cmake/check_rules.cmake
   cmake --build build/test --parallel 8
   cmake --build build/s0b --parallel 8
   ```
   Verify **0 compiler warnings** (`-Wall -Wextra -Werror`) and `-- check_rules: PASS`.
2. **Verify All Physical Wiring Against `TEAM.md` Section 2**:
   - [x] **Motors (`GP8`–`GP11`) & Encoders (`GP0`/`GP1`, `GP16`/`GP17`)**: Verified on 3.3 V Grove 1 & Grove 4; polarities and geometry (`ENC_CPR = 616U`, `WHEEL_CIRC_MM = 207U`, `MOTION_EFF_TRACK_MM = 114U`) set in `config/pins.h` and `config/thresholds.h`.
   - [ ] **Grove 6 `GP26` Lead Taped/Capped**: Grove 6 shares `GP26` with Grove 5 (`IR_R` line sensor `ADC0`); ensure the unused lead on Grove 6 cannot short.
   - [ ] **`GP28` `VBAT` Jumper OPEN**: So `ADC2` (`GP28`) reads the centre line sensor (`IR_C`) instead of the battery divider.
   - [ ] **Grove 2 Unused**: Because `GP2`/`GP3` (`I2C1`) are used by the GY-511 IMU on the Maker/Qwiic port.
   - [ ] **Ultrasonic (`GP4`/`GP5`, Grove 3) & Servo (`GP15`, PWM Slice 7B)**: `GP5` echo is 3.3 V-safe; servo signal is on `GP15` (never `GP12`).

---

## 4. Phase 1: Pairwise Integration Testing (`testing/integration/TEST_LOG.md`)

Run these five pairwise tests in order as each buddy finishes their subsystem. Record every run in `testing/integration/TEST_LOG.md` with its saved log in `testing/integration/logs/`.

| Test ID | Subsystems | Step-by-Step Procedure | Pass Criteria |
|---|---|---|---|
| **`INT-01`** | **S3 (Line) $\rightarrow$ S2 (VC + Motion)** | 1. Elevate wheels on the bench; transition VC to `NAV_LINE_TRACKING`.<br>2. Move a black tape strip under `IR_C` (`GP28`) and `IR_R` (`GP26`).<br>3. Place car on a 1 m floor line and run at `MOTION_SPEED_CRUISE_BRINGUP_CMPS` (`20 cm/s`). | - Positive `steer_err_q7` (line to the right) speeds up left wheel and slows right wheel.<br>- Car tracks the 1 m line smoothly.<br>- Lifting the car off the line (`TRACK_LOST`) stops both wheels and returns VC to `READY_WAITING`. |
| **`INT-02`** | **S3 (Barcode) $\leftrightarrow$ S2 (VC + Motion)** | 1. Drive over (or slide under `GP27`) a valid barcode card (`LEFT`, `RIGHT`, `U_TURN`, `STOP`).<br>2. Trigger junction detection (`DREP_NAV_LINE_JUNCTION`). | - S3 reads `motion_get_state()` under `MTX_MOTION` without blocking.<br>- VC transitions to `VC_SYS_NAV_TURN_LEFT` / `RIGHT` / `U_TURN` / `READY_WAITING` and executes the calibrated `90°` / `180°` turn. |
| **`INT-03`** | **S5 (Obstacle) $\rightarrow$ S2 (VC + Motion)** | 1. While driving in `NAV_LINE_TRACKING`, place a box `< 25 cm` ahead of `GP4`/`GP5`.<br>2. Observe servo scan (`GP15`) and bypass sequence.<br>3. Trigger line reacquisition (`EVF_NAV_LINE_REACQUIRED` + `DREP_BYPASS_RELEASE`). | - VC switches authority to `VC_AUTH_AVOID` (`VC_SYS_OBSTACLE_AVOIDANCE`) and ignores S3 line steering.<br>- Upon release, VC restores `VC_AUTH_LINE` (`VC_SYS_NAV_LINE_TRACKING`). |
| **`INT-04`** | **S4 (IMU/Safety) $\rightarrow$ S2 (Brake + VC)** | 1. With wheels spinning in `NAV_LINE_TRACKING`, tap the chassis (`> 2.5 g`) or disconnect IMU I2C to trip the `50 ms` watchdog.<br>2. Send remote `RESET` while wheels are spun by hand, then send `RESET` after `500 ms` stationary. | - `motion_emergency_brake()` drives `GP8`–`GP11` LOW immediately.<br>- VC enters `VC_SYS_SAFETY_FAULT_LATCHED` and rejects drive commands.<br>- First `RESET` (moving) is ignored; second `RESET` (`>= 500 ms` stationary) clears `EVF_SAFETY_FAULT_LATCHED` and returns to `READY_WAITING`. |
| **`INT-05`** | **S1 (Telemetry) $\leftrightarrow$ S2 (VC + Motion)** | 1. Boot car (`READY_WAITING`).<br>2. Send MQTT `cmd_start`, drive 1 m, send `cmd_stop`.<br>3. Inspect dashboard / MQTT `telemetry_motion`, `event_fault`, and `heartbeat` frames. | - `cmd_start` starts navigation; `cmd_stop` halts the car and returns to `READY_WAITING`.<br>- `heartbeat` reports `vc_get_system_state()` (`0..9`), and `telemetry_motion` matches `motion_get_state()`. |

> **How to start the car on the lab course before Buddy 1's WiFi/MQTT is merged:**  
> Because `vc_task_entry()` waits in `VC_SYS_READY_WAITING` until a `DREP_NAV_REMOTE_START` message arrives on `MBF_NAV_IN`, you can temporarily post `drep_nav_remote(DREP_NAV_REMOTE_START)` after a 5-second boot delay in `subsystem1.c` (or on pressing Robo Pico button `GP20`) when running untethered track tests without WiFi.

---

## 5. Phase 2: Full System & Lab Course Testing (`testing/system/TEST_LOG.md`)

Once `INT-01`..`INT-05` pass, test the complete car on the **lab course** using the production firmware (`build/s0b/robocar.uf2`, where `APP_TEST=OFF`).

### 5.1 Building & Flashing the Production Firmware
```powershell
cmake --build build/s0b --parallel 8
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
Copy-Item build/s0b/robocar.uf2 "$($v.DriveLetter):\"
```

### 5.2 Speed Progression on the Lab Course (`config/thresholds.h`)
- **Initial Lab Course Runs (`MOTION_BRINGUP_SPEEDS = 1U`, current default)**:
  - Cruise = `20 cm/s`, Slow/Turn = `10 cm/s`, Bypass = `15 cm/s`.
  - Keep `MOTION_BRINGUP_SPEEDS = 1U` while tuning S3's line-following gain and S5's obstacle bypass timings on the physical lab track.
- **Nominal Lab Course Runs (`MOTION_BRINGUP_SPEEDS = 0U`)**:
  - Cruise = `30 cm/s`, Slow/Turn = `15 cm/s`, Bypass = `20 cm/s`.
  - Flip `#define MOTION_BRINGUP_SPEEDS (0U)` in `config/thresholds.h` once the car completes a full lap reliably at bring-up speeds.

### 5.3 Planned System Test Cases (`SYS-02`..`SYS-06`)
Add these rows to `testing/system/TEST_LOG.md` before running them on the lab course:

| ID | Lab Course Test | Requirement | Method | Steps on the Lab Course | Expected Result |
|---|---|---|---|---|---|
| **`SYS-02`** | End-to-end line-following lap | FR2, FR5, FR6, NFR1 | manual + log + video | Place car at course start; send remote `START` via MQTT; run a full lap of straights and curves; send remote `STOP` at finish | Car stays on the line for the entire lap, reports live speed/distance over MQTT (`telemetry_motion`), and stops cleanly at finish |
| **`SYS-03`** | Barcode-guided junction turns | FR3, FR4 | manual + log + video | Place barcode cards (`LEFT`, `RIGHT`, `U_TURN`, `STOP`) before course junctions; run car along the line | Car decodes each barcode within `BC_D_CMD_CM` (`30 cm`), executes the commanded `90°`/`180°` turn or stop at the junction, and sends `event_barcode` to MQTT |
| **`SYS-04`** | Hump traversal & pitch detection | FR7, FR8 | manual + log + video | Drive car over the lab course hump/ramp during autonomous line tracking | IMU detects pitch $\ge 5^\circ$ (`IMU_HUMP_PITCH_DEG`); VC enters `VC_SYS_NAV_HUMP_SENSING` (slow speed), traverses hump without losing the line, and sends `event_hump` |
| **`SYS-05`** | Obstacle detection, scan & bypass | FR9, FR10, FR11 | manual + log + video | Place an obstacle box on the track line; run car toward the obstacle | Car stops at `< 25 cm`, sweeps servo (`GP15`), sends `event_obstacle`, steers around the clearer side (`VC_AUTH_AVOID`), reacquires the black line, and resumes `NAV_LINE_TRACKING` |
| **`SYS-06`** | Collision fault latch & stationary reset | FR12, IF-06, IF-11 | manual + log + video | Trigger a collision (`> 2.5 g`) or run off-track (`TRACK_LOST`); attempt remote `RESET` while moving, then after `500 ms` stationary | Motors brake immediately (`motion_emergency_brake()`); `event_fault` sent; moving `RESET` rejected; stationary `RESET` clears latch and returns to `READY_WAITING` |

---

## 6. Phase 3: Recording Evidence & Demo Videos (`testing/` & `videos/README.md`)

Every integration and system test requires **both** a saved log and (for physical motion/course tests) a short unlisted YouTube video:

1. **Capture Console Logs (`testing/integration/logs/` or `testing/system/logs/`)**:
   ```powershell
   powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Port COM4 -Seconds 60 -Out testing/system/logs/2026-11-XX_sys02_lap.txt
   ```
2. **Record 15–60 s Unlisted YouTube Videos (`videos/README.md`)**:
   - State or show the **Test ID** (`INT-01`..`05`, `SYS-02`..`06`) and the **git commit hash** (`git rev-parse --short HEAD`) at the start of the clip.
   - Upload to YouTube as **Unlisted** (never commit video files into the git repository).
   - Add the YouTube link to `videos/README.md` and to the **Log / evidence** column of `testing/integration/TEST_LOG.md` or `testing/system/TEST_LOG.md`.
3. **Final Pre-Submission Sanity Gate**:
   - Ensure `git status` is clean, `cmake -P cmake/check_rules.cmake` reports `PASS`, and all `TEST_LOG.md` tables have committed log files and video links for every run.

