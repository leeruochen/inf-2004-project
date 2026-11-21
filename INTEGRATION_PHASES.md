# System Integration & Lab Course Testing Guide (`INTEGRATION_PHASES.md`)

**Project:** INF2004 Autonomous Line-Following RoboCar (Pico W + Cytron Robo Pico, micro T-Kernel 3.0)  
**Maintainer:** Buddy 2 (Ruo Chen, `2500530` — Vehicle Controller & Motion Control Hub)  
**Related Docs:** `HANDOVER.md`, `TEAM.md`, `docs/DATA_REPRESENTATION.md`, `docs/DEBUG.md`, `docs/EMBEDDED_OPTIMIZATIONS.md`, `testing/README.md`, `videos/README.md`

---

## 0. Current Execution Roadmap (Track Validation $\rightarrow$ Final Logs $\rightarrow$ Nominal Speeds / Optimizations)

As of **22 Nov 2026**, **all five subsystems are implemented, merged on `main`, and running together on the car** over Wi-Fi/MQTT and the browser dashboard (`tools/dashboard/tel_web.py`):

1. **Current State of All 5 Subsystems (Integrated on `main`)**:
   - **S1 (Buddy 1 — Done & Live):** Telemetry task (`tel`), console and Wi-Fi/MQTT backends (`-DTEL_BACKEND_WIFI=ON`), IF-12 snapshots (`tel_set_ir_raw`, `tel_set_range_cm`, `tel_set_attitude`, `tel_post_obstacle`), decoded `A–Z` barcode events (`tel_post_barcode_letter`), framed non-retained `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`), single buffered state-change heartbeats, `test_tel` (`S1-H01..H04`), and all 7 dashboard test files (`test_tel_*.py`) PASS.
   - **S2 (Buddy 2 — Done):** `S2-U01..U28` PASS, safe optimizations `OPT-01..04` applied, `S2-U24/U25` fault-latch reset fix applied, `BC_D_CMD_CM` updated to `60 cm`, `-DMOTION_STEER_TARGETS=1` differential wheel targets, and gradual IMU-pitch hump climb speed/torque assist (`MOTION_HUMP_*`) active.
   - **S3 (Buddy 3 — Done & Merged):** Line-Following (`task_lf`, `drv_ir.c`, `lf_core.c`, plus straddle mode `-DS3_LF_STRADDLE=ON -DLF_SD_SIMPLE=1 -DLF_SD_BANG=1`) and Code 39 Barcode (`task_bc`, `drv_bc.c`, `bc_edge.c`, `bc_frame.c`, `bc_decode.c`, `BC_MIN_SPEED_MMPS = 20 mm/s`, runtime 4-letter `bc_task_set_map()`) are live; all 7 S3 host tests PASS.
   - **S4 (Buddy 4 — Done & Calibrated):** `drv_imu.c` auto-detects LSM303D (`0x1D`) or LSM303DLHC (`0x19`/`0x1E`); lift/overturn (`imu_lift.c`, `> 30°` or `> 0.10 m/s` for `50 ms`) verified in `S4-U23` (`134 ms` brake); hump height estimator (`imu_hump.c`) calibrated to `IMU_HUMP_WHEELBASE_MM = 95 mm`, `3°` entry / `2.5°` exit, `15 mm` min travel, `9 mm` min height; `imu_lift` and `imu_hump` host tests PASS.
   - **S5 (Buddy 5 — Done & Track-Tuned):** `CAR_WIDTH_MM = 130 mm`; `90°` watch-ahead + halt + standstill confirm + stationary `5°` sweep (`50°–130°`, 4 pings / `12 ms` / `400 mm` cap -> 3-echo median + oblique dropout bridging + `40°` beam removal) + encoder-closed 4-turn `45°` bypass (`OBS_OBSTACLE_DEPTH_MM = 100 mm`) + directional rejoin search. Currently `OBS_BYPASS_SLOW_BRINGUP = 1U` (`5 cm/s`, `60 s` budget).
2. **Active Car Build Configuration (`build/wifi`)**:
   - The current on-car bring-up firmware uses `-DTEL_BACKEND_WIFI=ON -DS3_LF_STRADDLE=ON` with `-DLF_SD_SIMPLE=1 -DLF_SD_BANG=1 -DMOTION_STEER_TARGETS=1` and bring-up speed `5 / 5 / 5 cm/s` (`OBS_BYPASS_SLOW_BRINGUP = 1U`).
3. **Final Verification & Recording Remaining**:
   - Capture and commit formal saved console logs (`testing/integration/logs/`, `testing/system/logs/`) and unlisted YouTube links (`videos/README.md`) for `INT-01..05`, `SYS-02..06`, `S1-M03/M04/M05`, `S4-U22` / `S4-U26`, and `S5-U33` / `INT-S5-02..04`.
   - For the nominal `NFR14` (`<= 5.0 s`) bypass timing run: clear `OBS_BYPASS_SLOW_BRINGUP` to `0U` and restore `MOTION_SPEED_BYPASS_BRINGUP_CMPS` to `25U` and `MOTION_SPEED_BYPASS_MIN_CMPS` to `15U` (`config/thresholds.h`, `config/timing.h`), or set `MOTION_BRINGUP_SPEEDS = 0U` for full nominal speeds (`30 / 15 / 25 cm/s`).
4. **Post-Track Optimizations (`docs/EMBEDDED_OPTIMIZATIONS.md`)**:
   - After final baseline logs are saved: optionally adopt **Tier 1** (`OPT-05..07`, `OPT-09..13`, `OPT-19`), **Tier 2** (`OPT-08`, `OPT-17`), and **Tier 3** (`OPT-14..16`, `OPT-18`, `OPT-20`) with team agreement.

---

## 1. How All 5 Subsystems Wire Together (`app_program/app_main.c`)

No custom "glue file" is required: the platform two-phase startup in `usermain()` (`app_program/app_main.c`) initializes and starts all five subsystems in deterministic order:

1. **`ipc_init()` (`src/common/ipc.c`)**: Creates all 8 shared kernel IPC objects (`MBF_NAV_IN`, `MBF_MOTION_CMD`, `MBF_OBSTACLE`, `MBF_TELEMETRY`, `SEM_ECHO`, `EVF_SAFETY`, `EVF_NAV`, `MTX_MOTION`) before any subsystem code runs.
2. **`subsystem1_init()` .. `subsystem5_init()`**: Each subsystem initializes its hardware drivers in a safe state (for example, `subsystem2_init()` calls `motion_emergency_brake()` so all four motor channels `GP8`–`GP11` are LOW before tasks start, and `subsystem3_init()` checks both line-sensor ADC channels and initializes GP27 edge capture).
3. **`subsystem1_start()` .. `subsystem5_start()`**: Each subsystem spawns its real-time tasks via `app_start_task()` at the fixed priorities in `config/priorities.h`:

| Priority | Task Name | Subsystem | Owner | Period / Trigger |
|---|---|---|---|---|
| **1 (Highest)** | `safety` | Subsystem 4 | Buddy 4 (Vincent) | Event-driven (`tk_wai_flg` on `EVF_SAFETY` with `SAFETY_IMU_WDOG_MS = 50 ms` timeout) |
| **2** | `vc` | Subsystem 2 | Buddy 2 (Ruo Chen) | Message-driven (`tk_rcv_mbf` on `MBF_NAV_IN`, `20 ms` timeout poll) |
| **3** | `motion` | Subsystem 2 | Buddy 2 (Ruo Chen) | Periodic **`20 ms`** (`app_period_wait`, `MOTION_PERIOD_MS`) |
| **4** | `lf` | Subsystem 3 | Buddy 3 (Jin Yong) | Periodic **`5 ms`** (`app_period_wait`, `LF_SAMPLE_PERIOD_MS`), posts steering every `20 ms` (`LF_STEER_POST_PERIOD_MS`) |
| **5** | `bc` | Subsystem 3 | Buddy 3 (Jin Yong) | ISR-woken on GP27 edge, then **`2 ms`** capture poll (`BC_POLL_PERIOD_MS`, D3) |
| **6** | `obs_scan` | Subsystem 5 | Buddy 5 (Chay Han) | Periodic **`60 ms`** (`app_period_wait`, `OBS_SERVO_STEP_PERIOD_MS`) + `SEM_ECHO` (`30 ms` watch / `12 ms` sweep timeout) |
| **7** | `obs_avoid` | Subsystem 5 | Buddy 5 (Chay Han) | `50 ms` idle poll / `MBF_OBSTACLE` receive -> `20/40 ms` closed-loop bypass poll |
| **8** | `imu` | Subsystem 4 | Buddy 4 (Vincent) | Periodic **`10 ms`** (`app_period_wait`, `IMU_SAMPLE_PERIOD_MS`) |
| **9 (Lowest app)** | `tel` | Subsystem 1 | Buddy 1 (Damian) | Periodic **`20 ms`** wake (`TEL_WAKE_PERIOD_MS`), publishes telemetry every **`200 ms`** (`TEL_PUBLISH_PERIOD_MS`), heartbeat every **`1 s`** (`TEL_HEARTBEAT_PERIOD_MS`) + `MBF_TELEMETRY` drain |
| **10** | `dbg_drain` | Platform | Platform | Periodic **`10 ms`** (`DBG_DRAIN_PERIOD_MS`) USB CDC non-blocking drain (`DBG_ENABLE` builds) |
| **11** | `app_test` | Platform | Team (`APP_TEST=ON`) | Runs `subsystem1_test()..subsystem5_test()` once (`APP_TEST` builds only) |

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
       int32_t  enc_l     = st.enc_count[0];              /* [0] left, [1] right */
       int32_t  enc_r     = st.enc_count[1];
   }
   uint32_t sys_fsm = vc_get_system_state(); /* 4-bit state: VC_SYS_INITIALISING (0) .. VC_SYS_NAV_TURN_RIGHT (9) */
   ```
3. **Receiving Events (`IF-04` `MBF_TELEMETRY`) & Applying Runtime Barcode Maps (`car/cmd/bc_map`)**:
   - Every time the 10-state System FSM changes state, `task_vc` posts `drep_tel_make(DREP_TEL_STATE, now_ms, state, false)` (`DREP_TEL_STATE = 4U`) to `MBF_TELEMETRY` with `TMO_POL`. `task_tel` pushes state-change heartbeats straight into `s_outbox` alongside event frames (`DREP_TEL_HUMP`, `DREP_TEL_BARCODE` with A–Z letter bits, `DREP_TEL_FAULT`) and IF-12 obstacle profiles (`tel_post_obstacle()`).
   - A validated `car/cmd/bc_map` frame (`DREP_FT_CMD_BC_MAP = 11`, non-retained) calls `bc_task_set_map(L, R, S, U)` in `../subsystem3/subsystem3.h`.

---

### 2.2 Subsystem 3 (Buddy 3 — Line-Following & Barcode Decode)
Include `#include "../subsystem2/subsystem2.h"` and `#include "src/common/data_repr.h"`:

1. **Line-Following Steering (`IF-01`, `MBF_NAV_IN`, sample `5 ms` / steer `20 ms`)**:
   - **Hold-Window Rule (`MOTION_T_HOLD_MS = 100 ms`)**: Motion Control holds a continuous `DREP_MC_STEER` command for at most `100 ms` (`5` motion periods). `task_lf` samples at `5 ms` and posts steering every `20 ms` aligned to Motion's poll slot (`lf_link_post_slot()`). If `task_lf` stops sending commands for `> 100 ms`, the car automatically stops.
   - **Steering Sign (`D11`)**: `steer_err_q7` is in Q0.7 (`-128 .. +127`, representing `-1.0 .. +0.992`). In `task_motion.c`, **positive `steer_err_q7 > 0` steers RIGHT** (`MOTION_STEER_TARGETS=1` adjusts left/right target speeds symmetrically; open-loop fallback scales duty directly).
   ```c
   /* Continuous steering while on the line: */
   nav_input_t steer_msg = drep_nav_line_steer(steer_err_q7);
   (void)tk_snd_mbf(MBF_NAV_IN, &steer_msg, (INT)sizeof(steer_msg), TMO_POL);

   /* Junction or track-lost events (DREP_NAV_LINE_JUNCTION / DREP_NAV_LINE_TRACK_LOST): */
   nav_input_t evt_msg = drep_nav_line_event(DREP_NAV_LINE_TRACK_LOST);
   (void)tk_snd_mbf(MBF_NAV_IN, &evt_msg, (INT)sizeof(evt_msg), (TMO)NAV_MBF_SEND_TMO_MS);
   ```
2. **Barcode Commands (`IF-01`, `MBF_NAV_IN`, `5 ms` send timeout) & Distance Reading (`IF-08`)**:
   - `task_bc` decodes Code 39 letters `'A'..'Z'` using speed from `motion_get_state(&st)` (`BC_MIN_SPEED_MMPS = 20 mm/s`), reports every decoded letter via `tel_post_barcode_letter(letter, bc_cmd, mapped)`, and maps letters via `bc_task_set_map()` (`A=LEFT`, `B=RIGHT`, `C=STRAIGHT`, `D=UTURN` by default; `BC_D_CMD_CM = 60 cm` expiry inside `task_vc.c`, `BC_REPEAT_IGNORE_CM = 10 cm`).
   - When sending a mapped action (`DREP_BC_CMD_LEFT = 0`, `DREP_BC_CMD_RIGHT = 1`, `DREP_BC_CMD_STRAIGHT = 2`, `DREP_BC_CMD_UTURN = 3`):
   ```c
   nav_input_t bc_msg = drep_nav_barcode(DREP_BC_CMD_LEFT);
   (void)tk_snd_mbf(MBF_NAV_IN, &bc_msg, (INT)sizeof(bc_msg), (TMO)NAV_MBF_SEND_TMO_MS);
   ```

---

### 2.3 Subsystem 4 (Buddy 4 — IMU Processing & Safety Task)
Include `#include "../subsystem2/subsystem2.h"`:

1. **Emergency Brake (`IF-11`) & Fault Latch (`IF-06`, `EVF_SAFETY`)**:
   - When `task_safety` (Priority 1) detects `EVF_SAFETY_LIFTED` (robot lifted `> 0.10 m/s` or tilted `> 30°` for `50 ms`) or a `SAFETY_IMU_WDOG_MS` (`50 ms`) timeout on `EVF_SAFETY_IMU_ALIVE`:
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
   - When an obstacle is confirmed within `OBS_D_DETECT_CM` (`25 cm`), set `EVF_NAV_AVOID_ACTIVE` and send bypass commands (`DREP_BYPASS_STOP = 0`, `DREP_BYPASS_CONTINUE = 1`, `DREP_BYPASS_LEFT = 2`, `DREP_BYPASS_RIGHT = 3`, `DREP_BYPASS_REVERSE = 4`) via `drep_nav_avoid(type, bypass_cmd)` (where `type` is `DREP_NAV_AVOID_SLOW`, `DREP_NAV_AVOID_BYPASS`, `DREP_NAV_AVOID_RELEASE`, or `DREP_NAV_AVOID_BLOCKED`):
   ```c
   (void)tk_set_flg(EVF_NAV, EVF_NAV_AVOID_ACTIVE);

   nav_input_t slow_msg  = drep_nav_avoid(DREP_NAV_AVOID_SLOW, DREP_BYPASS_STOP);
   (void)tk_snd_mbf(MBF_NAV_IN, &slow_msg, (INT)sizeof(slow_msg), (TMO)NAV_MBF_SEND_TMO_MS);

   nav_input_t avoid_msg = drep_nav_avoid(DREP_NAV_AVOID_BYPASS, DREP_BYPASS_LEFT);
   (void)tk_snd_mbf(MBF_NAV_IN, &avoid_msg, (INT)sizeof(avoid_msg), (TMO)NAV_MBF_SEND_TMO_MS);
   ```
2. **Releasing Authority Back to Line-Following (`DREP_NAV_AVOID_RELEASE`)**:
   - Once the line is reacquired during return-to-line:
   ```c
   (void)tk_set_flg(EVF_NAV, EVF_NAV_LINE_REACQUIRED);
   (void)tk_clr_flg(EVF_NAV, ~(UINT)EVF_NAV_AVOID_ACTIVE);

   nav_input_t rel_msg = drep_nav_avoid(DREP_NAV_AVOID_RELEASE, DREP_BYPASS_STOP);
   (void)tk_snd_mbf(MBF_NAV_IN, &rel_msg, (INT)sizeof(rel_msg), (TMO)NAV_MBF_SEND_TMO_MS);
   ```

---

## 3. Pre-Integration Hardware & Build Checklist

Before running pairwise or lab-course tests after teammates merge their branches:

1. **Pull `main` and Run the Compile + Rule + Host Tests**:
   ```powershell
   git pull origin main
   cmake -DRULES_ROOT=. -P cmake/check_rules.cmake
   cmake --build build/wifi --parallel 8
   cmake --build build/test --parallel 8
   ctest --test-dir build/host --output-on-failure
   ```
   Verify **0 compiler warnings** (`-Wall -Wextra -Werror`), `check_rules: PASS`, and `14/14` host tests passing.
2. **Verify All Physical Wiring Against `TEAM.md` Section 2**:
   - [x] **Motors (`GP8`–`GP11`) & Encoders (`GP0`/`GP1`, `GP16`/`GP17`)**: Verified on 3.3 V Grove 1 & Grove 4; polarities and geometry (`ENC_CPR = 616U`, `WHEEL_CIRC_MM = 207U`, `MOTION_EFF_TRACK_MM = 114U`, `CAR_WIDTH_MM = 130U`) set in `config/pins.h` and `config/thresholds.h`.
   - [ ] **Grove 6 `GP26` Lead Taped/Capped**: Grove 6 shares `GP26` with Grove 5 (`IR_R` line sensor `ADC0`); ensure the unused lead on Grove 6 cannot short.
   - [ ] **`GP28` `VBAT` Jumper OPEN**: So `ADC2` (`GP28`) reads the line sensor (`IR_C`) instead of the battery divider.
   - [ ] **Grove 2 Unused**: Because `GP2`/`GP3` (`I2C1`) are used by the GY-511 IMU on the Maker/Qwiic port.
   - [x] **Ultrasonic (`GP4`/`GP5`, Grove 3) & Servo (`GP15`, PWM Slice 7B)**: `GP5` echo is 3.3 V-safe; servo signal is on `GP15` (never `GP12`).

---

## 4. Phase 1: Pairwise Integration Testing (`testing/integration/TEST_LOG.md`)

Run these five pairwise tests in order and record every run in `testing/integration/TEST_LOG.md` with its saved log in `testing/integration/logs/`.

| Test ID | Subsystems | Step-by-Step Procedure | Pass Criteria |
|---|---|---|---|
| **`INT-01`** | **S3 (Line) $\rightarrow$ S2 (VC + Motion)** | 1. Elevate wheels on the bench; transition VC to `NAV_LINE_TRACKING`.<br>2. Move a black tape strip under `IR_C` (`GP28`) and `IR_R` (`GP26`).<br>3. Place car on a 1 m floor line and run at `MOTION_SPEED_CRUISE_CMPS` (`5 cm/s` bring-up). | - Positive `steer_err_q7` steers right (`MOTION_STEER_TARGETS=1` adjusts left/right speed targets smoothly).<br>- Car tracks the 1 m line without false `LINE_LOST`.<br>- Removing the car from the line (`TRACK_LOST`) stops both wheels and returns VC to `READY_WAITING`. |
| **`INT-02`** | **S3 (Barcode) $\leftrightarrow$ S2 (VC + Motion)** | 1. Configure mapping via `car/cmd/bc_map` (default `A=L, B=R, C=S, D=U`) or use printable cards.<br>2. Drive over a Code 39 barcode card (`GP27`), then cross a junction within `BC_D_CMD_CM` (`60 cm`). | - S3 reads `motion_get_state()` under `MTX_MOTION` without blocking (`BC_MIN_SPEED_MMPS = 20 mm/s`).<br>- S1 telemetry receives `event_barcode` with decoded `A–Z` letter; VC transitions to `VC_SYS_NAV_TURN_LEFT` / `RIGHT` / `GO_STRAIGHT` / `U_TURN` and executes the junction manoeuvre. |
| **`INT-03`** | **S5 (Obstacle) $\rightarrow$ S2 (VC + Motion) & S3 (Line)** | 1. While driving in `NAV_LINE_TRACKING`, place a box `< 25 cm` ahead of `GP4`/`GP5`.<br>2. Observe halt, stationary `5°` sweep (`50°–130°`), and encoder-closed 4-turn bypass.<br>3. Observe `EVF_NAV_LINE_FOUND` -> `EVF_NAV_LINE_REACQUIRED` handoff. | - VC switches authority to `VC_AUTH_AVOID` (`VC_SYS_OBSTACLE_AVOIDANCE`) and ignores S3 line steering during bypass.<br>- Upon release (`DREP_NAV_AVOID_RELEASE`), VC restores `VC_AUTH_LINE` (`VC_SYS_NAV_LINE_TRACKING`). |
| **`INT-04`** | **S4 (IMU/Safety) $\rightarrow$ S2 (Brake + VC)** | 1. With wheels spinning in `NAV_LINE_TRACKING`, lift/tilt the car (`> 30°` or `> 0.10 m/s` for `50 ms`) or disconnect IMU I2C (`50 ms` watchdog).<br>2. Send remote `RESET` while wheels are spun by hand, then send `RESET` after `500 ms` stationary. | - `motion_emergency_brake()` drives `GP8`–`GP11` LOW immediately (`<= 200 ms`, NFR10).<br>- VC enters `VC_SYS_SAFETY_FAULT_LATCHED` and rejects drive commands.<br>- First `RESET` (moving) is ignored; second `RESET` (`>= 500 ms` stationary) clears `EVF_SAFETY_FAULT_LATCHED` and returns to `READY_WAITING`. |
| **`INT-05`** | **S1 (Telemetry) $\leftrightarrow$ S2 / S3 / S4 / S5** | 1. Boot Wi-Fi build (`build/wifi`), connect web dashboard (`tel_web.py`).<br>2. Send `cmd_start` (`START`), optional `car/cmd/bc_map`, drive over barcode/hump/obstacle, send `cmd_stop`. | - `START`/`STOP` change `VC` state cleanly; `car/cmd/bc_map` updates S3 map (`[TEL] bc_map ...`); heartbeat reports `0..9`; `telemetry_motion`/`sensor`, `event_barcode` (`A–Z`), `event_hump`, `event_obstacle`, and `event_fault` match car behaviour. |

---

## 5. Phase 2: Full System & Lab Course Testing (`testing/system/TEST_LOG.md`)

Flash the active Wi-Fi car build (`build/wifi/robocar.uf2`) or USB console build (`build/usb/robocar.uf2`) with `APP_TEST=OFF`.

### 5.1 Building & Flashing the Production Firmware
```powershell
& $CM --build build/wifi --parallel 8
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
Copy-Item build/wifi/robocar.uf2 "$($v.DriveLetter):\"
```

### 5.2 Speed Progression on the Lab Course (`config/thresholds.h`, `config/timing.h`)
- **Current Bring-Up Demo Speeds (`MOTION_BRINGUP_SPEEDS = 1U`, `OBS_BYPASS_SLOW_BRINGUP = 1U`)**:
  - Cruise = `5 cm/s`, Slow/Turn = `5 cm/s`, Bypass = `5 cm/s` (`OBS_BYPASS_BUDGET_MS = 60000U` so obstacle bypass + `LINE_FOUND` handoff completes reliably; IMU hump climb boost ramps gradually up to `~9.5 cm/s` on steep inclines).
- **Nominal NFR14 Speeds (`OBS_BYPASS_SLOW_BRINGUP = 0U` or `MOTION_BRINGUP_SPEEDS = 0U`)**:
  - Restoring bypass speed to `25 cm/s` (`MOTION_SPEED_BYPASS_MIN_CMPS = 15U`) re-enables the compile-time `NFR14` (`<= 5.0 s`) guards; full nominal set (`MOTION_BRINGUP_SPEEDS = 0U`) uses Cruise = `30 cm/s`, Slow = `15 cm/s`, Bypass = `25 cm/s`.

### 5.3 System Test Cases (`SYS-02`..`SYS-06`)

| ID | Lab Course Test | Requirement | Method | Steps on the Lab Course | Expected Result |
|---|---|---|---|---|---|
| **`SYS-02`** | End-to-end line-following lap | FR1, FR4–FR8, NFR1 | manual + log + video | Place car at course start; send remote `START` via MQTT; run a full lap of straights and curves; send remote `STOP` at finish | Car stays on the line for the entire lap, reports live speed/distance over MQTT (`telemetry_motion`), and stops cleanly at finish |
| **`SYS-03`** | Barcode-guided junction turns | FR9, NFR8, NFR9 | manual + log + video | Set letter map via `car/cmd/bc_map` (default `A/B/C/D`); place Code 39 barcode cards before junctions; run car along the line | Car decodes each `A–Z` barcode, sends `event_barcode` to MQTT, and executes the mapped LEFT/RIGHT/STRAIGHT/UTURN at the junction within `BC_D_CMD_CM` (`60 cm`) |
| **`SYS-04`** | Hump traversal & pitch detection | FR11, NFR11, NFR12 | manual + log + video | Drive car over the lab course hump/ramp during autonomous line tracking | IMU detects pitch `>= 3°` (`IMU_HUMP_PITCH_DEG`, calibrated `95 mm` wheelbase), Motion applies hump climb assist without losing the line, no false lift fires, and `event_hump` reports height within NFR11 (`±20%` or `±5 mm`) |
| **`SYS-05`** | Obstacle detection, scan & bypass | FR13, FR14, NFR14, NFR15 | manual + log + video | Place an obstacle box on the track line; run car toward the obstacle | Car halts at `< 25 cm`, runs `5°` sweep (`50°–130°`), sends `event_obstacle` with width and clearances, bypasses on the clearer side (`VC_AUTH_AVOID`), reacquires the line (`LINE_FOUND`), and resumes `NAV_LINE_TRACKING` |
| **`SYS-06`** | Lift/overturn fault latch & stationary reset | FR12, NFR10, IF-06, IF-11 | manual + log + video | Lift the robot (`> 30°` or `> 0.10 m/s`) off the track mid-run (end-of-run signal) or tip it over; attempt remote `RESET` while moving, then after `500 ms` stationary | Motors brake within `200 ms` (`motion_emergency_brake()`); `event_fault` sent; moving `RESET` rejected; stationary `RESET` clears `FAULT_LATCHED` and returns to `READY_WAITING` |

---

### 5.4 Post-Track Optimization Phase (`docs/EMBEDDED_OPTIMIZATIONS.md`)
Do **not** mix hardware tuning on the practice track with Tier 2 / Tier 3 code or peripheral refactors. Once baseline `SYS-02`..`SYS-06` runs pass and are logged:
1. **Apply Tier 1 (Zero Behavioral Impact) Optimizations Across Remaining Subsystems**:
   - **Buddy 1 (`OPT-06`, `OPT-07`)**: Byte-chunked `drep_bits_put` / `drep_bits_get` and branchless SWAR hex/decimal helpers; verify with `ctest -R tel` (`S1-H01..H05`).
   - **Buddy 3 (`OPT-05`)**: `MULS`-based SWAR 9-bit mirror, SWAR popcount, and halved `uint16_t` LUT in `bc_decode.c`; verify with `ctest -R bc_decode` (`S3-U01`, `S3-U02`).
   - **Buddy 4 (`OPT-13`, `OPT-19`)**: 32-bit `atan2_half_deg` / lift dot-product and unified `isqrt` + `s_imu`/`s_safety` context structs; verify with `ctest -R imu_lift` (`S4-U21`) and a quick `S4-U23` re-check.
   - **Buddy 5 (`OPT-11`, `OPT-12`, `OPT-19`)**: 32-bit ultrasonic range division, 32-bit polar projection + 10-entry quadrant trig table in `task_obs_scan.c`, and `s_scan`/`s_avoid` context structs; verify with `S5-U04`, `S5-U06`..`S5-U16`.
2. **Apply Tier 2 (ISR Latency & Jitter Reduction) If Needed**:
   - **`OPT-08` & `OPT-17`**: Inline Thumb-1 `run_slot()` wrapper in `src/rtos/port/port_irq.c` and SRAM placement (`__not_in_flash_func`) for `echo_isr` (`GP5`) and `bc_gpio_isr` (`GP27`) if CYW43 WiFi load causes missed barcode edges or ultrasonic spread.
3. **Apply Tier 3 (RP2040 Hardware Offloads) With Team Agreement**:
   - **`OPT-14` (`interp0`/`interp1` hardware clamp/lerp)**, **`OPT-15` (DMA round-robin ADC for IR)**, **`OPT-16` (`PIO0 SM2`/`SM3` hardware echo & barcode capture)**, and **`OPT-20` (`--gc-sections` linker dead-code stripping)**, updating `TEAM.md` peripheral ownership before merging.

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

