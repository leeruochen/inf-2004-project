# Subsystem 2 — PID Speed Control & Straight-Line Tuning Report

**Subsystem:** Subsystem 2 (Motion Control and Vehicle Controller)  
**Owner:** Ruo Chen (Buddy 2, Student ID: `2500530`)  
**Source Files:** `app_program/subsystem2/task_motion.c`, `app_program/subsystem2/drv_motor.c`, `app_program/subsystem2/drv_encoder.c`  
**Config Headers:** `config/thresholds.h`, `config/timing.h`, `config/pins.h`

---

## 1. Control Architecture Overview

Motion Control runs as a periodic micro T-Kernel 3.0 task at **priority 3** (`TASK_PRI_MOTION`) on a drift-free **20 ms grid** (`MOTION_PERIOD_MS = 20U`, enforced via `app_period_init()` and `app_period_wait()`).

Each 20 ms control cycle performs:

1. **Non-blocking Quadrature Encoder Sampling (`IF-10`)**:
   - Reads the 32-bit signed cumulative count (`enc_get_count()`) from PIO0 SM0 (right wheel, `GP0`/`GP1`) and PIO0 SM1 (left wheel, `GP16`/`GP17`).
   - Computes wrap-safe two's-complement per-period deltas:
     ```c
     dl = (int32_t)((uint32_t)cnt[LEFT]  - (uint32_t)prev_cnt[LEFT]);
     dr = (int32_t)((uint32_t)cnt[RIGHT] - (uint32_t)prev_cnt[RIGHT]);
     ```
2. **Command Polling & Hold Window (`IF-02`, `MOTION_T_HOLD_MS`)**:
   - Polls `MBF_MOTION_CMD` non-blockingly (`TMO_POL`), validates every message with `drep_mc_valid()`, and clamps the speed setpoint to `MOTION_SPEED_MAX_CMPS` (50 cm/s, NFR2).
   - Holds the last valid setpoint for `MOTION_T_HOLD_MS` (100 ms = 5 periods); if no new command arrives within 100 ms, transitions to `DREP_MC_STOP` and brakes both motors.
3. **Feedforward + Per-Wheel Velocity PID + Cross-Coupled Straight-Line Correction**:
   - **Feedforward term (`base_pm`)**: Maps the speed setpoint (`0..50 cm/s`) linearly to permille duty (`0..MOTOR_DUTY_MAX_PCT * 10`), providing immediate response before integral accumulation.
   - **Per-wheel velocity PID (`pid_step_wheel`)**: When encoder geometry (`ENC_CPR` and `WHEEL_CIRC_MM`) is configured (`MOTION_HAS_GEOMETRY == 1`), computes target counts per 20 ms period:
     ```text
     target_counts_per_period = (setpoint_cmps * 10 * ENC_QUAD_MULT * ENC_CPR * 20)
                                / (WHEEL_CIRC_MM * 1000)
     ```
     and applies discrete PID with anti-windup clamping (`±MOTION_INTEGRAL_MAX`):
     ```text
     err       = target_counts - measured_delta
     integral  = clamp(integral + err, -MOTION_INTEGRAL_MAX, +MOTION_INTEGRAL_MAX)
     deriv     = err - prev_err
     u_pid_pm  = Kp * err + (Ki_x100 * integral) / 100 + Kd * deriv
     ```
   - **Cross-coupled straight-line correction (`MOTION_SYNC_KP_PM`, FR5)**:
     During straight-line driving (`DREP_MC_FWD`, `DREP_MC_BACK`, or `DREP_MC_STEER` with `arg == 0`), any encoder count mismatch between the left and right wheels (`sync_err = dl - dr`) applies a symmetric differential correction:
     ```text
     sync_corr_pm = MOTION_SYNC_KP_PM * (dl - dr)
     duty_left   -= sync_corr_pm
     duty_right  += sync_corr_pm
     ```
4. **Hardware PWM Actuation**:
   - Clamps total duty to `±(MOTOR_DUTY_MAX_PCT * 10)` permille (`±800` permille = 80% default cap) and drives PWM slice 4 (`GP8`/`GP9`, left motor) and slice 5 (`GP10`/`GP11`, right motor) at `MOTOR_PWM_FREQ_HZ = 20 kHz`.

---

## 2. Gain Summary (`app_program/subsystem2/task_motion.c`)

| Symbol | Initial Value | Final Tuned Value | Units | Role |
|---|---|---|---|---|
| `MOTION_PID_KP_PM` | `12` | **`3`** | permille duty / (count / 20 ms) | Per-wheel proportional velocity gain |
| `MOTION_PID_KI_X100` | `25` (`0.25`) | **`10` (`0.10`)** | (permille duty × 100) / accumulated count | Per-wheel integral gain (eliminates steady-state speed error) |
| `MOTION_PID_KD_PM` | `4` | **`1`** | permille duty / (count/period change) | Per-wheel derivative damping gain |
| `MOTION_SYNC_KP_PM` | `15` (per-period) | **`2` (cumulative)** | permille duty / `(total_dl - total_dr)` count diff | Cumulative heading synchronization gain for straight-line tracking |
| `MOTION_SYNC_KD_PM` | `0` | **`2` (per-period)** | permille duty / `(dl - dr)` rate diff | Per-period differential damping to prevent left-right yaw oscillation |
| `MOTION_SYNC_MAX_PM` | *(unclamped)* | **`120`** | permille (`±12.0 %` duty) | Clamp on cross-coupled steering correction so neither wheel stalls |
| `MOTION_INTEGRAL_MAX` | `400` | **`600`** | counts | Anti-windup cap on per-wheel integral accumulator |
| `MOTOR_DUTY_MAX_PCT` | `80U` | **`95U`** | `%` (`950` permille) | Hard PWM duty ceiling (`config/thresholds.h`) |

---

## 3. On-Vehicle Tuning Procedure & Key Engineering Findings

### Step 1: Polarity & Geometry Prerequisites (`S2-U03`, `S2-U07`, `S2-U10`, `S2-U11`)
1. With wheels off the ground (`testing/subsystem2/logs/2026-10-31_s2_phase1_calib.txt`), verified motor and encoder polarities:
   - `MOTOR_L_DIR_INVERT = 1U`, `MOTOR_R_DIR_INVERT = 0U`
   - `ENC_L_DIR_INVERT = 0U`, `ENC_R_DIR_INVERT = 1U`
2. Measured physical wheel diameter = `66 mm` (`WHEEL_CIRC_MM = 207U`, `S2-U11`).
3. Calibrated quadrature encoder resolution from floor runs (`S2-U10` / `S2-U18`): `ENC_CPR = 616U` (`2,464` quadrature counts per wheel revolution = `119.03` counts/cm).

### Step 2: Why Initial Gains Stuttered & How Tuning Solved It (`S2-U17`, `S2-U19`)
1. **Root Cause of Run 1 Stuttering (`floor_fwd100_run1.txt`):**
   - At `2,464` quadrature counts/rev (`11.9` counts/mm), a `20 cm/s` cruise speed produces `~48` counts per `20 ms` period.
   - With the initial pre-board gains (`KP_PM = 12`, `KD_PM = 4`, `SYNC_KP_PM = 15` on per-period `dl - dr`), a single encoder count difference between left and right wheels produced a `31 permille` (`3.1 %`) PWM swing, causing a 25 Hz left-right yaw stutter (`dL = 7320, dR = 7926, diff = -606 counts`).
2. **Velocity & Cumulative Heading Sync Tuning (`floor_fwd100_run2.txt` -> `floor_run3.txt`):**
   - Scaled per-wheel velocity gains to match the high-resolution `616 CPR` encoder: `MOTION_PID_KP_PM = 3`, `MOTION_PID_KI_X100 = 10`, `MOTION_PID_KD_PM = 1`.
   - Replaced unclamped per-period `dl - dr` sync with a **clamped PD heading synchronizer** on cumulative wheel travel (`sync_err = total_dl - total_dr`, `MOTION_SYNC_KP_PM = 2`, `MOTION_SYNC_KD_PM = 2`, clamped to `±MOTION_SYNC_MAX_PM = ±120 permille`).
   - Result in Floor Run 3 (`floor_run3.txt`): smooth, straight trajectory over `101 cm` with `dL = 12025, dR = 12018` (**`diff = +7 counts = 0.58 mm`** total wheel travel discrepancy!).

---

## 4. Tuning Run Log (`S2-U17`, `S2-U19`)

Record each tuning iteration below (never overwrite completed hardware run rows):

| Run | Commit | `KP_PM` | `KI_X100` | `KD_PM` | `SYNC_KP_PM` | Commanded Speed (cm/s) | Measured Avg Speed (cm/s) | Lateral Drift / L-R Count Diff | Notes |
|---|---|---|---|---|---|---|---|---|---|
| Baseline (pre-board) | `2290ff7` | `12` | `25` | `4` | `15` | `20` (bring-up) | *pending board run* | *pending board run* | Initial fixed-point gains compiled and verified in `S2-U12`..`S2-U16` |
| Run 1 (on-board bench) | `buddy2/motion` | `12` | `25` | `4` | `15` | `30` (100 ms step in `S2-U16`) | `30.0` (published), `encL=+460`, `encR=-547` counts/100 ms | *bench (wheels off ground)* | 20 ms period verified (`S2-U12`: 500 releases in 10 s, 0 missed); `ENC_L_DIR_INVERT=0`, `ENC_R_DIR_INVERT=1` identified from `S2-U16` log (`testing/subsystem2/logs/2026-10-31_s2_unit_all.txt`) |
| Run 2 (Floor Run 1) | `e794f8e` | `12` | `25` | `4` | `15` (per-period) | `25` (`moveForward(100)`) | `18.7` rep (`12.5` actual over `67 cm` in `5.375 s`) | Visible left-right stutter (`dL=7320, dR=7926, df=-606`) | High `KP`/`SYNC_KP` caused 25 Hz yaw oscillation on 2464-count/rev encoders (`testing/subsystem2/logs/floor_fwd100_run1.txt`) |
| Run 3 (Floor Run 2) | `e794f8e` | `3` | `10` | `1` | `2` (cumul) + `KD=2` | `25` (`moveForward(100)`) | `21.7` rep (`20.7` actual over `95 cm` in `4.598 s`, `+4.8%` err) | Smooth straight line (`< 2 cm` drift, `dL=11454, dR=11448, df=+6`) | Stuttering completely eliminated; updated `ENC_CPR` from `585` to `616` (`testing/subsystem2/logs/floor_fwd100_run2.txt`) |
| Run 4 (Floor Run 3) | `e794f8e` | `3` | `10` | `1` | `2` (cumul) + `KD=2` | `25` (`moveForward(100)`) | `21.1` rep (`21.4` actual over `101 cm` in `4.724 s`, `-1.4%` err) | Dead-straight (`< 1 cm` drift, `dL=12025, dR=12018, df=+7` counts = `0.58 mm`) | Final cruise PID & `ENC_CPR=616U` verified: `101 cm` actual vs `100 cm` target (`+1.0%` distance error, `testing/subsystem2/logs/floor_run3.txt`) |
| Run 5 (Floor Run 4 — 50 cm/s NFR2) | `e794f8e` | `3` | `10` | `1` | `2` (cumul) + `KD=2` | `50` (`MOTION_SPEED_MAX_CMPS`) | `56.7` rep (`58.4` actual over `103 cm` in `1.763 s`, `pk=51.1 cm/s`) | Straight at top speed (`dL=12418, dR=12374, df=+44` counts = `0.35%`) | Verified stable tracking and zero missed encoder counts at 50 cm/s top speed (`testing/subsystem2/logs/floor_run4_topspeed.txt`) |

