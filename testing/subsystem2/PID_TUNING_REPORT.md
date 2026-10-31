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

| Symbol | Initial Value | Units | Role |
|---|---|---|---|
| `MOTION_PID_KP_PM` | `12` | permille duty / (count / 20 ms) | Per-wheel proportional velocity gain |
| `MOTION_PID_KI_X100` | `25` (`0.25`) | (permille duty × 100) / accumulated count | Per-wheel integral gain (eliminates steady-state speed error) |
| `MOTION_PID_KD_PM` | `4` | permille duty / (count/period change) | Per-wheel derivative damping gain |
| `MOTION_SYNC_KP_PM` | `15` | permille duty / `(dl - dr)` count diff | Cross-coupled wheel synchronization gain for straight-line tracking |
| `MOTION_INTEGRAL_MAX` | `400` | counts | Anti-windup cap on per-wheel integral accumulator |
| `MOTOR_DUTY_MAX_PCT` | `80U` | `%` (`800` permille) | Hard PWM duty ceiling (`config/thresholds.h`) |

---

## 3. On-Vehicle Tuning Procedure

### Step 1: Polarity & Geometry Prerequisites (`S2-U03`, `S2-U07`, `S2-U10`, `S2-U11`)
1. With wheels off the ground, verify forward command spins both wheels forward; set `MOTOR_L_DIR_INVERT` and `MOTOR_R_DIR_INVERT` in `config/pins.h` if either wheel is reversed (`S2-U03`).
2. Spin each wheel forward by hand and verify `enc_get_count()` increases on the spun wheel; set `ENC_L_DIR_INVERT` and `ENC_R_DIR_INVERT` in `config/pins.h` if needed (`S2-U07`).
3. Measure `ENC_CPR` over 10 exact wheel revolutions (`S2-U10`) and `WHEEL_CIRC_MM` over 10 turns along a ruler (`S2-U11`), and set both in `config/thresholds.h`.

### Step 2: Velocity Loop Tuning (`MOTION_PID_KP_PM`, `MOTION_PID_KI_X100`, `MOTION_PID_KD_PM`)
1. Set `MOTION_PID_KI_X100 = 0` and `MOTION_PID_KD_PM = 0`. Increase `MOTION_PID_KP_PM` from `8` until the wheel speed tracks a step command (`20 cm/s` bring-up cruise) with < 15% overshoot.
2. Increase `MOTION_PID_KI_X100` in steps of `5` until steady-state speed error over a 1 m run is within ±5% (`S2-U17`).
3. Add `MOTION_PID_KD_PM` (`2..6`) to damp any start/stop oscillation.

### Step 3: Straight-Line Synchronization Tuning (`MOTION_SYNC_KP_PM`, `S2-U19`)
1. Place the car on a flat floor along a 2.0 m straight reference line.
2. Command a 2.0 m forward run at cruise speed (`moveForward(200)`) and measure lateral drift at the 2.0 m mark.
3. Adjust `MOTION_SYNC_KP_PM` (`10..25`) to minimize systematic heading drift caused by motor/gearbox asymmetry.

---

## 4. Tuning Run Log (`S2-U17`, `S2-U19`)

Record each tuning iteration below (never overwrite completed hardware run rows):

| Run | Commit | `KP_PM` | `KI_X100` | `KD_PM` | `SYNC_KP_PM` | Commanded Speed (cm/s) | Measured Avg Speed (cm/s) | 2.0 m Lateral Drift (cm) | Notes |
|---|---|---|---|---|---|---|---|---|---|
| Baseline (pre-board) | `2290ff7` | `12` | `25` | `4` | `15` | `20` (bring-up) | *pending board run* | *pending board run* | Initial fixed-point gains compiled and verified in `S2-U12`..`S2-U16` |
| Run 1 | | | | | | | | | |
| Run 2 | | | | | | | | | |
| Run 3 | | | | | | | | | |
| Run 4 | | | | | | | | | |
| Run 5 | | | | | | | | | |

