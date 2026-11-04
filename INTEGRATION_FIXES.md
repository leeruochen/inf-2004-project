# Subsystem 2 & 5 Integration Fixes: Fault Latch, Bypass Speed & NFR15 (`INTEGRATION_FIXES.md`)

**Date:** 2026-11-04  
**Author:** Buddy 2 (Ruo Chen, `2500530`)  
**Context:** Resolution of blockers and requests raised by Buddy 5 (Chay Han, `2500596`) in `testing/subsystem5/logs/2026-11-04_s5_bypass_runs.txt` and `testing/subsystem5/TEST_LOG.md`.

---

## Executive Summary

| Issue | Problem / Request | Root Cause | Proposed Action |
|---|---|---|---|
| **1. S2-U25 Failure & `FAULT_LATCHED` Deadlock** | S2-U25 fails in combined test builds; car is blocked on boot with `[S5] avoid ABORT: safety fault latched` until an explicit `REMOTE_RESET`. | 1. `task_vc.c` only clears its internal `s_sys_state` on `RESET` if the flag bit `FAULT_LATCHED` is currently 1; if cleared out-of-band, VC stays permanently deadlocked in fault state.<br>2. `subsystem2_test.c` (S2-U24) directly clears `FAULT_LATCHED` via `tk_clr_flg`, desyncing VC.<br>3. S2-U25 uses `EVF_SAFETY_STATUS_NO_ECHO` as a test bit, which S5's `task_obs_scan` actively clears in the background. | Patch `task_vc.c`, `subsystem2_test.c` S2-U24/S2-U25, and add test cleanup. |
| **2. `MOTION_SPEED_BYPASS_CMPS = 25 cm/s`** | Chay Han requested raising bypass speed to 25 cm/s to close NFR14 (3.0 s target). | Currently set to 15 cm/s (`BRINGUP`) / 20 cm/s (`NOMINAL`). | Update `config/thresholds.h` to 25 cm/s for both nominal and bring-up bypass. |
| **3. NFR15 Rewording (Obstacle Width Error)** | ±10% width estimation error cannot be met for small (e.g. 50 mm) boxes due to sensor angular resolution (10° fine scan = ~27.6 mm/slot). | Geometric quantization limit: 1-slot ambiguity is ±27.6 mm at 158 mm range. | Add Delta D12 to `ARCHITECTURE.md` and update `design.md`: "within ±10 % or ±1 bearing slot, whichever is larger". |

---

## 1. Issue 1: `S2-U25` Failure & `FAULT_LATCHED` Deadlock

### 1.1 Root Cause Analysis

In Chay Han's test run (`testing/subsystem5/logs/2026-11-04_s5_bypass_runs.txt`):
```text
[TEST] S2-U24 PASS  VC commands stop/brake when EVF_SAFETY_FAULT_LATCHED is set
[TEST] S2-U25 FAIL  VC clears FAULT_LATCHED only after SAFETY_T_STILL_MS stationary
...
[S5] avoid ABORT in SLOW: safety fault latched
```

Three interrelated defects cause this:
1. **VC Internal State Desynchronization (`app_program/subsystem2/task_vc.c`)**:
   In `handle_remote_msg()`:
   ```c
   if (type == DREP_NAV_REMOTE_RESET)
   {
       if (fault_latched) { ... }
       return;
   }
   ```
   If `s_sys_state == VC_SYS_SAFETY_FAULT_LATCHED`, but `fault_latched` (`is_fault_latched()`) is `false` (for example, if another task cleared the flag or it was cleared directly), `handle_remote_msg()` ignores the `RESET`! As a result, `s_sys_state` never transitions back to `VC_SYS_READY_WAITING`, and every subsequent navigation command triggers line 369:
   ```c
   if (fault_latched) {
       set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
       send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
   }
   ```
2. **Improper Test Cleanup in `test_s2_u24_vc_stops_on_fault_latched()` (`subsystem2_test.c`)**:
   At line 392 of `subsystem2_test.c`:
   ```c
   (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
   drain_motion_cmd_mbf();
   ```
   S2-U24 sets `EVF_SAFETY_FAULT_LATCHED`, feeds a line message to step VC, and verifies VC entered `VC_SYS_SAFETY_FAULT_LATCHED`. But then it directly clears `EVF_SAFETY_FAULT_LATCHED` via `tk_clr_flg()` without stepping VC through a proper `RESET`! This leaves `s_sys_state == VC_SYS_SAFETY_FAULT_LATCHED` while the bit in `EVF_SAFETY` is 0.
3. **Cross-Subsystem Flag Collision in `test_s2_u25_vc_reset_stationary_only()` (`subsystem2_test.c`)**:
   At line 422:
   ```c
   (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_STATUS_NO_ECHO);
   ```
   S2-U25 sets `EVF_SAFETY_STATUS_NO_ECHO` to verify that a `RESET` clears *only* `FAULT_LATCHED` and preserves other status bits. However, in the full multi-task firmware, Subsystem 5's `task_obs_scan.c` runs concurrently at Priority 6 and is the designated owner of `EVF_SAFETY_STATUS_NO_ECHO`. When `task_obs_scan` detects an echo from an obstacle (which was present during Chay Han's bench tests), it actively clears `EVF_SAFETY_STATUS_NO_ECHO` via `tk_clr_flg(EVF_SAFETY, ~(UINT)EVF_SAFETY_STATUS_NO_ECHO)`!
   When S2-U25 later asserts:
   ```c
   stationary_cleared = ((rflg2.flgptn & EVF_SAFETY_FAULT_LATCHED) == 0U)
                        && ((rflg2.flgptn & EVF_SAFETY_STATUS_NO_ECHO) != 0U)
   ```
   the test fails because `task_obs_scan` legitimately cleared `EVF_SAFETY_STATUS_NO_ECHO`.

### 1.2 Required Code Modifications

#### A. In `app_program/subsystem2/task_vc.c`
In `handle_remote_msg()` (around line 145), update the condition so `RESET` always clears the fault state whether the eventflag bit is still set or only `s_sys_state` is latched:
```c
    if (type == DREP_NAV_REMOTE_RESET)
    {
        if (fault_latched || (s_sys_state == VC_SYS_SAFETY_FAULT_LATCHED))
        {
            if (is_stationary_for_reset(now_ms))
            {
                /* Clear ONLY the FAULT_LATCHED bit (ARCHITECTURE.md D9). */
                (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
                s_authority = VC_AUTH_LINE;
                s_bc_pending = false;
                set_system_state(VC_SYS_READY_WAITING, now_ms);
                send_motion_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
            }
            else
            {
                set_system_state(VC_SYS_SAFETY_FAULT_LATCHED, now_ms);
                send_motion_cmd(drep_mc_pack(DREP_MC_BRAKE, 0, 0));
            }
        }
        return;
    }
```

#### B. In `app_program/subsystem2/subsystem2_test.c`
1. **In `test_s2_u24_vc_stops_on_fault_latched()` (line 392)**:
   Instead of abruptly zeroing the bit and leaving VC desynchronized, cleanly reset VC:
   ```c
   /* Reset VC state cleanly after verifying fault behaviour */
   (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
   {
       nav_input_t const rst = drep_nav_remote(DREP_NAV_REMOTE_RESET);
       (void)tk_snd_mbf(MBF_NAV_IN, &rst, (INT)sizeof(rst), TMO_POL);
       vc_drain_and_step(app_period_now_ms() + SAFETY_T_STILL_MS + 20U);
   }
   drain_motion_cmd_mbf();
   ```

2. **In `test_s2_u25_vc_reset_stationary_only()` (around line 422)**:
   Replace `EVF_SAFETY_STATUS_NO_ECHO` with a bit owned by Subsystem 2 (`EVF_SAFETY_FAULT_ENCODER`, bit 4):
   ```c
   /* Set FAULT_LATCHED plus FAULT_ENCODER (owned by S2) to verify only
    * FAULT_LATCHED is cleared on reset. */
   (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_FAULT_ENCODER);
   ...
   stationary_cleared = ((rflg2.flgptn & EVF_SAFETY_FAULT_LATCHED) == 0U)
                        && ((rflg2.flgptn & EVF_SAFETY_FAULT_ENCODER) != 0U)
                        && (vc_get_system_state() == VC_SYS_READY_WAITING);

   (void)tk_clr_flg(EVF_SAFETY, (UINT)~(EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_FAULT_ENCODER));
   ```

3. **At the end of `subsystem2_test()` (around line 850)**:
   Add a safety cleanup block to guarantee that when Subsystem 2's test finishes, the car is left in `VC_SYS_READY_WAITING` with zero fault bits active:
   ```c
   /* Ensure clean state for subsequent subsystem tests */
   (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
   vc_init();
   vc_complete_selftest(true);
   ```

---

## 2. Issue 2: `MOTION_SPEED_BYPASS_CMPS = 25 cm/s`

### 2.1 Context & Performance Analysis
- **Requirement NFR14**: Complete entire avoidance maneuver (leave line $\rightarrow$ pass obstacle $\rightarrow$ return to line) in $\le 3.0\text{ s}$ (grade-A) and $\le 5.0\text{ s}$ (hard limit).
- **Current Speeds in `config/thresholds.h`**:
  - `MOTION_SPEED_BYPASS_NOMINAL_CMPS = 20U`
  - `MOTION_SPEED_BYPASS_BRINGUP_CMPS = 15U`
  - When `MOTION_BRINGUP_SPEEDS = 1U`, bypass runs at only `15 cm/s`, yielding ~4056 ms before `LEG_BACK`.
- **Impact of 25 cm/s**:
  - `task_obs_avoid.c` calculates leg travel times as `(distance_mm * 100U) / MOTION_SPEED_BYPASS_CMPS`.
  - Increasing from 15 cm/s to 25 cm/s cuts straight leg traversal time by **40%**.
  - `task_vc.c` executes bypass 45° turns (`DREP_BYPASS_LEFT`, `DREP_BYPASS_RIGHT`) at `MOTION_SPEED_BYPASS_CMPS` (`task_vc.c:239-246`). A higher speed reduces turn duration from 550 ms down to ~350 ms, saving ~600 ms across the three turns.
  - Hard speed limit: `MOTION_SPEED_MAX_CMPS` is `50 cm/s`, and `MOTION_SPEED_BYPASS_MIN_CMPS` is `15 cm/s`. 25 cm/s is within all threshold guards (`config/thresholds.h:202-219`).

### 2.2 Required Modification in `config/thresholds.h`
Around lines 128 and 134:
```c
/** Nominal bypass speed. Closed to 25 cm/s on 2026-11-04 for NFR14. */
#define MOTION_SPEED_BYPASS_NOMINAL_CMPS    (25U)
...
/** Bring-up bypass speed. Aligned to 25 cm/s for NFR14. */
#define MOTION_SPEED_BYPASS_BRINGUP_CMPS    (25U)
```

---

## 3. Issue 3: Rewording NFR15 & Architecture Delta D12

### 3.1 Justification
- **The Problem**: Ultrasonic sensor `HC-SR04` fine scan steps every `OBS_FINE_STEP_DEG = 10^\circ`. At a distance $d = 158\text{ mm}$, one angular step subtends:
  $$\Delta w = d \cdot \frac{10 \cdot \pi}{180} \approx 27.6\text{ mm}$$
  For a 50 mm wide obstacle box, an unavoidable $\pm 1$-slot edge detection ambiguity causes an error of $\pm 27.6\text{ mm}$ ($\pm 55\%$), which makes the current NFR15 requirement of $\le \pm 10\%$ geometrically impossible for obstacles narrower than $\approx 276\text{ mm}$.
- **Precedent**: NFR11 (hump height detection by Buddy 4) already uses the exact same dual-threshold formulation:
  > *"within ±20% (or ±0.5 cm, whichever is larger)"*

### 3.2 Required Document Updates

#### A. In `ARCHITECTURE.md` (Add Delta D12)
Under Section *"Design deltas against design.md"*:
```markdown
| D12 | NFR15: obstacle width estimation error "within 10%" | Obstacle width estimation shall be within ±10% or ±1 fine-scan bearing slot (OBS_FINE_STEP_DEG = 10°, i.e. ±d·π/18 at range d), whichever is larger | Geometric sensor limit: with 10° servo steps, one bearing slot subtends ~27.6 mm at 158 mm range. For obstacles under 276 mm, ±10% is below the discrete angular resolution of the beam sweep. Matches NFR11's dual-threshold pattern. |
```

#### B. In `design.md`
1. Update Table row for NFR15 (around line 116):
   > **NFR15**: Ultrasonic measurements shall be filtered (median of 3 pings per bearing) before obstacle profiling, keeping obstacle width estimation error within **±10% (or ±1 bearing slot, whichever is larger)**.
2. Fix typo in Section 3.2 (line 312):
   Change:
   > *"median of three pings, which rejects outlier echoes (NFR13)."*
   To:
   > *"median of three pings, which rejects outlier echoes (NFR15)."*

---

## 4. Step-by-Step Implementation Checklist for the Next Device

1. [ ] **Pull Latest `main`**:
   ```powershell
   git pull origin main
   ```
2. [ ] **Apply Code Fixes**:
   - `app_program/subsystem2/task_vc.c` (handle `RESET` when `s_sys_state == VC_SYS_SAFETY_FAULT_LATCHED`).
   - `app_program/subsystem2/subsystem2_test.c` (proper S2-U24 reset, S2-U25 flag change to `EVF_SAFETY_FAULT_ENCODER`, and post-test clean state).
   - `config/thresholds.h` (set `MOTION_SPEED_BYPASS_*_CMPS` to `25U`).
   - `ARCHITECTURE.md` (add D12).
   - `design.md` (update NFR15 and fix typo in line 312).
3. [ ] **Verify Builds & Rule Checks**:
   ```powershell
   cmake -P cmake/check_rules.cmake
   cmake --build build/test --parallel 8
   cmake --build build/s0b --parallel 8
   ```
4. [ ] **Commit & Push to `main`**:
   ```powershell
   git add app_program/subsystem2/task_vc.c app_program/subsystem2/subsystem2_test.c config/thresholds.h ARCHITECTURE.md design.md INTEGRATION_FIXES.md
   git commit -m "fix(subsystem2): resolve fault latch reset deadlock, set bypass speed to 25 cm/s, and reword NFR15"
   git push origin main
   ```
5. [ ] **Notify Buddy 5 (Chay Han)**:
   - Let him know `main` is updated and `S2-U25` / `FAULT_LATCHED` is fixed so he can flash and re-run his bypass tests without manual `REMOTE_RESET` workarounds.
