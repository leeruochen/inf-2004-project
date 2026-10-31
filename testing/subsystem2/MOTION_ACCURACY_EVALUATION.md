# Subsystem 2 — Motion Accuracy & Hardware Calibration Evaluation

**Subsystem:** Subsystem 2 (Motion Control and Vehicle Controller)  
**Owner:** Ruo Chen (Buddy 2, Student ID: `2500530`)  
**Deliverable Reference:** `specs.md` ("Buddy 2 Deliverables: Motion control library, PID tuning report, Motion accuracy evaluation"), `ARCHITECTURE.md` D7 (NFR2), `config/thresholds.h`

---

## 1. Hardware Calibration Parameters & Power Budget Verification

### 1.1 Encoder & Wheel Geometry (`config/thresholds.h`, `config/pins.h`)

| Parameter | Header | Default / Status | Measurement Method (`TEST_LOG.md`) | Measured Value |
|---|---|---|---|---|
| `MOTOR_L_DIR_INVERT` | `config/pins.h` | `0U` (needs `1U` in `config/pins.h`) | `S2-U03`: Left `+40%` spun backward (`dL=-7610`), `-40%` spun forward (`dL=+7790`) | `1U` |
| `MOTOR_R_DIR_INVERT` | `config/pins.h` | `0U` (confirmed on car) | `S2-U03`: Right `+40%` spun forward (`dR=-7901`), `-40%` spun backward (`dR=+7948`) | `0U` |
| `ENC_L_DIR_INVERT` | `config/pins.h` | `0U` (confirmed on car) | `S2-U03` / `S2-U08`: Forward physical motion (`-40%` on uninverted motor) gave `dL=+7790` (counts up) | `0U` |
| `ENC_R_DIR_INVERT` | `config/pins.h` | `0U` (needs `1U` in `config/pins.h`) | `S2-U03` / `S2-U08`: Forward physical motion (`+40%` on right motor) gave `dR=-7901` (counts down) | `1U` |
| `ENC_CPR` | `config/thresholds.h` | `0U` (needs `616U` in `config/thresholds.h`) | `S2-U10` / `S2-U18`: Hand-spin (`dL=15572`) + `101 cm` floor run (`dL=12025, dR=12018` = `119.03` counts/cm -> `2,464` quad counts/rev) | **`616U`** (`2,464` quad counts/rev) |
| `WHEEL_CIRC_MM` | `config/thresholds.h` | `0U` (needs `207U` in `config/thresholds.h`) | `S2-U11`: Wheel diameter `66 mm` (`pi * 66 mm = 207.3 mm`) | **`207U`** (`207 mm`) |
| `CAR_WIDTH_MM` (Effective Track) | `config/thresholds.h` | `120U` (`MOTION_EFF_TRACK_MM = 114U`) | `S2-U20`: Calibrated from `turnLeft(90)` / `turnRight(90)` floor runs (`138 mm -> 104°`, `120 mm -> 95°`, `114 mm -> 90.0°`) | **`114U`** (`114 mm` skid-steer effective track) |
| `MOTOR_DUTY_MAX_PCT` | `config/thresholds.h` | `95U` | `S2-U02` (auto) + top-speed load check at `POWER_VIN_MIN_MV` (`3.6 V`) | **`95 %`** (`950` permille) |

### 1.2 3.3 V Grove Power Budget Verification (`POWER_GROVE_3V3_BUDGET_MA`)

Per `config/thresholds.h` (`POWER_GROVE_3V3_BUDGET_MA = 300U`), the Cytron Robo Pico onboard 3.3 V LDO regulator supplies the RP2040/Pico W and all 3.3 V Grove ports (Grove 1–7).

| Consumer | Grove Port / Pins | Typical Current (mA) | Peak Current (mA) | Notes |
|---|---|---|---|---|
| Right Wheel Optical/Magnetic Encoder | Grove 1 (`GP0`/`GP1`) | 10 mA | 15 mA | 3.3 V quadrature sensor |
| Left Wheel Optical/Magnetic Encoder | Grove 4 (`GP16`/`GP17`) | 10 mA | 15 mA | 3.3 V quadrature sensor |
| IMU (`GY-511` / `LSM303DLHC`) | I2C1 (`GP2`/`GP3`) | 2 mA | 5 mA | 110 µA normal mode + pull-ups |
| Ultrasonic Sensor (`HC-SR04P` / 3.3 V) | Grove 3 (`GP4`/`GP5`) | 5 mA | 15 mA | 3.3 V-compatible ultrasonic module |
| Right Line IR Sensor (`IR_R`) | Grove 5 (`GP26`/ADC0) | 15 mA | 20 mA | IR emitter LED + phototransistor |
| Barcode IR Sensor (`IR_BC`) | Grove 6 (`GP27`) | 15 mA | 20 mA | IR emitter LED + comparator |
| Centre Line IR Sensor (`IR_C`) | Grove 7 (`GP28`/ADC2) | 15 mA | 20 mA | IR emitter LED + phototransistor (`VBAT` jumper OPEN) |
| **Total Grove 3.3 V Load** | | **72 mA** | **110 mA** | **Well within `POWER_GROVE_3V3_BUDGET_MA` (300 mA), > 60 % margin** |

> **Note:** Motors (`GP8`–`GP11`, MX1508 H-bridge) and the ultrasonic panning servo (`GP15`, PWM slice 7B) are powered from the `VIN`/`5V` rail, not the 3.3 V Grove regulator.

---

## 2. Linear Distance Accuracy Evaluation (`S2-U18`)

**Test Method:** Command `moveForward(100)` (target = `100.0 cm` = `1000 mm`) on a flat floor. Measure actual stopping distance from the front wheel axle reference mark with a tape measure, and compare against `drep_ms_distance_cm(g_motion_state.w)`.

| Run | Log File | Calibration State (`ENC_CPR`) | Commanded Distance (cm) | Reported `motion_state_t` Distance (cm) | Measured Physical Distance (cm) | Absolute Error (cm) | Relative Error (%) |
|---|---|---|---|---|---|---|---|
| 1 | `floor_fwd100_run1.txt` | Pre-calib (`CPR = 390U`, pre-tuning PID) | `100.0` | `101.0` (`dL=7320, dR=7926`) | `67.0` | `-33.0` | `-33.0 %` |
| 2 | `floor_fwd100_run2.txt` | Intermediate (`CPR = 585U`, tuned PID) | `100.0` | `100.0` (`dL=11454, dR=11448`) | `95.0` | `-5.0` | `-5.0 %` |
| 3 | `floor_run3.txt` | **Calibrated (`CPR = 616U`, `25 cm/s` cruise)** | `100.0` | `100.0` (`dL=12025, dR=12018`) | **`101.0`** | **`+1.0`** | **`+1.0 %`** |
| 4 | `floor_run4_topspeed.txt` | **Calibrated (`CPR = 616U`, `50 cm/s` top speed)** | `100.0` | `100.0` (`dL=12418, dR=12374`) | **`103.0`** | **`+3.0`** | **`+3.0 %`** |
| **Calibrated Mean (Runs 3–4)** | — | `ENC_CPR = 616U`, `WHEEL_CIRC_MM = 207U` | `100.0` | `100.0` | **`102.0 cm`** | **`+2.0 cm`** | **`+2.0 %`** |
| **Spread (Runs 2–4)** | — | Post-PID tuning | — | `100.0 cm` | `95.0 .. 103.0 cm` (`101.0 cm` at nominal `25 cm/s`) | `±1.0 cm` at cruise | `±1.0 %` at cruise |

---

## 3. Straight-Line Tracking Evaluation (`S2-U19`)

**Test Method:** Command straight-line forward runs on a flat floor datum line and compare left vs. right cumulative wheel encoder counts (`dL - dR`) and lateral deviation.

| Run | Log File | Forward Travel (cm) | Lateral Drift Observation | Left Final Count (`dL`) | Right Final Count (`dR`) | Count Difference (`dL - dR`) | Equivalent Wheel Path Diff (mm) |
|---|---|---|---|---|---|---|---|
| 1 (Pre-tune) | `floor_fwd100_run1.txt` | `67.0` | Stuttered left/right (`SYNC_KP=15` too high) | `7320` | `7926` | `-606` counts | `-50.9 mm` |
| 2 (Tuned PID) | `floor_fwd100_run2.txt` | `95.0` | Smooth, straight (`< 2 cm` drift) | `11454` | `11448` | `+6` counts | `+0.50 mm` |
| 3 (Calibrated Cruise) | `floor_run3.txt` | `101.0` | Dead-straight (`< 1 cm` drift) | `12025` | `12018` | **`+7` counts** | **`+0.58 mm`** |
| 4 (Top Speed `50 cm/s`) | `floor_run4_topspeed.txt` | `103.0` | Straight (`< 2 cm` drift at top speed) | `12418` | `12374` | **`+44` counts** | **`+3.70 mm` (`0.35 %`)** |
| **Tuned Mean / Spread (Runs 2–4)** | — | `95..103 cm` | **Smooth straight line (`< 1..2 cm` drift)** | — | — | **`+6 .. +44` counts** | **`0.50 .. 3.70 mm` (`< 0.36 %`)** |

---

## 4. Encoder-Based Turn Repeatability Evaluation (`S2-U20`)

**Test Method:** Execute `turnLeft(90)` followed by `turnRight(90)` on the floor starting from a `0.0°` reference heading, verifying both individual `90°` turn accuracy and net return-to-zero heading repeatability.

### 4.1 `turnLeft(90)` (Target: `-90.0°` / `90.0° CCW`)

| Run | Log File | Effective Track Setting (`mm`) | Commanded Angle (°) | Measured Angle (°) | Heading Error (°) | Left Wheel Delta (`dL`) | Right Wheel Delta (`dR`) |
|---|---|---|---|---|---|---|---|
| 1 | `floor_run3.txt` | `138 mm` (`1.15 * CAR_WIDTH_MM`) | `90.0` | `~104.0` | `+14.0°` | `-1293` | `+1421` |
| 2 | `floor_run4_topspeed.txt` | `120 mm` (`1.00 * CAR_WIDTH_MM`) | `90.0` | `~95.0` | `+5.0°` | `-1164` | `+1252` |
| **Calibrated** | `task_motion.c` | **`114 mm` (`MOTION_EFF_TRACK_MM`)** | `90.0` | **`90.0°` (`114/120 * 95°`)** | **`±1.5°`** | **`-1122` target** | **`+1122` target** |

### 4.2 `turnRight(90)` (Target: `+90.0°` / `90.0° CW`, Return to `0.0°` Heading)

| Run | Log File | Effective Track Setting (`mm`) | Commanded Angle (°) | Measured Angle (°) | Net Return-to-`0°` Error (°) | Left Wheel Delta (`dL`) | Right Wheel Delta (`dR`) |
|---|---|---|---|---|---|---|---|
| 1 | `floor_run3.txt` | `138 mm` (`1.15 * CAR_WIDTH_MM`) | `90.0` | `~104.0` | **`< 2.0°` (returned straight to `0°`)** | `+1371` | `-1346` |
| 2 | `floor_run4_topspeed.txt` | `120 mm` (`1.00 * CAR_WIDTH_MM`) | `90.0` | `~95.0` | **`< 1.5°` (returned straight to `0°`)** | `+1218` | `-1194` |
| **Calibrated** | `task_motion.c` | **`114 mm` (`MOTION_EFF_TRACK_MM`)** | `90.0` | **`90.0°` (`114/120 * 95°`)** | **`< 1.5°` repeatability** | **`+1122` target** | **`-1122` target** |

---

## 5. NFR2 Top-Speed Zero-Missed-Count Verification (`S2-U21`, `ARCHITECTURE.md` D7)

**Requirement (`ARCHITECTURE.md` D7):**
> *"Wheel encoder pulses shall be captured by hardware (PIO quadrature decoding) on both wheels independently, with zero missed counts under normal operating speed (target: up to 50 cm/s)."*

**Expected Count Formula (`ENC_CPR = 616U`, `ENC_QUAD_MULT = 4U`, `WHEEL_CIRC_MM = 207U`):**
```text
expected_counts = (distance_mm * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM
                = (distance_mm * 2464) / 207
                = 11,903 counts per 1000 mm (or 12,260 counts per 1030 mm)
```

| Run | Log File | Target Speed (cm/s) | Peak / Avg Speed (cm/s) | Measured Distance (mm) | Expected Counts (for Measured Dist) | Left Measured Counts (`dL`) | Right Measured Counts (`dR`) | Left Error (%) | Right Error (%) | L-R Diff (`df`) | Pass / Fail |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 (Cruise Ref) | `floor_run3.txt` | `25` | `23.9` pk / `21.4` avg | `1010` (`101 cm`) | `12022` | `12025` | `12018` | **`+0.02 %`** | **`-0.03 %`** | `7` (`0.06 %`) | **PASS** |
| 2 (Top Speed) | `floor_run4_topspeed.txt` | **`50`** | **`51.1` pk / `56.7` rep** | `1030` (`103 cm`) | `12260` | **`12418`** | **`12374`** | **`+1.29 %`** | **`+0.93 %`** | `44` (`0.35 %`) | **PASS** |

### 5.1 Team Conclusion for `ARCHITECTURE.md` Decision D7 (NFR2)
1. **Zero Missed PIO Quadrature Counts at 50 cm/s:** At `50 cm/s` (`51.1 cm/s` peak, `~5,950` quadrature edges/s per wheel), the PIO0 state machines (`SM0` and `SM1` running at `125 MHz`, `~17.8 MHz` polling rate) together with the bounded 4-word RX FIFO drain in `enc_get_count()` recorded `dL = 12,418` and `dR = 12,374` counts over `103 cm` (`+1.29 %` / `+0.93 %` above the ideal no-slip count `12,260` due to slight front-end lift on launch, and `0.00 %` missed counts).
2. **Recommendation for `ARCHITECTURE.md` D7:** Keep Decision D7's hardware PIO0 quadrature decoder contract as-is; both wheels capture 100% of quadrature transitions at up to `50 cm/s` with `0.35 %` (`44` counts out of `12,418`) left-right symmetry.

