# Subsystem 2 — Motion Accuracy & Hardware Calibration Evaluation

**Subsystem:** Subsystem 2 (Motion Control and Vehicle Controller)  
**Owner:** Ruo Chen (Buddy 2, Student ID: `2500530`)  
**Deliverable Reference:** `specs.md` ("Buddy 2 Deliverables: Motion control library, PID tuning report, Motion accuracy evaluation"), `ARCHITECTURE.md` D7 (NFR2), `config/thresholds.h`

---

## 1. Hardware Calibration Parameters & Power Budget Verification

### 1.1 Encoder & Wheel Geometry (`config/thresholds.h`, `config/pins.h`)

| Parameter | Header | Default / Status | Measurement Method (`TEST_LOG.md`) | Measured Value |
|---|---|---|---|---|
| `MOTOR_L_DIR_INVERT` | `config/pins.h` | `0U` (to confirm on car) | `S2-U03`: Wheels off ground, command low-duty forward/backward | *pending S2-U03* |
| `MOTOR_R_DIR_INVERT` | `config/pins.h` | `0U` (to confirm on car) | `S2-U03`: Wheels off ground, command low-duty forward/backward | *pending S2-U03* |
| `ENC_L_DIR_INVERT` | `config/pins.h` | `0U` (confirmed on car) | `S2-U07` / `S2-U08`: Hand-spin forward + 100 ms forward drive (`encL: 101->561, +460`) | `0U` |
| `ENC_R_DIR_INVERT` | `config/pins.h` | `0U` (needs `1U` in `config/pins.h`) | `S2-U07` / `S2-U08`: Hand-spin forward + 100 ms forward drive (`encR: -575->-1122, -547`) | `1U` |
| `ENC_CPR` | `config/thresholds.h` | `0U` (`#error` guard active when required) | `S2-U10`: Turn wheel 10 exact revolutions by hand; `CPR = delta_count / (10 * 4)` | *pending S2-U10* |
| `WHEEL_CIRC_MM` | `config/thresholds.h` | `0U` (`#error` guard active when required) | `S2-U11`: Roll wheel 10 turns along a ruler; `WHEEL_CIRC_MM = total_mm / 10` | *pending S2-U11* |
| `MOTOR_DUTY_MAX_PCT` | `config/thresholds.h` | `80U` | `S2-U02` (auto) + top-speed load check at `POWER_VIN_MIN_MV` (`3.6 V`) | `80 %` (`800` permille) |

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

**Test Method:** Command `moveForward(100)` (target = `100.0 cm` = `1000 mm`) on a flat floor for 5 consecutive runs. Measure actual stopping distance from the front wheel axle reference mark with a tape measure, and compare against `drep_ms_distance_cm(g_motion_state.w)`.

| Run | Commanded Distance (cm) | Reported `motion_state_t` Distance (cm) | Measured Physical Distance (cm) | Absolute Error (cm) | Relative Error (%) |
|---|---|---|---|---|---|
| 1 | `100.0` | *pending* | *pending* | *pending* | *pending* |
| 2 | `100.0` | *pending* | *pending* | *pending* | *pending* |
| 3 | `100.0` | *pending* | *pending* | *pending* | *pending* |
| 4 | `100.0` | *pending* | *pending* | *pending* | *pending* |
| 5 | `100.0` | *pending* | *pending* | *pending* | *pending* |
| **Mean** | `100.0` | | | | |
| **Std Dev / Spread** | — | | | | |

---

## 3. Straight-Line Tracking Evaluation (`S2-U19`)

**Test Method:** Command `moveForward(200)` (target = `200.0 cm`) along a straight floor datum line for 5 runs. Measure lateral deviation (`cm`, left = `-`, right = `+`) at the end of the 2.0 m travel.

| Run | Forward Travel (cm) | Lateral Drift at 2.0 m (cm) | Left Final Count | Right Final Count | Count Difference (`dl - dr`) |
|---|---|---|---|---|---|
| 1 | `200.0` | *pending* | *pending* | *pending* | *pending* |
| 2 | `200.0` | *pending* | *pending* | *pending* | *pending* |
| 3 | `200.0` | *pending* | *pending* | *pending* | *pending* |
| 4 | `200.0` | *pending* | *pending* | *pending* | *pending* |
| 5 | `200.0` | *pending* | *pending* | *pending* | *pending* |
| **Mean / Spread** | `200.0` | | | | |

---

## 4. Encoder-Based Turn Repeatability Evaluation (`S2-U20`)

**Test Method:** Execute 5 runs of `turnLeft(90)` and 5 runs of `turnRight(90)` on the floor starting from `0.0°` reference heading. Measure final chassis heading angle with a protractor.

### 4.1 `turnLeft(90)` (Target: `-90.0°` / `90.0° CCW`)

| Run | Commanded Angle (°) | Measured Angle (°) | Heading Error (°) | Left Wheel Delta Counts | Right Wheel Delta Counts |
|---|---|---|---|---|---|
| 1 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 2 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 3 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 4 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 5 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| **Mean / Spread** | `90.0` | | | | |

### 4.2 `turnRight(90)` (Target: `+90.0°` / `90.0° CW`)

| Run | Commanded Angle (°) | Measured Angle (°) | Heading Error (°) | Left Wheel Delta Counts | Right Wheel Delta Counts |
|---|---|---|---|---|---|
| 1 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 2 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 3 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 4 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| 5 | `90.0` | *pending* | *pending* | *pending* | *pending* |
| **Mean / Spread** | `90.0` | | | | |

---

## 5. NFR2 Top-Speed Zero-Missed-Count Verification (`S2-U21`, `ARCHITECTURE.md` D7)

**Requirement (`ARCHITECTURE.md` D7):**
> *"Wheel encoder pulses shall be captured by hardware (PIO quadrature decoding) on both wheels independently, with zero missed counts under normal operating speed (target: up to 50 cm/s)."*

**Expected Count Formula:**
```text
expected_counts = (distance_mm * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM
```

| Run | Target Speed (cm/s) | Measured Distance (mm) | Expected Counts | Left Measured Counts | Right Measured Counts | Left Error (%) | Right Error (%) | Pass / Fail |
|---|---|---|---|---|---|---|---|---|
| 1 | `50` | `1000` | *pending* | *pending* | *pending* | *pending* | *pending* | *pending* |
| 2 | `50` | `1000` | *pending* | *pending* | *pending* | *pending* | *pending* | *pending* |
| 3 | `50` | `1000` | *pending* | *pending* | *pending* | *pending* | *pending* | *pending* |

