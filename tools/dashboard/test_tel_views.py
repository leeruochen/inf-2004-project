"""Tests for the dashboard integration: tel_views.py and the new tel_dashboard.py options.

    python -m unittest -v test_tel_views

Standard library only: listen is driven by a fake MQTT client, so neither
paho-mqtt nor a broker is needed.  The sensor-frame flag bytes are the same
as tests/host/test_tel.c checks against the C encoder.
"""

from __future__ import annotations

import io
import os
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

import tel_codec as codec
import tel_dashboard as dash
import tel_views as views

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


class _W:
    """LSB-first bit writer (drep_bits_put) for building test frames."""

    def __init__(self, t: int) -> None:
        self.t = t
        self.bits = []

    def put(self, v: int, n: int) -> "_W":
        v &= (1 << n) - 1
        self.bits += [(v >> i) & 1 for i in range(n)]
        return self

    def hex(self) -> str:
        n = codec.FRAME_LEN[self.t]
        b = bytearray(n)
        for i, bit in enumerate(self.bits):
            b[i >> 3] |= bit << (i & 7)
        return bytes([codec.WIRE_VER, self.t, n]).hex() + bytes(b).hex()


def sensor(ts, pitch=0.0, roll=0.0, rng=0, ir=(0, 0), flags=(False, False, True)):
    w = _W(codec.FT_TELEMETRY_SENSOR).put(ts, 24).put(ir[1], 12).put(ir[0], 12).put(rng, 9)
    w.put(int(round(pitch * 2)), 8).put(int(round(roll * 2)), 8)
    for f in flags:
        w.put(1 if f else 0, 1)
    return f"[TEL] car/telemetry/sensor {w.hex()}"


def heartbeat(up_ms, state="READY_WAITING", bits=()):
    mask = sum(1 << codec.SAFETY_BITS.index(b) for b in bits)
    w = _W(codec.FT_HEARTBEAT).put(up_ms, 32).put(codec.SYS_STATE.index(state), 4).put(mask, 10)
    return f"[TEL] car/telemetry/heartbeat {w.hex()}"


def hump(ts, height=0, pitch=6.0):
    w = _W(codec.FT_EVENT_HUMP).put(ts, 24).put(height, 7).put(int(pitch * 2), 8).put(0, 1)
    return f"[TEL] car/telemetry/hump {w.hex()}"


def obstacle(ts, cm, bearing_deg=90, width_mm=120, cl=300, cr=635):
    w = _W(codec.FT_EVENT_OBSTACLE).put(ts, 24).put(cm, 9).put((bearing_deg - 30) // 10, 4)
    w.put(width_mm // 5, 7).put(cl // 5, 7).put(1, 2).put(0, 1).put(cr // 5, 7)
    return f"[TEL] car/telemetry/obstacle {w.hex()}"


def run_check(lines, kind, **kw):
    s = views.Session(None)
    for ln in lines:
        s.feed_line(ln)
    fn = {"imu": views.check_imu, "obstacle": views.check_obstacle, "mqtt": views.check_mqtt}[kind]
    return {v.name: v for v in fn(s.ev, **kw)}


def imu_stationary_log():
    """What the stationary IMU procedure should produce on a healthy car."""
    out = ["[IMU] chip LSM303D, accel+mag at 0x1D", "[TEL] backend: console ([TEL] <topic> <hex>)"]
    t = 15000
    tilt = [0, 0, 0, 0, 0, 0, 8, 15, 15, 8, 0, 0, 0, 0, 0, 0, 0, 0]
    roll = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -6, -13, -13, -6, 0, 0, 0]
    for i, (p, r) in enumerate(zip(tilt, roll)):
        out.append(sensor(t, p + 0.5, r - 0.5))
        out.append(heartbeat(t, "READY_WAITING", ("IMU_ALIVE",)))
        out.append(f"[TEL] imu found=1 mag={995 + i % 7} mg pitch={int(p * 2)} half-deg")
        t += 1000
    out += ["[IMU] LIFTED (up): cos_tilt=990/1000 up_accel=300 mg v=120 mm/s",
            "[SAFETY] BRAKE: lifted or overturned",
            heartbeat(t, "SAFETY_FAULT_LATCHED", ("LIFTED", "FAULT_LATCHED"))]
    return out


class SensorFlags(unittest.TestCase):
    def test_flag_bytes_match_c(self):
        # tests/host/test_tel.c test_golden_frames: same frame, flags 1,1,1 and 0,1,0
        f = codec.decode(bytes.fromhex("01020ae80300ff0f80904d5b0e")).fields
        self.assertTrue(f["ir_valid"] and f["range_valid"] and f["att_valid"])
        self.assertEqual((f["ir_r_raw"], f["ir_c_raw"], f["range_cm"], f["pitch_deg"]), (4095, 2048, 400, -45.0))
        f = codec.decode(bytes.fromhex("01020ae80300ff0f80904d5b04")).fields
        self.assertEqual((f["ir_valid"], f["range_valid"], f["att_valid"]), (False, True, False))
        f = codec.decode(bytes.fromhex("01020ae80300ff0f80904d5b00")).fields
        self.assertFalse(f["flags_present"])

    def test_summary_marks_unavailable(self):
        s = codec.decode(bytes.fromhex("01020ae80300ff0f80904d5b04")).summary()
        self.assertIn("IR n/a", s)
        self.assertIn("range=400cm", s)
        self.assertIn("pitch/roll n/a", s)
        self.assertIn("validity flags absent", codec.decode(bytes.fromhex("01020ae80300ff0f80904d5b00")).summary())


class StatusLines(unittest.TestCase):
    def test_formats_from_task_tel(self):
        # The exact dbg_printf formats in task_tel.c log_status().
        k, d = codec.parse_status_line("12:00:00.000  [TEL] imu found=1 mag=1003 mg pitch=-3 half-deg")
        self.assertEqual((k, d["found"], d["accel_mag_mg"], d["pitch_deg"]), ("imu", True, 1003, -1.5))
        k, d = codec.parse_status_line("[TEL] obs sensors_ok=0 avoid_state=2 avoid_active=1")
        self.assertEqual((k, d["sensors_ok"], d["avoid_state"], d["avoid_active"]), ("obs", False, 2, True))
        k, d = codec.parse_status_line("[TEL] CONNECTED tx=10 txf=1 off=2 ev=3 bad=0 qd=0 od=0 cmd=4/1")
        self.assertEqual((k, d["link"], d["tx"], d["txf"], d["cmd_rej"]), ("link", "CONNECTED", 10, 1, 1))
        self.assertIsNone(codec.parse_status_line("[TEL] car/telemetry/heartbeat 010706"))


class ImuCheck(unittest.TestCase):
    def test_healthy_stationary_run(self):
        v = run_check(imu_stationary_log(), "imu")
        for name in ("IMU chip found", "No IMU I2C or watchdog fault",
                     "Acceleration magnitude at rest 900-1100 mg (1 g)",
                     "Pitch within +-2 deg while flat after boot",
                     "Pitch follows a hand tilt (span >= 10 deg)", "Roll follows a hand tilt (span >= 10 deg)",
                     "Lift detected and brakes latched", "Motors never commanded (stationary test)"):
            self.assertEqual(v[name].result, "PASS", f"{name}: {v[name].detail}")
        self.assertEqual(v["Hump height"].result, "NOT TESTED")

    def test_empty_log_never_passes(self):
        v = run_check([], "imu")
        self.assertFalse([n for n, x in v.items() if x.result == "PASS"])

    def test_no_tilt_is_not_tested(self):
        lines = [ln for ln in imu_stationary_log() if "sensor" not in ln] + \
                [sensor(20000 + i * 1000, 0.5, -0.5) for i in range(10)]
        v = run_check(lines, "imu")
        self.assertEqual(v["Pitch follows a hand tilt (span >= 10 deg)"].result, "NOT TESTED")

    def test_bad_magnitude_fails(self):
        lines = [f"[TEL] imu found=1 mag={m} mg pitch=0 half-deg" for m in (1450, 1460, 1470)]
        self.assertEqual(run_check(lines, "imu")["Acceleration magnitude at rest 900-1100 mg (1 g)"].result, "FAIL")

    def test_hump_height_zero_is_not_measured(self):
        v = run_check([hump(100), hump(900)], "imu")
        self.assertEqual(v["Hump height"].result, "NOT MEASURED")
        self.assertEqual(v["Hump event reported"].result, "INFO")

    def test_s1_injection_is_simulated(self):
        lines = ["[TEST] S1-U07 PASS  sensor snapshot clamps to wire fields"] + \
                [sensor(i * 1000, 15.0 * (i % 2)) for i in range(10)] + ["[TEST] subsystem1 SUMMARY 10 passed"]
        v = run_check(lines, "imu")
        self.assertEqual(v["Pitch follows a hand tilt (span >= 10 deg)"].result, "NO DATA")
        s = views.Session("imu")
        s.feed_line(lines[0])
        self.assertIn("SIMULATED", " ".join(s.feed_line(lines[1])))

    def test_motors_commanded_fails(self):
        v = run_check([heartbeat(1000), heartbeat(2000, "NAV_LINE_TRACKING")], "imu")
        self.assertEqual(v["Motors never commanded (stationary test)"].result, "FAIL")

    def test_unplug_test_faults_not_counted(self):
        lines = [heartbeat(i * 1000) for i in range(6)] + ["[S4] MANUAL S4-U14: unplug the IMU cable"] + \
                [heartbeat(9000, "SAFETY_FAULT_LATCHED", ("IMU_WDOG", "FAULT_LATCHED"))]
        self.assertEqual(run_check(lines, "imu")["No IMU I2C or watchdog fault"].result, "PASS")
        early = lines[:6] + [lines[7], lines[6]]       # the fault comes BEFORE the unplug step
        self.assertEqual(run_check(early, "imu")["No IMU I2C or watchdog fault"].result, "FAIL")
        late = lines[6:] + lines[:6]                    # unplug first: nothing before it to judge
        self.assertEqual(run_check(late, "imu")["No IMU I2C or watchdog fault"].result, "NO DATA")


class ObstacleCheck(unittest.TestCase):
    def log(self):
        rng = [0] * 3 + [20, 21, 20, 20, 20] + [35] + [50, 49, 50, 50, 51]
        out = [sensor(i * 1000, rng=r, flags=(False, True, True)) for i, r in enumerate(rng)]
        out += ["[TEL] obs sensors_ok=1 avoid_state=0 avoid_active=0"] * 3
        out += [obstacle(9000, 19, 90, 130), heartbeat(10000)]
        return out

    def test_expect_distances(self):
        v = run_check(self.log(), "obstacle", expect_cm=[20, 50, 30])
        self.assertEqual(v["Range matches a tape measure (+-2 cm): box at 20 cm"].result, "PASS")
        self.assertEqual(v["Range matches a tape measure (+-2 cm): box at 50 cm"].result, "PASS")
        self.assertEqual(v["Range matches a tape measure (+-2 cm): box at 30 cm"].result, "FAIL")
        self.assertEqual(v["Obstacle detected within 25 cm"].result, "PASS")
        self.assertEqual(v["Scanner sensors OK"].result, "PASS")
        self.assertEqual(v["Avoidance manoeuvre"].result, "NOT TESTED")

    def test_without_ground_truth_only_compare(self):
        v = run_check(self.log(), "obstacle")
        self.assertEqual(v["Range matches a tape measure (+-2 cm)"].result, "COMPARE")
        self.assertEqual(v["Obstacle width (DERIVED estimate; spec: errs wide)"].result, "COMPARE")

    def test_width_errs_wide(self):
        name = "Obstacle width (DERIVED estimate; spec: errs wide)"
        self.assertEqual(run_check(self.log(), "obstacle", expect_width_mm=100)[name].result, "PASS")
        self.assertEqual(run_check(self.log(), "obstacle", expect_width_mm=200)[name].result, "FAIL")

    def test_no_echo_only_fails(self):
        lines = [sensor(i * 1000, rng=0, flags=(False, True, True)) for i in range(8)]
        self.assertEqual(run_check(lines, "obstacle")["Forward ultrasonic readings"].result, "FAIL")

    def test_plateaus(self):
        self.assertEqual(views.plateaus([0, 20, 21, 20, 0, 20, 35, 50, 49, 50, 51, 50, 22]),
                         [(20.0, 4), (50.0, 5)])


class Mqtt(unittest.TestCase):
    def saved(self, gap_at=None):
        out = ["10:00:00.000  [DASH] source=mqtt broker=x:1883"]
        up = 5000
        for i in range(40):
            up += 5000 if i == gap_at else 1000
            out.append(f"10:00:{i:02d}.000  {heartbeat(up)}")
        return out

    def test_delivery(self):
        self.assertEqual(run_check(self.saved(), "mqtt")["Heartbeats delivered to the laptop"].result, "PASS")
        self.assertEqual(run_check(self.saved(gap_at=20), "mqtt")["Heartbeats delivered to the laptop"].result,
                         "FAIL")

    def test_console_backend_is_not_mqtt(self):
        v = run_check(["[TEL] backend: console ([TEL] <topic> <hex>)"], "mqtt")
        self.assertEqual(list(v.values())[0].result, "INFO")


class _FakeMsg:
    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = payload


class _FakeClient:
    """Stands in for paho: delivers queued messages when the loop starts."""

    def __init__(self, msgs):
        self.msgs = msgs
        self.on_connect = None
        self.on_message = None

    def connect(self, *a, **k):
        pass

    def subscribe(self, *a, **k):
        pass

    def loop_start(self):
        self.on_connect(self, None, {}, 0)
        for t, p in self.msgs:
            self.on_message(self, None, _FakeMsg(t, p))

    def loop_stop(self):
        pass

    def disconnect(self):
        pass


class Cli(unittest.TestCase):
    def test_listen_save_then_check(self):
        msgs = [("car/status", b"1")]
        for i in range(35):
            hx = heartbeat(5000 + i * 1000).split()[-1]
            msgs.append(("car/telemetry/heartbeat", bytes.fromhex(hx)))
        msgs.append(("car/telemetry/sensor", bytes.fromhex(sensor(9000, 3.0).split()[-1])))
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "mqtt.txt")
            fake = _FakeClient(msgs)
            with mock.patch.object(dash, "_new_client", return_value=(None, fake)), \
                    mock.patch.object(dash.time, "sleep", side_effect=KeyboardInterrupt), \
                    redirect_stdout(io.StringIO()) as out:
                self.assertEqual(dash.main(["listen", "--broker", "x", "--view", "imu", "--save", path]), 0)
            self.assertIn("pitch +3.0 deg [DERIVED", out.getvalue())
            with redirect_stdout(io.StringIO()) as out:
                dash.main(["check", "mqtt", path])
            self.assertIn("[PASS        ] Heartbeats delivered to the laptop", out.getvalue())

    def test_start_needs_allow_motion(self):
        with mock.patch.object(dash, "_new_client", side_effect=AssertionError("must not connect")), \
                redirect_stdout(io.StringIO()) as out:
            self.assertEqual(dash.main(["send", "start", "--broker", "x"]), 2)
        self.assertIn("--allow-motion", out.getvalue())

    def test_peak_hump_zero_is_not_a_height(self):
        s = dash.RunSummary()
        s.add(codec.decode(bytes.fromhex(hump(10).split()[-1])))
        self.assertIn("height not measured", s.text())


class RealLogs(unittest.TestCase):
    """Hardware logs already in the repo: the checks must stay honest on them."""

    def path(self, rel):
        p = os.path.join(REPO, rel)
        if not os.path.exists(p):
            self.skipTest(f"{rel} not in this checkout")
        return p

    def test_s4_app_test_log(self):
        with open(self.path("testing/subsystem4/logs/2026-11-13_1530_s4_console.txt"), encoding="utf-8",
                  errors="replace") as fh:
            v = run_check(fh.readlines(), "imu")
        self.assertEqual(v["IMU chip found"].result, "PASS")
        self.assertEqual(v["No IMU I2C or watchdog fault"].result, "PASS")     # U14 faults excluded
        self.assertEqual(v["Pitch follows a hand tilt (span >= 10 deg)"].result, "NO DATA")   # S1 simulated
        self.assertEqual(v["Hump height"].result, "NOT MEASURED")
        self.assertEqual(v["Motors never commanded (stationary test)"].result, "FAIL")   # U23 drives

    def test_s5_log(self):
        with open(self.path("testing/subsystem5/logs/2026-11-04_s5_unit_all.txt"), encoding="utf-8",
                  errors="replace") as fh:
            v = run_check(fh.readlines(), "obstacle")
        self.assertEqual(v["Obstacle detected within 25 cm"].result, "PASS")
        self.assertEqual(v["Forward ultrasonic readings"].result, "NO DATA")


if __name__ == "__main__":
    unittest.main()
