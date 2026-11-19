"""Tests for tel_calib.py: the line-sensor calibration helper (S1-H07).

    python -m unittest -v test_tel_calib

Standard library only.  The rules must match Buddy 3's firmware
(lf_sanity.c, lf_core.c); the last class checks them against his real LF-1
measurement log.
"""

from __future__ import annotations

import os
import tempfile
import unittest

import tel_calib as cal
import tel_codec as codec
import test_tel_views as tv

REPO = tv.REPO
RULES = cal.load_rules(REPO)


def sensor(ts: int, c: int, r: int, flags=(True, True, True)) -> codec.Frame:
    return codec.decode(bytes.fromhex(tv.sensor(ts, ir=(c, r), flags=flags).split()[-1]))


def stats(values) -> cal.Stats:
    s = cal.Stats()
    for v in values:
        s.add(v, 4095)
    return s


class Rules(unittest.TestCase):
    def test_read_from_the_firmware(self):
        for name in ("LF_LIGHT_MAX_PM", "LF_DARK_MIN_PM", "LF_MIN_SPAN_RAW", "ADC_NOISE_MAX_RAW"):
            with self.subTest(name=name):
                self.assertFalse(RULES[name]["source"].startswith("default"), RULES[name])

    def test_define_parser(self):
        with tempfile.NamedTemporaryFile("w", suffix=".h", delete=False) as fh:
            fh.write("#define A_B   (250U)   /* comment */\n#define C 7\n#define D (E + 1)\n#define F (12u) // x\n")
        try:
            self.assertEqual(cal.read_c_defines(fh.name, {"A_B", "C", "D", "F"}), {"A_B": 250, "C": 7, "F": 12})
        finally:
            os.unlink(fh.name)

    def test_missing_file_falls_back_to_documented_defaults(self):
        rules = cal.load_rules(tempfile.gettempdir())
        self.assertEqual(rules["LF_MIN_SPAN_RAW"]["value"], 400)
        self.assertTrue(rules["LF_MIN_SPAN_RAW"]["source"].startswith("default"))


class Maths(unittest.TestCase):
    def test_average_rounds_to_nearest(self):
        self.assertEqual(stats([1, 2]).avg, 2)          # (3 + 1) // 2, as lf_adc_stats_avg
        self.assertEqual(stats([1, 1, 2]).avg, 1)

    def test_adc_check(self):
        self.assertEqual(cal.adc_check(stats([3, 10, 16]), RULES)[0], "STUCK_LOW")
        self.assertEqual(cal.adc_check(stats([4079, 4095]), RULES)[0], "STUCK_HIGH")
        self.assertEqual(cal.adc_check(stats([100, 600]), RULES)[0], "NOISY")
        self.assertEqual(cal.adc_check(stats([100, 500]), RULES)[0], "OK")      # spread 400 is allowed
        self.assertEqual(cal.adc_check(cal.Stats(), RULES)[0], "NO_SAMPLES")

    def test_darkness_and_zone_match_lf_norm_and_lf_zone_of(self):
        self.assertEqual(cal.darkness(100, 140, 1144, 1000), 0)
        self.assertEqual(cal.darkness(2000, 140, 1144, 1000), 1000)
        self.assertEqual(cal.darkness(642, 140, 1144, 1000), 500)        # (502 * 1000) // 1004
        self.assertEqual(cal.darkness(500, 500, 500, 1000), 0)           # uncalibrated
        self.assertEqual([cal.zone(d, RULES) for d in (249, 250, 750, 751)], ["white", "edge", "edge", "black"])

    def test_channel_verdicts(self):
        ok = cal.channel_result(stats([130, 150]), stats([1100, 1180]), RULES)
        self.assertEqual((ok["verdict"], ok["span"]), ("OK", 1000))
        self.assertEqual(ok["light_max_raw"], 140 + 1000 * 250 // 1000)
        self.assertEqual(ok["dark_min_raw"], 140 + 1000 * 750 // 1000)
        flat = cal.channel_result(stats([140, 150]), stats([300, 320]), RULES)
        self.assertEqual(flat["verdict"], "NO_CONTRAST")
        bad = cal.channel_result(stats([140, 150]), stats([10, 12]), RULES)
        self.assertEqual(bad["verdict"], "BAD_SENSOR")
        self.assertEqual(cal.channel_result(None, None, RULES)["verdict"], "INCOMPLETE")


class GuidedCapture(unittest.TestCase):
    def run_capture(self, c, target, t0, values, simulated=False, flags=(True, True, True)):
        self.assertEqual(c.start(target, t0, 1.0), "")
        t = t0
        for cv, rv in values:
            t += 0.2
            c.on_frame(sensor(int(t * 1000), cv, rv, flags), t, simulated)
        c.tick(t0 + 1.5)
        return t0 + 2.0

    def test_white_then_black(self):
        c = cal.Calibration(RULES)
        t = self.run_capture(c, "white", 10.0, [(140, 150)] * 5)
        t = self.run_capture(c, "black", t, [(1140, 1150)] * 5)
        res = c.results()
        self.assertEqual((res["c"]["verdict"], res["r"]["verdict"]), ("OK", "OK"))
        live = c.live_view()
        self.assertEqual((live["c"]["zone"], live["c"]["darkness_pm"]), ("black", 1000))
        c.on_frame(sensor(99000, 640, 150), 99.0, False)
        self.assertEqual(c.live_view()["c"]["zone"], "edge")
        self.assertEqual(c.live_view()["r"]["zone"], "white")

    def test_simulated_and_missing_values_are_not_used(self):
        c = cal.Calibration(RULES)
        self.run_capture(c, "white", 10.0, [(123, 4095)] * 6, simulated=True)
        self.assertIn("failed", c.last_capture_note)
        self.assertIn("simulated", c.last_capture_note)
        self.assertIsNone(c.white["c"])
        self.run_capture(c, "white", 20.0, [(140, 150)] * 6, flags=(False, True, True))
        self.assertIn("without IR values", c.last_capture_note)
        self.assertTrue(any("tel_set_ir_raw" in n for n in c.snapshot(30.0)["notes"]))

    def test_bad_requests(self):
        c = cal.Calibration(RULES)
        self.assertNotEqual(c.start("grey", 1.0), "")
        self.assertNotEqual(c.start("white", 1.0, 120), "")
        self.assertIn("white/black measurement", c.use_car_values())


class BuddyThreeMeasurement(unittest.TestCase):
    """Buddy 3's LF-1 run of 3 Nov: CENTRE sees the line, RIGHT does not (his S3-U09 (c) FAIL)."""

    LOG = os.path.join(REPO, "testing", "subsystem3", "logs", "2026-11-03_s3_lf1_run2.txt")

    def setUp(self):
        if not os.path.exists(self.LOG):
            self.skipTest("log not in this checkout")
        self.c = cal.Calibration(RULES)
        with open(self.LOG, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if "[" in line:
                    self.c.on_console(line[line.find("["):])

    def test_same_verdicts_as_his_hardware_test(self):
        self.assertEqual(self.c.use_car_values(), "")
        res = self.c.results()
        self.assertEqual((res["c"]["white"]["avg"], res["c"]["black"]["avg"], res["c"]["span"]), (140, 1144, 1004))
        self.assertEqual(res["c"]["verdict"], "OK")             # S3-U09 (b) PASS
        self.assertEqual(res["r"]["verdict"], "NO_CONTRAST")    # S3-U09 (c) FAIL
        # The boot rule (CENTRE black vs RIGHT white) still passes: it cannot see a blind RIGHT sensor.
        self.assertEqual(res["boot_rule"]["verdict"], "OK")

    def test_both_steps_win_over_single_sensor_steps(self):
        meas = self.c.car["measured"]
        self.assertEqual(meas[("white", "c")].avg, 140)        # WHITE under both, not "line under RIGHT"
        self.assertEqual(meas[("black", "c")].avg, 1144)       # BLACK under both, not "line under CENTRE"

    def test_boot_calibration_lines(self):
        c = cal.Calibration(RULES)
        c.on_console("[LF] boot calibration ok: C avg 1144 (1099..1177), R avg 144 (117..169)")
        c.on_console("[LF] cal white 144 black 1144 (both sensors)")
        snap = c.snapshot(None)["car"]
        self.assertEqual(snap["boot"]["c_avg"], 1144)
        self.assertEqual(snap["cal"], {"white": 144, "black": 1144})


if __name__ == "__main__":
    unittest.main()
