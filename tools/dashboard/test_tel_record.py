"""Tests for tel_record.py: the recording table, sensor health and data gaps (S1-H06).

    python -m unittest -v test_tel_record

Standard library only.  Frames are built with the same bit writer as
test_tel_views.py; the last class replays every hardware log in testing/.
"""

from __future__ import annotations

import csv
import glob
import io
import os
import unittest

import tel_codec as codec
import tel_record as rec
import tel_views as views
import test_tel_views as tv

REPO = tv.REPO


def frame(line: str) -> codec.Frame:
    return codec.decode(bytes.fromhex(line.split()[-1]))


def motion(ts: int, speed: float = 0.0, dist: int = 0, enc=(0, 0)) -> codec.Frame:
    w = tv._W(codec.FT_TELEMETRY_MOTION).put(ts, 24).put(int(round(speed * 10)), 10).put(dist, 16)
    w.put(enc[0], 32).put(enc[1], 32)
    return codec.decode(bytes.fromhex(w.hex()))


def sensor(ts: int, **kw) -> codec.Frame:
    kw.setdefault("flags", (True, True, True))
    return frame(tv.sensor(ts, **kw))


def heartbeat(up_ms: int, state: str = "READY_WAITING") -> codec.Frame:
    return frame(tv.heartbeat(up_ms, state))


class Classify(unittest.TestCase):
    def setUp(self):
        self.limits = {"ADC_STUCK_LOW_RAW": 16, "ADC_STUCK_HIGH_RAW": 4079}

    def c(self, **kw):
        return rec.classify_sensor_frame(sensor(1000, **kw).fields, self.limits)

    def test_valid_readings(self):
        out = self.c(ir=(640, 150), rng=37, pitch=3.0, roll=-1.5)
        self.assertEqual({k: v[0] for k, v in out.items()},
                         {"ir_c": "VALID", "ir_r": "VALID", "ultrasonic": "VALID", "imu": "VALID"})
        self.assertEqual(out["ultrasonic"][2], "37 cm")

    def test_unavailable_when_no_producer(self):
        out = self.c(ir=(640, 150), flags=(False, True, True))
        self.assertEqual(out["ir_c"][0], "UNAVAILABLE")
        self.assertIn("IF-12", out["ir_c"][1])

    def test_old_firmware_without_flags(self):
        out = self.c(ir=(640, 150), flags=(False, False, False))
        self.assertTrue(all(v[0] == "UNAVAILABLE" for v in out.values()))
        self.assertIn("validity unknown", out["imu"][1])

    def test_invalid_readings(self):
        out = self.c(ir=(4095, 10), rng=400, pitch=45.0)
        self.assertEqual(out["ir_c"][0], "INVALID")         # top of the ADC
        self.assertEqual(out["ir_r"][0], "INVALID")         # bottom of the ADC
        self.assertEqual(out["ultrasonic"][0], "INVALID")   # the 400 cm clamp
        self.assertEqual(out["imu"][0], "INVALID")          # the 45 deg clamp

    def test_no_echo_is_a_valid_reading(self):
        out = self.c(rng=0)
        self.assertEqual(out["ultrasonic"][:1], ("VALID",))
        self.assertEqual(out["ultrasonic"][2], "no echo")

    def test_motion_saturation(self):
        self.assertEqual(rec.classify_motion(motion(0, 51.1).fields)[0], "INVALID")
        self.assertEqual(rec.classify_motion(motion(0, 20.0).fields)[0], "VALID")


class Recording(unittest.TestCase):
    def test_motion_and_sensor_of_one_wake_make_one_row(self):
        r = rec.Recorder()
        r.add(heartbeat(5000, "NAV_LINE_TRACKING"), 10.0)
        r.add(motion(5200, 12.3, 40), 10.20)
        r.add(sensor(5200, ir=(700, 160), rng=55, pitch=2.0, roll=1.0), 10.21)
        r.add(motion(5400, 12.5, 42), 10.40)
        r.add(sensor(5400, ir=(705, 162), rng=54, pitch=2.0, roll=1.0), 10.41)
        self.assertEqual(len(r.rows), 2)
        row = r.rows[0]
        self.assertEqual((row["speed"], row["distance_cm"], row["ir_c"], row["ir_r"], row["range_cm"], row["state"]),
                         (12.3, 40, 700, 160, 55, "NAV_LINE_TRACKING"))
        self.assertEqual(row["time"], "00:00:10.210")

    def test_motion_without_sensor_frames(self):
        r = rec.Recorder()                  # firmware where no producer wrote: no sensor frame at all
        for i in range(3):
            r.add(motion(1000 * i, 0.0, 5), 100.0 + i)
        r.flush(None)
        self.assertEqual(len(r.rows), 3)
        self.assertIsNone(r.rows[0]["ir_c"])
        self.assertEqual(r.rows[0]["speed"], 0.0)

    def test_unavailable_values_stay_empty(self):
        r = rec.Recorder()
        r.add(sensor(1000, ir=(700, 160), rng=55, flags=(False, True, True)), 1.0)
        row = r.rows[-1]
        self.assertIsNone(row["ir_c"])
        self.assertEqual(row["range_cm"], 55)
        self.assertEqual(row["status"]["ir_c"], "unavailable")

    def test_csv_and_recording_span(self):
        r = rec.Recorder()
        r.add(sensor(1000, ir=(700, 160)), 1.0)
        r.start_recording()
        r.add(sensor(1200, ir=(710, 161)), 1.2)
        r.add(sensor(1400, ir=(720, 162)), 1.4)
        r.stop_recording()
        r.add(sensor(1600, ir=(730, 163)), 1.6)
        self.assertEqual(r.recording()["rows"], 2)
        rows = list(csv.DictReader(io.StringIO(r.csv_text("recording"))))
        self.assertEqual([x["ir_centre_raw"] for x in rows], ["710", "720"])
        self.assertEqual(list(rows[0]), rec.CSV_COLUMNS)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(r.csv_text("all"))))), 4)

    def test_paging(self):
        r = rec.Recorder()
        for i in range(10):
            r.add(sensor(1000 + i * 200), 1.0 + i * 0.2)
        self.assertEqual([x["seq"] for x in r.page(limit=3)["rows"]], [8, 9, 10])
        self.assertEqual([x["seq"] for x in r.page(before=8, limit=3)["rows"]], [5, 6, 7])
        self.assertEqual([x["seq"] for x in r.page(after=8, limit=3)["rows"]], [9, 10])

    def test_simulated_rows_are_marked(self):
        r = rec.Recorder()
        r.add(sensor(1000, ir=(123, 4000)), 1.0, simulated=True)
        self.assertTrue(r.rows[-1]["simulated"])
        self.assertEqual(r.rows[-1]["status"]["ir_c"], "simulated")


class HealthStates(unittest.TestCase):
    def feed(self, h, t0, n, period=0.2, **kw):
        for i in range(n):
            t = t0 + i * period
            h.on_frame(sensor(int(t * 1000), **kw), t)
        return t0 + n * period

    def state(self, h, key):
        return next(x for x in h.snapshot(None) if x["key"] == key)

    def test_rate_from_the_car_clock(self):
        h = rec.Health()
        self.feed(h, 100.0, 20, ir=(700, 160))
        self.assertAlmostEqual(self.state(h, "ir_c")["rate_hz"], 5.0, places=1)
        self.assertEqual(self.state(h, "ir_c")["state"], "VALID")

    def test_live_silence_goes_stale_then_recovers(self):
        h = rec.Health()
        end = self.feed(h, 100.0, 10, ir=(700, 160))
        h.evaluate(end + 0.5)
        self.assertEqual(self.state(h, "ir_c")["state"], "VALID")
        h.evaluate(end + 3.0)
        self.assertEqual(self.state(h, "ir_c")["state"], "STALE")
        self.feed(h, end + 3.2, 2, ir=(701, 160))
        self.assertEqual(self.state(h, "ir_c")["state"], "VALID")
        gaps = [g for g in h.gap_rows(None) if g["sensor"] == "ir_c"]
        self.assertEqual(len(gaps), 1)
        self.assertFalse(gaps[0]["open"])
        self.assertEqual(gaps[0]["state"], "STALE")

    def test_replayed_gap_is_found_afterwards(self):
        h = rec.Health()
        end = self.feed(h, 100.0, 10, ir=(700, 160))
        self.feed(h, end + 5.0, 3, ir=(702, 161))       # 5 s without frames, inside a saved log
        gaps = [g for g in h.gap_rows(None) if g["sensor"] == "ir_c"]
        self.assertEqual(len(gaps), 1)
        self.assertGreaterEqual(gaps[0]["duration_s"], 5.0)

    def test_disconnect_and_back(self):
        h = rec.Health()
        end = self.feed(h, 100.0, 10, ir=(700, 160))
        h.set_link(False, "the car is OFFLINE", end)
        self.assertTrue(all(x["state"] == "DISCONNECTED" for x in h.snapshot(None) if x["key"] in ("ir_c", "imu")))
        self.assertEqual(self.state(h, "motion")["state"], "UNAVAILABLE")     # never seen: not a gap
        h.set_link(True, "", end + 10)
        self.feed(h, end + 10.1, 3, ir=(700, 160))
        self.assertEqual(self.state(h, "ir_c")["state"], "VALID")
        kinds = [g["state"] for g in h.gap_rows(None) if g["sensor"] == "ir_c"]
        self.assertEqual(kinds, ["DISCONNECTED", "STALE"])

    def test_invalid_then_valid_is_one_event(self):
        h = rec.Health()
        h.on_frame(sensor(1000, ir=(700, 160)), 1.0)
        h.on_frame(sensor(1200, ir=(4095, 160)), 1.2)
        h.on_frame(sensor(1400, ir=(4095, 160)), 1.4)
        h.on_frame(sensor(1600, ir=(700, 160)), 1.6)
        gaps = [g for g in h.gap_rows(None) if g["sensor"] == "ir_c"]
        self.assertEqual([(g["state"], g["start"], g["end"]) for g in gaps],
                         [("INVALID", "00:00:01.200", "00:00:01.600")])

    def test_frozen_ir_value(self):
        h = rec.Health()
        self.feed(h, 100.0, rec.IR_FROZEN_FRAMES + 2, ir=(700, 160))
        self.assertEqual(self.state(h, "ir_c")["state"], "STALE")
        self.assertIn("unchanged", self.state(h, "ir_c")["reason"])

    def test_car_restart_resets_the_rate_window(self):
        h = rec.Health()
        for i in range(5):
            h.on_frame(heartbeat(50000 + i * 1000), 10.0 + i)
        h.on_frame(heartbeat(400), 15.5)             # rebooted: uptime starts again
        h.on_frame(heartbeat(1400), 16.5)
        self.assertAlmostEqual(self.state(h, "heartbeat")["rate_hz"], 1.0, places=1)


class RealLogs(unittest.TestCase):
    def logs(self):
        found = sorted(glob.glob(os.path.join(REPO, "testing", "*", "logs", "*.txt")))
        if not found:
            self.skipTest("no hardware logs in this checkout")
        return found

    def replay(self, path):
        s, r, h = views.Session(None), rec.Recorder(), rec.Health()
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                body = line[line.find("["):] if "[" in line else line
                s.feed_line(line)
                p = codec.parse_log_line(body)
                if p is None:
                    continue
                try:
                    fr = codec.decode(p[1])
                except codec.FrameError:
                    continue
                sim = s.latest[fr.name]["sim"]
                r.add(fr, views._wall_s(line), sim)
                h.on_frame(fr, views._wall_s(line), sim)
        r.flush(None)
        return r, h

    def test_every_log_replays(self):
        for path in self.logs():
            with self.subTest(log=os.path.basename(path)):
                r, h = self.replay(path)
                r.csv_text("all")
                h.gap_rows(None)

    def test_usb_console_cadence_is_not_a_gap(self):
        # Regression: a USB console prints every 5th heartbeat (5 s apart) plus extra ones
        # on state changes; that cadence must not be reported as missing data.
        path = os.path.join(REPO, "testing", "subsystem3", "logs", "2026-11-03_s3_lf1_run2.txt")
        if not os.path.exists(path):
            self.skipTest("log not in this checkout")
        r, h = self.replay(path)
        self.assertEqual(h.gap_rows(None), [])
        self.assertEqual(next(x for x in h.snapshot(None) if x["key"] == "heartbeat")["state"], "VALID")
        self.assertTrue(all(row["simulated"] for row in r.rows))      # S1 self-test values persist

    def test_s1_unit_log_rates(self):
        path = os.path.join(REPO, "testing", "subsystem1", "logs", "2026-11-02_s1_unit.txt")
        if not os.path.exists(path):
            self.skipTest("log not in this checkout")
        r, h = self.replay(path)
        snap = {x["key"]: x for x in h.snapshot(None)}
        self.assertAlmostEqual(snap["motion"]["rate_hz"], 1.0, places=1)      # every 5th 200 ms frame
        self.assertEqual(len(r.rows), 204)


if __name__ == "__main__":
    unittest.main()
