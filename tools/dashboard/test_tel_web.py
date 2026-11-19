"""Tests for the web UI (tel_web.py): the HTTP API on a real local server.

    python -m unittest -v test_tel_web

Standard library only.  MQTT is a fake client (no broker); the page's script
is syntax-checked with node when node is installed.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest import mock

import tel_dashboard as dash
import tel_web as web
import test_tel_views as tv


class _Server:
    def __init__(self, monitor: web.Monitor) -> None:
        self.monitor = monitor
        self.srv = web.make_server(monitor, "127.0.0.1", 0)
        self.base = f"http://127.0.0.1:{self.srv.server_address[1]}"
        self.t = threading.Thread(target=self.srv.serve_forever, daemon=True)
        self.t.start()

    def get(self, path: str):
        try:
            with urllib.request.urlopen(self.base + path, timeout=5) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def jget(self, path: str):
        code, body = self.get(path)
        return code, json.loads(body)

    def post(self, path: str, obj):
        req = urllib.request.Request(self.base + path, data=json.dumps(obj).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def close(self):
        self.monitor.stop()
        self.srv.shutdown()
        self.srv.server_close()


def _wait(cond, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def _reading(state, view, name):
    return next(r for r in state["readings"][view] if r["name"] == name)


class WebApi(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.s = _Server(web.Monitor(roots=[web.REPO, self.tmp], out_dir=self.tmp))

    def tearDown(self):
        self.s.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _open(self, path, follow=False):
        code, j = self.s.post("/api/source", {"type": "file", "path": path, "follow": follow})
        self.assertEqual(code, 200, j)

    def _done(self):
        self.assertTrue(_wait(lambda: not self.s.jget("/api/state")[1]["source"]["running"]))
        return self.s.jget("/api/state?since=0")[1]

    def test_page_start_needs_arming(self):
        # Dashboard v2: START is on the page, but only behind "Arm motion" and a confirm
        # dialog; the server refuses it unless armed (test_start_refused_even_when_asked).
        code, body = self.s.get("/")
        self.assertEqual(code, 200)
        page = body.decode()
        self.assertIn("Robocar dashboard", page)
        self.assertIn("Arm motion", page)
        self.assertIn("EMERGENCY STOP", page)
        self.assertRegex(page, r'if \(!confirm\("START drives the motors[^"]*"\)\) return;\s*sendCmd\("start"\)')
        for label in ("MEASURED", "DERIVED", "REPORTED", "SIMULATED", "N/A"):
            self.assertIn(label, page)

    def test_start_refused_even_when_asked(self):
        code, j = self.s.post("/api/cmd", {"cmd": "start"})
        self.assertEqual(code, 403)
        self.assertIn("--allow-motion", j["error"])
        code, j = self.s.post("/api/cmd", {"cmd": "stop"})
        self.assertEqual(code, 409)        # no MQTT source

    def test_only_repo_files(self):
        code, j = self.s.post("/api/source", {"type": "file", "path": "/etc/passwd"})
        self.assertEqual(code, 400)
        code, j = self.s.post("/api/source", {"type": "file", "path": "../../../../etc/passwd"})
        self.assertEqual(code, 400)

    def test_usb_needs_windows(self):
        if os.name == "nt":
            self.skipTest("on Windows this would open the real serial port")
        code, j = self.s.post("/api/source", {"type": "usb", "seconds": 60})
        self.assertEqual(code, 400)
        self.assertIn("Windows", j["error"])

    def test_logs_listed(self):
        code, logs = self.s.jget("/api/logs")
        self.assertEqual(code, 200)
        paths = [l["path"].replace("\\", "/") for l in logs]
        if os.path.isdir(os.path.join(web.REPO, "testing", "subsystem4", "logs")):
            self.assertTrue(any(p.startswith("testing/subsystem4/logs/") for p in paths))

    def test_real_s4_log(self):
        rel = "testing/subsystem4/logs/2026-11-13_1530_s4_console.txt"
        if not os.path.exists(os.path.join(web.REPO, rel)):
            self.skipTest("log not in this checkout")
        self._open(rel)
        st = self._done()
        self.assertTrue(st["s1_ran"])
        self.assertEqual(_reading(st, "imu", "pitch")["kind"], "SIMULATED")     # S1-U07 values persist
        self.assertEqual(_reading(st, "imu", "hump height")["kind"], "N/A")
        self.assertEqual(_reading(st, "imu", "IMU chip")["kind"], "REPORTED")
        self.assertEqual(_reading(st, "lf", "IR CENTRE raw")["kind"], "SIMULATED")
        code, vs = self.s.jget("/api/check?kind=imu")
        res = {v["name"]: v["result"] for v in vs}
        self.assertEqual(res["IMU chip found"], "PASS")
        self.assertEqual(res["Pitch follows a hand tilt (span >= 10 deg)"], "NO DATA")
        self.assertTrue(st["feed"]["imu"])

    def test_real_lf_log_state(self):
        rel = "testing/subsystem3/logs/2026-11-05_s3_u15_run7.txt"
        if not os.path.exists(os.path.join(web.REPO, rel)):
            self.skipTest("log not in this checkout (line-following branch only)")
        self._open(rel)
        st = self._done()
        self.assertEqual(_reading(st, "lf", "LF state")["kind"], "REPORTED")
        # an APP_TEST build that ran the S1 tests: their injected IR values, never MEASURED
        self.assertTrue(st["s1_ran"])
        self.assertEqual(_reading(st, "lf", "IR CENTRE raw")["kind"], "SIMULATED")

    def test_lf_ir_from_sensor_frames(self):
        path = os.path.join(self.tmp, "lf.txt")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(tv.sensor(1000, ir=(1712, 180), flags=(True, False, False)) + "\n")
            fh.write("[LF] ON_LINE -> LINE_LOST  c=300 r=180 dc=90 dr=20 vc=4\n")
        self._open(path)
        st = self._done()
        self.assertEqual((_reading(st, "lf", "IR CENTRE raw")["value"], _reading(st, "lf", "IR CENTRE raw")["kind"]),
                         ("1712", "MEASURED"))
        self.assertEqual(_reading(st, "lf", "IR RIGHT raw")["value"], "180")
        self.assertEqual(_reading(st, "lf", "LF state")["value"], "LINE_LOST")
        self.assertEqual(_reading(st, "imu", "pitch")["kind"], "N/A")             # no attitude in this frame

    def test_follow_live_synthetic_run(self):
        path = os.path.join(self.tmp, "live.txt")
        self._open(path, follow=True)          # does not exist yet: the source waits for it
        lines = tv.imu_stationary_log()
        with open(path, "w", encoding="utf-8") as fh:
            for i, ln in enumerate(lines):
                fh.write(f"12:00:{i % 60:02d}.000  {ln}\n")
                fh.flush()
                if i == 5:
                    self.assertTrue(_wait(lambda: self.s.jget("/api/state")[1]["source"]["lines"] >= 6))
                    st = self.s.jget("/api/state")[1]
                    self.assertIn("data_age_s", st["source"])
        self.assertTrue(_wait(lambda: self.s.jget("/api/state")[1]["source"]["lines"] == len(lines)))
        st = self.s.jget("/api/state?since=0")[1]
        self.assertTrue(st["source"]["running"])          # still following
        self.assertEqual(_reading(st, "imu", "pitch")["kind"], "DERIVED")
        self.assertEqual(_reading(st, "imu", "|acceleration|")["kind"], "MEASURED")
        self.assertEqual(_reading(st, "vc", "motors commanded")["value"], "no")
        res = {v["name"]: v["result"] for v in self.s.jget("/api/check?kind=imu")[1]}
        self.assertEqual(res["Pitch follows a hand tilt (span >= 10 deg)"], "PASS")
        self.assertEqual(res["Lift detected and brakes latched"], "PASS")
        gen = st["gen"]
        self.assertEqual(self.s.post("/api/source", {"type": "stop"})[0], 200)
        st = self.s.jget("/api/state")[1]
        self.assertFalse(st["source"]["running"])
        self._open(path)                         # a new source: a new generation, fresh feeds
        self.assertGreater(self.s.jget("/api/state")[1]["gen"], gen)

    def test_obstacle_check_params(self):
        path = os.path.join(self.tmp, "obs.txt")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(tv.ObstacleCheck().log()) + "\n")
        self._open(path)
        st = self._done()
        self.assertEqual(_reading(st, "obstacle", "forward range")["kind"], "MEASURED")
        self.assertEqual(_reading(st, "obstacle", "obstacle width")["kind"], "DERIVED")
        code, vs = self.s.jget("/api/check?kind=obstacle&expect_cm=20,50&expect_width_mm=100")
        res = {v["name"]: v["result"] for v in vs}
        self.assertEqual(res["Range matches a tape measure (+-2 cm): box at 20 cm"], "PASS")
        self.assertEqual(res["Obstacle width (DERIVED estimate; spec: errs wide)"], "PASS")
        self.assertEqual(self.s.jget("/api/check?kind=obstacle&expect_cm=abc")[0], 400)
        self.assertEqual(self.s.jget("/api/check?kind=start")[0], 400)

    def test_mqtt_with_fake_client(self):
        msgs = [("car/status", b"1")]
        for i in range(35):
            msgs.append(("car/telemetry/heartbeat", bytes.fromhex(tv.heartbeat(5000 + i * 1000).split()[-1])))
        fake = tv._FakeClient(msgs)
        fake.published = []
        fake.on_disconnect = None
        fake.publish = lambda topic, frame, qos=0: fake.published.append((topic, frame))
        with mock.patch.dict("sys.modules", {"paho": mock.MagicMock(), "paho.mqtt": mock.MagicMock(),
                                             "paho.mqtt.client": mock.MagicMock()}), \
                mock.patch.object(dash, "_new_client", return_value=(None, fake)):
            code, j = self.s.post("/api/source", {"type": "mqtt", "broker": "10.0.0.2", "port": 1883})
        self.assertEqual(code, 200, j)
        st = self.s.jget("/api/state?since=0")[1]
        self.assertTrue(st["source"]["mqtt_connected"])
        self.assertEqual(st["source"]["car_status"], "ONLINE")
        self.assertEqual(_reading(st, "mqtt", "heartbeats received")["value"], "35")
        self.assertEqual(self.s.post("/api/cmd", {"cmd": "start"})[0], 403)
        self.assertEqual(self.s.post("/api/cmd", {"cmd": "stop"})[0], 200)
        self.assertEqual(fake.published, [("car/cmd/stop", bytes.fromhex("010900"))])
        res = {v["name"]: v["result"] for v in self.s.jget("/api/check?kind=mqtt")[1]}
        self.assertEqual(res["Heartbeats delivered to the laptop"], "PASS")
        saved = [f for f in os.listdir(self.tmp) if f.startswith("dash-mqtt-")]
        self.assertEqual(len(saved), 1)          # the session is kept like listen --save


class _Info:
    def __init__(self, mid):
        self.mid = mid


def _motion_hex(ts, speed=0.0):
    return tv._W(1).put(ts, 24).put(int(round(speed * 10)), 10).put(0, 16).put(0, 32).put(0, 32).hex()


class WebV2(unittest.TestCase):
    """Dashboard v2: commands with confirmation, the table, CSV export, data gaps, calibration."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.s = _Server(web.Monitor(roots=[web.REPO, self.tmp], out_dir=self.tmp))

    def tearDown(self):
        self.s.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _mqtt(self):
        fake = tv._FakeClient([("car/status", b"1"),
                               ("car/telemetry/heartbeat", bytes.fromhex(tv.heartbeat(5000).split()[-1]))])
        fake.on_disconnect = fake.on_publish = None
        fake.published = []

        def publish(topic, frame, qos=0):
            fake.published.append((topic, frame))
            mid = len(fake.published)
            if fake.on_publish:
                fake.on_publish(fake, None, mid)       # the broker's PUBACK, at once
            return _Info(mid)

        fake.publish = publish
        with mock.patch.dict("sys.modules", {"paho": mock.MagicMock(), "paho.mqtt": mock.MagicMock(),
                                             "paho.mqtt.client": mock.MagicMock()}), \
                mock.patch.object(dash, "_new_client", return_value=(None, fake)):
            code, j = self.s.post("/api/source", {"type": "mqtt", "broker": "10.0.0.2", "port": 1883})
        self.assertEqual(code, 200, j)
        return fake

    def _car(self, fake, topic, hexframe):
        fake.on_message(fake, None, tv._FakeMsg(topic, bytes.fromhex(hexframe)))

    def _history(self):
        return self.s.jget("/api/state")[1]["ctl"]["history"]

    def test_start_arm_confirm_and_estop(self):
        fake = self._mqtt()
        self.assertEqual(self.s.post("/api/cmd", {"cmd": "start"})[0], 403)           # not armed
        self.assertEqual(fake.published, [])
        self.assertEqual(self.s.post("/api/arm", {"armed": True})[0], 200)
        code, j = self.s.post("/api/cmd", {"cmd": "start"})
        self.assertEqual(code, 200, j)
        self.assertEqual(fake.published, [("car/cmd/start", bytes.fromhex("010800"))])
        self.assertEqual(self._history()[0]["result"], "PENDING")    # the broker has it, the car has not said
        self.assertEqual(self._history()[0]["broker"], "ok")
        self._car(fake, "car/telemetry/heartbeat", tv.heartbeat(5300, "NAV_LINE_TRACKING").split()[-1])
        self.assertEqual(self._history()[0]["result"], "CONFIRMED")
        self.assertEqual(self.s.post("/api/cmd", {"cmd": "start"})[0], 403)           # one START per arming
        code, j = self.s.post("/api/estop", {})
        self.assertEqual(code, 200, j)
        self.assertEqual([t for t, _ in fake.published[1:]], ["car/cmd/stop"] * 3)
        self._car(fake, "car/telemetry/motion", _motion_hex(5400, 0.0))
        self._car(fake, "car/telemetry/heartbeat", tv.heartbeat(5500, "READY_WAITING").split()[-1])
        h = self._history()[0]
        self.assertEqual((h["cmd"], h["result"]), ("estop", "CONFIRMED"))
        code, j = self.s.post("/api/cmd", {"cmd": "pause"})
        self.assertEqual(code, 501)
        self.assertIn("PAUSED", j["error"])
        self.assertEqual(self._history()[0]["result"], "REFUSED")

    def test_estop_without_broker(self):
        code, j = self.s.post("/api/estop", {})
        self.assertEqual(code, 409)
        self.assertIn("by hand", j["error"])
        self.assertEqual(self.s.post("/api/arm", {"armed": True})[0], 409)

    def _log(self):
        """5 rows/s for 4 s, a 6 s silence, then an IR reading at the top of the ADC."""
        lines, t = [], 36000.0
        def put(tt, txt):
            h, rem = divmod(tt, 3600)
            m, s = divmod(rem, 60)
            lines.append(f"{int(h):02d}:{int(m):02d}:{s:06.3f}  {txt}")
        for i in range(20):
            if i % 5 == 0:
                put(t, tv.heartbeat(int(t * 1000) % 10**9))
            put(t, f"[TEL] car/telemetry/motion {_motion_hex(int(t * 1000) % (1 << 24), 5.0)}")
            put(t + 0.01, tv.sensor(int(t * 1000) % (1 << 24), pitch=1.0, rng=30, ir=(600 + i, 150 + i),
                                    flags=(True, True, True)))
            t += 0.2
        t += 6.0
        for i in range(5):
            put(t, tv.sensor(int(t * 1000) % (1 << 24), pitch=1.0, rng=30, ir=(4095 if i == 2 else 640 + i, 150),
                             flags=(True, True, True)))
            t += 0.2
        path = os.path.join(self.tmp, "run.txt")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        return path

    def test_table_export_and_gaps(self):
        self.assertEqual(self.s.post("/api/source", {"type": "file", "path": self._log()})[0], 200)
        self.assertTrue(_wait(lambda: not self.s.jget("/api/state")[1]["source"]["running"]))
        code, page = self.s.jget("/api/table?limit=5")
        self.assertEqual(code, 200)
        self.assertEqual(page["total"], 25)
        self.assertEqual(len(page["rows"]), 5)
        first = self.s.jget("/api/table?before=3")[1]["rows"]
        self.assertEqual([r["seq"] for r in first], [1, 2])
        self.assertEqual((first[0]["speed"], first[0]["ir_c"], first[0]["range_cm"]), (5.0, 600, 30))
        code, body = self.s.get("/api/export.csv?what=rows&scope=all")
        self.assertEqual(code, 200)
        text = body.decode()
        self.assertTrue(text.startswith("seq,time,car_ms,ir_centre_raw"))
        self.assertEqual(len(text.strip().splitlines()), 26)
        st = self.s.jget("/api/state")[1]
        kinds = {(g["sensor"], g["state"]) for g in st["gaps"]}
        self.assertIn(("ir_c", "STALE"), kinds)       # the 6 s silence
        self.assertIn(("ir_c", "INVALID"), kinds)     # raw 4095
        self.assertIn("INVALID", self.s.get("/api/export.csv?what=gaps")[1].decode())
        self.assertEqual(self.s.get("/api/export.csv?what=pictures")[0], 400)
        self.assertEqual(self.s.post("/api/record", {"action": "start"})[0], 200)
        self.assertTrue(self.s.jget("/api/state")[1]["table"]["recording"]["active"])
        self.assertEqual(self.s.post("/api/record", {"action": "stop"})[0], 200)
        self.assertEqual(self.s.post("/api/record", {"action": "rewind"})[0], 400)

    def test_calibration_endpoints(self):
        rel = "testing/subsystem3/logs/2026-11-03_s3_lf1_run2.txt"
        if not os.path.exists(os.path.join(web.REPO, rel)):
            self.skipTest("log not in this checkout")
        self.assertEqual(self.s.post("/api/source", {"type": "file", "path": rel})[0], 200)
        self.assertTrue(_wait(lambda: not self.s.jget("/api/state")[1]["source"]["running"]))
        code, j = self.s.post("/api/calib", {"action": "capture", "target": "white"})
        self.assertEqual(code, 400)
        self.assertIn("live source", j["error"])
        self.assertEqual(self.s.post("/api/calib", {"action": "use_car"})[0], 200)
        res = self.s.jget("/api/state")[1]["calib"]["result"]
        self.assertEqual((res["c"]["verdict"], res["r"]["verdict"]), ("OK", "NO_CONTRAST"))
        self.assertEqual(self.s.post("/api/calib", {"action": "reset"})[0], 200)
        self.assertEqual(self.s.jget("/api/state")[1]["calib"]["result"]["c"]["verdict"], "INCOMPLETE")
        self.assertEqual(self.s.post("/api/calib", {"action": "capture", "seconds": "x"})[0], 400)


class OneConnection(unittest.TestCase):
    """Regression, 19 Nov 2026: two clients with one id took the broker connection off each other."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.s = _Server(web.Monitor(roots=[web.REPO, self.tmp], out_dir=self.tmp))

    def tearDown(self):
        self.s.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_every_client_gets_its_own_id(self):
        made = []

        class Client:
            def __init__(self, *a, client_id="", **k):
                made.append(client_id)

        client_mod = mock.MagicMock()
        client_mod.Client = Client
        mqtt_mod = mock.MagicMock(client=client_mod)
        with mock.patch.dict("sys.modules", {"paho": mock.MagicMock(mqtt=mqtt_mod), "paho.mqtt": mqtt_mod,
                                             "paho.mqtt.client": client_mod}):
            for base in ("robocar-web", "robocar-web", "robocar-dashboard"):
                dash._new_client(base)
        self.assertEqual(len(set(made)), 3)
        self.assertTrue(made[0].startswith("robocar-web-") and made[2].startswith("robocar-dashboard-"))

    def _connect(self, fake):
        with mock.patch.dict("sys.modules", {"paho": mock.MagicMock(), "paho.mqtt": mock.MagicMock(),
                                             "paho.mqtt.client": mock.MagicMock()}), \
                mock.patch.object(dash, "_new_client", return_value=(None, fake)):
            return self.s.post("/api/source", {"type": "mqtt", "broker": "10.0.0.2", "port": 1883})

    def test_second_connect_while_one_is_set_up(self):
        self.assertTrue(self.s.monitor._connecting.acquire(blocking=False))     # one is in progress
        try:
            code, j = self._connect(tv._FakeClient([]))
        finally:
            self.s.monitor._connecting.release()
        self.assertEqual(code, 400)
        self.assertIn("already being set up", j["error"])

    def test_a_replaced_client_is_ignored(self):
        hb = bytes.fromhex(tv.heartbeat(5000).split()[-1])
        old = tv._FakeClient([("car/telemetry/heartbeat", hb)])
        old.on_disconnect = old.on_publish = None
        self.assertEqual(self._connect(old)[0], 200)
        new = tv._FakeClient([("car/telemetry/heartbeat", hb)])
        new.on_disconnect = new.on_publish = None
        self.assertEqual(self._connect(new)[0], 200)
        n = self.s.jget("/api/state")[1]["source"]["frames"]
        old.on_message(old, None, tv._FakeMsg("car/telemetry/heartbeat", hb))     # the old one, late
        old.on_connect(old, None, {}, 0)
        self.assertEqual(self.s.jget("/api/state")[1]["source"]["frames"], n)
        new.on_message(new, None, tv._FakeMsg("car/telemetry/heartbeat", hb))
        self.assertEqual(self.s.jget("/api/state")[1]["source"]["frames"], n + 1)


class PageScript(unittest.TestCase):
    def test_node_syntax(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("node not installed")
        with open(web.PAGE, encoding="utf-8") as fh:
            js = re.search(r"<script>(.*)</script>", fh.read(), re.S).group(1)
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(js)
        try:
            r = subprocess.run([node, "--check", fh.name], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
        finally:
            os.unlink(fh.name)


if __name__ == "__main__":
    unittest.main()
