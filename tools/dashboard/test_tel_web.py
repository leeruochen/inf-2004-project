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

    def test_page_has_no_start_control(self):
        code, body = self.s.get("/")
        self.assertEqual(code, 200)
        page = body.decode()
        self.assertIn("Robocar dashboard", page)
        self.assertNotRegex(page, r'cmd:\s*"start"')
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
