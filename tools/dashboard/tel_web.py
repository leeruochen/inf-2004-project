#!/usr/bin/env python3
"""Robocar dashboard in the browser (dashboard integration, Buddy 3).

    python tools/dashboard/tel_web.py          # opens http://127.0.0.1:8765

A page over the existing tools, no rewrite: tel_codec decodes the frames,
tel_views labels every value (MEASURED / DERIVED / REPORTED / SIMULATED / N/A)
and runs the checks, tel_dashboard's helpers connect to MQTT.  Standard
library only; paho-mqtt only for the MQTT source.

Sources (one at a time):
    USB capture  Windows: runs tools/serial_capture.ps1 into build/dash-live-*.txt
                 and follows it
    log file     a saved log, or one still being written (follow)
    MQTT         subscribes to car/# and saves build/dash-mqtt-*.txt

Dashboard v2 (Buddy 1) adds, on the same frames: the recording table and CSV
export (tel_record.Recorder), sensor health and the data-gap table
(tel_record.Health), the line-sensor calibration helper (tel_calib) and
remote commands with confirmation (tel_control).

Safety: START is sent only after "arm motion" (one START per arming, see
tel_control); STOP, RESET and the emergency STOP go out over MQTT only.  A
command is shown as done only when the car's telemetry confirms it.  It
listens on 127.0.0.1 unless --host says otherwise.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import tel_calib as calib
import tel_codec as codec
import tel_control as control
import tel_dashboard as dash
import tel_record as record
import tel_views as views

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
PAGE = os.path.join(HERE, "tel_web.html")

VIEW_NAMES = ("imu", "obstacle", "lf", "barcode", "vc", "mqtt")
FEED_MAX = 400                  # event lines kept per view
CONSOLE_MAX = 600               # raw console lines kept
TRACKED = ("[LF]", "[BC]", "[VC]", "[S5] avoid", "[SAFETY]", "[capture]")
# task_lf.c: "[LF] ON_LINE -> LINE_LOST  c=1712 r=180 dc=... dr=... vc=4"
_LF_MOVE = re.compile(r"\[LF\] ([A-Z_]+) -> ([A-Z_]+)\s+c=(\d+) r=(\d+)")


def _r(name: str, value: object, kind: str, note: str = "") -> dict:
    return {"name": name, "value": str(value), "kind": kind, "note": note}


def _now_sod() -> float:
    """The laptop's local time as seconds of day: the clock of every line stamp."""
    t = time.time()
    lt = time.localtime(t)
    return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec + (t % 1)


class Monitor:
    """One telemetry source feeding one tel_views.Session per view."""

    def __init__(self, repo: str = REPO, roots: Optional[List[str]] = None, out_dir: Optional[str] = None) -> None:
        self.repo = repo
        self.roots = [os.path.realpath(p) for p in (roots or [repo])]
        self.out_dir = out_dir or os.path.join(repo, "build")
        self.lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._proc: Optional[subprocess.Popen] = None
        self._client = None
        self._save = None
        self.gen = 0
        self.rules = calib.load_rules(repo)
        self.limits = {k: int(v["value"]) for k, v in self.rules.items()}
        self._reset("none", "no source")

    # -- state ----------------------------------------------------------------

    def _reset(self, kind: str, label: str) -> None:
        self.gen += 1               # the page restarts its feeds when this changes
        self.sessions = {v: views.Session(v) for v in VIEW_NAMES}
        self.seq = 0
        self.feed: Dict[str, Deque[Tuple[int, str]]] = {v: deque(maxlen=FEED_MAX) for v in VIEW_NAMES}
        self.console: Deque[Tuple[int, str]] = deque(maxlen=CONSOLE_MAX)
        self.last_line: Dict[str, str] = {}
        self.lf_state: Optional[Tuple[str, int]] = None     # (state, transitions seen)
        self.src = {"kind": kind, "label": label, "running": False, "live": False, "error": "",
                    "lines": 0, "frames": 0, "mqtt_connected": None, "car_status": "", "file": ""}
        self.last_rx: Optional[float] = None
        self.last_hb_rx: Optional[float] = None
        # Dashboard v2: one of each per source, so a new source starts clean.
        self.rec = record.Recorder()
        self.rec.limits = self.limits
        self.health = record.Health(self.limits)
        self.cal = calib.Calibration(self.rules)
        self.ctl = control.Controller()
        self.last_t: Optional[float] = None     # newest line stamp, seconds of day

    @property
    def ev(self) -> views.Evidence:
        return self.sessions["imu"].ev

    def feed_line(self, line: str) -> None:
        line = line.rstrip("\r\n")
        if not line:
            return
        t = views._wall_s(line)
        stamp = line[:12] if t is not None else ""
        body = line[line.find("["):] if "[" in line else line
        with self.lock:
            self.src["lines"] += 1
            self.last_rx = time.monotonic()
            if t is not None:
                self.last_t = t
            parsed = codec.parse_log_line(body)
            if parsed is not None:
                self.src["frames"] += 1
                if "car/telemetry/heartbeat" in body:
                    self.last_hb_rx = self.last_rx
            for p in TRACKED:
                if body.startswith(p):
                    self.last_line[p] = body
            m = _LF_MOVE.search(body)
            if m:
                self.lf_state = (m.group(2), (self.lf_state[1] + 1) if self.lf_state else 1)
            self.seq += 1
            self.console.append((self.seq, line))
            for v, s in self.sessions.items():
                for out in s.feed_line(line):
                    self.seq += 1
                    self.feed[v].append((self.seq, f"{stamp} {out}" if stamp else out))
            if parsed is not None:
                self._frame_v2(parsed[1])
            elif not body.startswith("[DASH]"):
                self.cal.on_console(body)

    def _frame_v2(self, data: bytes) -> None:
        """Recording, health, calibration and command confirmation for one frame (lock held)."""
        try:
            fr = codec.decode(data)
        except codec.FrameError:
            return              # tel_views already shows the reject
        sim = bool(self.sessions["imu"].latest.get(fr.name, {}).get("sim"))
        t = self.last_t
        self.rec.add(fr, t, sim)
        self.health.on_frame(fr, t, sim)
        self.cal.on_frame(fr, t, sim)
        if self.src["kind"] == "mqtt":     # only a live car can confirm a command
            now = time.monotonic()
            if fr.type == codec.FT_HEARTBEAT:
                self.ctl.on_heartbeat(str(fr.fields["state"]), now)
            elif fr.type == codec.FT_TELEMETRY_MOTION:
                self.ctl.on_motion(float(fr.fields["speed_cmps"]), now)

    # -- sources --------------------------------------------------------------

    def _inside_roots(self, path: str) -> Optional[str]:
        full = os.path.realpath(path if os.path.isabs(path) else os.path.join(self.repo, path))
        for root in self.roots:
            if full == root or full.startswith(root + os.sep):
                return full
        return None

    def open_file(self, path: str, follow: bool) -> str:
        full = self._inside_roots(path)
        if full is None:
            return "only files inside the repository can be opened"
        if not follow and not os.path.isfile(full):
            return f"{path}: no such file"
        self.stop()
        with self.lock:
            self._reset("file", os.path.relpath(full, self.repo))
            self.src.update(running=True, live=follow, file=os.path.relpath(full, self.repo))
        self._start_thread(self._follow_file, full, follow)
        return ""

    def start_usb(self, seconds: int) -> str:
        if os.name != "nt":
            return ("USB capture runs tools/serial_capture.ps1, which needs Windows. Elsewhere, capture "
                    "the log another way and open it with Follow.")
        script = os.path.join(self.repo, "tools", "serial_capture.ps1")
        os.makedirs(self.out_dir, exist_ok=True)
        out = os.path.join(self.out_dir, "dash-live-" + time.strftime("%Y%m%d-%H%M%S") + ".txt")
        self.stop()
        try:
            self._proc = subprocess.Popen(
                ["powershell", "-ExecutionPolicy", "Bypass", "-File", script, "-Seconds", str(seconds), "-Out", out],
                cwd=self.repo, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except OSError as exc:
            return f"could not start serial_capture.ps1: {exc}"
        with self.lock:
            self._reset("usb", f"USB capture -> {os.path.relpath(out, self.repo)}")
            self.src.update(running=True, live=True, file=os.path.relpath(out, self.repo))
        self._start_thread(self._follow_file, out, True)
        return ""

    def connect_mqtt(self, broker: str, port: int) -> str:
        try:
            import paho.mqtt.client  # noqa: F401  (only to give a clear message)
        except ImportError:
            return "paho-mqtt is not installed: pip install paho-mqtt"
        self.stop()
        os.makedirs(self.out_dir, exist_ok=True)
        out = os.path.join(self.out_dir, "dash-mqtt-" + time.strftime("%Y%m%d-%H%M%S") + ".txt")
        _, client = dash._new_client("robocar-web")
        save = open(out, "w", encoding="utf-8", buffering=1)
        header = f"{dash._stamp()}  [DASH] source=mqtt broker={broker}:{port}"
        save.write(header + "\n")
        with self.lock:
            self._reset("mqtt", f"MQTT {broker}:{port} -> {os.path.relpath(out, self.repo)}")
            self.src.update(running=True, live=True, mqtt_connected=False, file=os.path.relpath(out, self.repo))
        self.feed_line(header)
        self._save = save

        def on_connect(c, userdata, flags, *rest):
            with self.lock:
                self.src["mqtt_connected"] = True
                self.ctl.set_connected(True, time.monotonic())
            c.subscribe("car/#", qos=1)

        def on_disconnect(c, userdata, *rest):
            with self.lock:
                self.src["mqtt_connected"] = False
                self.ctl.set_connected(False, time.monotonic())
                self.health.set_link(False, "the laptop lost the broker connection", _now_sod())

        def on_publish(c, userdata, mid, *rest):
            with self.lock:         # QoS 1: the broker's PUBACK for one of our commands
                self.ctl.on_puback(mid, time.monotonic())

        def on_message(c, userdata, msg):
            if msg.topic.startswith("car/cmd/"):
                return
            if msg.topic == codec.STATUS_TOPIC:
                state = "ONLINE" if msg.payload == b"1" else "OFFLINE (last will)"
                with self.lock:
                    self.src["car_status"] = state
                    self.ctl.set_car_status(state)
                line = f"{dash._stamp()}  [DASH] car/status {state}"
            else:
                line = f"{dash._stamp()}  [TEL] {msg.topic} {bytes(msg.payload).hex()}"
            if self._save:
                self._save.write(line + "\n")
            self.feed_line(line)

        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message
        client.on_publish = on_publish
        try:
            client.connect(broker, port, keepalive=30)
        except (OSError, ValueError) as exc:
            save.close()
            self._save = None
            with self.lock:
                self.src.update(running=False, error=f"cannot reach {broker}:{port}: {exc}")
            return self.src["error"]
        client.loop_start()
        self._client = client
        return ""

    def _publish(self, cmd: str) -> Optional[int]:
        """One command frame, QoS 1.  Returns paho's message id (None if the client gives none)."""
        topic, frame = codec.encode_cmd(cmd)
        info = self._client.publish(topic, frame, qos=1)
        self.feed_line(f"{dash._stamp()}  [DASH] sent {cmd}: {topic} {frame.hex()}")
        return getattr(info, "mid", None)

    def send_cmd(self, cmd: str) -> Tuple[int, str, Optional[dict]]:
        """START / STOP / RESET (and PAUSE, refused).  Returns (HTTP code, message, command entry)."""
        now = time.monotonic()
        with self.lock:
            if self._client is None and cmd in ("stop", "reset") and self.src["kind"] != "mqtt":
                return 409, ("STOP/RESET need the MQTT source connected (a USB console build takes no "
                             "commands)"), None
            reason = self.ctl.refuse_reason(cmd, now)
            if not reason and self._client is None:
                reason = "not connected to the broker"
            if reason:
                entry = self.ctl.refused(cmd, reason, now, dash._stamp())
                code = 501 if cmd == "pause" else 403 if reason == control.START_NOT_ARMED else 409
                return code, reason, entry.as_dict(now)
            before = self.ctl.state
        mid = self._publish(cmd)
        with self.lock:
            entry = self.ctl.sent(cmd, now, dash._stamp(), [mid], before)
            return 200, f"sent {cmd}", entry.as_dict(time.monotonic())

    def estop(self) -> Tuple[int, str, Optional[dict]]:
        """STOP, several times in a row.  Never blocked by the car's state."""
        now = time.monotonic()
        with self.lock:
            if self._client is None or not self.src["mqtt_connected"]:
                entry = self.ctl.refused("estop", "not connected to the broker: STOP cannot be sent. "
                                         "Stop the car by hand (pick it up or switch it off).", now, dash._stamp())
                return 409, entry.detail, entry.as_dict(now)
            before = self.ctl.state
        repeats, spacing = control.estop_plan()
        mids = []
        for i in range(repeats):
            mids.append(self._publish("stop"))
            if i + 1 < repeats:
                time.sleep(spacing)
        with self.lock:
            entry = self.ctl.sent("estop", now, dash._stamp(), mids, before)
            return 200, f"emergency STOP sent {repeats} times", entry.as_dict(time.monotonic())

    def arm(self, on: bool) -> str:
        with self.lock:
            if not on:
                self.ctl.disarm("disarmed by the user")
                return ""
            return self.ctl.arm(time.monotonic())

    # -- recording, calibration, export ------------------------------------------

    def record(self, action: str) -> str:
        with self.lock:
            if action == "start":
                self.rec.start_recording()
            elif action == "stop":
                self.rec.stop_recording()
            else:
                return "action must be start or stop"
        return ""

    def calib_action(self, action: str, target: str = "", seconds: float = calib.CAPTURE_S) -> str:
        with self.lock:
            if action == "capture":
                if not self.src["live"]:
                    return "a capture needs a live source (MQTT, USB capture, or a followed log)"
                return self.cal.start(target, self.last_t if self.src["kind"] == "file" else _now_sod(), seconds)
            if action == "cancel":
                self.cal.cancel()
            elif action == "reset":
                self.cal.reset()
            elif action == "use_car":
                return self.cal.use_car_values()
            else:
                return "action must be capture, cancel, reset or use_car"
        return ""

    def table(self, after: Optional[int], before: Optional[int], limit: int) -> dict:
        with self.lock:
            self.rec.flush(self._clock_now())
            return self.rec.page(after, before, limit)

    def export_csv(self, what: str, scope: str) -> Tuple[str, str]:
        """(filename, CSV text) for the table ('rows') or the data-gap events ('gaps')."""
        stamp = time.strftime("%Y%m%d-%H%M%S")
        with self.lock:
            if what == "gaps":
                return f"robocar-data-gaps-{stamp}.csv", record.gaps_csv(self.health.gap_rows(self._clock_now()))
            self.rec.flush(None)
            return f"robocar-telemetry-{scope}-{stamp}.csv", self.rec.csv_text(scope)

    def _clock_now(self) -> Optional[float]:
        """'Now' in the line clock for a live source; None for a finished replay."""
        if not self.src["live"]:
            return None
        return self.last_t if self.src["kind"] == "file" else _now_sod()

    def _update_link(self) -> None:
        """The source as a whole: broker up, car ONLINE, lines still arriving (lock held)."""
        if not self.src["live"]:
            return
        age = None if self.last_rx is None else time.monotonic() - self.last_rx
        ok, why = True, ""
        if self.src["kind"] == "mqtt" and not self.src["mqtt_connected"]:
            ok, why = False, "the laptop is not connected to the broker"
        elif self.src["kind"] == "mqtt" and self.src["car_status"].startswith("OFFLINE"):
            ok, why = False, "the car is OFFLINE (the broker published its last will)"
        elif age is not None and age > record.DISCONNECT_S:
            what = "from the car" if self.src["kind"] == "mqtt" else f"in the {self.src['kind']} source"
            ok, why = False, f"no data {what} for {age:.0f} s"
        if ok != self.health.link[0] or (not ok and why != self.health.link[1]):
            self.health.set_link(ok, why, self._clock_now())

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            self._proc.terminate()
            self._proc = None
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:       # paho raises various errors on a dead link; stopping anyway
                pass
            self._client = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._save is not None:
            self._save.close()
            self._save = None
        with self.lock:
            self.src["running"] = False
            if self.src["mqtt_connected"]:
                self.src["mqtt_connected"] = False
        self._stop.clear()

    def _start_thread(self, fn, *args) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=fn, args=args, daemon=True)
        self._thread.start()

    def _follow_file(self, path: str, follow: bool) -> None:
        try:
            while follow and not os.path.exists(path):
                if self._stop.wait(0.2):
                    return
            with open(path, encoding="utf-8", errors="replace") as fh:
                while not self._stop.is_set():
                    line = fh.readline()
                    if line:
                        self.feed_line(line)
                    elif not follow:
                        break
                    else:
                        self._stop.wait(0.2)
        except OSError as exc:
            with self.lock:
                self.src["error"] = str(exc)
        finally:
            with self.lock:
                if not follow:
                    self.src["running"] = False

    # -- what the page reads ----------------------------------------------------

    def logs(self) -> List[dict]:
        found = []
        for d in [self.out_dir] + [os.path.join(self.repo, "testing", s, "logs")
                                   for s in sorted(os.listdir(os.path.join(self.repo, "testing")))
                                   if os.path.isdir(os.path.join(self.repo, "testing", s, "logs"))]:
            if not os.path.isdir(d):
                continue
            for name in os.listdir(d):
                p = os.path.join(d, name)
                if name.endswith(".txt") and os.path.isfile(p):
                    found.append((os.path.getmtime(p), p))
        found.sort(reverse=True)
        return [{"path": os.path.relpath(p, self.repo), "mtime": time.strftime("%d %b %H:%M", time.localtime(t))}
                for t, p in found[:300]]

    def state(self, since: int) -> dict:
        now = time.monotonic()
        with self.lock:
            src = dict(self.src)
            if src["live"] and self.last_rx is not None:
                src["data_age_s"] = round(now - self.last_rx, 1)
            if src["live"] and self.last_hb_rx is not None:
                src["hb_age_s"] = round(now - self.last_hb_rx, 1)
            clk = self._clock_now()
            self._update_link()
            self.health.evaluate(clk)
            self.rec.flush(clk)
            page = self.rec.page(limit=1)
            return {
                "source": src,
                "gen": self.gen,
                "seq": self.seq,
                "readings": self.readings(),
                "feed": {v: [t for s, t in q if s > since] for v, q in self.feed.items()},
                "console": [t for s, t in self.console if s > since],
                "app_test": self.ev.app_test, "s1_ran": self.ev.s1_ran,
                # Dashboard v2
                "health": self.health.snapshot(clk),
                "gaps": self.health.gap_rows(clk)[-200:],
                "ctl": self.ctl.snapshot(now),
                "calib": self.cal.snapshot(clk),
                "table": {"total": page["total"], "last_seq": page["last_seq"], "recording": page["recording"],
                          "latest": page["rows"][-1] if page["rows"] else None},
            }

    def check(self, kind: str, expect_cm: Optional[List[int]], expect_width_mm: Optional[int]) -> List[dict]:
        with self.lock:
            ev = self.ev
            if kind == "imu":
                vs = views.check_imu(ev)
            elif kind == "obstacle":
                vs = views.check_obstacle(ev, expect_cm, expect_width_mm)
            else:
                vs = views.check_mqtt(ev)
        return [{"name": v.name, "result": v.result, "detail": v.detail} for v in vs]

    def readings(self) -> Dict[str, List[dict]]:
        ev = self.ev
        latest = self.sessions["imu"].latest
        sens = latest.get("sensor")
        hb = latest.get("heartbeat")
        out: Dict[str, List[dict]] = {}

        def sensor_value(valid_key: str, text: str, kind: str, note: str, missing: str) -> dict:
            if sens is None:
                return ("", "N/A", "no sensor frames yet")
            f = sens["f"]
            if sens["sim"]:
                return (text.format(**f), "SIMULATED", "the S1 self-test wrote this (APP_TEST build)")
            if not f["flags_present"]:
                return (text.format(**f), "N/A", "validity unknown: firmware without the validity flags")
            if f[valid_key]:
                return (text.format(**f), kind, note)
            return ("", "N/A", missing)

        # IMU
        imu = []
        if ev.imu_not_ready:
            imu.append(_r("IMU chip", "NOT READY", "REPORTED"))
        elif ev.imu_status:
            imu.append(_r("IMU chip", "found" if ev.imu_status[-1]["found"] else "NOT FOUND", "REPORTED"))
        elif ev.imu_chip:
            imu.append(_r("IMU chip", ev.imu_chip[-1][len("[IMU] chip "):], "REPORTED"))
        else:
            imu.append(_r("IMU chip", "", "N/A", "no [IMU] chip line yet"))
        if ev.imu_status:
            imu.append(_r("|acceleration|", f"{ev.imu_status[-1]['accel_mag_mg']} mg", "MEASURED",
                          "filtered magnitude; 1 g = 1000 mg (this car reads about 920 at rest)"))
        else:
            imu.append(_r("|acceleration|", "", "N/A", "needs the [TEL] imu status line (stationary firmware)"))
        v = sensor_value("att_valid", "{pitch_deg:+.1f} deg", "DERIVED", "from the accelerometer, levelled at boot; "
                         "nose up = positive", "no attitude producer in this firmware")
        imu.append(_r("pitch", *v))
        v = sensor_value("att_valid", "{roll_deg:+.1f} deg", "DERIVED", "from the accelerometer, not levelled, "
                         "sign not yet checked on the car", "no attitude producer in this firmware")
        imu.append(_r("roll", *v))
        n_h = len(ev.humps) or ev.hump_lines
        last_h = ev.humps[-1] if ev.humps else None
        imu.append(_r("hump events", f"{n_h}" + (f" (last pitch {last_h['pitch_deg']:+.1f} deg)" if last_h else ""),
                      "REPORTED", "pitch-only detector: a tilt or hard braking also fires it"))
        if ev.humps and all(int(h["height_mm"]) == 0 for h in ev.humps):
            imu.append(_r("hump height", "", "N/A", "not measured: the firmware sends 0"))
        elif ev.humps:
            imu.append(_r("hump height", f"{last_h['height_mm']} mm", "DERIVED", "estimate"))
        else:
            imu.append(_r("hump height", "", "N/A", "no hump event"))
        if hb:
            names = [n for n in hb["f"]["fault_names"] if n in views.IMU_BITS]
            imu.append(_r("safety bits", ", ".join(names) or "none", "REPORTED", "from the newest heartbeat"))
        else:
            imu.append(_r("safety bits", "", "N/A", "no heartbeat yet"))
        imu.append(_r("lift events", ev.lifted_lines, "REPORTED", self.last_line.get("[SAFETY]", "")))
        out["imu"] = imu

        # Obstacle
        obs = []
        v = sensor_value("range_valid", "{range_cm} cm", "MEASURED", "ultrasonic, servo straight ahead (0 = no echo)",
                         "no ultrasonic producer in this firmware")
        obs.append(_r("forward range", *v))
        if ev.obs_status:
            d = ev.obs_status[-1]
            obs.append(_r("scanner sensors", "OK" if d["sensors_ok"] else "NOT OK", "REPORTED"))
            obs.append(_r("avoidance", f"state {d['avoid_state']}, active {int(d['avoid_active'])}", "REPORTED",
                          self.last_line.get("[S5] avoid", "")))
        else:
            obs.append(_r("scanner sensors", "", "N/A", "needs the [TEL] obs status line (stationary firmware)"))
        lo = latest.get("obstacle")
        prof = lo["f"] if lo else (ev.s5_obstacles[-1] if ev.s5_obstacles else None)
        sim = bool(lo and lo["sim"])
        if prof is None:
            obs.append(_r("last obstacle", "", "N/A", "none reported (needs something within 25 cm)"))
        else:
            k1, k2 = ("SIMULATED", "SIMULATED") if sim else ("MEASURED", "DERIVED")
            src = "" if lo else "from [S5] console lines"
            obs.append(_r("last obstacle", f"{prof['distance_cm']} cm @ {prof['bearing_deg']} deg", k1, src))
            obs.append(_r("obstacle width", f"{prof['width_mm']} mm", k2, "estimate; errs wide on purpose"))
            obs.append(_r("clearance L / R", f"{prof['clear_l_mm']} / {prof['clear_r_mm']} mm", k2, "estimate"))
        if hb:
            obs.append(_r("NO_ECHO", "set" if "NO_ECHO" in hb["f"]["fault_names"] else "clear", "REPORTED",
                          "set = nothing in range for a whole sweep"))
        out["obstacle"] = obs

        # Line following
        lf = []
        miss = "no line-following task in this firmware (subsystem 3 is a stub on main)"
        lf.append(_r("IR CENTRE raw", *sensor_value("ir_valid", "{ir_c_raw}", "MEASURED", "12-bit ADC", miss)))
        lf.append(_r("IR RIGHT raw", *sensor_value("ir_valid", "{ir_r_raw}", "MEASURED", "12-bit ADC", miss)))
        if self.lf_state:
            lf.append(_r("LF state", self.lf_state[0], "REPORTED",
                         f"from the LF task's state-change lines ({self.lf_state[1]} so far); SUSPENDED = avoidance"))
        else:
            lf.append(_r("LF state", "", "N/A", "no [LF] state-change line yet"))
        lf.append(_r("last LF log line", self.last_line.get("[LF]", ""), "REPORTED" if "[LF]" in self.last_line
                     else "N/A", "" if "[LF]" in self.last_line else "no [LF] lines (LF branch firmware only)"))
        out["lf"] = lf

        # Barcode
        bc = []
        lb = latest.get("barcode")
        if lb:
            bc.append(_r("last command", lb["f"]["command"], "SIMULATED" if lb["sim"] else "REPORTED",
                         "S1 self-test" if lb["sim"] else "decoded barcode"))
            bc.append(_r("barcodes", len(ev.barcodes), "REPORTED", f"{ev.barcodes_simulated} simulated not counted"))
        else:
            bc.append(_r("last command", "", "N/A", "no barcode task in this firmware yet"))
        bc.append(_r("last BC log line", self.last_line.get("[BC]", ""), "REPORTED" if "[BC]" in self.last_line
                     else "N/A", "" if "[BC]" in self.last_line else "no [BC] lines"))
        out["barcode"] = bc

        # Vehicle Controller
        vc = []
        if hb:
            vc.append(_r("VC state", hb["f"]["state"], "REPORTED", f"up {hb['f']['uptime_ms'] / 1000:.1f} s"))
        else:
            vc.append(_r("VC state", "", "N/A", "no heartbeat yet"))
        moving = sorted(set(ev.states) & set(views.MOVING_STATES))
        vc.append(_r("motors commanded", ("yes: " + ", ".join(moving)) if moving else "no", "REPORTED",
                     "any VC state that drives, seen so far"))
        mo = latest.get("motion")
        if mo:
            f = mo["f"]
            vc.append(_r("speed", f"{f['speed_cmps']:+.1f} cm/s", "MEASURED", "encoders"))
            vc.append(_r("distance", f"{f['distance_cm']} cm", "DERIVED", "from the encoders"))
            vc.append(_r("encoders L / R", f"{f['enc_l']} / {f['enc_r']}", "MEASURED"))
        else:
            vc.append(_r("speed", "", "N/A", "no motion frames yet"))
        lf_ = latest.get("fault")
        if lf_:
            vc.append(_r("last fault event", ", ".join(lf_["f"]["names"]) or "-",
                         "SIMULATED" if lf_["sim"] else "REPORTED"))
        out["vc"] = vc

        # MQTT
        mq = []
        mq.append(_r("car telemetry backend", ev.backend or "", "REPORTED" if ev.backend else "N/A",
                     "" if ev.backend else "no [TEL] backend line (it prints at boot)"))
        if ev.link_status:
            d = ev.link_status[-1]
            mq.append(_r("car link", d["link"], "REPORTED",
                         f"sent {d['tx']}, publish fails {d['txf']}, skipped offline {d['off']}, outbox dropped {d['od']}"))
        elif ev.link_moves:
            mq.append(_r("car link", ev.link_moves[-1], "REPORTED"))
        if self.src["kind"] == "mqtt":
            mq.append(_r("laptop to broker", "connected" if self.src["mqtt_connected"] else "NOT connected",
                         "MEASURED", "seen by this laptop"))
            mq.append(_r("car/status", self.src["car_status"] or "", "REPORTED" if self.src["car_status"] else "N/A",
                         "retained ONLINE, or the broker's last will"))
        mq.append(_r("heartbeats received", len(ev.heartbeats), "MEASURED", "counted by this laptop"))
        out["mqtt"] = mq
        return out


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    monitor: Monitor = None     # set by make_server

    def log_message(self, fmt, *args):     # quiet: the page polls twice a second
        pass

    def _json(self, code: int, obj) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _csv(self, filename: str, text: str) -> None:
        body = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _int(q, key: str) -> Optional[int]:
        v = q.get(key, [""])[0]
        return int(v) if v.strip().lstrip("-").isdigit() else None

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        m = self.monitor
        if u.path == "/":
            with open(PAGE, "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")     # an updated page shows at the next reload
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/api/state":
            self._json(200, m.state(int(q.get("since", ["0"])[0] or 0)))
        elif u.path == "/api/logs":
            self._json(200, m.logs())
        elif u.path == "/api/check":
            kind = q.get("kind", [""])[0]
            if kind not in ("imu", "obstacle", "mqtt"):
                self._json(400, {"error": "kind must be imu, obstacle or mqtt"})
                return
            try:
                cm = [int(x) for x in q["expect_cm"][0].split(",") if x.strip()] if q.get("expect_cm") else None
                w = int(q["expect_width_mm"][0]) if q.get("expect_width_mm", [""])[0] else None
            except ValueError:
                self._json(400, {"error": "expected distances and width must be whole numbers"})
                return
            self._json(200, m.check(kind, cm, w))
        elif u.path == "/api/table":
            self._json(200, m.table(self._int(q, "after"), self._int(q, "before"), self._int(q, "limit") or 200))
        elif u.path == "/api/export.csv":
            what = q.get("what", ["rows"])[0]
            scope = q.get("scope", ["all"])[0]
            if what not in ("rows", "gaps") or scope not in ("all", "recording"):
                self._json(400, {"error": "what must be rows or gaps; scope all or recording"})
                return
            self._csv(*m.export_csv(what, scope))
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            n = int(self.headers.get("Content-Length", "0"))
            req = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._json(400, {"error": "bad JSON"})
            return
        m = self.monitor
        if u.path == "/api/source":
            t = req.get("type")
            if t == "file":
                err = m.open_file(str(req.get("path", "")), bool(req.get("follow")))
            elif t == "usb":
                err = m.start_usb(int(req.get("seconds", 1800)))
            elif t == "mqtt":
                err = m.connect_mqtt(str(req.get("broker", "")).strip(), int(req.get("port", 1883)))
            elif t == "stop":
                m.stop()
                err = ""
            else:
                err = "type must be file, usb, mqtt or stop"
            self._json(400 if err else 200, {"error": err} if err else {"ok": True})
        elif u.path == "/api/cmd":
            code, msg, entry = m.send_cmd(str(req.get("cmd", "")))
            self._json(code, {"error": msg, "cmd": entry} if code != 200 else {"ok": msg, "cmd": entry})
        elif u.path == "/api/estop":
            code, msg, entry = m.estop()
            self._json(code, {"error": msg, "cmd": entry} if code != 200 else {"ok": msg, "cmd": entry})
        elif u.path == "/api/arm":
            err = m.arm(bool(req.get("armed")))
            self._json(409 if err else 200, {"error": err} if err else {"ok": True})
        elif u.path == "/api/record":
            err = m.record(str(req.get("action", "")))
            self._json(400 if err else 200, {"error": err} if err else {"ok": True})
        elif u.path == "/api/calib":
            try:
                seconds = float(req.get("seconds", calib.CAPTURE_S))
            except (TypeError, ValueError):
                self._json(400, {"error": "seconds must be a number"})
                return
            err = m.calib_action(str(req.get("action", "")), str(req.get("target", "")), seconds)
            self._json(400 if err else 200, {"error": err} if err else {"ok": True})
        else:
            self._json(404, {"error": "not found"})


def make_server(monitor: Monitor, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"monitor": monitor})
    srv = ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    return srv


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description="Robocar dashboard in the browser")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--host", default="127.0.0.1", help="127.0.0.1 (default) = this laptop only")
    p.add_argument("--no-browser", action="store_true")
    p.add_argument("--open", metavar="LOG", help="start with this log file")
    p.add_argument("--follow", action="store_true", help="with --open: keep reading as it grows")
    args = p.parse_args(argv)

    monitor = Monitor()
    srv = None
    for port in range(args.port, args.port + 10):
        try:
            srv = make_server(monitor, args.host, port)
            break
        except OSError:
            continue
    if srv is None:
        print(f"no free port in {args.port}..{args.port + 9}")
        return 1
    url = f"http://{'127.0.0.1' if args.host in ('0.0.0.0', '') else args.host}:{srv.server_address[1]}/"
    if args.open:
        err = monitor.open_file(args.open, args.follow)
        if err:
            print(err)
    print(f"Robocar dashboard: {url}   (Ctrl+C to quit)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
