#!/usr/bin/env python3
"""Robocar telemetry dashboard (Buddy 1, subsystem 1).  Runs on the laptop.

    listen      subscribe to car/#, decode every frame, track the run
    send        publish a remote command: start | stop | reset
    decode-log  decode "[TEL] <topic> <hex>" lines from a saved USB console log
                (console backend, TEL_BACKEND_WIFI=OFF): no broker needed
    decode      decode one frame given as hex
    check       judge a subsystem test from a saved log: imu | obstacle | mqtt
                (PASS only where the log holds the supporting measurement)

Subsystem views (dashboard integration): --view imu|obstacle|barcode|lf|vc|mqtt|all
on listen and decode-log labels every value MEASURED / DERIVED / REPORTED /
SIMULATED / N/A (tel_views.py).  decode-log --follow watches a log that
tools/serial_capture.ps1 is still writing (live USB view).

Examples:
    python tel_dashboard.py listen --broker 192.168.1.10
    python tel_dashboard.py listen --broker 192.168.1.10 --view imu --save imu_mqtt.txt
    python tel_dashboard.py send stop --broker 192.168.1.10
    python tel_dashboard.py send start --broker 192.168.1.10 --allow-motion
    python tel_dashboard.py decode-log ../../testing/subsystem1/logs/run.txt
    python tel_dashboard.py decode-log ../../build/imu-test.txt --view imu --follow
    python tel_dashboard.py check imu ../../build/imu-test.txt
    python tel_dashboard.py check obstacle ../../build/obs-test.txt --expect-cm 20,50 --expect-width-mm 100
    python tel_dashboard.py decode 010706efbeaddef93f

listen/send need paho-mqtt (pip install paho-mqtt; 1.x and 2.x both work).
Frame format: app_program/subsystem1/MQTT.md and tel_codec.py.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import tel_codec as codec
import tel_views as views

HEARTBEAT_STALE_S = 3.0     # 3 missed 1 s heartbeats = car silent


@dataclass
class RunSummary:
    """What the operator wants at the end of a run (mission requirement 5: peak hump)."""

    frames: Dict[str, int] = field(default_factory=dict)
    rejects: int = 0
    peak_hump_mm: int = -1
    barcodes: list = field(default_factory=list)
    obstacles: int = 0
    last_state: str = "?"
    last_distance_cm: int = 0
    faults_seen: int = 0

    def add(self, fr: codec.Frame) -> None:
        self.frames[fr.name] = self.frames.get(fr.name, 0) + 1
        f = fr.fields
        if fr.type == codec.FT_EVENT_HUMP:
            self.peak_hump_mm = max(self.peak_hump_mm, int(f["height_mm"]))
        elif fr.type == codec.FT_EVENT_BARCODE:
            self.barcodes.append(f["command"])
        elif fr.type == codec.FT_EVENT_OBSTACLE:
            self.obstacles += 1
        elif fr.type == codec.FT_EVENT_FAULT:
            self.faults_seen += 1
        elif fr.type == codec.FT_HEARTBEAT:
            self.last_state = str(f["state"])
        elif fr.type == codec.FT_TELEMETRY_MOTION:
            self.last_distance_cm = int(f["distance_cm"])

    def text(self) -> str:
        counts = ", ".join(f"{k}={v}" for k, v in sorted(self.frames.items())) or "none"
        peak = f"{self.peak_hump_mm} mm" if self.peak_hump_mm >= 0 else "no hump reported"
        if self.peak_hump_mm == 0:     # dashboard integration: 0 = the firmware sent no height
            peak = f"height not measured ({self.frames.get('hump', 0)} hump events, every height 0)"
        return (
            "---- run summary ----\n"
            f"frames: {counts}; rejected: {self.rejects}\n"
            f"PEAK HUMP: {peak}\n"
            f"barcodes: {', '.join(self.barcodes) or 'none'}\n"
            f"obstacles: {self.obstacles}; fault events: {self.faults_seen}\n"
            f"last state: {self.last_state}; distance travelled: {self.last_distance_cm} cm"
        )


def _print_frame(topic: str, data: bytes, summary: RunSummary, quiet_periodic: bool) -> None:
    try:
        fr = codec.decode(data)
    except codec.FrameError as exc:
        summary.rejects += 1
        print(f"{time.strftime('%H:%M:%S')} {topic:<26} REJECT {exc} ({data.hex()})")
        return
    expected = codec.TOPIC.get(fr.type)
    if expected is not None and expected != topic:
        print(f"{time.strftime('%H:%M:%S')} {topic:<26} WARNING frame type {fr.name} on wrong topic")
    summary.add(fr)
    if quiet_periodic and fr.type in (codec.FT_TELEMETRY_MOTION, codec.FT_TELEMETRY_SENSOR):
        return
    print(f"{time.strftime('%H:%M:%S')} {fr.name:<10} {fr.summary()}")


def _new_client(client_id: str):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("paho-mqtt is not installed: pip install paho-mqtt")
    try:
        return mqtt, mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:      # paho-mqtt 1.x
        return mqtt, mqtt.Client(client_id=client_id)


def _stamp() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"


def cmd_listen(args: argparse.Namespace) -> int:
    mqtt, client = _new_client("robocar-dashboard")
    summary = RunSummary()
    last_hb = [time.monotonic()]
    stale_reported = [False]
    session = views.Session(args.view) if args.view else None
    save = open(args.save, "w", encoding="utf-8", buffering=1) if args.save else None
    if save:
        # Same line format as tools/serial_capture.ps1, so decode-log and check read it.
        save.write(f"{_stamp()}  [DASH] source=mqtt broker={args.broker}:{args.port}\n")

    def on_connect(c, userdata, flags, *rest):
        print(f"connected to {args.broker}:{args.port}; subscribing car/#")
        c.subscribe("car/#", qos=1)

    def on_message(c, userdata, msg):
        if msg.topic == codec.STATUS_TOPIC:
            state = "ONLINE" if msg.payload == b"1" else "OFFLINE (last will)"
            print(f"{time.strftime('%H:%M:%S')} status     car {state}")
            if save:
                save.write(f"{_stamp()}  [DASH] car/status {state}\n")
            return
        if msg.topic.startswith("car/cmd/"):
            return      # our own commands echoed back
        if msg.topic.endswith("/heartbeat"):
            last_hb[0] = time.monotonic()
            stale_reported[0] = False
        data = bytes(msg.payload)
        if save:
            save.write(f"{_stamp()}  [TEL] {msg.topic} {data.hex()}\n")
        if session is None:
            _print_frame(msg.topic, data, summary, args.quiet)
            return
        try:
            fr = codec.decode(data)
        except codec.FrameError as exc:
            summary.rejects += 1
            print(f"{time.strftime('%H:%M:%S')} REJECT {exc} ({data.hex()})")
            return
        summary.add(fr)
        for line in session.feed_frame(fr):
            print(f"{time.strftime('%H:%M:%S')} {line}")

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()
    try:
        while True:
            time.sleep(0.5)
            if not stale_reported[0] and time.monotonic() - last_hb[0] > HEARTBEAT_STALE_S:
                print(f"{time.strftime('%H:%M:%S')} WARNING no heartbeat for {HEARTBEAT_STALE_S:.0f} s")
                stale_reported[0] = True
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()
        if save:
            save.close()
    print(summary.text())
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    if args.command == "start" and not args.allow_motion:
        # Dashboard integration: START drives the motors (VC -> NAV_LINE_TRACKING at
        # cruise speed; with no line-following task on main it drives straight).
        print("refused: 'start' drives the motors. Stationary sensor tests never need it.\n"
              "Car on the track, area clear, someone ready to catch it: add --allow-motion")
        return 2
    mqtt, client = _new_client("robocar-dashboard-cmd")
    topic, frame = codec.encode_cmd(args.command)
    client.connect(args.broker, args.port, keepalive=30)
    client.loop_start()
    info = client.publish(topic, frame, qos=1)
    info.wait_for_publish(timeout=5)
    client.loop_stop()
    client.disconnect()
    print(f"sent {args.command}: {topic} {frame.hex()}")
    return 0


def _log_lines(path: str, follow: bool):
    """Lines of a log; with follow, keep waiting for new lines until Ctrl+C."""
    if follow:
        import os
        while not os.path.exists(path):     # the capture has not created it yet
            time.sleep(0.2)
    with open(path, encoding="utf-8", errors="replace") as fh:
        while True:
            line = fh.readline()
            if line:
                yield line
            elif not follow:
                return
            else:
                time.sleep(0.2)


def cmd_decode_log(args: argparse.Namespace) -> int:
    summary = RunSummary()
    session = views.Session(args.view) if args.view else None
    try:
        for line in _log_lines(args.file, args.follow):
            parsed = codec.parse_log_line(line)
            if session is not None:
                if parsed is not None:
                    try:
                        summary.add(codec.decode(parsed[1]))
                    except codec.FrameError:
                        summary.rejects += 1
                stamp = line[:12] if views._wall_s(line) is not None else ""
                for out in session.feed_line(line):
                    print(f"{stamp} {out}" if stamp else out)
            elif parsed is not None:
                _print_frame(parsed[0], parsed[1], summary, args.quiet)
    except KeyboardInterrupt:
        pass
    print(summary.text())
    return 0


def cmd_check(args: argparse.Namespace) -> int:
    session = views.Session(None)
    for line in _log_lines(args.file, False):
        session.feed_line(line)
    ev = session.ev
    if args.subsystem == "imu":
        verdicts = views.check_imu(ev)
    elif args.subsystem == "obstacle":
        expect = [int(x) for x in args.expect_cm.split(",")] if args.expect_cm else None
        verdicts = views.check_obstacle(ev, expect, args.expect_width_mm)
    else:
        verdicts = views.check_mqtt(ev)
    print(views.format_verdicts(f"{args.subsystem} check: {args.file}", verdicts))
    return 1 if any(v.result == "FAIL" for v in verdicts) else 0


def cmd_decode(args: argparse.Namespace) -> int:
    try:
        fr = codec.decode(bytes.fromhex(args.hex))
    except (ValueError, codec.FrameError) as exc:
        print(f"REJECT {exc}")
        return 1
    print(f"{fr.name}: {fr.summary()}")
    for k, v in fr.fields.items():
        print(f"  {k} = {v}")
    return 0


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    pl = sub.add_parser("listen", help="subscribe to car/# and decode")
    pl.add_argument("--broker", required=True)
    pl.add_argument("--port", type=int, default=1883)
    pl.add_argument("--quiet", action="store_true", help="hide the 200 ms motion/sensor frames")
    pl.add_argument("--view", choices=views.VIEWS, help="one subsystem's data, labelled by source")
    pl.add_argument("--save", metavar="FILE", help="also write every frame to FILE (decode-log/check read it)")
    pl.set_defaults(func=cmd_listen)

    ps = sub.add_parser("send", help="publish a remote command")
    ps.add_argument("command", choices=sorted(codec.CMD_TYPE))
    ps.add_argument("--broker", required=True)
    ps.add_argument("--port", type=int, default=1883)
    ps.add_argument("--allow-motion", action="store_true", help="required for start: it drives the motors")
    ps.set_defaults(func=cmd_send)

    pd = sub.add_parser("decode-log", help="decode [TEL] lines from a console log")
    pd.add_argument("file")
    pd.add_argument("--quiet", action="store_true", help="hide motion/sensor frames")
    pd.add_argument("--view", choices=views.VIEWS, help="one subsystem's data and console lines, labelled")
    pd.add_argument("--follow", action="store_true", help="keep reading as the log grows (Ctrl+C stops)")
    pd.set_defaults(func=cmd_decode_log)

    pc = sub.add_parser("check", help="judge a subsystem test from a saved log")
    pc.add_argument("subsystem", choices=("imu", "obstacle", "mqtt"))
    pc.add_argument("file")
    pc.add_argument("--expect-cm", help="obstacle: tape-measured box distances, e.g. 20,50")
    pc.add_argument("--expect-width-mm", type=int, help="obstacle: the box's real width")
    pc.set_defaults(func=cmd_check)

    px = sub.add_parser("decode", help="decode one frame given as hex")
    px.add_argument("hex")
    px.set_defaults(func=cmd_decode)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
