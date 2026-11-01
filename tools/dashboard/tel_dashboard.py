#!/usr/bin/env python3
"""Robocar telemetry dashboard (Buddy 1, subsystem 1).  Runs on the laptop.

    listen      subscribe to car/#, decode every frame, track the run
    send        publish a remote command: start | stop | reset
    decode-log  decode "[TEL] <topic> <hex>" lines from a saved USB console log
                (console backend, TEL_BACKEND_WIFI=OFF): no broker needed
    decode      decode one frame given as hex

Examples:
    python tel_dashboard.py listen --broker 192.168.1.10
    python tel_dashboard.py send stop --broker 192.168.1.10
    python tel_dashboard.py decode-log ../../testing/subsystem1/logs/run.txt
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


def cmd_listen(args: argparse.Namespace) -> int:
    mqtt, client = _new_client("robocar-dashboard")
    summary = RunSummary()
    last_hb = [time.monotonic()]
    stale_reported = [False]

    def on_connect(c, userdata, flags, *rest):
        print(f"connected to {args.broker}:{args.port}; subscribing car/#")
        c.subscribe("car/#", qos=1)

    def on_message(c, userdata, msg):
        if msg.topic == codec.STATUS_TOPIC:
            state = "ONLINE" if msg.payload == b"1" else "OFFLINE (last will)"
            print(f"{time.strftime('%H:%M:%S')} status     car {state}")
            return
        if msg.topic.startswith("car/cmd/"):
            return      # our own commands echoed back
        if msg.topic.endswith("/heartbeat"):
            last_hb[0] = time.monotonic()
            stale_reported[0] = False
        _print_frame(msg.topic, bytes(msg.payload), summary, args.quiet)

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
    print(summary.text())
    return 0


def cmd_send(args: argparse.Namespace) -> int:
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


def cmd_decode_log(args: argparse.Namespace) -> int:
    summary = RunSummary()
    with open(args.file, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            parsed = codec.parse_log_line(line)
            if parsed is not None:
                _print_frame(parsed[0], parsed[1], summary, args.quiet)
    print(summary.text())
    return 0


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
    pl.set_defaults(func=cmd_listen)

    ps = sub.add_parser("send", help="publish a remote command")
    ps.add_argument("command", choices=sorted(codec.CMD_TYPE))
    ps.add_argument("--broker", required=True)
    ps.add_argument("--port", type=int, default=1883)
    ps.set_defaults(func=cmd_send)

    pd = sub.add_parser("decode-log", help="decode [TEL] lines from a console log")
    pd.add_argument("file")
    pd.add_argument("--quiet", action="store_true", help="hide motion/sensor frames")
    pd.set_defaults(func=cmd_decode_log)

    px = sub.add_parser("decode", help="decode one frame given as hex")
    px.add_argument("hex")
    px.set_defaults(func=cmd_decode)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
