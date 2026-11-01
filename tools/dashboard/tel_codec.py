"""Decoder/encoder for the robocar MQTT binary frames (Buddy 1, subsystem 1).

Mirror of src/common/data_repr.h section "MQTT binary frames" and
docs/DATA_REPRESENTATION.md section 8:

    frame = ver u8 | type u8 | len u8 | payload[len]
    payload fields are packed LSB first, little-endian (drep_bits_put)

Pure Python 3, standard library only, so it also runs where paho-mqtt is not
installed (decoding a saved serial log).  tools/dashboard/test_tel_decode.py
checks it against GOLDEN frames that tests/host/test_tel.c also checks against
the C encoders, so the two sides cannot drift apart silently (risk R3).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

WIRE_VER = 1
HDR_BYTES = 3

FT_TELEMETRY_MOTION = 1
FT_TELEMETRY_SENSOR = 2
FT_EVENT_BARCODE = 3
FT_EVENT_HUMP = 4
FT_EVENT_OBSTACLE = 5
FT_EVENT_FAULT = 6
FT_HEARTBEAT = 7
FT_CMD_START = 8
FT_CMD_STOP = 9
FT_CMD_RESET = 10

# Payload length per frame type (drep_frame_len).
FRAME_LEN = {1: 15, 2: 10, 3: 4, 4: 5, 5: 8, 6: 5, 7: 6, 8: 0, 9: 0, 10: 0}

FRAME_NAME = {
    1: "motion", 2: "sensor", 3: "barcode", 4: "hump", 5: "obstacle",
    6: "fault", 7: "heartbeat", 8: "cmd_start", 9: "cmd_stop", 10: "cmd_reset",
}

TOPIC = {
    1: "car/telemetry/motion", 2: "car/telemetry/sensor", 3: "car/telemetry/barcode",
    4: "car/telemetry/hump", 5: "car/telemetry/obstacle", 6: "car/telemetry/fault",
    7: "car/telemetry/heartbeat",
}
CMD_TOPIC = {"start": "car/cmd/start", "stop": "car/cmd/stop", "reset": "car/cmd/reset"}
CMD_TYPE = {"start": FT_CMD_START, "stop": FT_CMD_STOP, "reset": FT_CMD_RESET}
STATUS_TOPIC = "car/status"

# Vehicle Controller system FSM, 4 bits (app_program/subsystem2/subsystem2.h VC_SYS_*).
SYS_STATE = [
    "INITIALISING", "READY_WAITING", "OBSTACLE_AVOIDANCE", "SAFETY_FAULT_LATCHED",
    "NAV_LINE_TRACKING", "NAV_HUMP_SENSING", "NAV_TURN_LEFT", "NAV_GO_STRAIGHT",
    "NAV_U_TURN", "NAV_TURN_RIGHT",
]

# EVF_SAFETY bits b0..b9 (src/common/ipc_types.h).
SAFETY_BITS = [
    "COLLISION", "IMU_ALIVE", "FAULT_LATCHED", "STALL", "ENCODER",
    "IMU_I2C", "IMU_WDOG", "NO_ECHO", "WIFI_LOSS", "IR_SENSOR",
]

BARCODE_CMD = ["LEFT (A)", "RIGHT (B)", "STRAIGHT (C)", "U-TURN (D)"]

# Obstacle layout (data_repr.h DREP_OBS_*).
OBS_MM_PER_UNIT = 5
OBS_BEARING_BASE_DEG = 30
OBS_BEARING_STEP_DEG = 10


class FrameError(ValueError):
    """The bytes are not a valid frame (same reject rules as drep_frame_check)."""


class _Bits:
    """LSB-first little-endian bit reader (drep_bits_get)."""

    def __init__(self, data: bytes, start_bit: int = HDR_BYTES * 8) -> None:
        self._data = data
        self._pos = start_bit

    def u(self, bits: int) -> int:
        v = 0
        for i in range(bits):
            p = self._pos + i
            v |= ((self._data[p >> 3] >> (p & 7)) & 1) << i
        self._pos += bits
        return v

    def s(self, bits: int) -> int:
        v = self.u(bits)
        sign = 1 << (bits - 1)
        return (v ^ sign) - sign


def _bit_names(mask: int, names: List[str]) -> List[str]:
    return [n for i, n in enumerate(names) if mask & (1 << i)]


def check(data: bytes) -> int:
    """Return the frame type, or raise FrameError with the drep reject reason."""
    if len(data) < HDR_BYTES:
        raise FrameError("SHORT")
    if data[0] != WIRE_VER:
        raise FrameError("BAD_VERSION")
    t = data[1]
    if t not in FRAME_LEN:
        raise FrameError("UNKNOWN_TYPE")
    if data[2] != FRAME_LEN[t] or len(data) != HDR_BYTES + FRAME_LEN[t]:
        raise FrameError("BAD_LENGTH")
    return t


@dataclass
class Frame:
    type: int
    name: str
    fields: Dict[str, object] = field(default_factory=dict)

    def summary(self) -> str:
        f = self.fields
        if self.type == FT_TELEMETRY_MOTION:
            return (f"t={f['ts_ms']}ms v={f['speed_cmps']:+.1f}cm/s d={f['distance_cm']}cm "
                    f"encL={f['enc_l']} encR={f['enc_r']}")
        if self.type == FT_TELEMETRY_SENSOR:
            return (f"t={f['ts_ms']}ms IR_R={f['ir_r_raw']} IR_C={f['ir_c_raw']} range={f['range_cm']}cm "
                    f"pitch={f['pitch_deg']:+.1f} roll={f['roll_deg']:+.1f}")
        if self.type == FT_EVENT_BARCODE:
            return f"t={f['ts_ms']}ms {f['command']}"
        if self.type == FT_EVENT_HUMP:
            oor = " OOR" if f["oor"] else ""
            return f"t={f['ts_ms']}ms height={f['height_mm']}mm pitch={f['pitch_deg']:+.1f}{oor}"
        if self.type == FT_EVENT_OBSTACLE:
            if f["no_echo"]:
                return f"t={f['ts_ms']}ms no echo (bearing {f['bearing_deg']} deg)"
            return (f"t={f['ts_ms']}ms {f['distance_cm']}cm @ {f['bearing_deg']}deg width={f['width_mm']}mm "
                    f"clearL={f['clear_l_mm']}mm clearR={f['clear_r_mm']}mm seq={f['seq']}"
                    + (" OOR" if f["oor"] else ""))
        if self.type == FT_EVENT_FAULT:
            return f"t={f['ts_ms']}ms bits=0x{f['bits']:03x} {','.join(f['names']) or '-'}"
        if self.type == FT_HEARTBEAT:
            return (f"up={f['uptime_ms'] / 1000:.1f}s state={f['state']} "
                    f"faults={','.join(f['fault_names']) or '-'}")
        return ""


def decode(data: bytes) -> Frame:
    """Decode one frame.  Raises FrameError on any reject rule."""
    t = check(data)
    b = _Bits(data)
    f: Dict[str, object] = {}
    if t == FT_TELEMETRY_MOTION:
        f["ts_ms"] = b.u(24)
        f["speed_cmps"] = b.s(10) / 10.0
        f["distance_cm"] = b.u(16)
        f["enc_l"] = b.s(32)
        f["enc_r"] = b.s(32)
    elif t == FT_TELEMETRY_SENSOR:
        f["ts_ms"] = b.u(24)
        f["ir_r_raw"] = b.u(12)
        f["ir_c_raw"] = b.u(12)
        f["range_cm"] = b.u(9)
        f["pitch_deg"] = b.s(8) / 2.0
        f["roll_deg"] = b.s(8) / 2.0
    elif t == FT_EVENT_BARCODE:
        f["ts_ms"] = b.u(24)
        f["cmd"] = b.u(2)
        f["command"] = BARCODE_CMD[f["cmd"]]
    elif t == FT_EVENT_HUMP:
        f["ts_ms"] = b.u(24)
        f["height_mm"] = b.u(7)
        f["pitch_deg"] = b.s(8) / 2.0
        f["oor"] = bool(b.u(1))
    elif t == FT_EVENT_OBSTACLE:
        f["ts_ms"] = b.u(24)
        f["distance_cm"] = b.u(9)
        bearing_idx = b.u(4)
        f["bearing_idx"] = bearing_idx
        f["bearing_deg"] = OBS_BEARING_BASE_DEG + OBS_BEARING_STEP_DEG * bearing_idx
        f["width_mm"] = b.u(7) * OBS_MM_PER_UNIT
        f["clear_l_mm"] = b.u(7) * OBS_MM_PER_UNIT
        f["seq"] = b.u(2)
        f["oor"] = bool(b.u(1))
        f["clear_r_mm"] = b.u(7) * OBS_MM_PER_UNIT
        f["no_echo"] = f["distance_cm"] == 0
    elif t == FT_EVENT_FAULT:
        f["ts_ms"] = b.u(24)
        f["bits"] = b.u(10)
        f["names"] = _bit_names(f["bits"], SAFETY_BITS)
    elif t == FT_HEARTBEAT:
        f["uptime_ms"] = b.u(32)
        s = b.u(4)
        f["state_id"] = s
        f["state"] = SYS_STATE[s] if s < len(SYS_STATE) else f"?{s}"
        f["fault_bits"] = b.u(10)
        f["fault_names"] = _bit_names(f["fault_bits"], SAFETY_BITS)
    return Frame(t, FRAME_NAME[t], f)


def encode_cmd(cmd: str) -> Tuple[str, bytes]:
    """(topic, frame) for a remote command: 'start', 'stop' or 'reset'."""
    t = CMD_TYPE[cmd]
    return CMD_TOPIC[cmd], bytes([WIRE_VER, t, 0])


_LOG_LINE = re.compile(r"\[TEL\]\s+(car/\S+)\s+([0-9a-fA-F]+)\s*$")


def parse_log_line(line: str) -> Optional[Tuple[str, bytes]]:
    """(topic, frame bytes) from a console-backend line '[TEL] <topic> <hex>', else None."""
    m = _LOG_LINE.search(line.strip())
    if not m:
        return None
    return m.group(1), bytes.fromhex(m.group(2))
