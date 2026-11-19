"""Telemetry recording, update rates and sensor health (Buddy 1, dashboard v2).

Fed one decoded frame at a time by tel_web.Monitor (the same frames tel_views
labels), with the time of the line it came from:

    Recorder   one row per Telemetry wake.  task_tel.c publish_periodic sends
               the motion frame and then the sensor frame in the same wake, so
               a row holds speed next to the IR raws, pitch/roll and range.
               Rows are kept for review and written out as CSV.
    Health     per sensor: VALID, STALE, INVALID, UNAVAILABLE or DISCONNECTED,
               the measured update rate, and a table of every gap (when a
               sensor stopped being VALID, why, and when it came back).

Times are seconds of day from the line stamp ("HH:MM:SS.fff  "), the clock both
the USB capture and the MQTT source write; they are unwrapped across midnight.
Update rates come from the car's own frame times (ts_ms, uptime_ms), so
network jitter does not show in them.

What "STALE" can and cannot see: the sensor frame repeats the newest value a
producer wrote (IF-12 snapshot), so a producer that stops writing still looks
fresh.  STALE therefore means "frames stopped arriving", plus, for the IR
raws only, "the value has not moved for a while" (a live ADC always jitters
by a few counts).  Standard library only.
"""

from __future__ import annotations

import csv
import io
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import tel_calib as calib
import tel_codec as codec

VALID, STALE, INVALID, UNAVAILABLE, DISCONNECTED = "VALID", "STALE", "INVALID", "UNAVAILABLE", "DISCONNECTED"
STATES = (VALID, STALE, INVALID, UNAVAILABLE, DISCONNECTED)

SENSORS = ("ir_c", "ir_r", "ultrasonic", "imu", "motion", "heartbeat")
LABEL = {
    "ir_c": "IR line sensor, CENTRE", "ir_r": "IR line sensor, RIGHT", "ultrasonic": "Ultrasonic range",
    "imu": "IMU pitch / roll", "motion": "Encoders and speed", "heartbeat": "Car heartbeat",
}
OWNER = {"ir_c": "Buddy 3", "ir_r": "Buddy 3", "ultrasonic": "Buddy 5", "imu": "Buddy 4",
         "motion": "Buddy 2", "heartbeat": "Buddy 1"}

MAX_ROWS = 36000            # 2 h at 5 rows/s
MAX_GAPS = 500
RATE_WINDOW = 25            # samples per rate estimate
STALE_FACTOR = 3.0          # STALE after this many missed periods
STALE_MIN_S = {"heartbeat": 3.0}        # tel_dashboard HEARTBEAT_STALE_S
STALE_MIN_DEFAULT_S = 1.0
STALE_UNKNOWN_S = 6.0       # until two intervals have been seen
IR_FROZEN_FRAMES = 25       # identical IR raw for 25 frames (5 s at 5 Hz): producer stopped?
DISCONNECT_S = 3.0          # live source: no line at all for this long
PAIR_S = 0.15               # motion and sensor frame of the same wake
TS_WRAP_MS = 1 << 24        # 24-bit frame timestamps
# Which sensors share one car clock (one frame type).
_CLOCK_TRACKS = {"sensor": ("ir_c", "ir_r", "ultrasonic", "imu"), "motion": ("motion",), "heartbeat": ("heartbeat",)}

SPEED_FIELD_MAX_CMPS = 51.1     # 10-bit, x10 cm/s (data_repr.h)
ATT_CLAMP_DEG = 45.0            # pitch/roll clamp (ARCHITECTURE.md D8)
RANGE_CLAMP_CM = 400            # tel_set_range_cm clamp (S1-U07)


def clock(t: Optional[float]) -> str:
    """'HH:MM:SS.mmm' for seconds of day (unwrapped values fold back into one day)."""
    if t is None:
        return ""
    t = t % 86400.0
    h, rem = divmod(t, 3600.0)
    m, s = divmod(rem, 60.0)
    return f"{int(h):02d}:{int(m):02d}:{s:06.3f}"


class _Unwrap:
    """Seconds of day -> a monotonic seconds value across midnight."""

    def __init__(self) -> None:
        self.offset = 0.0
        self.last: Optional[float] = None

    def __call__(self, t: Optional[float]) -> Optional[float]:
        if t is None:
            return self.last
        v = t + self.offset
        if self.last is not None and v < self.last - 43200.0:
            self.offset += 86400.0
            v += 86400.0
        self.last = max(v, self.last) if self.last is not None else v
        return v


# ---------------------------------------------------------------------------
# Per-sensor classification of one frame
# ---------------------------------------------------------------------------

def classify_sensor_frame(f: dict, limits: Dict[str, int]) -> Dict[str, Tuple[str, str, str]]:
    """{sensor: (state, reason, value text)} for the sensors a sensor frame carries."""
    out: Dict[str, Tuple[str, str, str]] = {}
    lo, hi = limits["ADC_STUCK_LOW_RAW"], limits["ADC_STUCK_HIGH_RAW"]
    flags = bool(f["flags_present"])
    no_flag = "validity unknown: firmware before the 19 Nov validity flags" if not flags else ""
    for key, raw_key in (("ir_c", "ir_c_raw"), ("ir_r", "ir_r_raw")):
        raw = int(f[raw_key])
        if not flags:
            out[key] = (UNAVAILABLE, no_flag, str(raw))
        elif not f["ir_valid"]:
            out[key] = (UNAVAILABLE, "no producer has written it (line-following task, IF-12)", "")
        elif raw <= lo:
            out[key] = (INVALID, f"raw {raw} <= {lo}: at the bottom of the ADC (shorted or unplugged?)", str(raw))
        elif raw >= hi:
            out[key] = (INVALID, f"raw {raw} >= {hi}: at the top of the ADC (open input?)", str(raw))
        else:
            out[key] = (VALID, "", str(raw))
    rng = int(f["range_cm"])
    if not flags:
        out["ultrasonic"] = (UNAVAILABLE, no_flag, f"{rng} cm")
    elif not f["range_valid"]:
        out["ultrasonic"] = (UNAVAILABLE, "no producer has written it (obstacle scan task, IF-12)", "")
    elif rng >= RANGE_CLAMP_CM:
        out["ultrasonic"] = (INVALID, f"{rng} cm: at the {RANGE_CLAMP_CM} cm clamp (beyond the sensor's range)",
                             f"{rng} cm")
    else:
        out["ultrasonic"] = (VALID, "no echo (nothing in range)" if rng == 0 else "", "no echo" if rng == 0 else f"{rng} cm")
    p, r = float(f["pitch_deg"]), float(f["roll_deg"])
    att = f"pitch {p:+.1f}, roll {r:+.1f} deg"
    if not flags:
        out["imu"] = (UNAVAILABLE, no_flag, att)
    elif not f["att_valid"]:
        out["imu"] = (UNAVAILABLE, "no producer has written it (IMU task, IF-12)", "")
    elif abs(p) >= ATT_CLAMP_DEG or abs(r) >= ATT_CLAMP_DEG:
        out["imu"] = (INVALID, f"at the +-{ATT_CLAMP_DEG:g} deg clamp (saturated)", att)
    else:
        out["imu"] = (VALID, "", att)
    return out


def classify_motion(f: dict) -> Tuple[str, str, str]:
    v = float(f["speed_cmps"])
    text = f"{v:+.1f} cm/s, {f['distance_cm']} cm"
    if abs(v) >= SPEED_FIELD_MAX_CMPS:
        return INVALID, f"speed at the +-{SPEED_FIELD_MAX_CMPS} cm/s field limit (saturated)", text
    return VALID, "", text


def classify_heartbeat(f: dict) -> Tuple[str, str, str]:
    if str(f["state"]).startswith("?"):
        return INVALID, f"unknown VC state {f['state_id']}", str(f["state"])
    return VALID, "", str(f["state"])


# ---------------------------------------------------------------------------
# Health
# ---------------------------------------------------------------------------

@dataclass
class _Track:
    key: str
    state: str = UNAVAILABLE
    reason: str = "no data yet"
    value: str = ""
    simulated: bool = False
    since: Optional[float] = None
    last_seen: Optional[float] = None       # laptop time of the newest frame carrying it
    last_valid: Optional[float] = None
    arrivals: Deque[float] = field(default_factory=lambda: deque(maxlen=RATE_WINDOW))   # laptop time
    car_times: Deque[float] = field(default_factory=lambda: deque(maxlen=RATE_WINDOW))  # car clock, s
    same_raw: int = 0
    last_raw: Optional[int] = None
    gap: Optional[dict] = None              # the open gap, if any

    def stale_after(self) -> float:
        """Seconds without an update before the sensor is STALE: a few of its usual periods.

        The usual period is the upper quartile of recent intervals, not the median, so the
        extra heartbeats the car sends on each state change (task_tel.c) do not make a
        5 s USB-console cadence look like a 1 s one.
        """
        floor = STALE_MIN_S.get(self.key, STALE_MIN_DEFAULT_S)
        if len(self.arrivals) < 3:
            return max(floor, STALE_UNKNOWN_S)
        ts = list(self.arrivals)
        gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
        period = gaps[(3 * (len(gaps) - 1)) // 4]
        return max(floor, STALE_FACTOR * period)

    def rate_hz(self) -> Optional[float]:
        ts = list(self.car_times) if len(self.car_times) >= 2 else list(self.arrivals)
        if len(ts) < 2 or ts[-1] <= ts[0]:
            return None
        return (len(ts) - 1) / (ts[-1] - ts[0])


class Health:
    """Sensor states and the data-gap table for one source."""

    def __init__(self, limits: Optional[Dict[str, int]] = None) -> None:
        rules = calib.load_rules()
        self.limits = limits or {k: int(v["value"]) for k, v in rules.items()}
        self.tracks = {k: _Track(k) for k in SENSORS}
        self.gaps: Deque[dict] = deque(maxlen=MAX_GAPS)
        self.gap_seq = 0
        self._t = _Unwrap()
        self.link: Tuple[bool, str] = (True, "")
        self.car_ts: Dict[str, Tuple[float, float]] = {}     # sensor -> (last car seconds, wrap offset)

    # -- gaps -------------------------------------------------------------------

    def _open(self, tr: _Track, state: str, reason: str, start: Optional[float]) -> None:
        self.gap_seq += 1
        tr.gap = {"id": self.gap_seq, "sensor": tr.key, "label": LABEL[tr.key], "state": state,
                  "reason": reason, "start": start, "end": None}
        self.gaps.append(tr.gap)

    def _close(self, tr: _Track, end: Optional[float]) -> None:
        if tr.gap is not None:
            tr.gap["end"] = end
            tr.gap = None

    def _move(self, tr: _Track, state: str, reason: str, t: Optional[float]) -> None:
        if state == tr.state and reason == tr.reason:
            return
        if state != tr.state:
            self._close(tr, t)
            if state != VALID and tr.last_seen is not None:     # never-seen sensors are not "gaps"
                self._open(tr, state, reason, t)
            tr.since = t
        elif tr.gap is not None:
            tr.gap["reason"] = reason
        tr.state, tr.reason = state, reason

    # -- input ------------------------------------------------------------------

    def _car_seconds(self, key: str, ms: int, wrap_ms: Optional[int]) -> float:
        last = self.car_ts.get(key)
        offset = last[1] if last else 0.0
        s = ms / 1000.0 + offset
        if last is not None and wrap_ms is not None and s < last[0] - (wrap_ms / 2000.0):
            offset += wrap_ms / 1000.0
            s += wrap_ms / 1000.0
        if last is not None and s < last[0]:        # the car restarted: a new clock
            for k in _CLOCK_TRACKS[key]:
                self.tracks[k].car_times.clear()
        self.car_ts[key] = (s, offset)
        return s

    def _seen(self, key: str, t: Optional[float], car_s: Optional[float], state: str, reason: str,
              value: str, simulated: bool) -> None:
        tr = self.tracks[key]
        if t is not None and tr.state == VALID and tr.last_valid is not None and state == VALID:
            if t - tr.last_valid > tr.stale_after():     # a gap that is over already (replayed logs)
                self._close(tr, tr.last_valid)
                self._open(tr, STALE, f"no update for {t - tr.last_valid:.1f} s", tr.last_valid)
                self._close(tr, t)
        if tr.state == DISCONNECTED and not self.link[0]:
            tr.last_seen = t
            return
        tr.last_seen = t
        if state != UNAVAILABLE:                    # rates count frames that carry a value
            if t is not None:
                tr.arrivals.append(t)
            if car_s is not None:
                tr.car_times.append(car_s)
        tr.value, tr.simulated = value, simulated
        if state == VALID:
            tr.last_valid = t
        self._move(tr, state, reason, t)

    def on_frame(self, fr: codec.Frame, t: Optional[float], simulated: bool = False) -> None:
        t = self._t(t)
        f = fr.fields
        if fr.type == codec.FT_TELEMETRY_SENSOR:
            car_s = self._car_seconds("sensor", int(f["ts_ms"]), TS_WRAP_MS)
            for key, (state, reason, value) in classify_sensor_frame(f, self.limits).items():
                if key in ("ir_c", "ir_r") and state == VALID and not simulated:
                    state, reason = self._frozen(key, int(value), state, reason)
                self._seen(key, t, car_s, state, reason, value, simulated)
        elif fr.type == codec.FT_TELEMETRY_MOTION:
            car_s = self._car_seconds("motion", int(f["ts_ms"]), TS_WRAP_MS)
            self._seen("motion", t, car_s, *classify_motion(f), simulated=False)
        elif fr.type == codec.FT_HEARTBEAT:
            car_s = self._car_seconds("heartbeat", int(f["uptime_ms"]), None)
            self._seen("heartbeat", t, car_s, *classify_heartbeat(f), simulated=False)

    def _frozen(self, key: str, raw: int, state: str, reason: str) -> Tuple[str, str]:
        tr = self.tracks[key]
        tr.same_raw = tr.same_raw + 1 if raw == tr.last_raw else 0
        tr.last_raw = raw
        if tr.same_raw >= IR_FROZEN_FRAMES:
            return STALE, f"raw {raw} unchanged for {tr.same_raw + 1} frames: producer stopped?"
        return state, reason

    def set_link(self, ok: bool, reason: str, t: Optional[float]) -> None:
        """The whole source: MQTT up and car ONLINE (or the capture still producing lines)."""
        t = self._t(t)
        was_ok = self.link[0]
        self.link = (ok, reason)
        for tr in self.tracks.values():
            if not ok and tr.last_seen is not None:
                # The gap began with the sensor's last frame, not when the loss was noticed.
                self._move(tr, DISCONNECTED, reason, tr.last_seen if tr.state != DISCONNECTED else t)
            elif ok and not was_ok and tr.state == DISCONNECTED:
                self._move(tr, STALE, "waiting for data after the link came back", t)

    def evaluate(self, now: Optional[float]) -> None:
        """Live sources: marks sensors STALE whose frames stopped.  'now' in the line clock."""
        if now is None or not self.link[0]:
            return
        now = self._t(now)
        for tr in self.tracks.values():
            if tr.state == VALID and tr.last_valid is not None and now - tr.last_valid > tr.stale_after():
                self._move(tr, STALE, f"no update for {now - tr.last_valid:.1f} s", tr.last_valid)

    # -- output -----------------------------------------------------------------

    def snapshot(self, now: Optional[float]) -> List[dict]:
        out = []
        now_u = self._t.last if now is None else max(self._t.last or now, now + self._t.offset)
        for key in SENSORS:
            tr = self.tracks[key]
            rate = tr.rate_hz()
            out.append({"key": key, "label": LABEL[key], "owner": OWNER[key], "state": tr.state,
                        "reason": tr.reason, "value": tr.value, "simulated": tr.simulated,
                        "age_s": None if tr.last_seen is None or now_u is None else round(max(0.0, now_u - tr.last_seen), 1),
                        "rate_hz": None if rate is None else round(rate, 2)})
        return out

    def gap_rows(self, now: Optional[float]) -> List[dict]:
        rows = []
        now_u = self._t.last if now is None else now + self._t.offset
        for g in self.gaps:
            end = g["end"]
            dur = None
            if g["start"] is not None:
                ref = end if end is not None else now_u
                dur = None if ref is None else round(max(0.0, ref - g["start"]), 1)
            rows.append({"id": g["id"], "sensor": g["sensor"], "label": g["label"], "state": g["state"],
                         "reason": g["reason"], "start": clock(g["start"]), "end": clock(end) if end is not None else "",
                         "open": end is None, "duration_s": dur})
        return rows


# ---------------------------------------------------------------------------
# Recorder
# ---------------------------------------------------------------------------

CSV_COLUMNS = ["seq", "time", "car_ms", "ir_centre_raw", "ir_right_raw", "pitch_deg", "roll_deg",
               "speed_cmps", "distance_cm", "range_cm", "vc_state", "ir_centre", "ir_right",
               "ultrasonic", "imu", "simulated"]


class Recorder:
    """Rows for the live and historical table, and the CSV export."""

    def __init__(self) -> None:
        self.rows: Deque[dict] = deque(maxlen=MAX_ROWS)
        self.seq = 0
        self.state = ""
        self._pending: Optional[Tuple[float, dict]] = None     # motion frame waiting for its sensor frame
        self._t = _Unwrap()
        self.rec_from: Optional[int] = None
        self.rec_to: Optional[int] = None
        self.limits = {k: int(v["value"]) for k, v in calib.load_rules().items()}

    # -- recording --------------------------------------------------------------

    def start_recording(self) -> None:
        self.flush(None)
        self.rec_from, self.rec_to = self.seq + 1, None

    def stop_recording(self) -> None:
        self.flush(None)
        if self.rec_from is not None and self.rec_to is None:
            self.rec_to = self.seq

    def recording(self) -> dict:
        if self.rec_from is None:
            return {"active": False, "rows": 0}
        last = self.rec_to if self.rec_to is not None else self.seq
        return {"active": self.rec_to is None, "from_seq": self.rec_from, "to_seq": last,
                "rows": max(0, last - self.rec_from + 1)}

    # -- input --------------------------------------------------------------------

    def _emit(self, t: Optional[float], motion: Optional[dict], sensor: Optional[dict], simulated: bool) -> None:
        self.seq += 1
        row = {"seq": self.seq, "t": t, "time": clock(t), "car_ms": None, "ir_c": None, "ir_r": None,
               "pitch": None, "roll": None, "range_cm": None, "speed": None, "distance_cm": None,
               "state": self.state, "status": {}, "simulated": simulated}
        if motion is not None:
            row.update(car_ms=motion["ts_ms"], speed=motion["speed_cmps"], distance_cm=motion["distance_cm"])
        if sensor is not None:
            row["car_ms"] = sensor["ts_ms"]
            cls = classify_sensor_frame(sensor, self.limits)
            for key in ("ir_c", "ir_r", "ultrasonic", "imu"):
                st = cls[key][0]
                row["status"][key] = "simulated" if simulated and st != UNAVAILABLE else st.lower()
            if cls["ir_c"][0] != UNAVAILABLE:
                row["ir_c"] = sensor["ir_c_raw"]
            if cls["ir_r"][0] != UNAVAILABLE:
                row["ir_r"] = sensor["ir_r_raw"]
            if cls["ultrasonic"][0] != UNAVAILABLE:
                row["range_cm"] = sensor["range_cm"]
            if cls["imu"][0] != UNAVAILABLE:
                row["pitch"], row["roll"] = sensor["pitch_deg"], sensor["roll_deg"]
        self.rows.append(row)

    def add(self, fr: codec.Frame, t: Optional[float], simulated: bool = False) -> None:
        t = self._t(t)
        if fr.type == codec.FT_HEARTBEAT:
            self.state = str(fr.fields["state"])
        elif fr.type == codec.FT_TELEMETRY_MOTION:
            self.flush(None)                     # the previous wake had no sensor frame
            self._pending = (t if t is not None else 0.0, dict(fr.fields))
        elif fr.type == codec.FT_TELEMETRY_SENSOR:
            motion = None
            if self._pending is not None and (t is None or abs(t - self._pending[0]) <= PAIR_S):
                motion = self._pending[1]
            elif self._pending is not None:
                self.flush(None)
            self._pending = None
            self._emit(t, motion, dict(fr.fields), simulated)

    def flush(self, now: Optional[float]) -> None:
        """Writes out a motion frame whose sensor frame never came (none in this firmware, or late)."""
        if self._pending is None:
            return
        if now is not None and now + self._t.offset - self._pending[0] <= PAIR_S * 3:
            return
        t, motion = self._pending
        self._pending = None
        self._emit(t, motion, None, False)

    # -- output -------------------------------------------------------------------

    def page(self, after: Optional[int] = None, before: Optional[int] = None, limit: int = 200) -> dict:
        """Up to 'limit' rows: newest by default, rows after 'after', or rows before 'before'."""
        limit = max(1, min(int(limit), 2000))
        rows = list(self.rows)
        if after is not None:
            sel = [r for r in rows if r["seq"] > after][:limit]
        elif before is not None:
            sel = [r for r in rows if r["seq"] < before][-limit:]
        else:
            sel = rows[-limit:]
        return {"rows": sel, "total": len(rows), "first_seq": rows[0]["seq"] if rows else None,
                "last_seq": rows[-1]["seq"] if rows else None, "recording": self.recording()}

    def csv_text(self, scope: str = "all") -> str:
        rows = list(self.rows)
        if scope == "recording":
            rec = self.recording()
            if rec.get("rows"):
                rows = [r for r in rows if rec["from_seq"] <= r["seq"] <= rec["to_seq"]]
            else:
                rows = []
        buf = io.StringIO()
        w = csv.writer(buf, lineterminator="\n")
        w.writerow(CSV_COLUMNS)
        for r in rows:
            st = r["status"]
            w.writerow([r["seq"], r["time"], _cell(r["car_ms"]), _cell(r["ir_c"]), _cell(r["ir_r"]),
                        _cell(r["pitch"]), _cell(r["roll"]), _cell(r["speed"]), _cell(r["distance_cm"]),
                        _cell(r["range_cm"]), r["state"], st.get("ir_c", ""), st.get("ir_r", ""),
                        st.get("ultrasonic", ""), st.get("imu", ""), int(bool(r["simulated"]))])
        return buf.getvalue()


def gaps_csv(rows: List[dict]) -> str:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["id", "sensor", "state", "start", "end", "duration_s", "reason"])
    for g in rows:
        w.writerow([g["id"], g["label"], g["state"], g["start"], g["end"] or "(still open)",
                    _cell(g["duration_s"]), g["reason"]])
    return buf.getvalue()


def _cell(v) -> str:
    return "" if v is None else str(v)
