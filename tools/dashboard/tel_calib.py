"""Line-sensor calibration helper for the dashboard (Buddy 1, dashboard v2).

The car calibrates itself: Buddy 3's Line-Following task measures the IR
sensors at boot and works on a darkness scale from calibrated white (0) to
calibrated black (1000 per mille).  This module does the same sums on the
laptop, from the raw IR values in the telemetry sensor frame, so a teammate
can check the sensors over white and over the line before a run:

    capture over WHITE, capture over BLACK  ->  per-sensor verdict, span,
                                               raw thresholds
    every new frame                         ->  darkness and zone
                                               (white / edge / black)

The rules are Buddy 3's, read from the firmware sources at start-up so the
two cannot drift apart:

    lf_sanity.c lf_adc_check     STUCK_LOW / STUCK_HIGH / NOISY / OK
    lf_sanity.c lf_adc_stats_avg average rounded to nearest
    lf_core.c   lf_cal_from_start_pose   black >= white + LF_MIN_SPAN_RAW
    lf_core.c   lf_norm, lf_zone_of      darkness and zone (no hysteresis here)

Nothing here is sent to the car.  It cannot change the car's own
calibration, which needs a firmware change by Buddy 3.  Standard library only.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import tel_codec as codec

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))

# Firmware constant -> (file, value used when the file or the macro is missing).
RULES = {
    "LF_PM_FULL": ("app_program/subsystem3/lf_core.h", 1000),
    "LF_LIGHT_MAX_PM": ("app_program/subsystem3/lf_core.h", 250),
    "LF_DARK_MIN_PM": ("app_program/subsystem3/lf_core.h", 750),
    "LF_MIN_SPAN_RAW": ("app_program/subsystem3/lf_core.h", 400),
    "ADC_FULL_SCALE_RAW": ("config/thresholds.h", 4095),
    "ADC_STUCK_LOW_RAW": ("config/thresholds.h", 16),
    "ADC_STUCK_HIGH_RAW": ("config/thresholds.h", 4079),
    "ADC_NOISE_MAX_RAW": ("config/thresholds.h", 400),
}

CAPTURE_S = 3.0             # default capture window
MIN_SAMPLES = 5             # a capture needs at least this many IR frames
CHANNELS = ("c", "r")       # CENTRE, RIGHT
CHANNEL_NAME = {"c": "CENTRE", "r": "RIGHT"}

_DEFINE = re.compile(r"^\s*#define\s+([A-Z0-9_]+)\s+\(?\s*(\d+)\s*[uU]?\s*\)?\s*(?:/[*/].*)?$")


def read_c_defines(path: str, names) -> Dict[str, int]:
    """Plain integer macros '#define NAME (123U)' from a C header; others are skipped."""
    out: Dict[str, int] = {}
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                m = _DEFINE.match(line)
                if m and m.group(1) in names:
                    out[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    return out


def load_rules(repo: str = REPO) -> Dict[str, dict]:
    """{name: {"value", "source"}}: from the firmware files, else the documented default."""
    by_file: Dict[str, List[str]] = {}
    for name, (rel, _) in RULES.items():
        by_file.setdefault(rel, []).append(name)
    found: Dict[str, int] = {}
    for rel, names in by_file.items():
        found.update(read_c_defines(os.path.join(repo, rel), names))
    rules = {}
    for name, (rel, default) in RULES.items():
        if name in found:
            rules[name] = {"value": found[name], "source": rel}
        else:
            rules[name] = {"value": default, "source": f"default (not found in {rel})"}
    return rules


def _v(rules: Dict[str, dict], name: str) -> int:
    return int(rules[name]["value"])


# ---------------------------------------------------------------------------
# Statistics and verdicts (lf_sanity.c / lf_core.c on the laptop)
# ---------------------------------------------------------------------------

@dataclass
class Stats:
    n: int = 0
    total: int = 0
    lo: int = 0
    hi: int = 0

    def add(self, raw: int, full_scale: int) -> None:
        v = min(int(raw), full_scale)
        if self.n == 0 or v < self.lo:
            self.lo = v
        if self.n == 0 or v > self.hi:
            self.hi = v
        self.n += 1
        self.total += v

    @property
    def avg(self) -> int:
        """Rounded to nearest, as lf_adc_stats_avg."""
        return (self.total + self.n // 2) // self.n if self.n else 0

    @property
    def spread(self) -> int:
        return self.hi - self.lo if self.n else 0

    def as_dict(self) -> dict:
        return {"n": self.n, "min": self.lo, "avg": self.avg, "max": self.hi, "spread": self.spread}


def adc_check(s: Stats, rules: Dict[str, dict]) -> Tuple[str, str]:
    """(verdict, reason) as lf_adc_check: NO_SAMPLES, STUCK_LOW, STUCK_HIGH, NOISY or OK."""
    if s.n == 0:
        return "NO_SAMPLES", "no samples"
    if s.hi <= _v(rules, "ADC_STUCK_LOW_RAW"):
        return "STUCK_LOW", f"every sample <= {_v(rules, 'ADC_STUCK_LOW_RAW')}: unplugged or shorted to ground?"
    if s.lo >= _v(rules, "ADC_STUCK_HIGH_RAW"):
        return "STUCK_HIGH", f"every sample >= {_v(rules, 'ADC_STUCK_HIGH_RAW')}: open input or no light return?"
    if s.spread > _v(rules, "ADC_NOISE_MAX_RAW"):
        return "NOISY", (f"spread {s.spread} > {_v(rules, 'ADC_NOISE_MAX_RAW')} raw: the sensor moved during "
                         "the capture, or it sat on the line's edge")
    return "OK", "plausible"


def channel_result(white: Optional[Stats], black: Optional[Stats], rules: Dict[str, dict]) -> dict:
    """One sensor's verdict from its own white and black captures."""
    out = {"white": white.as_dict() if white else None, "black": black.as_dict() if black else None,
           "span": None, "verdict": "INCOMPLETE", "reason": "capture white and black first",
           "light_max_raw": None, "dark_min_raw": None}
    if white is None or black is None:
        return out
    for label, s in (("white", white), ("black", black)):
        v, why = adc_check(s, rules)
        if v != "OK":
            out.update(verdict="BAD_SENSOR", reason=f"{label} capture {v}: {why}")
            return out
    span = black.avg - white.avg
    out["span"] = span
    need = _v(rules, "LF_MIN_SPAN_RAW")
    if black.avg < white.avg + need:
        out.update(verdict="NO_CONTRAST",
                   reason=(f"black {black.avg} is not at least {need} above white {white.avg} "
                           f"(span {span}): the sensor does not see the line"))
        return out
    full = _v(rules, "LF_PM_FULL")
    out.update(verdict="OK", reason=f"span {span} >= {need}",
               light_max_raw=white.avg + (span * _v(rules, "LF_LIGHT_MAX_PM")) // full,
               dark_min_raw=white.avg + (span * _v(rules, "LF_DARK_MIN_PM")) // full)
    return out


def darkness(raw: int, white: int, black: int, full: int) -> int:
    """Per mille of the white..black span, as lf_norm (0 when uncalibrated)."""
    if black <= white or raw <= white:
        return 0
    if raw >= black:
        return full
    return ((raw - white) * full) // (black - white)


def zone(d_pm: int, rules: Dict[str, dict]) -> str:
    """'white' / 'edge' / 'black', as lf_zone_of (LIGHT / EDGE / DARK), without its hysteresis."""
    if d_pm < _v(rules, "LF_LIGHT_MAX_PM"):
        return "white"
    if d_pm > _v(rules, "LF_DARK_MIN_PM"):
        return "black"
    return "edge"


# ---------------------------------------------------------------------------
# The car's own measurements, from its USB console (Buddy 3's [LF] lines)
# ---------------------------------------------------------------------------

# LF-1 measurement test (lf_hw_test.c):
#   "[LF] WHITE under both: C min/avg/max 105/140/171  R 117/144/168  n=1000"
_LF1 = re.compile(r"\[LF\] (WHITE under both|BLACK under both|line under CENTRE|line under RIGHT): "
                  r"C min/avg/max (\d+)/(\d+)/(\d+)\s+R (\d+)/(\d+)/(\d+)\s+n=(\d+)")
# Line-Following task boot calibration (branch buddy3/lf3-link, task_lf.c):
#   "[LF] boot calibration ok: C avg 1144 (1099..1177), R avg 144 (117..169)"
#   "[LF] cal white 144 black 1144 (both sensors)"
_BOOT = re.compile(r"\[LF\] boot calibration (\S+): C avg (\d+) \((\d+)\.\.(\d+)\), R avg (\d+) \((\d+)\.\.(\d+)\)")
_CAL = re.compile(r"\[LF\] cal white (\d+) black (\d+)")

# Which surface each LF-1 step puts under each sensor.
_LF1_SURFACE = {
    "WHITE under both": {"c": "white", "r": "white"},
    "BLACK under both": {"c": "black", "r": "black"},
    "line under CENTRE": {"c": "black", "r": "white"},
    "line under RIGHT": {"c": "white", "r": "black"},
}


def _stats(lo: str, avg: str, hi: str, n: str) -> Stats:
    """A Stats with the car's min/avg/max (total chosen so avg rounds back to the car's value)."""
    k = max(int(n), 1)
    return Stats(n=k, total=int(avg) * k, lo=int(lo), hi=int(hi))


# ---------------------------------------------------------------------------
# A calibration session
# ---------------------------------------------------------------------------

@dataclass
class _Capture:
    target: str                 # "white" | "black"
    start_t: float
    seconds: float
    c: Stats = field(default_factory=Stats)
    r: Stats = field(default_factory=Stats)
    ignored_simulated: int = 0
    ignored_invalid: int = 0


class Calibration:
    """Guided white/black capture plus live darkness, one per dashboard source."""

    def __init__(self, rules: Optional[Dict[str, dict]] = None) -> None:
        self.rules = rules or load_rules()
        self.reset()

    def reset(self) -> None:
        self.white: Dict[str, Optional[Stats]] = {"c": None, "r": None}
        self.black: Dict[str, Optional[Stats]] = {"c": None, "r": None}
        self.source = ""                # "dashboard capture" or the car console line used
        self.capture: Optional[_Capture] = None
        self.last_capture_note = ""
        self.live: Optional[dict] = None
        self.ir_frames = 0              # sensor frames with a real IR reading
        self.ir_simulated = 0           # IR values written by the S1 self-test
        self.ir_missing = 0             # sensor frames with validity flags, IR not set
        self.no_flags = 0               # sensor frames from firmware before the validity flags
        self.car: Dict[str, object] = {"lines": []}   # what the car printed about its calibration

    # -- capture ------------------------------------------------------------

    def start(self, target: str, now: float, seconds: float = CAPTURE_S) -> str:
        """Begin capturing (target 'white' or 'black').  Returns '' or why it cannot start."""
        if target not in ("white", "black"):
            return "target must be white or black"
        if not 0.5 <= seconds <= 30:
            return "capture length must be 0.5 to 30 s"
        self.capture = _Capture(target, now, seconds)
        self.last_capture_note = ""
        return ""

    def cancel(self) -> None:
        self.capture = None
        self.last_capture_note = "capture cancelled"

    def _finish(self) -> None:
        cap = self.capture
        self.capture = None
        if cap is None:
            return
        if cap.c.n < MIN_SAMPLES:
            why = f"only {cap.c.n} IR frame(s) in {cap.seconds:g} s (need {MIN_SAMPLES})"
            if cap.ignored_simulated:
                why += f"; {cap.ignored_simulated} simulated by a self-test, not used"
            if cap.ignored_invalid:
                why += f"; {cap.ignored_invalid} without IR values (no producer)"
            self.last_capture_note = f"{cap.target} capture failed: {why}"
            return
        store = self.white if cap.target == "white" else self.black
        store["c"], store["r"] = cap.c, cap.r
        self.source = "dashboard capture"
        self.last_capture_note = f"{cap.target} captured: {cap.c.n} frames"

    def tick(self, now: Optional[float]) -> None:
        """Ends a capture whose window has passed (call with the current time)."""
        cap = self.capture
        if cap is not None and now is not None and now >= cap.start_t + cap.seconds:
            self._finish()

    def use_car_values(self) -> str:
        """Take the white/black the car measured (from its console lines) as the calibration."""
        meas = self.car.get("measured")
        if not meas:
            return "the car has not printed a white/black measurement in this source"
        for ch in CHANNELS:
            if meas.get(("white", ch)) is None or meas.get(("black", ch)) is None:
                return f"the car's lines give no {CHANNEL_NAME[ch]} white and black pair yet"
        for ch in CHANNELS:
            self.white[ch] = meas[("white", ch)]
            self.black[ch] = meas[("black", ch)]
        self.source = "car console: " + str(self.car.get("measured_from", ""))
        return ""

    # -- input --------------------------------------------------------------

    def on_frame(self, fr: codec.Frame, t: Optional[float], simulated: bool) -> None:
        if fr.type != codec.FT_TELEMETRY_SENSOR:
            return
        f = fr.fields
        valid = bool(f["ir_valid"])
        if valid and simulated:
            self.ir_simulated += 1
        elif valid:
            self.ir_frames += 1
        elif f["flags_present"]:
            self.ir_missing += 1
        else:
            self.no_flags += 1
        cap = self.capture
        if cap is not None and t is not None and t >= cap.start_t:
            if simulated:
                cap.ignored_simulated += 1
            elif not valid or not f["flags_present"]:
                cap.ignored_invalid += 1
            else:
                fs = _v(self.rules, "ADC_FULL_SCALE_RAW")
                cap.c.add(int(f["ir_c_raw"]), fs)
                cap.r.add(int(f["ir_r_raw"]), fs)
            self.tick(t)
        self.live = {"t": t, "valid": valid, "simulated": simulated,
                     "c": int(f["ir_c_raw"]), "r": int(f["ir_r_raw"])}

    def on_console(self, body: str) -> None:
        """Picks up the car's own white/black measurements from [LF] lines."""
        m = _LF1.search(body)
        if m:
            step = m.group(1)
            steps = self.car.setdefault("steps", {})
            steps[step] = {"c": _stats(m.group(2), m.group(3), m.group(4), m.group(8)),
                           "r": _stats(m.group(5), m.group(6), m.group(7), m.group(8))}
            # "... under both" is what each surface reads; the one-sensor steps only fill in
            # a sensor still missing (the block may also cover the other sensor).
            meas: Dict[Tuple[str, str], Stats] = {}
            for name in ("WHITE under both", "BLACK under both", "line under CENTRE", "line under RIGHT"):
                if name in steps:
                    for ch in CHANNELS:
                        meas.setdefault((_LF1_SURFACE[name][ch], ch), steps[name][ch])
            self.car["measured"] = meas
            self.car["measured_from"] = "LF-1 measurement lines"
            self._note_car_line(body)
            return
        m = _BOOT.search(body)
        if m:
            self.car["boot"] = {"verdict": m.group(1), "c_avg": int(m.group(2)), "c_range": [int(m.group(3)), int(m.group(4))],
                                "r_avg": int(m.group(5)), "r_range": [int(m.group(6)), int(m.group(7))]}
            self._note_car_line(body)
            return
        m = _CAL.search(body)
        if m:
            self.car["cal"] = {"white": int(m.group(1)), "black": int(m.group(2))}
            self._note_car_line(body)

    def _note_car_line(self, body: str) -> None:
        lines = self.car["lines"]
        lines.append(body)
        del lines[:-8]

    # -- results --------------------------------------------------------------

    def results(self) -> Dict[str, dict]:
        out = {ch: channel_result(self.white[ch], self.black[ch], self.rules) for ch in CHANNELS}
        # What the car's boot rule (lf_cal_from_start_pose) would compute: CENTRE on the
        # line gives black, RIGHT on white gives white, and both sensors then share them.
        w, b = self.white["r"], self.black["c"]
        boot = {"white": w.avg if w else None, "black": b.avg if b else None, "verdict": "INCOMPLETE",
                "reason": "needs RIGHT over white and CENTRE over black"}
        if w and b:
            vw, why_w = adc_check(w, self.rules)
            vb, why_b = adc_check(b, self.rules)
            need = _v(self.rules, "LF_MIN_SPAN_RAW")
            if vw != "OK" or vb != "OK":
                boot.update(verdict="BAD_SENSOR", reason=f"RIGHT white {vw}, CENTRE black {vb}")
            elif b.avg >= w.avg + need:
                boot.update(verdict="OK", reason=f"black {b.avg} >= white {w.avg} + {need}")
            else:
                boot.update(verdict="NO_CONTRAST", reason=f"black {b.avg} < white {w.avg} + {need}")
        out["boot_rule"] = boot
        return out

    def live_view(self) -> Optional[dict]:
        if self.live is None:
            return None
        res = self.results()
        full = _v(self.rules, "LF_PM_FULL")
        view = {"t": self.live["t"], "valid": self.live["valid"], "simulated": self.live["simulated"]}
        for ch in CHANNELS:
            raw = self.live[ch]
            r = res[ch]
            entry = {"raw": raw if self.live["valid"] else None, "darkness_pm": None, "zone": None}
            if self.live["valid"] and r["verdict"] == "OK":
                d = darkness(raw, r["white"]["avg"], r["black"]["avg"], full)
                entry.update(darkness_pm=d, zone=zone(d, self.rules))
            view[ch] = entry
        return view

    def snapshot(self, now: Optional[float]) -> dict:
        self.tick(now)
        cap = self.capture
        notes = []
        if self.ir_frames == 0:
            if self.ir_missing:
                notes.append("Sensor frames arrive but carry no IR values: the line-following task does not "
                             "call tel_set_ir_raw() yet (Buddy 3, IF-12). Live calibration needs it.")
            elif self.no_flags:
                notes.append("This firmware predates the sensor-frame validity flags (19 Nov), so its IR values "
                             "cannot be trusted for a capture. Flash a build from main.")
            elif not self.ir_simulated:
                notes.append("No sensor frames from this source yet.")
        if self.live and self.live["simulated"]:
            notes.append("These IR values were written by the subsystem 1 self-test (APP_TEST build), "
                         "not by a sensor: captures ignore them.")
        car = {k: v for k, v in self.car.items() if k not in ("measured", "steps")}
        meas = self.car.get("measured") or {}
        car["measured"] = {f"{surface} {CHANNEL_NAME[ch]}": s.as_dict() for (surface, ch), s in meas.items()}
        return {
            "rules": self.rules,
            "capturing": None if cap is None else {
                "target": cap.target, "seconds": cap.seconds, "n": cap.c.n,
                "left_s": None if now is None else max(0.0, round(cap.start_t + cap.seconds - now, 1)),
                "ignored_simulated": cap.ignored_simulated, "ignored_invalid": cap.ignored_invalid},
            "note": self.last_capture_note,
            "source": self.source,
            "result": self.results(),
            "live": self.live_view(),
            "car": car,
            "ir_frames": self.ir_frames,
            "notes": notes,
        }
