"""Per-subsystem views and evidence checks for the telemetry dashboard.

Dashboard integration (Buddy 3, 19 Nov 2026), on top of Buddy 1's tel_codec.py
and tel_dashboard.py.  Every value is printed with where it comes from:

    MEASURED   read from a sensor: IR ADC, ultrasonic echo, encoder counts,
               accelerometer
    DERIVED    computed on the car from measured values: pitch/roll from the
               accelerometer, obstacle width and clearances, distance
    REPORTED   a state or decision the firmware reports: VC state, fault bits,
               decoded barcode, hump event
    SIMULATED  injected by a self-test (APP_TEST build: the subsystem 1 tests
               write made-up sensor values, a barcode and an obstacle)
    N/A        no producer in this firmware, or it has not written yet

A check prints PASS only when the log holds the measurement that supports
it.  Otherwise it says FAIL, NOT TESTED (the step was not done), NO DATA
(the firmware did not send what the check needs), NOT MEASURED (the firmware
has no such measurement) or COMPARE (only a person with a ruler can judge).
"""

from __future__ import annotations

import re
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import tel_codec as codec

VIEWS = ("imu", "obstacle", "barcode", "lf", "vc", "mqtt", "all")

# Console line prefixes shown by each view (firmware dbg lines).
_PREFIXES = {
    "imu": ("[IMU]", "[SAFETY]", "[S4]"),
    "obstacle": ("[S5]", "[S5RAW]"),
    "barcode": ("[BC]",),
    "lf": ("[LF]",),
    "vc": ("[VC]", "[SAFETY] BRAKE", "[TEL] cmd"),
    "mqtt": ("[TEL] link", "[TEL] backend", "[TEL] teardown", "[TEL] cmd", "[TEL] CYW43", "[NET]", "[WIFI]"),
}

IMU_BITS = ("LIFTED", "IMU_ALIVE", "FAULT_LATCHED", "IMU_I2C", "IMU_WDOG")
MOVING_STATES = ("OBSTACLE_AVOIDANCE", "NAV_LINE_TRACKING", "NAV_HUMP_SENSING", "NAV_TURN_LEFT",
                 "NAV_GO_STRAIGHT", "NAV_U_TURN", "NAV_TURN_RIGHT")

# Plausibility limits for the checks (stated in each check's text).
REST_MAG_MIN_MG = 900           # 1 g = 1000 mg; +-10 % covers sensor scale error
REST_MAG_MAX_MG = 1100
LEVEL_MAX_DEG = 2.0             # pitch after the boot level reference, car flat
TILT_SPAN_DEG = 10.0            # a deliberate hand tilt
RANGE_TOL_CM = 2                # HC-SR04 reading vs a tape measure
PLATEAU_MIN_N = 3               # samples for a stable range (console: 1 sensor frame/s)
OBS_DETECT_CM = 25              # config/thresholds.h OBS_D_DETECT_CM
HB_GAP_S = 3.0                  # tel_dashboard HEARTBEAT_STALE_S

# task_obs_scan.c: "[S5] obstacle %lu mm @ %lu deg w=%lu clrL=%lu clrR=%lu"
_S5_OBSTACLE = re.compile(r"\[S5\] obstacle (\d+) mm @ (\d+) deg w=(\d+) clrL=(\d+) clrR=(\d+)")


@dataclass
class Evidence:
    """Everything the checks need, gathered from one log or MQTT session."""

    app_test: bool = False          # "[TEST] S..." lines: an APP_TEST build
    s1_ran: bool = False            # the subsystem 1 tests ran: they inject sensor values
    in_s1_test: bool = False        # between the first S1 test line and its SUMMARY
    unplug_hb: Optional[int] = None  # heartbeat count when S4-U14 (unplug the IMU) began
    mqtt_source: bool = False       # a file written by "listen --save"
    backend: str = ""               # "console" | "wifi" from "[TEL] backend:"
    imu_chip: List[str] = field(default_factory=list)
    imu_not_ready: bool = False
    imu_i2c_fault_lines: int = 0
    imu_status: List[dict] = field(default_factory=list)
    obs_status: List[dict] = field(default_factory=list)
    link_status: List[dict] = field(default_factory=list)
    link_moves: List[str] = field(default_factory=list)
    attitude: List[Tuple[float, float]] = field(default_factory=list)   # (pitch, roll), real only
    ranges: List[int] = field(default_factory=list)                     # cm, real only (0 = no echo)
    ir: List[Tuple[int, int]] = field(default_factory=list)             # (centre, right), real only
    sensor_frames: int = 0
    sensor_simulated: int = 0
    sensor_no_flags: int = 0
    humps: List[dict] = field(default_factory=list)
    hump_lines: int = 0
    lifted_lines: int = 0
    brake_lines: List[str] = field(default_factory=list)
    obstacles: List[dict] = field(default_factory=list)
    obstacles_simulated: int = 0
    s5_obstacles: List[dict] = field(default_factory=list)   # from [S5] console lines
    s5_avoid_lines: List[str] = field(default_factory=list)
    barcodes: List[str] = field(default_factory=list)
    barcodes_simulated: int = 0
    heartbeats: List[dict] = field(default_factory=list)                # fields + "rx" wall time if known
    states: List[str] = field(default_factory=list)
    fault_frames: List[dict] = field(default_factory=list)
    motion: List[dict] = field(default_factory=list)


def _wall_s(line: str) -> Optional[float]:
    """Seconds of day from a 'HH:MM:SS.fff  ' capture prefix, else None."""
    head = line[:12]
    try:
        h, m, s = head.split(":")
        return int(h) * 3600 + int(m) * 60 + float(s)
    except ValueError:
        return None


class Session:
    """Feeds lines/frames into the evidence and renders one view."""

    def __init__(self, view: Optional[str] = None) -> None:
        self.view = view
        self.ev = Evidence()
        self._last_bits: Optional[int] = None
        self._last_state: Optional[str] = None
        self._last_motion_moving = False

    # -- input --------------------------------------------------------------

    def feed_line(self, line: str) -> List[str]:
        """One console (or saved listen) line.  Returns the lines to print."""
        text = line.rstrip("\r\n")
        body = text[text.find("[") :] if "[" in text else text
        ev = self.ev
        if body.startswith("[DASH] source=mqtt"):
            ev.mqtt_source = True
            return []
        if body.startswith("[TEST] S1-"):
            ev.app_test = True
            ev.s1_ran = True
            ev.in_s1_test = True
        elif body.startswith("[TEST] subsystem1 SUMMARY"):
            ev.in_s1_test = False
        elif body.startswith("[TEST] S"):
            ev.app_test = True
        if "MANUAL S4-U14" in body and ev.unplug_hb is None:
            ev.unplug_hb = len(ev.heartbeats)
        parsed = codec.parse_log_line(body)
        if parsed is not None:
            try:
                fr = codec.decode(parsed[1])
            except codec.FrameError as exc:
                return [f"REJECT {exc} ({parsed[1].hex()})"] if self.view else []
            return self.feed_frame(fr, _wall_s(text))
        if body.startswith("[DASH] car/status"):
            return self._show("mqtt", body[7:]) if self.view else []
        self._scan_console(body)
        return self._console_view(body)

    def _scan_console(self, body: str) -> None:
        ev = self.ev
        st = codec.parse_status_line(body)
        if st is not None:
            kind, d = st
            {"imu": ev.imu_status, "obs": ev.obs_status, "link": ev.link_status}[kind].append(d)
            return
        if body.startswith("[TEL] backend:"):
            ev.backend = "wifi" if "WiFi" in body else "console"
        elif body.startswith("[TEL] link "):
            ev.link_moves.append(body[len("[TEL] link "):])
        elif body.startswith("[IMU] chip"):
            ev.imu_chip.append(body)
        elif "IMU NOT READY" in body or body.startswith("[IMU] unknown chip"):
            ev.imu_not_ready = True
        elif body.startswith("[IMU] I2C fault"):
            ev.imu_i2c_fault_lines += 1
        elif body.startswith("[IMU] hump"):
            ev.hump_lines += 1
        elif body.startswith("[IMU] LIFTED"):
            ev.lifted_lines += 1
        elif body.startswith("[SAFETY] BRAKE"):
            ev.brake_lines.append(body)
        elif _S5_OBSTACLE.search(body):
            m = _S5_OBSTACLE.search(body)
            mm, deg, w, cl, cr = (int(x) for x in m.groups())
            # Same keys as a decoded obstacle frame (distance in whole cm).
            ev.s5_obstacles.append({"distance_cm": mm // 10, "bearing_deg": deg, "width_mm": w,
                                    "clear_l_mm": cl, "clear_r_mm": cr, "no_echo": mm == 0})
        elif body.startswith("[S5] avoid") or body.startswith("[S5] bypass") or body.startswith("[S5] blocked"):
            ev.s5_avoid_lines.append(body)

    def feed_frame(self, fr: codec.Frame, wall: Optional[float] = None) -> List[str]:
        """One decoded frame (from a log line or MQTT)."""
        ev = self.ev
        f = fr.fields
        simulated = ev.in_s1_test
        if fr.type == codec.FT_TELEMETRY_SENSOR:
            ev.sensor_frames += 1
            # Once the S1 tests ran, their snapshot values stay in every later
            # sensor frame until a real producer overwrites them, and nothing
            # tells the two apart: treat all of them as simulated.
            if ev.s1_ran:
                ev.sensor_simulated += 1
            elif not f["flags_present"]:
                ev.sensor_no_flags += 1
            else:
                if f["att_valid"]:
                    ev.attitude.append((float(f["pitch_deg"]), float(f["roll_deg"])))
                if f["range_valid"]:
                    ev.ranges.append(int(f["range_cm"]))
                if f["ir_valid"]:
                    ev.ir.append((int(f["ir_c_raw"]), int(f["ir_r_raw"])))
        elif fr.type == codec.FT_EVENT_HUMP and not simulated:
            ev.humps.append(dict(f))
        elif fr.type == codec.FT_EVENT_OBSTACLE:
            if simulated:
                ev.obstacles_simulated += 1
            else:
                ev.obstacles.append(dict(f))
        elif fr.type == codec.FT_EVENT_BARCODE:
            if simulated:
                ev.barcodes_simulated += 1
            else:
                ev.barcodes.append(str(f["command"]))
        elif fr.type == codec.FT_HEARTBEAT:
            hb = dict(f)
            hb["rx"] = wall
            ev.heartbeats.append(hb)
            ev.states.append(str(f["state"]))
        elif fr.type == codec.FT_EVENT_FAULT and not simulated:
            ev.fault_frames.append(dict(f))
        elif fr.type == codec.FT_TELEMETRY_MOTION:
            ev.motion.append(dict(f))
        return self._frame_view(fr, simulated)

    # -- views --------------------------------------------------------------

    def _show(self, view: str, text: str) -> List[str]:
        if self.view in (view, "all"):
            return [text]
        return []

    def _console_view(self, body: str) -> List[str]:
        if not self.view:
            return []
        st = codec.parse_status_line(body)
        if st is not None:
            kind, d = st
            if kind == "imu":
                found = "found" if d["found"] else "NOT FOUND"
                return self._show("imu", f"imu        chip {found}; |accel| {d['accel_mag_mg']} mg [MEASURED, "
                                         f"filtered]; pitch {d['pitch_deg']:+.1f} deg [DERIVED]")
            if kind == "obs":
                ok = "OK" if d["sensors_ok"] else "NOT OK"
                return self._show("obstacle", f"obstacle   sensors {ok}; avoidance state {d['avoid_state']} "
                                              f"active={int(d['avoid_active'])} [REPORTED]")
            return self._show("mqtt", f"mqtt       link {d['link']}: frames sent {d['tx']}, publish fails "
                                      f"{d['txf']}, skipped offline {d['off']}, events dropped {d['qd']}, "
                                      f"outbox dropped {d['od']}, commands {d['cmd_ok']} ok/{d['cmd_rej']} "
                                      f"rejected [REPORTED by the car]")
        for view, prefixes in _PREFIXES.items():
            if any(body.startswith(p) for p in prefixes) and self.view in (view, "all"):
                return [f"console    {body}"]
        return []

    def _frame_view(self, fr: codec.Frame, simulated: bool) -> List[str]:
        if not self.view:
            return []
        f = fr.fields
        tag_sim = " [SIMULATED: S1 self-test]" if simulated else ""
        out: List[str] = []
        if fr.type == codec.FT_TELEMETRY_SENSOR:
            if self.ev.s1_ran:
                why = " [SIMULATED/UNTRUSTED: the S1 tests ran, their S1-U07 values persist]"
                out += self._show("imu", f"attitude   pitch {f['pitch_deg']:+.1f} roll {f['roll_deg']:+.1f}{why}")
                out += self._show("obstacle", f"range      {f['range_cm']} cm{why}")
                out += self._show("lf", f"IR         CENTRE {f['ir_c_raw']} RIGHT {f['ir_r_raw']}{why}")
                return out
            if not f["flags_present"]:
                why = " [validity unknown: firmware without the flags]"
                out += self._show("imu", f"attitude   pitch {f['pitch_deg']:+.1f} roll {f['roll_deg']:+.1f}{why}")
                out += self._show("obstacle", f"range      {f['range_cm']} cm{why}")
                out += self._show("lf", f"IR         CENTRE {f['ir_c_raw']} RIGHT {f['ir_r_raw']}{why}")
                return out
            if f["att_valid"]:
                out += self._show("imu", f"attitude   pitch {f['pitch_deg']:+.1f} deg [DERIVED from accelerometer, "
                                         f"levelled at boot]; roll {f['roll_deg']:+.1f} deg [DERIVED, not "
                                         f"levelled, sign unchecked]")
            else:
                out += self._show("imu", "attitude   N/A (no attitude producer in this firmware yet)")
            if f["range_valid"]:
                r = f"{f['range_cm']} cm" if f["range_cm"] else "no echo (0)"
                out += self._show("obstacle", f"range      {r} straight ahead [MEASURED ultrasonic at servo 90 deg]")
            else:
                out += self._show("obstacle", "range      N/A (no ultrasonic producer in this firmware)")
            if f["ir_valid"]:
                out += self._show("lf", f"IR         CENTRE {f['ir_c_raw']} RIGHT {f['ir_r_raw']} raw [MEASURED ADC]")
            else:
                out += self._show("lf", "IR         N/A (no line-following task in this firmware: subsystem 3 "
                                        "is a stub on main)")
            return out
        if fr.type == codec.FT_EVENT_HUMP:
            h = (f"height {f['height_mm']} mm [DERIVED estimate]" if f["height_mm"]
                 else "height NOT MEASURED (firmware sends 0)")
            return self._show("imu", f"HUMP       pitch {f['pitch_deg']:+.1f} deg [REPORTED: pitch past 5 deg "
                                     f"for 3 samples]; {h}{tag_sim}")
        if fr.type == codec.FT_EVENT_OBSTACLE:
            if f["no_echo"]:
                return self._show("obstacle", f"OBSTACLE   profile with no echo{tag_sim}")
            return self._show("obstacle", f"OBSTACLE   {f['distance_cm']} cm @ {f['bearing_deg']} deg [MEASURED]; "
                                          f"width {f['width_mm']} mm, clear L {f['clear_l_mm']} / R "
                                          f"{f['clear_r_mm']} mm [DERIVED estimates; width errs wide on "
                                          f"purpose]{tag_sim}")
        if fr.type == codec.FT_EVENT_BARCODE:
            return self._show("barcode", f"BARCODE    {f['command']} [REPORTED decoded command]{tag_sim}")
        if fr.type == codec.FT_EVENT_FAULT:
            names = ",".join(f["names"]) or "-"
            out += self._show("imu", f"FAULT      {names} [REPORTED]{tag_sim}")
            if self.view not in ("imu", "all"):
                out += self._show("vc", f"FAULT      {names} [REPORTED]{tag_sim}")
            return out
        if fr.type == codec.FT_HEARTBEAT:
            bits = int(f["fault_bits"])
            state = str(f["state"])
            if self.view == "mqtt":
                return [f"heartbeat  up {f['uptime_ms'] / 1000:.1f} s, state {state}"]
            if state != self._last_state:
                prev = self._last_state or "-"
                out += self._show("vc", f"VC state   {prev} -> {state} at up {f['uptime_ms'] / 1000:.1f} s [REPORTED]")
                self._last_state = state
            if bits != self._last_bits:
                names = ",".join(n for n in f["fault_names"] if n in IMU_BITS) or "-"
                prev_names = (",".join(n for i, n in enumerate(codec.SAFETY_BITS)
                                       if (self._last_bits or 0) & (1 << i) and n in IMU_BITS) or "-")
                if names != prev_names:
                    out += self._show("imu", f"safety     {names} [REPORTED bits]")
                no_echo = "NO_ECHO" in f["fault_names"]
                was_no_echo = bool((self._last_bits or 0) & (1 << codec.SAFETY_BITS.index("NO_ECHO")))
                if self._last_bits is None or no_echo != was_no_echo:
                    out += self._show("obstacle", "status     NO_ECHO: nothing in range for a whole sweep [REPORTED]"
                                                  if no_echo else "status     echo present [REPORTED]")
                self._last_bits = bits
            return out
        if fr.type == codec.FT_TELEMETRY_MOTION:
            moving = abs(float(f["speed_cmps"])) > 0.0
            if moving or moving != self._last_motion_moving:
                self._last_motion_moving = moving
                return self._show("vc", f"motion     v {f['speed_cmps']:+.1f} cm/s, encoders L {f['enc_l']} R "
                                        f"{f['enc_r']} [MEASURED]; distance {f['distance_cm']} cm [DERIVED]")
            return []
        return []


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

@dataclass
class Verdict:
    name: str
    result: str         # PASS FAIL NOT TESTED NO DATA NOT MEASURED COMPARE INFO
    detail: str


def _motors_not_commanded(ev: Evidence) -> Verdict:
    name = "Motors never commanded (stationary test)"
    if not ev.states:
        return Verdict(name, "NO DATA", "no heartbeat frames, so the VC state is unknown")
    moving = sorted(set(s for s in ev.states if s in MOVING_STATES))
    if moving:
        return Verdict(name, "FAIL", f"the VC entered {', '.join(moving)}: the motors were commanded")
    return Verdict(name, "PASS", f"{len(ev.states)} heartbeats, VC only in {', '.join(sorted(set(ev.states)))}")


def _app_test_note(ev: Evidence) -> List[Verdict]:
    if ev.s1_ran:
        return [Verdict("Firmware build", "INFO", "APP_TEST build with the subsystem 1 tests: sensor-frame "
                        "values and the S1-window barcode/obstacle frames are SIMULATED and are not used "
                        "below. Use the stationary test firmware (APP_TEST=OFF) for subsystem tests.")]
    if ev.app_test:
        return [Verdict("Firmware build", "INFO", "APP_TEST build (subsystem tests ran, not subsystem 1's): "
                        "telemetry values are real; the tests may move the car or provoke faults")]
    return []


def check_imu(ev: Evidence) -> List[Verdict]:
    v: List[Verdict] = _app_test_note(ev)

    name = "IMU chip found"
    st_found = [d["found"] for d in ev.imu_status]
    if ev.imu_not_ready or (st_found and not any(st_found)):
        v.append(Verdict(name, "FAIL", "[IMU] task started, IMU NOT READY / found=0"))
    elif ev.imu_chip or any(st_found):
        v.append(Verdict(name, "PASS", ev.imu_chip[0] if ev.imu_chip else "[TEL] imu found=1"))
    else:
        v.append(Verdict(name, "NO DATA", "no [IMU] chip line and no [TEL] imu status line"))

    name = "No IMU I2C or watchdog fault"
    hbs = ev.heartbeats if ev.unplug_hb is None else ev.heartbeats[:ev.unplug_hb]
    bad = [hb for hb in hbs if {"IMU_I2C", "IMU_WDOG"} & set(hb["fault_names"])]
    before = " before S4-U14 unplugged the IMU on purpose" if ev.unplug_hb is not None else ""
    if bad or (ev.imu_i2c_fault_lines and ev.unplug_hb is None):
        first = bad[0]["uptime_ms"] / 1000 if bad else None
        when = f"first at up {first:.1f} s" if first is not None else "[IMU] I2C fault line"
        v.append(Verdict(name, "FAIL", f"{len(bad)} heartbeats{before} with IMU_I2C/IMU_WDOG, "
                         f"{ev.imu_i2c_fault_lines} I2C fault lines ({when})"))
    elif len(hbs) >= 5:
        v.append(Verdict(name, "PASS", f"{len(hbs)} heartbeats{before}, none with IMU_I2C or IMU_WDOG"
                         + (" (faults after it are provoked and not counted)" if before else "")))
    else:
        v.append(Verdict(name, "NO DATA", f"only {len(hbs)} heartbeats{before} (need 5)"))

    name = f"Acceleration magnitude at rest {REST_MAG_MIN_MG}-{REST_MAG_MAX_MG} mg (1 g)"
    mags = [int(d["accel_mag_mg"]) for d in ev.imu_status if d["found"]]
    if len(mags) >= 3:
        med = statistics.median(mags)
        res = "PASS" if REST_MAG_MIN_MG <= med <= REST_MAG_MAX_MG else "FAIL"
        v.append(Verdict(name, res, f"median {med:.0f} mg of {len(mags)} samples (min {min(mags)}, max {max(mags)})"))
    else:
        v.append(Verdict(name, "NO DATA", f"{len(mags)} [TEL] imu status lines (need 3; build with "
                         f"-DTEL_STATUS_PERIOD_MS=1000)"))

    name = f"Pitch within +-{LEVEL_MAX_DEG:.0f} deg while flat after boot"
    if len(ev.attitude) >= 5:
        first = [p for p, _ in ev.attitude[:5]]
        worst = max(abs(p) for p in first)
        res = "PASS" if worst <= LEVEL_MAX_DEG else "FAIL"
        v.append(Verdict(name, res, f"first 5 attitude samples {', '.join(f'{p:+.1f}' for p in first)} deg "
                         f"(car must be flat and still for the first 15 s)"))
    else:
        v.append(Verdict(name, "NO DATA", _att_missing(ev)))

    pitches = [p for p, _ in ev.attitude]
    rolls = [r for _, r in ev.attitude]
    for axis, vals, how in (("Pitch", pitches, "lift the FRONT about 15 deg, hold 3 s, put it down"),
                            ("Roll", rolls, "lift the LEFT side about 15 deg, hold 3 s, put it down")):
        name = f"{axis} follows a hand tilt (span >= {TILT_SPAN_DEG:.0f} deg)"
        if len(vals) < 5:
            v.append(Verdict(name, "NO DATA", _att_missing(ev)))
        elif max(vals) - min(vals) >= TILT_SPAN_DEG:
            v.append(Verdict(name, "PASS", f"{min(vals):+.1f} .. {max(vals):+.1f} deg. Sign not judged: "
                             f"nose-up / left-up should read positive; compare with what you did"))
        else:
            v.append(Verdict(name, "NOT TESTED", f"span only {max(vals) - min(vals):.1f} deg: {how}"))

    name = "Hump event reported"
    n_h = len(ev.humps) or ev.hump_lines
    if n_h:
        v.append(Verdict(name, "INFO", f"{n_h} events. The detector is pitch-only (past 5 deg for 3 samples), "
                         f"so a hand tilt or hard braking also fires it: this is not a hump-detection PASS"))
    else:
        v.append(Verdict(name, "NOT TESTED", "no hump event (needs a real hump at speed, or a tilt past 5 deg)"))

    name = "Hump height"
    if ev.humps and all(int(h["height_mm"]) == 0 for h in ev.humps):
        v.append(Verdict(name, "NOT MEASURED", "every hump frame carries height 0: the firmware has no "
                         "height estimate yet"))
    elif ev.humps:
        hs = [int(h["height_mm"]) for h in ev.humps]
        v.append(Verdict(name, "COMPARE", f"reported {', '.join(map(str, hs))} mm (DERIVED estimate): "
                         f"compare with the hump's measured height"))
    else:
        v.append(Verdict(name, "NOT TESTED", "no hump frames"))

    name = "Lift detected and brakes latched"
    lift_hb = any("LIFTED" in hb["fault_names"] for hb in ev.heartbeats) or \
        any("LIFTED" in fr["names"] for fr in ev.fault_frames)
    latched = bool(ev.brake_lines) or any("FAULT_LATCHED" in hb["fault_names"] for hb in ev.heartbeats)
    if ev.lifted_lines or lift_hb:
        if latched:
            v.append(Verdict(name, "PASS", f"{ev.lifted_lines} [IMU] LIFTED lines; "
                             f"{ev.brake_lines[0] if ev.brake_lines else 'FAULT_LATCHED in a heartbeat'}"))
        else:
            v.append(Verdict(name, "FAIL", "LIFTED seen but no [SAFETY] BRAKE / FAULT_LATCHED"))
    else:
        v.append(Verdict(name, "NOT TESTED", "lift the car straight up about 5 cm and hold it 1 s"))

    v.append(_motors_not_commanded(ev))
    return v


def _att_missing(ev: Evidence) -> str:
    if ev.sensor_simulated:
        return "sensor frames are from an APP_TEST build (simulated)"
    if ev.sensor_no_flags:
        return "sensor frames have no validity flags (firmware without the dashboard integration)"
    return f"{len(ev.attitude)} attitude samples (need 5; is the IMU producer in this firmware?)"


def plateaus(values: List[int], tol: int = RANGE_TOL_CM, min_n: int = PLATEAU_MIN_N) -> List[Tuple[float, int]]:
    """Stable stretches of non-zero readings, [(median cm, samples)]; 0 (no echo) is skipped."""
    out: List[Tuple[float, int]] = []
    cur: List[int] = []

    def close() -> None:
        if len(cur) >= min_n:
            med = float(statistics.median(cur))
            if out and abs(out[-1][0] - med) <= tol:
                out[-1] = (out[-1][0], out[-1][1] + len(cur))
            else:
                out.append((med, len(cur)))

    for x in values:
        if x <= 0:
            continue
        if cur and abs(x - statistics.median(cur)) > tol:
            close()
            cur = []
        cur.append(x)
    close()
    return out


def check_obstacle(ev: Evidence, expect_cm: Optional[List[int]] = None,
                   expect_width_mm: Optional[int] = None) -> List[Verdict]:
    v: List[Verdict] = _app_test_note(ev)

    name = "Scanner sensors OK"
    oks = [d["sensors_ok"] for d in ev.obs_status]
    if oks:
        v.append(Verdict(name, "PASS" if all(oks) else "FAIL",
                         f"{sum(oks)}/{len(oks)} [TEL] obs status lines say sensors_ok=1"))
    else:
        v.append(Verdict(name, "NO DATA", "no [TEL] obs status lines"))

    name = "Forward ultrasonic readings"
    nz = [r for r in ev.ranges if r > 0]
    if not ev.ranges:
        why = ("sensor frames are from an APP_TEST build (simulated)" if ev.sensor_simulated else
               "sensor frames have no validity flags (firmware without the dashboard integration)"
               if ev.sensor_no_flags else "no range samples")
        v.append(Verdict(name, "NO DATA", why))
    elif len(nz) >= 5:
        v.append(Verdict(name, "PASS", f"{len(nz)} echoes of {len(ev.ranges)} samples, "
                         f"{min(nz)}-{max(nz)} cm (MEASURED, servo at 90 deg)"))
    else:
        v.append(Verdict(name, "FAIL", f"only {len(nz)} echoes in {len(ev.ranges)} samples (0 = no echo): "
                         f"is the sensor wired, and is there a box 20 cm in front?"))

    name = f"Range matches a tape measure (+-{RANGE_TOL_CM} cm)"
    pls = plateaus(ev.ranges)
    pl_txt = ", ".join(f"{m:.0f} cm x{n}" for m, n in pls) or "none"
    if expect_cm:
        for want in expect_cm:
            hit = [m for m, _ in pls if abs(m - want) <= RANGE_TOL_CM]
            nm = f"{name}: box at {want} cm"
            if hit:
                v.append(Verdict(nm, "PASS", f"stable reading {hit[0]:.0f} cm (stable readings: {pl_txt})"))
            elif pls:
                v.append(Verdict(nm, "FAIL", f"no stable reading within {RANGE_TOL_CM} cm (stable readings: {pl_txt})"))
            else:
                v.append(Verdict(nm, "NO DATA", f"no stable reading ({PLATEAU_MIN_N}+ samples within "
                                 f"{RANGE_TOL_CM} cm)"))
    elif len(pls) >= 2:
        v.append(Verdict(name, "COMPARE", f"stable readings {pl_txt}: compare with your tape measure, or "
                         f"rerun the check with --expect-cm"))
    else:
        v.append(Verdict(name, "NOT TESTED", f"stable readings: {pl_txt}. Hold a box at 20 cm, then 50 cm, "
                         f"about 10 s each"))

    # Obstacle frames when the log has them, else the scanner's own console lines
    # (the same profiles; never both, so nothing is counted twice).
    profs = ev.obstacles if ev.obstacles else ev.s5_obstacles
    src = "obstacle frames" if ev.obstacles else "[S5] obstacle console lines"
    name = f"Obstacle detected within {OBS_DETECT_CM} cm"
    if profs:
        near = min((o for o in profs if not o["no_echo"]), key=lambda o: o["distance_cm"], default=None)
        if near is None:
            v.append(Verdict(name, "FAIL", f"{len(profs)} profiles ({src}), all with no echo"))
        elif near["distance_cm"] > OBS_DETECT_CM:
            v.append(Verdict(name, "FAIL", f"nearest profile {near['distance_cm']} cm, beyond {OBS_DETECT_CM} cm"))
        else:
            v.append(Verdict(name, "PASS", f"{len(profs)} profiles ({src}); nearest {near['distance_cm']} cm @ "
                             f"{near['bearing_deg']} deg (MEASURED)"))
    else:
        v.append(Verdict(name, "NOT TESTED", "hold a box 15-20 cm in front for 3 s (2 coarse sweeps)"))

    name = "Obstacle width (DERIVED estimate; spec: errs wide)"
    widths = [int(o["width_mm"]) for o in profs if not o["no_echo"]]
    if not widths:
        v.append(Verdict(name, "NOT TESTED", "no obstacle profile"))
    elif expect_width_mm:
        narrow = [w for w in widths if w < expect_width_mm]
        if narrow:
            v.append(Verdict(name, "FAIL", f"{len(narrow)}/{len(widths)} profiles narrower than the "
                             f"{expect_width_mm} mm box (min {min(narrow)} mm)"))
        else:
            v.append(Verdict(name, "PASS", f"all {len(widths)} profiles >= {expect_width_mm} mm "
                             f"({min(widths)}-{max(widths)} mm): errs wide as required"))
    else:
        v.append(Verdict(name, "COMPARE", f"{min(widths)}-{max(widths)} mm: rerun with --expect-width-mm "
                         f"<box width>"))

    name = "Clearances left/right (DERIVED estimates)"
    if profs:
        o = profs[-1]
        v.append(Verdict(name, "INFO", f"last profile: clear L {o['clear_l_mm']} mm, R {o['clear_r_mm']} mm"))

    name = "Avoidance manoeuvre"
    if ev.s5_avoid_lines:
        v.append(Verdict(name, "INFO", f"{len(ev.s5_avoid_lines)} avoidance lines, first: {ev.s5_avoid_lines[0]}"))
    else:
        v.append(Verdict(name, "NOT TESTED", "needs the car moving under the VC (NAV state); a stationary "
                         "test leaves avoidance idle"))

    v.append(_motors_not_commanded(ev))
    return v


def check_mqtt(ev: Evidence) -> List[Verdict]:
    v: List[Verdict] = []
    if ev.mqtt_source:
        hbs = ev.heartbeats
        name = "Heartbeats delivered to the laptop"
        if len(hbs) < 2:
            v.append(Verdict(name, "NO DATA", f"{len(hbs)} heartbeats received"))
            return v
        span = (hbs[-1]["uptime_ms"] - hbs[0]["uptime_ms"]) / 1000.0
        gaps = [(b["uptime_ms"] - a["uptime_ms"]) / 1000.0 for a, b in zip(hbs, hbs[1:])]
        expected = int(span) + 1
        worst = max(gaps)
        detail = (f"{len(hbs)} of about {expected} heartbeats over {span:.0f} s of car uptime "
                  f"({100.0 * len(hbs) / expected:.0f}%); longest gap {worst:.1f} s")
        if span < 30:
            v.append(Verdict(name, "NOT TESTED", detail + " (listen for at least 30 s)"))
        elif worst > HB_GAP_S or min(gaps) < 0:
            v.append(Verdict(name, "FAIL", detail + (" (the car rebooted)" if min(gaps) < 0 else "")))
        else:
            v.append(Verdict(name, "PASS", detail))
        mot = [int(m["ts_ms"]) for m in ev.motion]
        if len(mot) >= 2 and mot[-1] > mot[0]:
            exp_m = (mot[-1] - mot[0]) // 200 + 1
            v.append(Verdict("Motion frames delivered (5 per second)", "INFO",
                             f"{len(mot)} of about {exp_m} ({100.0 * len(mot) / exp_m:.0f}%)"))
        return v
    name = "Telemetry backend"
    if ev.backend == "console":
        v.append(Verdict(name, "INFO", "console backend: frames go to USB, not MQTT (build with "
                         "-DTEL_BACKEND_WIFI=ON to test MQTT)"))
        return v
    if ev.backend == "wifi":
        v.append(Verdict(name, "INFO", "WiFi/MQTT backend"))
    name = "Link reached CONNECTED"
    if any(m.endswith("-> CONNECTED") for m in ev.link_moves):
        drops = sum(1 for m in ev.link_moves if m.startswith("CONNECTED ->"))
        v.append(Verdict(name, "PASS", f"{len(ev.link_moves)} link changes, {drops} drops from CONNECTED"))
    elif ev.link_moves or ev.backend == "wifi":
        v.append(Verdict(name, "FAIL", f"never CONNECTED; changes: {', '.join(ev.link_moves) or 'none'}"))
    else:
        v.append(Verdict(name, "NO DATA", "no [TEL] backend or link lines"))
    if ev.link_status:
        d = ev.link_status[-1]
        v.append(Verdict("Car-side publish counters (last status line)", "INFO",
                         f"sent {d['tx']}, publish fails {d['txf']}, skipped offline {d['off']}, outbox "
                         f"dropped {d['od']}. Delivery is only proven by the laptop: run listen --save"))
    return v


def format_verdicts(title: str, verdicts: List[Verdict]) -> str:
    lines = [f"==== {title} ===="]
    for x in verdicts:
        lines.append(f"[{x.result:<12}] {x.name}")
        lines.append(f"               {x.detail}")
    counts: Dict[str, int] = {}
    for x in verdicts:
        counts[x.result] = counts.get(x.result, 0) + 1
    lines.append("totals: " + ", ".join(f"{k} {n}" for k, n in sorted(counts.items())))
    return "\n".join(lines)
