"""Demo rubric & live obstacle/navigation tracker for the dashboard Overview.

Standard library only.  Watches decoded frames (tel_codec) and optional
console lines to keep track of:
  - instantaneous states: obstacle detection, line status (ON_LINE / LINE_LOST),
    junctions, barcode, hump, pickup/safety latch;
  - a newest-first run event timeline;
  - the 10-item demo rubric on one screen.

Never fabricates progress: simulated frames from APP_TEST self-tests are
marked SIMULATED rather than DONE, and TRACK_LOST is inferred from a drop out
of a moving VC state to READY_WAITING without a recent dashboard STOP command
(since no MQTT frame directly carries [LF] LINE_LOST or JUNCTION yet).
"""

from __future__ import annotations

import re
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import tel_codec as codec
import tel_views as views

OBS_DETECT_CM = views.OBS_DETECT_CM
REJOIN_HOLD_S = 3.0
POST_AVOID_LOST_S = 4.0
LOG_STOP_WINDOW_S = 5.0

JUNCTION_STATES = ("NAV_TURN_LEFT", "NAV_TURN_RIGHT", "NAV_U_TURN", "NAV_GO_STRAIGHT")
BC_TO_VC = {
    "LEFT (A)": "NAV_TURN_LEFT",
    "RIGHT (B)": "NAV_TURN_RIGHT",
    "STRAIGHT (C)": "NAV_GO_STRAIGHT",
    "U-TURN (D)": "NAV_U_TURN",
}
ACTION_TO_VC = {
    "LEFT": "NAV_TURN_LEFT",
    "RIGHT": "NAV_TURN_RIGHT",
    "STRAIGHT": "NAV_GO_STRAIGHT",
    "UTURN": "NAV_U_TURN",
    "U-TURN": "NAV_U_TURN",
}
ACTION_LABEL = {
    "LEFT": "LEFT",
    "RIGHT": "RIGHT",
    "STRAIGHT": "STRAIGHT",
    "UTURN": "U-TURN",
}

_LF_MOVE = re.compile(r"\[LF\] ([A-Z_]+) -> ([A-Z_]+)")
_BC_OK = re.compile(r"\[BC\]\s+#\d+\s+OK\s+([A-Z])\b")
_S5_OBS = views._S5_OBSTACLE
_S5_REACQ = re.compile(r"\[S5\] avoid -> IDLE \(reacquired=(\d)\)")

_DAY_S = 86400.0


def _elapsed_sod(earlier: float, later: float) -> float:
    """Seconds from earlier to later on a seconds-of-day clock (midnight-safe)."""
    d = later - earlier
    if d < -(_DAY_S / 2.0):
        d += _DAY_S
    return d


class DemoTracker:
    """Tracks live use cases, run events, and the 10 demo rubric checks."""

    def __init__(self) -> None:
        self.last_vc_state: str = ""
        self.prev_vc_state: str = ""
        self.last_uptime_s: Optional[float] = None
        self.last_fault_names: List[str] = []
        self.no_echo_active: bool = False
        self.last_speed: Optional[float] = None
        self.last_distance_cm: Optional[int] = None
        self.last_range_cm: Optional[int] = None
        self.last_ir: Optional[Tuple[int, int]] = None
        self.last_att: Optional[Tuple[float, float]] = None
        self.lf_state: str = ""
        self.last_dash_stop_t: Optional[float] = None
        self.last_dash_start_t: Optional[float] = None
        self.bc_map: Dict[str, str] = {"LEFT": "A", "RIGHT": "B", "STRAIGHT": "C", "UTURN": "D"}
        self._reset_run()

    def _reset_run(self) -> None:
        self.frames_seen: int = 0
        self.hb_seen: int = 0

        self.obs_count: int = 0
        self.obs_sim_count: int = 0
        self.obs_pending: bool = False
        self.last_obs: Optional[dict] = None
        self.obs_widths: List[int] = []
        self.console_obs_count: int = 0
        self.last_console_obs: Optional[dict] = None

        self.avoid_entered_count: int = 0
        self.avoid_rejoin_count: int = 0
        self.avoid_fail_count: int = 0
        self.avoid_start_up_s: Optional[float] = None
        self.avoid_release_up_s: Optional[float] = None
        self.avoid_last_dur_s: Optional[float] = None
        self.avoid_outcome: str = "NONE"    # NONE | ACTIVE | REJOINING | REJOINED | LOST_LINE
        self.console_reacquired: Optional[bool] = None
        self.last_avoid_note: str = ""

        self.line_follow_seen: bool = False
        self.line_follow_moved: bool = False
        self.line_lost_count: int = 0
        self.line_lost_active: bool = False
        self.line_lost_reason: str = ""
        self.line_lost_time: str = ""

        self.junction_count: int = 0
        self.last_junction: Optional[dict] = None

        self.bc_detect_count: int = 0
        self.bc_sim_count: int = 0
        self.bc_console_seen: bool = False
        self.bc_console_letter: str = ""
        self.bc_decoded: List[str] = []
        self.last_barcode: str = ""
        self.last_barcode_letter: str = ""
        self.last_barcode_action: str = ""
        self.last_barcode_vc: str = ""
        self.last_barcode_time: str = ""
        self.bc_followed: Optional[dict] = None
        self.bc_history: Deque[dict] = deque(maxlen=40)

        self.hump_count: int = 0
        self.hump_sim_count: int = 0
        self.last_hump: Optional[dict] = None
        self.peak_hump_mm: int = 0

        self.lift_seen: bool = False
        self.pickup_stop_seen: bool = False
        self.fault_latched_seen: bool = False
        self.last_fault_names_evt: List[str] = []
        self.pickup_time: str = ""

        self.events: Deque[dict] = deque(maxlen=80)

    def clear(self) -> None:
        """Reset rubric progress and event timeline while keeping live sensor/VC snapshots."""
        self._reset_run()
        self.prev_vc_state = self.last_vc_state
        if self.last_vc_state == "OBSTACLE_AVOIDANCE":
            self.avoid_outcome = "ACTIVE"
            self.avoid_entered_count = 1
            self.avoid_start_up_s = self.last_uptime_s
        elif self.last_vc_state == "NAV_LINE_TRACKING":
            self.line_follow_seen = True

    def _add_event(self, stamp: str, tag: str, severity: str, text: str) -> None:
        t = stamp or (f"up {self.last_uptime_s:.1f}s" if self.last_uptime_s is not None else "")
        self.events.append({"time": t, "tag": tag, "severity": severity, "text": text})

    def _resolve_letter(self, letter: str, fallback_action: str = "") -> Tuple[str, str, str]:
        """Return (action_label, command_display, vc_target) for a decoded letter A-Z."""
        ltr = (letter or "").strip().upper()[:1]
        for key in ("LEFT", "RIGHT", "STRAIGHT", "UTURN"):
            if self.bc_map.get(key) == ltr:
                act_lbl = ACTION_LABEL[key]
                return act_lbl, f"{act_lbl} ({ltr})", ACTION_TO_VC[key]
        if fallback_action and fallback_action != "UNMAPPED" and fallback_action in ACTION_TO_VC:
            act_lbl = "U-TURN" if fallback_action in ("UTURN", "U-TURN") else fallback_action
            return act_lbl, f"{act_lbl} ({ltr})" if ltr else act_lbl, ACTION_TO_VC[fallback_action]
        return "UNMAPPED", (f"LETTER {ltr}" if ltr else "UNMAPPED"), ""

    def set_barcode_map(self, mapping: Dict[str, str], stamp: str = "") -> Dict[str, str]:
        """Validate and update the 4-action barcode letter mapping (A-Z)."""
        norm: Dict[str, str] = {}
        for key in ("LEFT", "RIGHT", "STRAIGHT", "UTURN"):
            raw = mapping.get(key) or (mapping.get("U-TURN") if key == "UTURN" else None) or self.bc_map[key]
            ltr = str(raw).strip().upper()
            if len(ltr) != 1 or not ("A" <= ltr <= "Z"):
                raise ValueError(f"{key} must be a single letter A-Z")
            norm[key] = ltr
        if len(set(norm.values())) != 4:
            raise ValueError("Each event must be assigned to a distinct letter A-Z")
        self.bc_map = norm
        if self.last_barcode_letter:
            act, cmd, vc = self._resolve_letter(self.last_barcode_letter)
            self.last_barcode_action = act
            self.last_barcode = cmd
            self.last_barcode_vc = vc
        self._add_event(
            stamp, "BARCODE", "info",
            f"map updated: L={norm['LEFT']} R={norm['RIGHT']} S={norm['STRAIGHT']} U={norm['UTURN']}",
        )
        return dict(self.bc_map)

    def recent_log_stop(self, wall_s: Optional[float]) -> bool:
        """True if a replayed/console log saw '[DASH] sent stop:' shortly before wall_s."""
        if self.last_dash_stop_t is None or wall_s is None:
            return False
        if self.last_dash_start_t is not None and _elapsed_sod(self.last_dash_start_t, self.last_dash_stop_t) < 0.0:
            return False
        dt = _elapsed_sod(self.last_dash_stop_t, wall_s)
        return 0.0 <= dt <= LOG_STOP_WINDOW_S

    def on_command(self, cmd: str, result: str, detail: str, stamp: str) -> None:
        sev = "warn" if cmd in ("stop", "estop") else "info"
        text = f"{cmd.upper()} ({result})" + (f" · {detail}" if detail and detail != f"sent {cmd}" else "")
        self._add_event(stamp, "CMD", sev, text)

    def on_console(self, body: str, stamp: str, wall_s: Optional[float]) -> None:
        if body.startswith("[DASH] sent stop:"):
            self.last_dash_stop_t = wall_s
            return
        if body.startswith("[DASH] sent start:"):
            self.last_dash_start_t = wall_s
            return

        m = _LF_MOVE.search(body)
        if m:
            prev_lf, next_lf = m.group(1), m.group(2)
            self.lf_state = next_lf
            if next_lf == "LINE_LOST":
                self.line_lost_count += 1
                self.line_lost_active = True
                self.line_lost_reason = f"[LF] {prev_lf} -> LINE_LOST"
                self.line_lost_time = stamp
                self._add_event(stamp, "LINE LOST", "bad", body)
            elif next_lf == "JUNCTION":
                self.junction_count += 1
                self.last_junction = {"state": "JUNCTION", "time": stamp, "source": "[LF] console"}
                self._add_event(stamp, "JUNCTION", "info", body)
            elif next_lf == "ON_LINE":
                self.line_follow_seen = True
                self.line_lost_active = False
            return

        m = _S5_REACQ.search(body)
        if m:
            reacq = m.group(1) == "1"
            self.console_reacquired = reacq
            self.last_avoid_note = body
            if reacq:
                if self.avoid_outcome in ("ACTIVE", "REJOINING"):
                    self.avoid_outcome = "REJOINED"
                    self.avoid_rejoin_count += 1
                self._add_event(stamp, "REJOIN", "ok", body)
            else:
                self.avoid_outcome = "LOST_LINE"
                self.avoid_fail_count += 1
                self._add_event(stamp, "AVOID", "bad", body)
            return

        if body.startswith("[S5] avoid handoff") or body.startswith("[S5] avoid ABORT"):
            self.console_reacquired = False
            self.last_avoid_note = body
            self.avoid_outcome = "LOST_LINE"
            self._add_event(stamp, "AVOID", "bad", body)
            return

        if body.startswith("[S5] bypass") or body.startswith("[S5] blocked:"):
            self.last_avoid_note = body
            self._add_event(stamp, "AVOID", "warn", body)
            return

        m = _S5_OBS.search(body)
        if m:
            mm, deg, w, cl, cr = (int(x) for x in m.groups())
            if mm > 0:
                cm = mm // 10
                self.console_obs_count += 1
                if cm <= OBS_DETECT_CM:
                    self.obs_pending = True
                self.last_console_obs = {
                    "distance_cm": cm,
                    "bearing_deg": deg,
                    "width_mm": w,
                    "clear_l_mm": cl,
                    "clear_r_mm": cr,
                    "no_echo": False,
                    "time": stamp,
                    "source": "[S5] console",
                }
                if self.obs_count == 0:
                    self._add_event(stamp, "OBSTACLE", "warn", f"{cm} cm @ {deg} deg · width {w} mm (console)")
            return

        if body.startswith("[BC]"):
            low = body.lower()
            if "stub" not in low and "init" not in low and "sanity" not in low:
                self.bc_console_seen = True
                m_bc = _BC_OK.search(body)
                if m_bc:
                    self.bc_console_letter = m_bc.group(1)
                self._add_event(stamp, "BARCODE", "info", body)
            return

        if body.startswith("[IMU] LIFTED"):
            self.lift_seen = True
            self._add_event(stamp, "PICKUP", "bad", body)
            return

        if body.startswith("[SAFETY] BRAKE"):
            self.pickup_stop_seen = True
            self.pickup_time = stamp
            self._add_event(stamp, "PICKUP", "bad", body)

    def on_frame(self, fr: codec.Frame, wall_s: Optional[float], stamp: str,
                 sim: bool = False, recent_stop: bool = False) -> None:
        del wall_s
        self.frames_seen += 1
        f = fr.fields

        if fr.type == codec.FT_HEARTBEAT:
            self._on_heartbeat(f, stamp, recent_stop)
        elif fr.type == codec.FT_EVENT_OBSTACLE:
            if sim:
                self.obs_sim_count += 1
                return
            if f["no_echo"]:
                self.obs_pending = False
                return
            self.obs_count += 1
            if int(f["distance_cm"]) <= OBS_DETECT_CM:
                self.obs_pending = True
            obs = dict(f)
            obs["time"] = stamp
            obs["source"] = "MQTT"
            self.last_obs = obs
            if int(f["width_mm"]) > 0:
                self.obs_widths.append(int(f["width_mm"]))
            self._add_event(
                stamp, "OBSTACLE", "warn",
                f"{f['distance_cm']} cm @ {f['bearing_deg']} deg · width {f['width_mm']} mm "
                f"· clr L {f['clear_l_mm']} / R {f['clear_r_mm']} mm",
            )
        elif fr.type == codec.FT_EVENT_BARCODE:
            if sim:
                self.bc_sim_count += 1
                return
            letter = str(f.get("letter") or "")
            raw_action = str(f.get("action") or "")
            if letter:
                act, cmd, vc_target = self._resolve_letter(letter, "" if not f.get("mapped", True) else raw_action)
            else:
                cmd = str(f["command"])
                act = raw_action or cmd.split(" ")[0]
                vc_target = BC_TO_VC.get(cmd, "")
            self.bc_detect_count += 1
            self.bc_decoded.append(cmd)
            self.last_barcode = cmd
            self.last_barcode_letter = letter
            self.last_barcode_action = act
            self.last_barcode_vc = vc_target
            self.last_barcode_time = stamp
            self.bc_history.append({
                "seq": self.bc_detect_count,
                "time": stamp,
                "letter": letter or "?",
                "action": act,
                "command": cmd,
                "vc_target": vc_target,
                "source": "MQTT",
            })
            self._add_event(stamp, "BARCODE", "info", f"decoded {cmd}")
        elif fr.type == codec.FT_EVENT_HUMP:
            if sim:
                self.hump_sim_count += 1
                return
            if self.last_vc_state in ("READY_WAITING", "SAFETY_FAULT_LATCHED", "INITIALISING"):
                return
            h = int(f["height_mm"])
            p = float(f["pitch_deg"])
            self.hump_count += 1
            self.last_hump = {"height_mm": h, "pitch_deg": p, "oor": bool(f["oor"]), "time": stamp}
            if h > 0:
                self.peak_hump_mm = h
            self._add_event(
                stamp, "HUMP", "info",
                f"height {h} mm · pitch {p:+.1f} deg" if h > 0 else f"pitch {p:+.1f} deg (height 0 mm)",
            )
        elif fr.type == codec.FT_EVENT_FAULT:
            if sim:
                return
            names = list(f["names"])
            self.last_fault_names_evt = names
            if "LIFTED" in names:
                self.lift_seen = True
            if "FAULT_LATCHED" in names:
                self.fault_latched_seen = True
            self._add_event(stamp, "FAULT", "bad", f"fault: {', '.join(names) or '-'}")
        elif fr.type == codec.FT_TELEMETRY_MOTION:
            self.last_speed = float(f["speed_cmps"])
            self.last_distance_cm = int(f["distance_cm"])
            if abs(self.last_speed) > 1.0 and self.last_vc_state == "NAV_LINE_TRACKING":
                self.line_follow_moved = True
        elif fr.type == codec.FT_TELEMETRY_SENSOR:
            if not sim and f["flags_present"]:
                if f["range_valid"]:
                    self.last_range_cm = int(f["range_cm"])
                if f["ir_valid"]:
                    self.last_ir = (int(f["ir_c_raw"]), int(f["ir_r_raw"]))
                if f["att_valid"]:
                    self.last_att = (float(f["pitch_deg"]), float(f["roll_deg"]))

    def _on_heartbeat(self, f: dict, stamp: str, recent_stop: bool) -> None:
        self.hb_seen += 1
        state = str(f["state"])
        uptime_s = int(f["uptime_ms"]) / 1000.0
        fault_names = list(f["fault_names"])
        self.last_uptime_s = uptime_s
        self.last_fault_names = fault_names
        self.no_echo_active = "NO_ECHO" in fault_names
        if "LIFTED" in fault_names:
            self.lift_seen = True
        if state == "NAV_LINE_TRACKING":
            self.line_follow_seen = True

        prev = self.prev_vc_state
        if prev and state != prev:
            self._handle_vc_transition(prev, state, uptime_s, fault_names, stamp, recent_stop)

        if (self.avoid_outcome == "REJOINING"
                and state in views.MOVING_STATES
                and state != "OBSTACLE_AVOIDANCE"
                and self.avoid_release_up_s is not None
                and (uptime_s - self.avoid_release_up_s) >= REJOIN_HOLD_S):
            self.avoid_outcome = "REJOINED"
            self.avoid_rejoin_count += 1
            self._add_event(stamp, "REJOIN", "ok", f"confirmed line rejoin (tracking steady >= {REJOIN_HOLD_S:.0f} s)")

        if state == "SAFETY_FAULT_LATCHED" and (
            "LIFTED" in fault_names or "FAULT_LATCHED" in fault_names or self.lift_seen
        ):
            self.pickup_stop_seen = True
            self.fault_latched_seen = True

        self.last_vc_state = state
        self.prev_vc_state = state

    def _handle_vc_transition(self, prev: str, state: str, uptime_s: float,
                              fault_names: List[str], stamp: str, recent_stop: bool) -> None:
        if state == "OBSTACLE_AVOIDANCE":
            self.avoid_entered_count += 1
            self.avoid_start_up_s = uptime_s
            self.avoid_release_up_s = None
            self.console_reacquired = None
            self.avoid_outcome = "ACTIVE"
            self.line_lost_active = False
            self._add_event(stamp, "AVOID", "warn", f"entered OBSTACLE_AVOIDANCE (from {prev})")
            return

        if state == "NAV_LINE_TRACKING":
            self.line_follow_seen = True
            self.line_lost_active = False
            if prev == "OBSTACLE_AVOIDANCE":
                self.obs_pending = False
                dur = max(0.0, uptime_s - self.avoid_start_up_s) if self.avoid_start_up_s is not None else None
                self.avoid_last_dur_s = dur
                self.avoid_release_up_s = uptime_s
                if self.console_reacquired is False:
                    self.avoid_outcome = "LOST_LINE"
                    self.avoid_fail_count += 1
                    self._add_event(stamp, "AVOID", "bad", "left OBSTACLE_AVOIDANCE without reacquiring line")
                elif self.console_reacquired is True:
                    self.avoid_outcome = "REJOINED"
                    self.avoid_rejoin_count += 1
                    msg = f"rejoined line after {dur:.1f} s" if dur is not None else "rejoined line"
                    self._add_event(stamp, "REJOIN", "ok", msg)
                else:
                    self.avoid_outcome = "REJOINING"
                    msg = (f"OBSTACLE_AVOIDANCE -> NAV_LINE_TRACKING ({dur:.1f} s) · checking line hold"
                           if dur is not None else "OBSTACLE_AVOIDANCE -> NAV_LINE_TRACKING · checking line hold")
                    self._add_event(stamp, "REJOIN", "info", msg)
            else:
                self._add_event(stamp, "VC", "ok", f"{prev} -> NAV_LINE_TRACKING")
            return

        if state in JUNCTION_STATES:
            self.junction_count += 1
            self.line_lost_active = False
            self.last_junction = {"state": state, "time": stamp, "source": "VC heartbeat"}
            expected_vc = self.last_barcode_vc or BC_TO_VC.get(self.last_barcode, "")
            if self.last_barcode and expected_vc == state:
                self.bc_followed = {
                    "barcode": self.last_barcode,
                    "letter": self.last_barcode_letter,
                    "action": self.last_barcode_action,
                    "vc_state": state,
                    "time": stamp,
                }
                self._add_event(stamp, "JUNCTION", "ok", f"junction -> {state} (followed barcode {self.last_barcode})")
            else:
                self._add_event(stamp, "JUNCTION", "info", f"junction detected -> {state}")
            return

        if state == "READY_WAITING" and prev in views.MOVING_STATES:
            if recent_stop:
                if self.avoid_outcome == "REJOINING":
                    self.avoid_outcome = "REJOINED"
                    self.avoid_rejoin_count += 1
                self._add_event(stamp, "VC", "info", f"{prev} -> READY_WAITING (STOP commanded)")
                return
            self.line_lost_count += 1
            self.line_lost_active = True
            self.line_lost_time = stamp
            after_avoid = (
                prev == "OBSTACLE_AVOIDANCE"
                or (self.avoid_outcome == "REJOINING"
                    and self.avoid_release_up_s is not None
                    and (uptime_s - self.avoid_release_up_s) <= POST_AVOID_LOST_S)
            )
            if after_avoid:
                self.line_lost_reason = (
                    "car stopped to READY_WAITING after obstacle avoidance without STOP (did not rejoin line)"
                )
                self.avoid_outcome = "LOST_LINE"
                self.avoid_fail_count += 1
                self._add_event(stamp, "LINE LOST", "bad",
                                f"TRACK_LOST after obstacle avoidance ({prev} -> READY_WAITING)")
            else:
                self.line_lost_reason = f"VC dropped {prev} -> READY_WAITING without a STOP command (TRACK_LOST)"
                self._add_event(stamp, "LINE LOST", "bad",
                                f"TRACK_LOST: {prev} -> READY_WAITING (no STOP command)")
            return

        if state == "SAFETY_FAULT_LATCHED":
            if self.avoid_outcome == "REJOINING" and prev == "NAV_LINE_TRACKING":
                self.avoid_outcome = "REJOINED"
                self.avoid_rejoin_count += 1
            self.line_lost_active = False
            self.fault_latched_seen = True
            if "LIFTED" in fault_names or "FAULT_LATCHED" in fault_names or self.lift_seen:
                self.pickup_stop_seen = True
                self.pickup_time = stamp
                self._add_event(stamp, "PICKUP", "bad",
                                f"picked up / SAFETY_FAULT_LATCHED ({', '.join(fault_names) or 'FAULT_LATCHED'})")
            else:
                self._add_event(stamp, "FAULT", "bad",
                                f"entered SAFETY_FAULT_LATCHED ({', '.join(fault_names) or '-'})")
            return

        self._add_event(stamp, "VC", "info", f"{prev} -> {state}")

    def _active_obs(self) -> Optional[dict]:
        return self.last_obs or self.last_console_obs

    def _live_tiles(self, obs: Optional[dict]) -> Dict[str, dict]:
        tiles: Dict[str, dict] = {}

        # 1. Obstacle
        if self.last_vc_state == "OBSTACLE_AVOIDANCE":
            val = (f"{obs['distance_cm']} cm @ {obs['bearing_deg']} deg (w {obs['width_mm']} mm)"
                   if obs else "avoidance active")
            det = self.last_avoid_note or (
                f"L {obs['clear_l_mm']} / R {obs['clear_r_mm']} mm" if obs else "VC in OBSTACLE_AVOIDANCE"
            )
            tiles["obstacle"] = {"status": "AVOIDING", "tone": "warn", "value": val, "detail": det}
        elif self.avoid_outcome == "LOST_LINE":
            tiles["obstacle"] = {
                "status": "REJOIN FAILED", "tone": "bad",
                "value": f"width {obs['width_mm']} mm" if obs else "line not reacquired",
                "detail": "TRACK_LOST after obstacle avoidance",
            }
        elif self.obs_pending and obs and int(obs["distance_cm"]) <= OBS_DETECT_CM:
            fwd = f" · fwd {self.last_range_cm} cm" if self.last_range_cm is not None else ""
            tiles["obstacle"] = {
                "status": "DETECTED", "tone": "warn",
                "value": f"{obs['distance_cm']} cm @ {obs['bearing_deg']} deg",
                "detail": f"w {obs['width_mm']} mm · L {obs['clear_l_mm']} / R {obs['clear_r_mm']} mm{fwd}",
            }
        elif self.last_range_cm is not None and 0 < self.last_range_cm <= OBS_DETECT_CM:
            tiles["obstacle"] = {
                "status": "NEAR", "tone": "warn",
                "value": f"fwd {self.last_range_cm} cm",
                "detail": "inside 25 cm threshold (waiting for profile)",
            }
        elif self.avoid_outcome in ("REJOINING", "REJOINED"):
            val = (f"fwd {self.last_range_cm} cm" if self.last_range_cm
                   else ("no echo" if self.no_echo_active or self.last_range_cm == 0 else "bypass complete"))
            tiles["obstacle"] = {
                "status": "CLEARED", "tone": "ok" if self.avoid_outcome == "REJOINED" else "info",
                "value": val,
                "detail": (f"rejoined line ({self.avoid_rejoin_count}x)" if self.avoid_outcome == "REJOINED"
                           else "bypass ended; checking line hold"),
            }
        elif self.obs_sim_count and not self.obs_count:
            tiles["obstacle"] = {
                "status": "SIMULATED", "tone": "warn",
                "value": "S1 self-test only",
                "detail": "APP_TEST simulation, not a real sensor",
            }
        else:
            if self.last_range_cm == 0 or (self.last_range_cm is None and self.no_echo_active):
                val = "no echo"
            elif self.last_range_cm is not None:
                val = f"fwd {self.last_range_cm} cm"
            else:
                val = "—"
            tiles["obstacle"] = {
                "status": "CLEAR",
                "tone": "ok" if (self.last_range_cm is not None or self.hb_seen > 0) else "off",
                "value": val,
                "detail": (f"last profile {obs['distance_cm']} cm @ {obs['bearing_deg']} deg"
                           if obs else "nothing within 25 cm"),
            }

        # 2. Line
        if self.line_lost_active:
            tiles["line"] = {
                "status": "LINE LOST", "tone": "bad",
                "value": "TRACK_LOST",
                "detail": self.line_lost_reason or "car stopped without STOP command",
            }
        elif self.avoid_outcome == "REJOINING":
            tiles["line"] = {
                "status": "REJOINING", "tone": "info",
                "value": "NAV_LINE_TRACKING",
                "detail": "checking 3 s steady tracking after avoidance",
            }
        elif self.last_vc_state == "NAV_LINE_TRACKING" or self.lf_state in ("ON_LINE", "REACQUIRING"):
            det = (f"IR C={self.last_ir[0]} R={self.last_ir[1]}" if self.last_ir
                   else (f"speed {self.last_speed:+.1f} cm/s" if self.last_speed is not None else "following line"))
            tiles["line"] = {
                "status": "ON LINE", "tone": "ok",
                "value": self.lf_state or "NAV_LINE_TRACKING",
                "detail": det,
            }
        elif self.last_vc_state == "OBSTACLE_AVOIDANCE":
            tiles["line"] = {
                "status": "SUSPENDED", "tone": "warn",
                "value": "AVOIDANCE",
                "detail": "line following paused during obstacle bypass",
            }
        elif self.last_vc_state:
            det = (f"line lost seen {self.line_lost_count}x earlier" if self.line_lost_count
                   else (f"IR C={self.last_ir[0]} R={self.last_ir[1]}" if self.last_ir else "waiting to start"))
            tiles["line"] = {"status": "IDLE", "tone": "off", "value": self.last_vc_state, "detail": det}
        else:
            tiles["line"] = {"status": "WAITING", "tone": "off", "value": "—", "detail": "no heartbeat yet"}

        # 3. Junction
        if self.last_vc_state in JUNCTION_STATES:
            bc_tail = f" · barcode {self.last_barcode}" if self.last_barcode else ""
            tiles["junction"] = {
                "status": "ACTIVE", "tone": "info",
                "value": self.last_vc_state,
                "detail": f"junction #{self.junction_count}{bc_tail}",
            }
        elif self.last_junction:
            tiles["junction"] = {
                "status": "DETECTED", "tone": "ok",
                "value": self.last_junction["state"],
                "detail": f"count {self.junction_count} · {self.last_junction['time'] or 'seen'}",
            }
        else:
            tiles["junction"] = {
                "status": "NONE", "tone": "off",
                "value": "none yet",
                "detail": "no junction seen in this run",
            }

        # 4. Barcode
        if self.bc_detect_count > 0:
            det = (f"followed ({self.bc_followed['vc_state']})" if self.bc_followed
                   else f"decoded {self.bc_detect_count}x · waiting for junction")
            tiles["barcode"] = {"status": "DECODED", "tone": "ok", "value": self.last_barcode, "detail": det}
        elif self.bc_sim_count > 0:
            tiles["barcode"] = {
                "status": "SIMULATED", "tone": "warn",
                "value": "S1 self-test only",
                "detail": "APP_TEST simulation",
            }
        elif self.bc_console_seen:
            tiles["barcode"] = {
                "status": "DETECTED", "tone": "info",
                "value": "console [BC]",
                "detail": "console activity seen; awaiting decoded frame",
            }
        else:
            tiles["barcode"] = {
                "status": "NONE", "tone": "off",
                "value": "none yet",
                "detail": "no barcode decoded yet",
            }

        # 5. Hump
        if self.hump_count > 0 and self.last_hump:
            h = int(self.last_hump["height_mm"])
            tiles["hump"] = {
                "status": "MEASURED" if h > 0 else "EVENT (0 mm)",
                "tone": "ok" if h > 0 else "warn",
                "value": f"{h} mm (peak {self.peak_hump_mm} mm)",
                "detail": f"pitch {self.last_hump['pitch_deg']:+.1f} deg · count {self.hump_count}",
            }
        elif self.hump_sim_count > 0:
            tiles["hump"] = {
                "status": "SIMULATED", "tone": "warn",
                "value": "S1 self-test only",
                "detail": "APP_TEST simulation",
            }
        else:
            tiles["hump"] = {
                "status": "NONE", "tone": "off",
                "value": "none yet",
                "detail": f"live pitch {self.last_att[0]:+.1f} deg" if self.last_att else "no hump event yet",
            }

        # 6. Pickup / Safety
        latched_now = (
            self.last_vc_state == "SAFETY_FAULT_LATCHED"
            or "LIFTED" in self.last_fault_names
            or "FAULT_LATCHED" in self.last_fault_names
        )
        if latched_now:
            tiles["pickup"] = {
                "status": "PICKED UP / STOPPED", "tone": "bad",
                "value": ", ".join(self.last_fault_names) or self.last_vc_state,
                "detail": "motors braked · put car flat & still 0.5 s, then RESET",
            }
        elif self.pickup_stop_seen:
            tiles["pickup"] = {
                "status": "VERIFIED", "tone": "ok",
                "value": "pickup stop seen",
                "detail": f"reset back to {self.last_vc_state or 'READY_WAITING'}",
            }
        else:
            tiles["pickup"] = {
                "status": "ARMED", "tone": "ok" if self.hb_seen > 0 else "off",
                "value": "on ground" if self.hb_seen > 0 else "—",
                "detail": "pick car up ~5 cm to verify stop & end demo",
            }

        return tiles

    def _rubric_items(self, src: dict, obs: Optional[dict]) -> List[dict]:
        items: List[dict] = []

        def add(rid: str, num: int, title: str, status: str, value: str, detail: str) -> None:
            items.append({"id": rid, "num": num, "title": title, "status": status, "value": value, "detail": detail})

        # 1. Detect a barcode
        if self.bc_detect_count > 0:
            add("bc_detect", 1, "Detect a barcode", "DONE", f"{self.bc_detect_count} detected",
                f"last: {self.last_barcode}" + (f" at {self.last_barcode_time}" if self.last_barcode_time else ""))
        elif self.bc_console_seen:
            add("bc_detect", 1, "Detect a barcode", "ACTIVE", "[BC] seen",
                "console barcode line seen; waiting for decoded frame")
        elif self.bc_sim_count > 0:
            add("bc_detect", 1, "Detect a barcode", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only, not a real barcode")
        else:
            add("bc_detect", 1, "Detect a barcode", "WAITING", "—", "no barcode detected in this run")

        # 2. Decode the barcode and obtain the code
        if self.bc_detect_count > 0:
            add("bc_decode", 2, "Decode the barcode and obtain the code", "DONE", self.last_barcode,
                "history: " + ", ".join(self.bc_decoded))
        elif self.bc_sim_count > 0:
            add("bc_decode", 2, "Decode the barcode and obtain the code", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only")
        else:
            add("bc_decode", 2, "Decode the barcode and obtain the code", "WAITING", "—", "no barcode frame yet")

        # 3. Measure hump height
        if self.hump_count > 0 and self.peak_hump_mm > 0:
            last_h = int(self.last_hump["height_mm"]) if self.last_hump else self.peak_hump_mm
            add("hump_height", 3, "Measure hump height", "DONE", f"peak {self.peak_hump_mm} mm",
                f"{self.hump_count} event(s), last {last_h} mm")
        elif self.hump_count > 0:
            add("hump_height", 3, "Measure hump height", "WARN", "0 mm (pitch only)",
                f"{self.hump_count} hump event(s), all reported height 0 mm")
        elif self.hump_sim_count > 0:
            add("hump_height", 3, "Measure hump height", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only")
        else:
            add("hump_height", 3, "Measure hump height", "WAITING", "—", "no hump event yet")

        # 4. Follow the line
        if self.line_lost_active:
            add("follow_line", 4, "Follow the line", "FAIL", "LINE LOST",
                self.line_lost_reason or "TRACK_LOST")
        elif self.last_vc_state == "NAV_LINE_TRACKING":
            spd = f"{self.last_speed:+.1f} cm/s" if self.last_speed is not None else "NAV_LINE_TRACKING"
            add("follow_line", 4, "Follow the line", "ACTIVE", spd,
                f"dist {self.last_distance_cm} cm" if self.last_distance_cm is not None else "following line now")
        elif self.line_follow_moved or self.line_follow_seen:
            det = (f"line lost {self.line_lost_count}x in run" if self.line_lost_count
                   else f"last VC {self.last_vc_state or '—'}")
            add("follow_line", 4, "Follow the line", "DONE", "tracked line", det)
        else:
            add("follow_line", 4, "Follow the line", "WAITING", "—", "press START to follow the line")

        # 5. Follow the instruction from the barcode
        if self.bc_followed:
            add("bc_follow", 5, "Follow the instruction from the barcode", "DONE",
                f"{self.bc_followed['barcode']} -> {self.bc_followed['vc_state']}",
                f"executed at {self.bc_followed['time'] or 'junction'}")
        elif self.bc_detect_count > 0:
            exp_vc = self.last_barcode_vc or BC_TO_VC.get(self.last_barcode, self.last_barcode)
            add("bc_follow", 5, "Follow the instruction from the barcode", "ACTIVE",
                f"pending: {self.last_barcode}",
                f"waiting for junction to execute {exp_vc}" if exp_vc else f"letter {self.last_barcode_letter} is not assigned to a turn event")
        elif self.bc_sim_count > 0:
            add("bc_follow", 5, "Follow the instruction from the barcode", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only")
        else:
            add("bc_follow", 5, "Follow the instruction from the barcode", "WAITING", "—",
                "needs decoded barcode + junction")

        # 6. Detect an obstacle
        if self.obs_count > 0 or self.console_obs_count > 0 or self.avoid_entered_count > 0:
            if obs and int(obs["distance_cm"]) <= OBS_DETECT_CM:
                add("obs_detect", 6, "Detect an obstacle", "DONE",
                    f"{obs['distance_cm']} cm @ {obs['bearing_deg']} deg",
                    f"source {obs.get('source', 'telemetry')}, {self.obs_count or self.console_obs_count} profile(s)")
            elif self.avoid_entered_count > 0:
                add("obs_detect", 6, "Detect an obstacle", "DONE", "avoidance triggered",
                    f"entered OBSTACLE_AVOIDANCE {self.avoid_entered_count}x")
            else:
                add("obs_detect", 6, "Detect an obstacle", "WARN",
                    f"{obs['distance_cm']} cm (> {OBS_DETECT_CM} cm)",
                    "profile beyond 25 cm detect threshold")
        elif self.last_range_cm is not None and 0 < self.last_range_cm <= OBS_DETECT_CM:
            add("obs_detect", 6, "Detect an obstacle", "ACTIVE", f"fwd {self.last_range_cm} cm",
                "within 25 cm; waiting for scan profile")
        elif self.obs_sim_count > 0:
            add("obs_detect", 6, "Detect an obstacle", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only")
        else:
            add("obs_detect", 6, "Detect an obstacle", "WAITING", "—", "no obstacle profile yet")

        # 7. Measure obstacle width
        if self.obs_widths or (obs and int(obs["width_mm"]) > 0):
            w_mm = int(obs["width_mm"]) if obs and int(obs["width_mm"]) > 0 else self.obs_widths[-1]
            clr = f"L {obs['clear_l_mm']} / R {obs['clear_r_mm']} mm" if obs else ""
            add("obs_width", 7, "Measure obstacle width", "DONE", f"{w_mm} mm",
                clr or "measured by ultrasonic sweep")
        elif obs:
            add("obs_width", 7, "Measure obstacle width", "WARN", "0 mm",
                "obstacle profile reported width 0 mm")
        elif self.obs_sim_count > 0:
            add("obs_width", 7, "Measure obstacle width", "SIMULATED", "S1 self-test",
                "APP_TEST simulation only")
        else:
            add("obs_width", 7, "Measure obstacle width", "WAITING", "—", "no obstacle profile yet")

        # 8. Navigate around the obstacle and rejoin the line
        if self.last_vc_state == "OBSTACLE_AVOIDANCE" or self.avoid_outcome == "ACTIVE":
            add("obs_rejoin", 8, "Navigate around the obstacle and rejoin the line", "ACTIVE",
                "avoiding now", self.last_avoid_note or "VC in OBSTACLE_AVOIDANCE")
        elif self.avoid_outcome == "REJOINING":
            add("obs_rejoin", 8, "Navigate around the obstacle and rejoin the line", "ACTIVE",
                "checking line hold",
                f"returned to NAV_LINE_TRACKING; confirming {REJOIN_HOLD_S:.0f} s without TRACK_LOST")
        elif self.avoid_outcome == "LOST_LINE":
            add("obs_rejoin", 8, "Navigate around the obstacle and rejoin the line", "FAIL",
                "LOST LINE", self.last_avoid_note or self.line_lost_reason or "did not rejoin line")
        elif self.avoid_outcome == "REJOINED" or self.avoid_rejoin_count > 0:
            dur = f" ({self.avoid_last_dur_s:.1f} s)" if self.avoid_last_dur_s is not None else ""
            add("obs_rejoin", 8, "Navigate around the obstacle and rejoin the line", "DONE",
                f"rejoined{dur}",
                f"bypass complete ({self.avoid_rejoin_count}x)")
        else:
            add("obs_rejoin", 8, "Navigate around the obstacle and rejoin the line", "WAITING",
                "—", "waiting for obstacle avoidance run")

        # 9. Send telemetry data via MQTT
        if src.get("kind") == "mqtt":
            if src.get("mqtt_connected") and self.frames_seen > 0:
                add("mqtt_tel", 9, "Send telemetry data via MQTT", "ACTIVE",
                    f"{self.frames_seen} frames", f"{self.hb_seen} heartbeats · car {src.get('car_status') or 'seen'}")
            else:
                add("mqtt_tel", 9, "Send telemetry data via MQTT", "WARN",
                    src.get("car_status") or "no frames yet",
                    "broker connected" if src.get("mqtt_connected") else "broker NOT connected")
        else:
            add("mqtt_tel", 9, "Send telemetry data via MQTT", "WAITING",
                src.get("kind") or "none", "not live MQTT (connect broker above to test)")

        # 10. Stop motors when the car is picked up (signals end of demo)
        if self.pickup_stop_seen:
            faults = ", ".join(self.last_fault_names or self.last_fault_names_evt) or "LIFTED / FAULT_LATCHED"
            val = "STOPPED & LATCHED" if self.last_vc_state == "SAFETY_FAULT_LATCHED" else "verified"
            add("pickup_stop", 10, "Stop motors when picked up (end of demo)", "DONE", val, faults)
        elif self.lift_seen:
            add("pickup_stop", 10, "Stop motors when picked up (end of demo)", "ACTIVE",
                "LIFTED seen", "waiting for SAFETY_FAULT_LATCHED / BRAKE")
        else:
            add("pickup_stop", 10, "Stop motors when picked up (end of demo)", "WAITING", "—",
                "pick car up ~5 cm at end of run")

        return items

    def _alerts(self, obs: Optional[dict]) -> List[dict]:
        alerts: List[dict] = []
        if (self.last_vc_state == "SAFETY_FAULT_LATCHED"
                or "LIFTED" in self.last_fault_names
                or "FAULT_LATCHED" in self.last_fault_names):
            faults = ", ".join(self.last_fault_names) or "FAULT_LATCHED"
            alerts.append({
                "severity": "bad",
                "text": f"CAR STOPPED / {faults} — put car flat & still for 0.5 s, then press RESET.",
            })
        if self.line_lost_active:
            alerts.append({
                "severity": "bad",
                "text": f"LINE LOST (TRACK_LOST): {self.line_lost_reason}",
            })
        if self.last_vc_state == "OBSTACLE_AVOIDANCE":
            extra = f" ({obs['distance_cm']} cm, width {obs['width_mm']} mm)" if obs else ""
            alerts.append({
                "severity": "warn",
                "text": f"OBSTACLE DETECTED — car is in OBSTACLE_AVOIDANCE{extra}",
            })
        if self.last_vc_state in JUNCTION_STATES:
            alerts.append({
                "severity": "info",
                "text": f"JUNCTION DETECTED — VC executing {self.last_vc_state}",
            })
        return alerts

    def snapshot(self, src: dict, ctl_state: str = "") -> dict:
        del ctl_state
        obs = self._active_obs()
        rubric = self._rubric_items(src, obs)
        done = sum(1 for r in rubric if r["status"] in ("DONE", "ACTIVE"))
        return {
            "live": self._live_tiles(obs),
            "rubric": rubric,
            "summary": {"done": done, "total": len(rubric)},
            "events": list(reversed(self.events)),
            "alerts": self._alerts(obs),
            "barcode": {
                "detected": self.bc_detect_count > 0 or self.bc_console_seen,
                "decoded": self.bc_detect_count > 0,
                "count": self.bc_detect_count,
                "last_letter": self.last_barcode_letter or self.bc_console_letter,
                "last_action": self.last_barcode_action,
                "last_command": self.last_barcode,
                "last_time": self.last_barcode_time,
                "expected_vc": self.last_barcode_vc,
                "followed": self.bc_followed,
                "mapping": dict(self.bc_map),
                "history": list(reversed(self.bc_history)),
            },
        }
