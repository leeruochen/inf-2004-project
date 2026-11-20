"""Remote commands with honest confirmation (Buddy 1, dashboard v2).

The car accepts START, STOP and RESET on car/cmd/* (tel_wire.c, data_repr.h
DREP_FT_CMD_*).  It sends no acknowledgement of its own, so a command counts
as done only when the car's telemetry shows the result:

    sent        published, QoS 1
    broker      the broker's PUBACK came back (it holds the message; the car
                may still be offline)
    CONFIRMED   a heartbeat received after the command shows the expected
                state (the car sends an extra heartbeat on every state change,
                task_tel.c), and for STOP the newest speed is ~0
    NOT CONFIRMED  nothing showed it within CONFIRM_S, with the reason

Safety rules, on top of the car's own (the VC and Safety decide; nothing
here can override them):
    START   needs the broker connected, the car ONLINE, a fresh heartbeat and
            the car waiting in READY_WAITING.  (The "arm motion" step and its
            confirm dialog were removed on 19 Nov 2026 at the owner's request:
            one press of START drives the car.)
    STOP    is never blocked while the broker is connected.  If the car is
            silent it still goes out, with a warning that it may not arrive.
    E-STOP  STOP sent ESTOP_REPEATS times in a row.
    PAUSE   refused: the firmware has no such command or VC state yet.

Pure logic with injected times (time.monotonic() in tel_web); tel_web does
the publishing.  Standard library only.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Set, Tuple

import tel_views as views

CONFIRM_S = 2.0             # a heartbeat follows a state change within ~0.2 s
HB_FRESH_S = 3.0            # tel_dashboard HEARTBEAT_STALE_S
ESTOP_REPEATS = 3
ESTOP_SPACING_S = 0.1
STILL_CMPS = 1.0            # |speed| at or below this counts as stopped
HISTORY = 50

SUPPORTED = ("start", "stop", "reset")
PAUSE_MISSING = ("PAUSE is not in the car firmware yet: there is no pause command (data_repr.h DREP_FT_CMD_*) "
                 "and no PAUSED state in the Vehicle Controller (Buddy 2). Use STOP; the proposal is in "
                 "tools/dashboard/README.md.")
MOVING = tuple(views.MOVING_STATES)
STOPPED = ("READY_WAITING", "SAFETY_FAULT_LATCHED", "INITIALISING")


@dataclass
class Command:
    id: int
    cmd: str                    # start | stop | reset | estop | pause
    sent_at: float              # monotonic
    wall: str                   # laptop clock, for the table
    state_before: str
    mids: Set[int] = field(default_factory=set)
    acked: Set[int] = field(default_factory=set)
    broker: str = "waiting"     # waiting | ok (every publish acknowledged) | n/a | -
    result: str = "PENDING"     # PENDING | CONFIRMED | NOT CONFIRMED | REFUSED
    detail: str = ""
    warning: str = ""
    done_at: Optional[float] = None

    def as_dict(self, now: float) -> dict:
        return {"id": self.id, "cmd": self.cmd, "time": self.wall, "broker": self.broker, "result": self.result,
                "detail": self.detail, "warning": self.warning, "state_before": self.state_before,
                "age_s": round(now - self.sent_at, 1),
                "took_s": None if self.done_at is None else round(self.done_at - self.sent_at, 2)}


class Controller:
    """What the car is doing, what may be sent, and what each command achieved."""

    def __init__(self) -> None:
        self.connected = False
        self.car_status = ""            # ONLINE | OFFLINE (last will) | ""
        self.state = ""                 # newest heartbeat state
        self.hb_at: Optional[float] = None
        self.speed: Optional[float] = None
        self.speed_at: Optional[float] = None
        self.history: Deque[Command] = deque(maxlen=HISTORY)
        self._next = 1
        self._early_acks: List[int] = []

    # -- what the car reports ----------------------------------------------------

    def set_connected(self, ok: bool, now: float) -> None:
        self.connected = ok

    def set_car_status(self, status: str) -> None:
        self.car_status = status

    def on_heartbeat(self, state: str, now: float) -> None:
        self.state, self.hb_at = state, now
        self._judge(now)

    def on_motion(self, speed: float, now: float) -> None:
        self.speed, self.speed_at = speed, now
        self._judge(now)

    def on_puback(self, mid: int, now: float) -> None:
        for c in self.history:
            if mid in c.mids:
                c.acked.add(mid)
                if c.acked >= c.mids:
                    c.broker = "ok"
                return
        # On a local broker the PUBACK can arrive before sent() has logged the command.
        self._early_acks.append(mid)

    # -- checks ----------------------------------------------------------------------

    def hb_age(self, now: float) -> Optional[float]:
        return None if self.hb_at is None else now - self.hb_at

    def _link_problem(self, now: float, need_car: bool) -> str:
        if not self.connected:
            return "not connected to the broker"
        if not need_car:
            return ""
        if self.car_status and self.car_status != "ONLINE":
            return f"the car is {self.car_status}"
        age = self.hb_age(now)
        if age is None:
            return "no heartbeat from the car yet"
        if age > HB_FRESH_S:
            return f"no heartbeat for {age:.1f} s"
        return ""

    def checklist(self, now: float) -> List[dict]:
        """The pre-START checks the Drive page shows, each with ok and a short text."""
        age = self.hb_age(now)
        items = [
            ("broker", self.connected, "connected to the broker" if self.connected else "not connected"),
            ("car", self.car_status == "ONLINE", f"car {self.car_status or 'status unknown'}"),
            ("heartbeat", age is not None and age <= HB_FRESH_S,
             "no heartbeat yet" if age is None else f"last heartbeat {age:.1f} s ago"),
            ("state", self.state in ("READY_WAITING", "INITIALISING"),
             f"VC {self.state or 'unknown'}" + (" (RESET first)" if self.state == "SAFETY_FAULT_LATCHED" else "")),
        ]
        return [{"name": n, "ok": bool(ok), "text": t} for n, ok, t in items]

    def refuse_reason(self, cmd: str, now: float) -> str:
        """'' if cmd may be sent now, else why not."""
        if cmd == "pause":
            return PAUSE_MISSING
        if cmd not in SUPPORTED and cmd != "estop":
            return f"unknown command {cmd!r}"
        if cmd in ("stop", "estop", "reset"):
            return self._link_problem(now, need_car=False)
        why = self._link_problem(now, need_car=True)
        if why:
            return why
        if self.state == "SAFETY_FAULT_LATCHED":
            return "the car is in SAFETY_FAULT_LATCHED: RESET it first (car still for 0.5 s)"
        if self.state not in ("READY_WAITING", "INITIALISING"):
            return f"the car is in {self.state}: START only works from READY_WAITING"
        return ""

    # -- the command log ------------------------------------------------------------

    def _new(self, cmd: str, now: float, wall: str, state_before: Optional[str] = None) -> Command:
        c = Command(self._next, cmd, now, wall, self.state if state_before is None else state_before)
        self._next += 1
        self.history.append(c)
        return c

    def refused(self, cmd: str, reason: str, now: float, wall: str) -> Command:
        c = self._new(cmd, now, wall)
        c.result, c.detail, c.broker, c.done_at = "REFUSED", reason, "-", now
        return c

    def sent(self, cmd: str, now: float, wall: str, mids: List[Optional[int]],
             state_before: Optional[str] = None) -> Command:
        """Record a command that was published (mids: paho message ids; None when unknown).

        'now' and 'state_before' are taken just before publishing: the car can answer
        (a new heartbeat) before this is called, especially after an E-STOP burst.
        """
        c = self._new(cmd, now, wall, state_before)
        c.mids = {m for m in mids if m is not None}
        if not c.mids:
            c.broker = "n/a"
        early = [m for m in self._early_acks if m in c.mids]
        self._early_acks = [m for m in self._early_acks if m not in c.mids]
        for m in early:
            self.on_puback(m, now)
        if cmd in ("stop", "estop"):
            why = self._link_problem(now, need_car=True)
            if why:
                c.warning = f"{why}: STOP may not reach the car. Be ready to stop it by hand."
        return c

    def _judge(self, now: float) -> None:
        for c in self.history:
            if c.result != "PENDING":
                continue
            fresh_hb = self.hb_at is not None and self.hb_at > c.sent_at
            fresh_v = self.speed_at is not None and self.speed_at > c.sent_at
            if c.cmd == "start" and fresh_hb:
                if self.state in MOVING:
                    self._done(c, "CONFIRMED", f"car in {self.state}", now)
                elif self.state == "SAFETY_FAULT_LATCHED":
                    self._done(c, "NOT CONFIRMED", "the car is latched: RESET first", now)
            elif c.cmd in ("stop", "estop") and fresh_hb and self.state in STOPPED:
                if fresh_v and self.speed is not None and abs(self.speed) > STILL_CMPS:
                    continue                    # braking: wait for the speed to drop
                tail = f", speed {self.speed:+.1f} cm/s" if fresh_v and self.speed is not None else ""
                already = " (it was already stopped)" if c.state_before in STOPPED else ""
                self._done(c, "CONFIRMED", f"car in {self.state}{tail}{already}", now)
            elif c.cmd == "reset" and fresh_hb and self.state != "SAFETY_FAULT_LATCHED":
                text = ("latch cleared" if c.state_before == "SAFETY_FAULT_LATCHED" else "nothing was latched")
                self._done(c, "CONFIRMED", f"{text}: car in {self.state}", now)

    def _done(self, c: Command, result: str, detail: str, now: float) -> None:
        c.result, c.detail, c.done_at = result, detail, now

    def tick(self, now: float) -> None:
        for c in self.history:
            if c.result != "PENDING" or now - c.sent_at < CONFIRM_S:
                continue
            fresh_hb = self.hb_at is not None and self.hb_at > c.sent_at
            if not fresh_hb:
                why = "no heartbeat since the command: the car may be offline or out of range"
            elif c.cmd == "start":
                why = f"the car stayed in {self.state}"
                if self.state == "READY_WAITING":
                    # Seen 19 Nov: a refused boot calibration makes the line follower report
                    # TRACK_LOST at once, faster than a heartbeat can show NAV_LINE_TRACKING.
                    why += (" (if this repeats: the line follower may have lost the line at once; power the "
                            "car up in the start pose, CENTRE on black and RIGHT on white)")
            elif c.cmd in ("stop", "estop"):
                if self.state not in STOPPED:
                    why = f"the car is still in {self.state}"
                elif self.speed is not None:
                    why = f"still moving at {self.speed:+.1f} cm/s"
                else:
                    why = "no speed reading after the command"
            else:
                why = "still latched: keep the car still for 0.5 s; the fault itself must be gone"
            self._done(c, "NOT CONFIRMED", why, now)

    # -- output ----------------------------------------------------------------------------

    def snapshot(self, now: float) -> dict:
        self.tick(now)
        age = self.hb_age(now)
        return {
            "connected": self.connected, "car_status": self.car_status, "state": self.state,
            "hb_age_s": None if age is None else round(age, 1),
            "speed": self.speed, "checklist": self.checklist(now),
            "can": {cmd: self.refuse_reason(cmd, now) for cmd in ("start", "stop", "reset", "pause")},
            "history": [c.as_dict(now) for c in reversed(self.history)],
        }


def estop_plan() -> Tuple[int, float]:
    """(repeats, spacing seconds) for the emergency stop burst."""
    return ESTOP_REPEATS, ESTOP_SPACING_S
