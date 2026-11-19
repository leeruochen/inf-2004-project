"""Tests for tel_control.py: remote commands, arming and confirmation (S1-H08).

    python -m unittest -v test_tel_control

Standard library only, times injected.  A command may show CONFIRMED only
after a heartbeat newer than the command shows the expected state.
"""

from __future__ import annotations

import unittest

import tel_control as ctl


class Base(unittest.TestCase):
    def live(self, state="READY_WAITING", t=100.0):
        c = ctl.Controller()
        c.set_connected(True, t)
        c.set_car_status("ONLINE")
        c.on_heartbeat(state, t)
        return c


class Refusals(unittest.TestCase):
    def test_start_needs_arming_even_with_no_car(self):
        c = ctl.Controller()
        self.assertEqual(c.refuse_reason("start", 1.0), ctl.START_NOT_ARMED)
        self.assertIn("--allow-motion", ctl.START_NOT_ARMED)

    def test_pause_is_refused_with_the_reason(self):
        self.assertIn("no PAUSED state", ctl.Controller().refuse_reason("pause", 1.0))

    def test_stop_needs_only_the_broker(self):
        c = ctl.Controller()
        self.assertIn("broker", c.refuse_reason("stop", 1.0))
        c.set_connected(True, 1.0)
        self.assertEqual(c.refuse_reason("stop", 1.0), "")          # car silent: still allowed
        self.assertEqual(c.refuse_reason("estop", 1.0), "")

    def test_unknown_command(self):
        self.assertIn("unknown", ctl.Controller().refuse_reason("jump", 1.0))


class Arming(Base):
    def test_arm_needs_a_live_car(self):
        c = ctl.Controller()
        self.assertIn("broker", c.arm(1.0))
        c.set_connected(True, 1.0)
        self.assertIn("no heartbeat", c.arm(1.0))
        c.on_heartbeat("READY_WAITING", 1.0)
        c.set_car_status("OFFLINE (last will)")
        self.assertIn("OFFLINE", c.arm(1.0))

    def test_one_start_per_arming(self):
        c = self.live()
        self.assertEqual(c.arm(100.0), "")
        self.assertEqual(c.refuse_reason("start", 100.5), "")
        c.sent("start", 100.5, "t", [7])
        self.assertFalse(c.armed(100.6))
        self.assertEqual(c.refuse_reason("start", 100.6), ctl.START_NOT_ARMED)

    def test_arming_expires(self):
        c = self.live()
        c.arm(100.0)
        c.on_heartbeat("READY_WAITING", 100.0 + ctl.ARM_S - 1)
        self.assertTrue(c.armed(100.0 + ctl.ARM_S - 1))
        c.tick(100.0 + ctl.ARM_S + 0.1)
        self.assertFalse(c.armed(100.0 + ctl.ARM_S + 0.1))
        self.assertIn("expired", c.disarm_reason)

    def test_losing_the_link_disarms(self):
        c = self.live()
        c.arm(100.0)
        c.set_connected(False, 101.0)
        self.assertFalse(c.armed(101.0))

    def test_start_refused_when_latched_or_moving(self):
        c = self.live("SAFETY_FAULT_LATCHED")
        c.armed_until = 200.0
        self.assertIn("RESET it first", c.refuse_reason("start", 100.5))
        c.on_heartbeat("NAV_LINE_TRACKING", 100.6)
        self.assertIn("only works from READY_WAITING", c.refuse_reason("start", 100.7))

    def test_stale_heartbeat_blocks_start(self):
        c = self.live()
        c.arm(100.0)
        self.assertIn("no heartbeat for", c.refuse_reason("start", 100.0 + ctl.HB_FRESH_S + 1))


class Confirmation(Base):
    def test_start_confirmed_by_a_newer_heartbeat_only(self):
        c = self.live()
        c.arm(100.0)
        cmd = c.sent("start", 101.0, "t", [3])
        c.on_heartbeat("NAV_LINE_TRACKING", 100.9)       # older than the command: no proof
        self.assertEqual(cmd.result, "PENDING")
        c.on_heartbeat("NAV_LINE_TRACKING", 101.2)
        self.assertEqual(cmd.result, "CONFIRMED")
        self.assertIn("NAV_LINE_TRACKING", cmd.detail)

    def test_start_not_confirmed_when_nothing_changes(self):
        c = self.live()
        c.arm(100.0)
        cmd = c.sent("start", 101.0, "t", [3])
        c.on_heartbeat("READY_WAITING", 101.5)
        c.tick(101.0 + ctl.CONFIRM_S + 0.1)
        self.assertEqual(cmd.result, "NOT CONFIRMED")
        self.assertIn("stayed in READY_WAITING", cmd.detail)

    def test_no_heartbeat_after_the_command(self):
        c = self.live()
        cmd = c.sent("stop", 101.0, "t", [4])
        c.tick(101.0 + ctl.CONFIRM_S + 0.1)
        self.assertEqual(cmd.result, "NOT CONFIRMED")
        self.assertIn("no heartbeat since the command", cmd.detail)

    def test_stop_waits_for_the_speed_to_drop(self):
        c = self.live("NAV_LINE_TRACKING")
        cmd = c.sent("stop", 101.0, "t", [5])
        c.on_motion(18.0, 101.1)
        c.on_heartbeat("READY_WAITING", 101.15)
        self.assertEqual(cmd.result, "PENDING")           # VC stopped, wheels still turning
        c.on_motion(0.0, 101.4)
        self.assertEqual(cmd.result, "CONFIRMED")
        self.assertIn("speed +0.0", cmd.detail)

    def test_stop_when_already_stopped(self):
        c = self.live()
        cmd = c.sent("stop", 101.0, "t", [5])
        c.on_heartbeat("READY_WAITING", 102.0)
        self.assertEqual(cmd.result, "CONFIRMED")
        self.assertIn("already stopped", cmd.detail)

    def test_answer_before_logging_keeps_the_true_state_before(self):
        # Regression (found end to end, 19 Nov): an E-STOP burst takes 0.2 s, and the car's
        # READY_WAITING heartbeat came back before sent() ran, so the entry said "already stopped".
        c = self.live("NAV_LINE_TRACKING")
        before = c.state
        c.on_heartbeat("READY_WAITING", 101.15)
        cmd = c.sent("estop", 101.0, "t", [1, 2, 3], before)
        c.on_heartbeat("READY_WAITING", 101.3)
        self.assertEqual(cmd.result, "CONFIRMED")
        self.assertNotIn("already stopped", cmd.detail)

    def test_stop_to_a_silent_car_warns(self):
        c = ctl.Controller()
        c.set_connected(True, 1.0)
        cmd = c.sent("estop", 1.0, "t", [1, 2, 3])
        self.assertIn("may not reach the car", cmd.warning)
        self.assertEqual(cmd.mids, {1, 2, 3})

    def test_reset(self):
        c = self.live("SAFETY_FAULT_LATCHED")
        cmd = c.sent("reset", 101.0, "t", [6])
        c.on_heartbeat("SAFETY_FAULT_LATCHED", 101.3)
        self.assertEqual(cmd.result, "PENDING")
        c.on_heartbeat("READY_WAITING", 101.6)
        self.assertEqual(cmd.result, "CONFIRMED")
        self.assertIn("latch cleared", cmd.detail)
        noop = c.sent("reset", 102.0, "t", [8])
        c.on_heartbeat("READY_WAITING", 102.2)
        self.assertEqual(noop.result, "CONFIRMED")
        self.assertIn("nothing was latched", noop.detail)
        c2 = self.live("SAFETY_FAULT_LATCHED")
        stuck = c2.sent("reset", 101.0, "t", [9])
        c2.on_heartbeat("SAFETY_FAULT_LATCHED", 101.5)
        c2.tick(101.0 + ctl.CONFIRM_S + 0.1)
        self.assertEqual(stuck.result, "NOT CONFIRMED")
        self.assertIn("still latched", stuck.detail)

    def test_broker_ack(self):
        c = self.live()
        cmd = c.sent("stop", 101.0, "t", [11])
        self.assertEqual(cmd.broker, "waiting")
        c.on_puback(11, 101.05)
        self.assertEqual(cmd.broker, "ok")
        self.assertEqual(c.sent("stop", 101.1, "t", [None]).broker, "n/a")

    def test_ack_before_the_command_is_logged(self):
        c = self.live()
        c.on_puback(21, 101.0)                  # a local broker answers within a millisecond
        self.assertEqual(c.sent("stop", 101.0, "t", [21]).broker, "ok")

    def test_estop_needs_every_ack(self):
        c = self.live()
        cmd = c.sent("estop", 101.0, "t", [31, 32, 33])
        c.on_puback(31, 101.01)
        c.on_puback(32, 101.02)
        self.assertEqual(cmd.broker, "waiting")
        c.on_puback(33, 101.03)
        self.assertEqual(cmd.broker, "ok")

    def test_refused_entries_are_logged(self):
        c = ctl.Controller()
        cmd = c.refused("pause", ctl.PAUSE_MISSING, 1.0, "t")
        snap = c.snapshot(1.0)
        self.assertEqual(snap["history"][0]["result"], "REFUSED")
        self.assertEqual(cmd.broker, "-")
        self.assertEqual(set(snap["can"]), {"start", "stop", "reset", "pause"})


if __name__ == "__main__":
    unittest.main()
