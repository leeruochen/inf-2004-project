"""Unit tests for tel_demo.DemoTracker (Overview live state & 10-item rubric)."""

from __future__ import annotations

import unittest

import tel_codec as codec
import tel_demo as demo
import test_tel_views as tv


def _decode_log(line: str) -> codec.Frame:
    parsed = codec.parse_log_line(line)
    assert parsed is not None
    return codec.decode(parsed[1])


def _barcode(ts: int, cmd_idx: int) -> codec.Frame:
    w = tv._W(codec.FT_EVENT_BARCODE).put(ts, 24).put(cmd_idx, 2)
    return codec.decode(bytes.fromhex(w.hex()))


def _motion(ts: int, speed: float, dist: int = 0) -> codec.Frame:
    w = tv._W(codec.FT_TELEMETRY_MOTION).put(ts, 24).put(int(round(speed * 10)), 10).put(dist, 16).put(0, 32).put(0, 32)
    return codec.decode(bytes.fromhex(w.hex()))


def _fault(ts: int, *bits: str) -> codec.Frame:
    mask = sum(1 << codec.SAFETY_BITS.index(b) for b in bits)
    w = tv._W(codec.FT_EVENT_FAULT).put(ts, 24).put(mask, 10)
    return codec.decode(bytes.fromhex(w.hex()))


def _rubric_map(snap: dict) -> dict:
    return {r["id"]: r for r in snap["rubric"]}


class DemoRubricTests(unittest.TestCase):
    def _mqtt_src(self, connected: bool = True, car_status: str = "ONLINE") -> dict:
        return {"kind": "mqtt", "mqtt_connected": connected, "car_status": car_status}

    def test_initial_state(self):
        tr = demo.DemoTracker()
        snap = tr.snapshot({"kind": "none"})
        self.assertEqual(len(snap["rubric"]), 10)
        self.assertEqual(snap["summary"], {"done": 0, "total": 10})
        self.assertTrue(all(r["status"] == "WAITING" for r in snap["rubric"]))
        self.assertEqual(snap["alerts"], [])

    def test_obstacle_avoidance_and_rejoin_steady_hold(self):
        tr = demo.DemoTracker()
        tr.on_frame(_decode_log(tv.heartbeat(1000, "READY_WAITING")), 10.0, "12:00:01.000")
        tr.on_frame(_decode_log(tv.heartbeat(2000, "NAV_LINE_TRACKING")), 11.0, "12:00:02.000")
        tr.on_frame(_motion(2100, 15.0, 20), 11.1, "12:00:02.100")

        # Obstacle detected + width measured
        tr.on_frame(_decode_log(tv.obstacle(2500, 18, 90, 120, 300, 400)), 11.5, "12:00:02.500")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["obstacle"]["status"], "DETECTED")
        rm = _rubric_map(snap)
        self.assertEqual(rm["obs_detect"]["status"], "DONE")
        self.assertEqual(rm["obs_width"]["status"], "DONE")
        self.assertEqual(rm["obs_width"]["value"], "120 mm")

        # Enter avoidance -> return to line tracking (checking hold) -> steady 3 s -> DONE
        tr.on_frame(_decode_log(tv.heartbeat(3000, "OBSTACLE_AVOIDANCE")), 12.0, "12:00:03.000")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["obstacle"]["status"], "AVOIDING")
        self.assertEqual(snap["live"]["line"]["status"], "SUSPENDED")
        self.assertEqual(_rubric_map(snap)["obs_rejoin"]["status"], "ACTIVE")

        tr.on_frame(_decode_log(tv.heartbeat(5000, "NAV_LINE_TRACKING")), 14.0, "12:00:05.000")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["line"]["status"], "REJOINING")
        self.assertEqual(snap["live"]["obstacle"]["status"], "CLEARED")
        self.assertEqual(_rubric_map(snap)["obs_rejoin"]["status"], "ACTIVE")

        tr.on_frame(_decode_log(tv.heartbeat(8100, "NAV_LINE_TRACKING")), 17.1, "12:00:08.100")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["obstacle"]["status"], "CLEARED")
        self.assertEqual(snap["live"]["line"]["status"], "ON LINE")
        self.assertEqual(_rubric_map(snap)["obs_rejoin"]["status"], "DONE")

    def test_obstacle_avoidance_console_reacquired(self):
        tr = demo.DemoTracker()
        tr.on_frame(_decode_log(tv.heartbeat(1000, "NAV_LINE_TRACKING")), 10.0, "12:00:01.000")
        tr.on_frame(_decode_log(tv.heartbeat(2000, "OBSTACLE_AVOIDANCE")), 11.0, "12:00:02.000")
        tr.on_console("[S5] avoid -> IDLE (reacquired=1)", "12:00:03.500", 12.5)
        tr.on_frame(_decode_log(tv.heartbeat(3600, "NAV_LINE_TRACKING")), 12.6, "12:00:03.600")
        snap = tr.snapshot({"kind": "usb"})
        self.assertEqual(_rubric_map(snap)["obs_rejoin"]["status"], "DONE")
        self.assertEqual(snap["live"]["obstacle"]["status"], "CLEARED")

    def test_obstacle_avoidance_rejoin_failed_track_lost(self):
        tr = demo.DemoTracker()
        tr.on_frame(_decode_log(tv.heartbeat(1000, "NAV_LINE_TRACKING")), 10.0, "12:00:01.000")
        tr.on_frame(_decode_log(tv.obstacle(1500, 18, 90, 110, 300, 350)), 10.5, "12:00:01.500")
        tr.on_frame(_decode_log(tv.heartbeat(2000, "OBSTACLE_AVOIDANCE")), 11.0, "12:00:02.000")
        tr.on_frame(_decode_log(tv.heartbeat(4000, "NAV_LINE_TRACKING")), 13.0, "12:00:04.000")
        # Car drops to READY_WAITING within 4 s without STOP -> failed rejoin / TRACK_LOST
        tr.on_frame(_decode_log(tv.heartbeat(5500, "READY_WAITING")), 14.5, "12:00:05.500", recent_stop=False)
        snap = tr.snapshot(self._mqtt_src())
        rm = _rubric_map(snap)
        self.assertEqual(rm["obs_rejoin"]["status"], "FAIL")
        self.assertEqual(rm["follow_line"]["status"], "FAIL")
        self.assertEqual(snap["live"]["line"]["status"], "LINE LOST")
        self.assertEqual(snap["live"]["obstacle"]["status"], "REJOIN FAILED")
        self.assertTrue(any("LINE LOST" in a["text"] for a in snap["alerts"]))

    def test_line_lost_vs_user_stop(self):
        tr = demo.DemoTracker()
        tr.on_frame(_decode_log(tv.heartbeat(1000, "NAV_LINE_TRACKING")), 10.0, "12:00:01.000")
        tr.on_frame(_decode_log(tv.heartbeat(2000, "READY_WAITING")), 11.0, "12:00:02.000", recent_stop=True)
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["line"]["status"], "IDLE")
        self.assertNotEqual(_rubric_map(snap)["follow_line"]["status"], "FAIL")

        # Now start again and drop without STOP
        tr.on_frame(_decode_log(tv.heartbeat(3000, "NAV_LINE_TRACKING")), 12.0, "12:00:03.000")
        tr.on_frame(_decode_log(tv.heartbeat(4000, "READY_WAITING")), 13.0, "12:00:04.000", recent_stop=False)
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["line"]["status"], "LINE LOST")
        self.assertEqual(_rubric_map(snap)["follow_line"]["status"], "FAIL")

    def test_console_lf_transitions(self):
        tr = demo.DemoTracker()
        tr.on_console("[LF] IDLE -> ON_LINE  c=1700 r=200 dc=900 dr=50 vc=4", "12:00:01.000", 1.0)
        tr.on_console("[LF] ON_LINE -> JUNCTION  c=1800 r=1750 dc=950 dr=900 vc=4", "12:00:02.000", 2.0)
        tr.on_console("[LF] ON_LINE -> LINE_LOST  c=200 r=180 dc=20 dr=20 vc=4", "12:00:03.000", 3.0)
        snap = tr.snapshot({"kind": "usb"})
        self.assertEqual(snap["live"]["junction"]["status"], "DETECTED")
        self.assertEqual(snap["live"]["line"]["status"], "LINE LOST")
        self.assertEqual(tr.line_lost_count, 1)
        self.assertEqual(tr.junction_count, 1)

    def test_barcode_and_junction_follow(self):
        tr = demo.DemoTracker()
        tr.on_frame(_decode_log(tv.heartbeat(1000, "NAV_LINE_TRACKING")), 10.0, "12:00:01.000")
        # Junction without barcode -> junction detected, barcode still waiting
        tr.on_frame(_decode_log(tv.heartbeat(2000, "NAV_GO_STRAIGHT")), 11.0, "12:00:02.000")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(snap["live"]["junction"]["status"], "ACTIVE")
        self.assertEqual(_rubric_map(snap)["bc_detect"]["status"], "WAITING")
        self.assertEqual(_rubric_map(snap)["bc_follow"]["status"], "WAITING")

        # Return to tracking, decode LEFT (A), then take NAV_TURN_LEFT
        tr.on_frame(_decode_log(tv.heartbeat(3000, "NAV_LINE_TRACKING")), 12.0, "12:00:03.000")
        tr.on_frame(_barcode(3500, 0), 12.5, "12:00:03.500")
        tr.on_frame(_decode_log(tv.heartbeat(4000, "NAV_TURN_LEFT")), 13.0, "12:00:04.000")
        snap = tr.snapshot(self._mqtt_src())
        rm = _rubric_map(snap)
        self.assertEqual(rm["bc_detect"]["status"], "DONE")
        self.assertEqual(rm["bc_decode"]["status"], "DONE")
        self.assertEqual(rm["bc_decode"]["value"], "LEFT (A)")
        self.assertEqual(rm["bc_follow"]["status"], "DONE")
        self.assertIn("NAV_TURN_LEFT", rm["bc_follow"]["value"])

    def test_hump_and_pickup_and_simulated_and_clear(self):
        tr = demo.DemoTracker()
        # Simulated frames never mark DONE
        tr.on_frame(_decode_log(tv.obstacle(1000, 15, 90, 100, 200, 200)), 1.0, "12:00:01.000", sim=True)
        tr.on_frame(_barcode(1100, 2), 1.1, "12:00:01.100", sim=True)
        tr.on_frame(_decode_log(tv.hump(1200, height=12, pitch=7.5)), 1.2, "12:00:01.200", sim=True)
        rm = _rubric_map(tr.snapshot(self._mqtt_src()))
        self.assertEqual(rm["obs_detect"]["status"], "SIMULATED")
        self.assertEqual(rm["bc_decode"]["status"], "SIMULATED")
        self.assertEqual(rm["hump_height"]["status"], "SIMULATED")

        # Real hump height=0 -> WARN, then height=14 -> DONE
        tr.on_frame(_decode_log(tv.hump(2000, height=0, pitch=6.0)), 2.0, "12:00:02.000")
        self.assertEqual(_rubric_map(tr.snapshot(self._mqtt_src()))["hump_height"]["status"], "WARN")
        tr.on_frame(_decode_log(tv.hump(3000, height=14, pitch=8.0)), 3.0, "12:00:03.000")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(_rubric_map(snap)["hump_height"]["status"], "DONE")
        self.assertEqual(_rubric_map(snap)["hump_height"]["value"], "peak 14 mm")
        self.assertEqual(_rubric_map(snap)["mqtt_tel"]["status"], "ACTIVE")

        # Pickup: LIFTED + SAFETY_FAULT_LATCHED
        tr.on_frame(_fault(3500, "LIFTED", "FAULT_LATCHED"), 3.5, "12:00:03.500")
        tr.on_frame(_decode_log(tv.heartbeat(3600, "SAFETY_FAULT_LATCHED", ("LIFTED", "FAULT_LATCHED"))),
                    3.6, "12:00:03.600")
        snap = tr.snapshot(self._mqtt_src())
        self.assertEqual(_rubric_map(snap)["pickup_stop"]["status"], "DONE")
        self.assertEqual(snap["live"]["pickup"]["status"], "PICKED UP / STOPPED")
        self.assertTrue(any("RESET" in a["text"] for a in snap["alerts"]))

        # clear() resets rubric & events while keeping live VC state
        tr.clear()
        snap2 = tr.snapshot(self._mqtt_src())
        self.assertEqual(tr.last_vc_state, "SAFETY_FAULT_LATCHED")
        self.assertEqual(_rubric_map(snap2)["hump_height"]["status"], "WAITING")
        self.assertEqual(_rubric_map(snap2)["pickup_stop"]["status"], "WAITING")
        self.assertEqual(snap2["events"], [])


if __name__ == "__main__":
    unittest.main()
