"""Dashboard decoder tests (Buddy 1).  Run from tools/dashboard/:

    python -m unittest -v test_tel_decode

GOLDEN holds the exact bytes the firmware's C encoders produce for fixed
inputs; tests/host/test_tel.c checks the same strings against
src/common/data_repr.h.  If either side changes, one of the two tests fails
(Week 8 risk R3: dashboard and firmware disagree on the layout).
"""

import unittest

import tel_codec as codec

# (frame hex, expected decoded fields).  Inputs as in tests/host/test_tel.c.
GOLDEN = [
    ("01010f563412855f4760f0ffff0389070000",            # motion(0x123456, speed -12.3, 4567 cm, -1000, 123456)
     {"ts_ms": 0x123456, "speed_cmps": -12.3, "distance_cm": 4567, "enc_l": -1000, "enc_r": 123456}),
    ("01020ae80300ff0f80904d5b00",                      # sensor(1000, 4095, 2048, 400, -90 Q1, 45 Q1)
     {"ts_ms": 1000, "ir_r_raw": 4095, "ir_c_raw": 2048, "range_cm": 400, "pitch_deg": -45.0, "roll_deg": 22.5}),
    ("01030409030003",                                   # barcode(777, U-TURN)
     {"ts_ms": 777, "cmd": 3, "command": "U-TURN (D)"}),
    ("010405881300aa70",                                 # hump(5000, 42 mm, -31 Q1)
     {"ts_ms": 5000, "height_mm": 42, "pitch_deg": -15.5, "oor": False}),
    ("01050840e201192c43d41f",                           # obstacle(25 cm, idx 6, 123 mm, 340, 635, seq 2)
     {"ts_ms": 123456, "distance_cm": 25, "bearing_idx": 6, "bearing_deg": 90, "width_mm": 125,
      "clear_l_mm": 340, "clear_r_mm": 635, "seq": 2, "oor": False, "no_echo": False}),
    ("0106056300000501",                                 # fault(99, 0x105)
     {"ts_ms": 99, "bits": 0x105, "names": ["COLLISION", "FAULT_LATCHED", "WIFI_LOSS"]}),
    ("010706efbeaddef93f",                               # heartbeat(0xDEADBEEF, 9, 0x3FF)
     {"uptime_ms": 0xDEADBEEF, "state_id": 9, "state": "NAV_TURN_RIGHT", "fault_bits": 0x3FF}),
]


class GoldenFrames(unittest.TestCase):
    def test_golden_decode(self):
        for hx, expected in GOLDEN:
            with self.subTest(frame=hx):
                fr = codec.decode(bytes.fromhex(hx))
                for k, v in expected.items():
                    self.assertEqual(fr.fields[k], v, k)
                self.assertTrue(fr.summary())

    def test_commands_match_firmware(self):
        self.assertEqual(codec.encode_cmd("start"), ("car/cmd/start", bytes.fromhex("010800")))
        self.assertEqual(codec.encode_cmd("stop"), ("car/cmd/stop", bytes.fromhex("010900")))
        self.assertEqual(codec.encode_cmd("reset"), ("car/cmd/reset", bytes.fromhex("010a00")))


class Rejects(unittest.TestCase):
    """Same reject rules as drep_frame_check()."""

    def reject(self, hx, reason):
        with self.assertRaises(codec.FrameError) as ctx:
            codec.decode(bytes.fromhex(hx))
        self.assertEqual(str(ctx.exception), reason)

    def test_rules(self):
        self.reject("0107", "SHORT")
        self.reject("020900", "BAD_VERSION")
        self.reject("010000", "UNKNOWN_TYPE")
        self.reject("010b00", "UNKNOWN_TYPE")
        self.reject("01090100", "BAD_LENGTH")             # len field 1 for a 0-byte command
        self.reject("010706efbeaddef9", "BAD_LENGTH")     # heartbeat one byte short
        self.reject("010706efbeaddef93f00", "BAD_LENGTH")  # one byte too many


class ConsoleLog(unittest.TestCase):
    def test_parse_line(self):
        line = "[TEL] car/telemetry/heartbeat 010706efbeaddef93f\r\n"
        topic, data = codec.parse_log_line(line)
        self.assertEqual(topic, "car/telemetry/heartbeat")
        self.assertEqual(codec.decode(data).fields["state"], "NAV_TURN_RIGHT")
        self.assertIsNone(codec.parse_log_line("[STACK] tel hw=1234"))
        self.assertIsNone(codec.parse_log_line("[TEL] CONNECTED tx=10 txf=0"))

    def test_signed_edges(self):
        # motion speed field: 10-bit two's complement, x10 cm/s; -51.1 .. +51.1
        b = bytearray(bytes.fromhex("01010f" + "00" * 15))
        b[3 + 3] = 0x01
        b[3 + 4] = 0x02                                   # raw 0x201 = -511
        self.assertEqual(codec.decode(bytes(b)).fields["speed_cmps"], -51.1)


if __name__ == "__main__":
    unittest.main()
