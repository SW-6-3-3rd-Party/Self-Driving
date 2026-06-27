import math
import struct
import unittest

from HPVC.hpvc_aeb import (
    AebConfig,
    AebState,
    HpvcAebController,
    decode_brake_command,
    decode_front_packet,
    decode_middle_packet,
    encode_brake_command,
)
from MIDDLE.generate_contract_fixture import canonical_packet
from MIDDLE.protocol import pack_packet


class HpvcAebTest(unittest.TestCase):
    def test_decodes_front_aeb1_packet(self):
        payload = (
            struct.pack("<4sBBHI", b"AEB1", 1, 0x07, 0xEE03, 42)
            + struct.pack("<IHHH", 1234, 180, 250, 260)
        )
        frame = decode_front_packet(payload, arrival_time=10.0)
        self.assertEqual(frame.sequence, 42)
        self.assertTrue(frame.tof_valid)
        self.assertTrue(frame.left_valid)
        self.assertTrue(frame.right_valid)
        self.assertTrue(math.isclose(frame.tof_front_m, 0.18, abs_tol=1e-9))
        self.assertTrue(math.isclose(frame.ultrasonic_left_m, 0.25, abs_tol=1e-9))

    def test_decodes_middle_v2_person_packet(self):
        frame = decode_middle_packet(pack_packet(canonical_packet()), arrival_time=20.0)
        self.assertEqual(frame.sequence, 0xFFFFFFFE)
        self.assertTrue(frame.person_detection_valid)
        self.assertEqual(len(frame.persons), 3)
        self.assertTrue(math.isclose(frame.persons[0].confidence, 0.86, abs_tol=1e-6))

    def test_controller_full_brakes_for_close_person_and_tof(self):
        front = decode_front_packet(
            struct.pack("<4sBBHI", b"AEB1", 1, 0x04, 0, 1)
            + struct.pack("<IHHH", 100, 150, 0, 0),
            arrival_time=100.0,
        )
        middle = decode_middle_packet(pack_packet(canonical_packet()), arrival_time=100.0)
        controller = HpvcAebController(AebConfig())
        decision = controller.decide(front, middle, now=100.1)
        self.assertEqual(decision.state, AebState.FULL_BRAKE)
        self.assertEqual(decision.brake_percent, 100)
        self.assertTrue(decision.person_in_path)

    def test_brake_command_round_trip(self):
        front = decode_front_packet(
            struct.pack("<4sBBHI", b"AEB1", 1, 0x04, 0, 1)
            + struct.pack("<IHHH", 100, 150, 0, 0),
            arrival_time=100.0,
        )
        middle = decode_middle_packet(pack_packet(canonical_packet()), arrival_time=100.0)
        decision = HpvcAebController(AebConfig()).decide(front, middle, now=100.0)
        packet = encode_brake_command(decision, timestamp_us=123)
        decoded = decode_brake_command(packet)
        self.assertEqual(decoded["state"], AebState.FULL_BRAKE)
        self.assertEqual(decoded["brake_percent"], 100)
        self.assertEqual(decoded["timestamp_us"], 123)


if __name__ == "__main__":
    unittest.main()
