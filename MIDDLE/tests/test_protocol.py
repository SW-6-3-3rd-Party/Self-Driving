import math
import unittest

from MIDDLE.protocol import (
    FLAG_CAMERA_VALID,
    FLAG_LANE_VALID,
    FLAG_PERSON_DETECTION_VALID,
    PACKET_SIZE,
    PAYLOAD_FLOAT_COUNT,
    LaneBoundary,
    PerceptionPacket,
    PersonDetection,
    pack_packet,
    unpack_packet,
)
from MIDDLE.generate_contract_fixture import canonical_packet


class ProtocolTest(unittest.TestCase):
    def test_round_trip(self):
        source = PerceptionPacket(
            sequence=123,
            frame_timestamp_us=456,
            ultrasonic_timestamp_us=789,
            flags=FLAG_CAMERA_VALID | FLAG_LANE_VALID | FLAG_PERSON_DETECTION_VALID,
            left=LaneBoundary(0.2, 0.0, 0.1, 0.18, 0.9),
            right=LaneBoundary(0.2, 0.0, 0.1, -0.22, 0.8),
            side_left_distance_m=0.75,
            side_right_distance_m=0.80,
            person_count=1,
            persons=(PersonDetection(1.0, 0.91, 0.50, 0.45, 0.20, 0.40),),
        )
        data = pack_packet(source)
        self.assertEqual(len(data), PACKET_SIZE)
        result = unpack_packet(data)
        self.assertEqual(result.sequence, source.sequence)
        self.assertEqual(result.flags, source.flags)
        self.assertTrue(math.isclose(result.left.lateral_offset_m, 0.18, abs_tol=1e-6))
        self.assertTrue(math.isclose(result.right.lateral_offset_m, -0.22, abs_tol=1e-6))
        self.assertEqual(result.person_count, 1)
        self.assertTrue(math.isclose(result.persons[0].confidence, 0.91, abs_tol=1e-6))
        self.assertTrue(math.isclose(result.persons[0].center_x_norm, 0.50, abs_tol=1e-6))

    def test_crc_rejects_corruption(self):
        packet = PerceptionPacket(
            sequence=1,
            frame_timestamp_us=2,
            ultrasonic_timestamp_us=3,
            flags=0,
            left=LaneBoundary(),
            right=LaneBoundary(),
            side_left_distance_m=math.nan,
            side_right_distance_m=math.nan,
        )
        data = bytearray(pack_packet(packet))
        data[20] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC"):
            unpack_packet(bytes(data))

    def test_frozen_matlab_contract_fixture(self):
        data = pack_packet(canonical_packet())
        self.assertEqual(len(data), PACKET_SIZE)
        self.assertEqual(data[:4], b"MID2")
        self.assertEqual(data[4], 2)
        self.assertEqual(data[5], 31)
        self.assertEqual(data[6:8], PAYLOAD_FLOAT_COUNT.to_bytes(2, "little"))
        result = unpack_packet(data)
        self.assertEqual(result.sequence, 0xFFFFFFFE)
        self.assertEqual(result.frame_timestamp_us, 1_234_567_890_123)
        self.assertEqual(result.person_count, 1)


if __name__ == "__main__":
    unittest.main()
