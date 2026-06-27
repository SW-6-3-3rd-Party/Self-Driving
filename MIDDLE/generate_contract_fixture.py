"""Generate the canonical protocol-v2 packet used by MATLAB contract tests."""

from __future__ import annotations

import argparse

from .protocol import (
    FLAG_CAMERA_VALID,
    FLAG_LANE_VALID,
    FLAG_LEFT_ULTRASONIC_VALID,
    FLAG_PERSON_DETECTION_VALID,
    FLAG_RIGHT_ULTRASONIC_VALID,
    LaneBoundary,
    PerceptionPacket,
    PersonDetection,
    pack_packet,
)


def canonical_packet() -> PerceptionPacket:
    return PerceptionPacket(
        sequence=0xFFFFFFFE,
        frame_timestamp_us=1_234_567_890_123,
        ultrasonic_timestamp_us=1_234_567_890_456,
        flags=(
            FLAG_CAMERA_VALID
            | FLAG_LANE_VALID
            | FLAG_LEFT_ULTRASONIC_VALID
            | FLAG_RIGHT_ULTRASONIC_VALID
            | FLAG_PERSON_DETECTION_VALID
        ),
        left=LaneBoundary(0.2, 0.01, 0.1, 0.18, 0.9),
        right=LaneBoundary(-0.3, -0.02, -0.12, -0.22, 0.8),
        side_left_distance_m=0.75,
        side_right_distance_m=0.80,
        person_count=1,
        persons=(
            PersonDetection(
                valid=1.0,
                confidence=0.86,
                center_x_norm=0.52,
                center_y_norm=0.48,
                width_norm=0.18,
                height_norm=0.42,
            ),
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    args = parser.parse_args()
    with open(args.output, "wb") as output:
        output.write(pack_packet(canonical_packet()))


if __name__ == "__main__":
    main()
