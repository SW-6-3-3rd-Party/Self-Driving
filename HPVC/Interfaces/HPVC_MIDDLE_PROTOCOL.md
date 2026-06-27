# MIDDLE -> HPVC UDP Contract v2

This file is the frozen wire contract. Protocol changes require a new version.

## Transport

- UDP unicast: MIDDLE source port `5006` to HPVC destination port `5005`
- One complete message per datagram; datagrams are never concatenated
- Byte order: little endian
- Packet rate: nominally 20 Hz
- Packet size: exactly 156 bytes

## Packet layout

| Offset | Type | Name | Unit / rule |
|---:|---|---|---|
| 0 | `char[4]` | Magic | ASCII `MID2` |
| 4 | `uint8` | Version | `2` |
| 5 | `uint8` | Flags | Validity bits below |
| 6 | `uint16` | Float count | `31` |
| 8 | `uint32` | Sequence | Increment once per camera frame |
| 12 | `uint64` | Frame timestamp | MIDDLE monotonic microseconds |
| 20 | `uint64` | Ultrasonic timestamp | MIDDLE monotonic microseconds |
| 28 | `float32[5]` | Left boundary | Field order below |
| 48 | `float32[5]` | Right boundary | Field order below |
| 68 | `float32` | Left-side distance | m |
| 72 | `float32` | Right-side distance | m |
| 76 | `float32` | Person count | 0..3, integer value stored as float |
| 80 | `float32[6]` | Person detection 0 | Field order below |
| 104 | `float32[6]` | Person detection 1 | Field order below |
| 128 | `float32[6]` | Person detection 2 | Field order below |
| 152 | `uint32` | CRC32 | IEEE CRC32 over bytes 0..151 |

Boundary order: curvature `[1/m]`, curvature derivative `[1/m^2]`, heading
`[rad]`, lateral offset `[m]`, strength `[0..1]`.

Person detection order: valid flag (`0.0` or `1.0`), confidence `[0..1]`,
center x normalized `[0..1]`, center y normalized `[0..1]`, width normalized
`[0..1]`, height normalized `[0..1]`. Version 2 sends at most three person
detections sorted by confidence.

Flags: bit 0 camera valid, bit 1 lane valid, bit 2 left ultrasonic valid,
bit 3 right ultrasonic valid, bit 4 person detection valid. Bits 5..7 are
reserved and sent as zero.

Positive heading and lateral offset point to the vehicle's left. Invalid sensor
values must be ignored unless the corresponding validity flag is set.

## Receiver rules

HPVC accepts a datagram only when length, magic, version, float count, and
CRC are valid. It then checks that Sequence is newer using uint32 modular
ordering. Duplicate or out-of-order packets do not refresh the link watchdog.

The MIDDLE and HPVC modules have different monotonic-clock epochs. HPVC must never
subtract a MIDDLE timestamp from its own clock. It records local arrival time
for every accepted new Sequence and declares the link stale after 0.20 s.
Sender timestamps remain useful for source-side interval and latency diagnostics
when clocks are separately synchronized.

On a stale/invalid link, LKAS, LCA, and HPVC-side AEB perception fusion must
drop the stale MIDDLE data and follow the actuator fail-safe policy. UDP loss is
expected; there is no retransmission because old perception data is not useful
for steering or braking.

## Contract tests

From the project root:

```bash
python3 -m unittest discover -s MIDDLE/tests -v
```

From MATLAB:

```matlab
setupRCCarProject
testMiddleContract
testMiddleLinkMonitor
```
