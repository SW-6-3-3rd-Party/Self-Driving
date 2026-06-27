# RPi #2 -> RPi #1 UDP Contract v1

This file is the frozen wire contract. Protocol changes require a new version.

## Transport

- UDP unicast: RPi #2 source port `5006` to RPi #1 destination port `5005`
- One complete message per datagram; datagrams are never concatenated
- Byte order: little endian
- Packet rate: nominally 20 Hz
- Packet size: exactly 80 bytes

## Packet layout

| Offset | Type | Name | Unit / rule |
|---:|---|---|---|
| 0 | `char[4]` | Magic | ASCII `RP2L` |
| 4 | `uint8` | Version | `1` |
| 5 | `uint8` | Flags | Validity bits below |
| 6 | `uint16` | Float count | `12` |
| 8 | `uint32` | Sequence | Increment once per camera frame |
| 12 | `uint64` | Frame timestamp | RPi #2 monotonic microseconds |
| 20 | `uint64` | Ultrasonic timestamp | RPi #2 monotonic microseconds |
| 28 | `float32[5]` | Left boundary | Field order below |
| 48 | `float32[5]` | Right boundary | Field order below |
| 68 | `float32` | Left-side distance | m |
| 72 | `float32` | Right-side distance | m |
| 76 | `uint32` | CRC32 | IEEE CRC32 over bytes 0..75 |

Boundary order: curvature `[1/m]`, curvature derivative `[1/m^2]`, heading
`[rad]`, lateral offset `[m]`, strength `[0..1]`.

Flags: bit 0 camera valid, bit 1 lane valid, bit 2 left ultrasonic valid,
bit 3 right ultrasonic valid. Bits 4..7 are reserved and sent as zero.

Positive heading and lateral offset point to the vehicle's left. Invalid sensor
values must be ignored unless the corresponding validity flag is set.

## Receiver rules

RPi #1 accepts a datagram only when length, magic, version, float count, and
CRC are valid. It then checks that Sequence is newer using uint32 modular
ordering. Duplicate or out-of-order packets do not refresh the link watchdog.

The two Raspberry Pis have different monotonic-clock epochs. RPi #1 must never
subtract an RPi #2 timestamp from its own clock. It records local arrival time
for every accepted new Sequence and declares the link stale after 0.20 s.
Sender timestamps remain useful for source-side interval and latency diagnostics
when clocks are separately synchronized.

On a stale/invalid link, LKAS and LCA must be disabled and the steering command
must follow the actuator fail-safe policy. UDP loss is expected; there is no
retransmission because old perception data is not useful for steering.

## Contract tests

From the project root:

```bash
python3 -m unittest discover -s RPI2/tests -v
```

From MATLAB:

```matlab
setupRCCarProject
testRpi2Contract
testRpi2LinkMonitor
```
