# RPi #1 -> Front TC375 Steering UDP Contract v1

This file is the frozen wire contract. Protocol changes require a new version.

## Transport

- UDP unicast: HPVC/RPi #1 Ethernet `192.168.10.1` source port `5101` to
  Front TC375 `192.168.10.11` destination port `5100`
- One complete message per datagram
- Byte order: little endian
- Nominal rate: 20 Hz (`0.05 s` period)
- Packet size: exactly 40 bytes

## Packet layout

| Offset | Type | Name | Unit / rule |
|---:|---|---|---|
| 0 | `char[4]` | Magic | ASCII `R1SC` |
| 4 | `uint8` | Version | `1` |
| 5 | `uint8` | Control mode | `0` disabled, `1` steering-angle control |
| 6 | `uint8` | Flags | Defined below |
| 7 | `uint8` | Header size | `32` |
| 8 | `uint32` | Sequence | Increment once per datagram |
| 12 | `uint64` | Timestamp | RPi #1 monotonic microseconds |
| 20 | `float32` | Steering angle | rad, positive means left |
| 24 | `float32` | Maximum steering rate | rad/s, positive |
| 28 | `uint16` | Alive count | Increment once per datagram, wraps naturally |
| 30 | `uint16` | Reserved | Must be zero |
| 32 | `uint32` | Reserved | Must be zero |
| 36 | `uint32` | CRC32 | IEEE CRC32 over bytes 0..35 |

Flags:

- Bit 0, `SteeringValid`: angle command may be applied.
- Bit 1, `EmergencyCenter`: ignore the requested angle and request controlled
  return to calibrated center.
- Bit 2, `UpstreamControlValid`: RPi #2 link and perception are currently valid.
- Bits 3..7 are reserved and must be zero.

`EmergencyCenter` has priority over `SteeringValid`. Version 1 transmitters never
set both bits. A receiver must still choose center if both arrive set.

## Transmitter rules

RPi #1 sets `SteeringValid` only when LKAS is enabled, upstream control is valid,
and the angle is finite and within the configured limit. Otherwise it sends angle
`0`, clears `SteeringValid`, and sets `EmergencyCenter`. RPi #1 starts with LKAS
disabled, so connecting the network alone cannot authorize servo movement.

## TC375 receiver rules

The TC375 must not drive a servo merely because a UDP datagram arrived. It accepts
a command only after checking:

1. Datagram length, magic, version, header size, reserved fields, and CRC32.
2. A newer Sequence using uint32 half-range modular comparison.
3. Finite angle/rate values and locally configured physical limits.
4. `SteeringValid=1` and `EmergencyCenter=0`.

The TC375 uses its own local receive time for the watchdog. It must enter controlled
center mode when no accepted new Sequence arrives for `0.20 s`, when a packet is
invalid, or when `EmergencyCenter=1`. The RPi timestamp is diagnostic only because
the two devices do not share a monotonic-clock epoch.

Until the servo is physically installed and center/direction calibration is
complete, the TC375 application must keep its PWM output disabled. Decoder tests
do not authorize actuator output.
