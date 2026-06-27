# Front TC375 -> HPVC AEB Sensor UDP Contract v1

This contract carries converted front range data from the Front TC375 to the
HPVC AEB logic.

## Transport

- UDP unicast: Front TC375 `192.168.10.11` source port `5011` to HPVC
  `192.168.10.1` destination port `5011`
- One complete message per datagram
- Byte order: little endian
- Nominal rate: 25 Hz
- UDP payload size: exactly 22 bytes

## Packet Layout

| Offset | Type | Name | Unit / rule |
|---:|---|---|---|
| 0 | `char[4]` | Magic | ASCII `AEB1` |
| 4 | `uint8` | Version | `1` |
| 5 | `uint8` | Valid mask | Bit order below |
| 6 | `uint16` | ToF diagnostic | High byte model ID, low byte diagnostic |
| 8 | `uint32` | Sequence | Increment once per datagram |
| 12 | `uint32` | Timestamp | TC375 local milliseconds |
| 16 | `uint16` | Front ToF distance | centimeters x 10 |
| 18 | `uint16` | Left ultrasonic distance | centimeters x 10 |
| 20 | `uint16` | Right ultrasonic distance | centimeters x 10 |

Valid mask:

- Bit 0: left ultrasonic distance is fresh
- Bit 1: right ultrasonic distance is fresh
- Bit 2: front ToF distance is fresh
- Bits 3..7 are reserved and sent as zero

The HPVC converts `cm x 10` values to meters by dividing by `1000.0`. Invalid
distances must be ignored unless the corresponding valid bit is set.
