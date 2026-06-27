# FRONT

FRONT is the TC375 front-zone module.

It performs two jobs:

- Reads the front ToF and left/right ultrasonic sensors, then sends `AEB1`
  sensor packets to HPVC on UDP `5011`.
- Receives HPVC/PC `HPSC` steering commands on UDP `5100`, validates CRC,
  sequence, flags, angle limits, and watchdog freshness, then drives the servo
  PWM output.

## Network

```text
FRONT TC375 IP : 192.168.10.11
HPVC/PC IP     : 192.168.10.1

FRONT -> HPVC/PC sensors : UDP 5011, magic AEB1
HPVC/PC -> FRONT steering: UDP 5100, magic HPSC
```

If the PC sends UDP directly to `192.168.10.11`, add a static ARP entry for the
FRONT MAC used in `AebSensorNode.h`.

## Servo PWM

Servo output is implemented in:

```text
FrontSteeringNode.c
FrontSteeringNode.h
```

Default calibration:

```text
P02.3 / TOM0 channel 3 servo signal
50 Hz period = 20000 us
1150 us = left
1650 us = center
2000 us = right
max command angle = +/-0.50 rad
watchdog = 200 ms
```

Positive steering angle maps to left, negative steering angle maps to right.

Default PWM pin is `IfxGtm_TOM0_3_TOUT3_P02_3_OUT`. Change
`FRONT_STEERING_SERVO_PIN`, `FRONT_STEERING_SERVO_TOM`, and
`FRONT_STEERING_SERVO_TOM_CHANNEL` in `FrontSteeringNode.c` if your board routes
the servo signal to a different TC375 pin.

## PC Bench Test

Listen for sensor packets:

```bash
python FRONT/sensor_test.py sensor --bind-ip 0.0.0.0
```

Send one small steering command:

```bash
python FRONT/sensor_test.py steer --front-host 192.168.10.11 --source-ip 192.168.10.1 --angle-rad 0.15 --duration 2 --arm
```

Keyboard-only servo test:

```bash
python FRONT/servo.py --front-host 192.168.10.11 --source-ip 192.168.10.1
```

Interactive sensor/steering test:

```bash
python FRONT/sensor_test.py interactive --front-host 192.168.10.11 --source-ip 192.168.10.1 --arm
```
