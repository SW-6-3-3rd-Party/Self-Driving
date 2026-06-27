<<<<<<< HEAD
﻿# MIDDLE

MIDDLE is the perception module. It reads the camera, detects lane boundaries
and person objects, samples left/right side ultrasonic distances, and sends one
validated UDP perception packet to HPVC.

## Runtime

```bash
python3 -m MIDDLE.app \
  --camera /dev/video0 \
  --udp-host <hpvc-ip> \
  --udp-port 5005 \
  --udp-source-port 5006
```

For bench testing without physical sensors:

```bash
python3 -m MIDDLE.app \
  --synthetic-camera \
  --udp-host <hpvc-ip> \
  --mock-ultrasonic 1.0 1.0
```

The web preview runs on:

```text
http://<middle-ip>:8000
```

## Protocol

MIDDLE sends the `MID2` v2 packet to HPVC:

```text
MIDDLE source port 5006 -> HPVC destination port 5005
packet size 156 bytes
magic MID2
version 2
payload float count 31
CRC32 over bytes 0..151
```

The shared wire contract is documented in:

```text
HPVC/Interfaces/HPVC_MIDDLE_PROTOCOL.md
```

## Tests

From the project root:

```bash
python3 -m unittest discover -s MIDDLE/tests -v
python3 -m compileall -q MIDDLE
```

From MATLAB:

```matlab
setupRCCarProject
buildMiddleUdpReceiveModel(true)
open_system('MiddleUdpReceiveTest')
testMiddleContract
testMiddleLinkMonitor
```
=======
# MIDDLE

RPi #2 중간 계층 코드 묶음입니다. 실제 실행 파일과 시험 코드는
[`RPI2/`](./RPI2/)에 두고, 이 폴더는 그 상위 진입점 역할만 합니다.

포함 범위:

- `RPI2/app.py`
- `RPI2/protocol.py`
- `RPI2/lane_detector.py`
- `RPI2/ultrasonic.py`
- `RPI2/udp_receiver.py`
- `RPI2/run_rpi2.sh`
- `RPI2/stop_rpi2.sh`
- `RPI2/tests/`
- `RPI2/README.md`

배포나 검증 시에는 `RPI2/README.md`의 절차를 따른다.
>>>>>>> 951cd7e9bc5caa71f7dfd4bd77623e64cc167d9b
