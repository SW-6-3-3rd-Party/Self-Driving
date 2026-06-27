# Raspberry Pi #2 인지 ECU

이 폴더에는 RPi #2에서 실행할 단일 카메라 차선 검출기, 좌·우 측면
초음파 센서 수집 기능, UDP 송신기, 읽기 전용 웹 미리보기 기능이 들어 있습니다.

## 데이터 흐름

```text
USB 카메라 -> 단일 차선 처리 루프 -> 차선 경계 특징값
좌·우 초음파 -> 교대 측정 -> 측면 거리
차선 + 초음파 -> 80바이트 UDP 패킷 -> RPi #1 HPVC
최신 처리 영상/측정값 -> 웹 미리보기 (영상처리 중복 실행 없음)
```

## Raspberry Pi 설치

```bash
cd RCCarAutonomousSystem
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r RPI2/requirements.txt
```

HC-SR04 계열 GPIO 초음파 센서를 실제로 사용할 경우:

```bash
python3 -m pip install rpi-lgpio
```

초음파 센서의 Echo 출력은 Raspberry Pi GPIO에 연결하기 전에 반드시
3.3V로 레벨 시프팅해야 합니다.

## 카메라 및 가상 초음파 시험

`192.168.1.10`을 실제 RPi #1 주소로 변경하십시오.

```bash
python3 -m RPI2.app \
  --camera /dev/video0 \
  --udp-host 192.168.1.10 \
  --udp-port 5005 \
  --udp-source-port 5006 \
  --lane-width-m 0.40 \
  --visible-length-m 2.0 \
  --mock-ultrasonic 1.0 1.0
```

브라우저에서 `http://<rpi2-ip>:8000`을 열면 처리 영상과 측정값을
확인할 수 있습니다.

## 트랙 없이 전체 차선 데이터 시험

실제 트랙과 웹캠을 사용하지 않고 움직이는 가상 차선을 생성할 수 있습니다.
`172.30.1.89`는 Simulink를 실행하는 Mac의 주소입니다.

```bash
python3 app.py \
  --synthetic-camera \
  --udp-host 172.30.1.89 \
  --udp-port 5005 \
  --udp-source-port 5006 \
  --mock-ultrasonic 1.0 1.0
```

브라우저에서 `http://172.30.1.49:8000`을 열어 가상 차선 검출을 확인하고,
Mac에서는 `RPI1/Models/RPI2UdpReceiveTest.slx`를 실행합니다. 이 모드는 통신과 차선 데이터
처리 검증용이며 실제 카메라 성능을 검증하지는 않습니다.

## 실제 측면 초음파 센서

GPIO 번호는 물리 핀 번호가 아닌 BCM 번호 체계를 사용합니다. 현재 기본 설정은
다음과 같으며 Trigger와 Echo는 모두 서로 다른 GPIO를 사용합니다.

| 위치 | 신호 | BCM GPIO | Raspberry Pi 물리 핀 |
|---|---|---:|---:|
| 왼쪽 | Trigger | GPIO23 | 16번 |
| 왼쪽 | Echo | GPIO24 | 18번 |
| 오른쪽 | Trigger | GPIO17 | 11번 |
| 오른쪽 | Echo | GPIO27 | 13번 |

기본 핀 설정으로 실행:

```bash
python3 -m RPI2.app \
  --camera /dev/video0 \
  --udp-host 192.168.1.10
```

다른 핀을 사용할 때만 실행 인자로 덮어씁니다.

```bash
python3 -m RPI2.app \
  --camera /dev/video0 \
  --udp-host 192.168.1.10 \
  --left-trigger 23 --left-echo 24 \
  --right-trigger 17 --right-echo 27
```

카메라만 시험하고 초음파를 사용하지 않을 때:

```bash
python3 -m RPI2.app \
  --camera /dev/video0 \
  --udp-host 192.168.1.10 \
  --disable-ultrasonic
```

두 센서는 초음파 상호 간섭을 줄이기 위해 일정한 시간 간격을 두고
순차적으로 측정합니다.

## UDP 패킷 v1

고정된 송수신 계약과 RPi #1 watchdog 규칙은
`Interfaces/RPI1_RPI2_PROTOCOL.md`를 기준으로 합니다.

모든 정수와 실수는 리틀 엔디언 형식이며 전체 패킷 크기는 80바이트입니다.

```text
매직 값                 4바이트  "RP2L"
프로토콜 버전           uint8    1
상태 플래그             uint8
Payload 실수 개수       uint16   12
패킷 순서 번호          uint32
카메라 Timestamp        uint64   단조 증가 마이크로초
초음파 Timestamp        uint64   단조 증가 마이크로초
좌측 차선 경계          5 x float32
우측 차선 경계          5 x float32
좌측 측면 거리          float32, 미터
우측 측면 거리          float32, 미터
CRC32                   uint32
```

각 차선 경계에는 다음 값이 포함됩니다.

```text
curvature_1pm
curvature_derivative_1pm2
heading_rad
lateral_offset_m
strength
```

상태 플래그:

```text
bit 0: 카메라 데이터 유효
bit 1: 차선 검출 유효
bit 2: 좌측 초음파 데이터 유효
bit 3: 우측 초음파 데이터 유효
```

횡방향 오프셋과 Heading의 양수 방향은 차량의 왼쪽입니다. 폐루프 주행을
시작하기 전에 실제 RC 트랙에서 카메라 보정값과 원근 변환 비율을 측정해야
합니다.

## 개발 PC 테스트

`RCCarAutonomousSystem` 폴더에서 다음 명령을 실행하십시오.

```bash
python3 -m unittest discover -s RPI2/tests -v
python3 -m compileall -q RPI2
```

Simulink UDP 블록을 연결하기 전에 RPi #1 또는 PC에서 다음 수신기를 실행하여
패킷을 확인할 수 있습니다.

```bash
python3 -m RPI2.udp_receiver --port 5005
```

## Simulink에서 UDP 수신

먼저 `Parameters/rccarParameters.m`에서 RPi #2 주소를 실제 값으로 변경합니다.

```matlab
rccar.Rpi2Address = '172.30.1.49';
rccar.Rpi2ListenPort = 5005;
rccar.Rpi2SourcePort = 5006;
```

MATLAB에서 데스크톱 수신 시험 모델을 생성하고 실행합니다.

```matlab
cd('/Users/taegon/Documents/MATLAB/Examples/R2026a/autonomous_control/RCCarAutonomousSystem')
setupRCCarProject
buildRpi2UdpReceiveModel(true)
open_system('RPI2UdpReceiveTest')
sim('RPI2UdpReceiveTest')
```

수신 모델은 80바이트 패킷의 Magic, Version, Payload 개수, CRC32를 검사하고
다음 변수를 기록합니다.

```text
rpi2LaneFeatures
rpi2Ultrasonic
rpi2Flags
rpi2Sequence
rpi2FrameTimestampUs
rpi2UltrasonicTimestampUs
rpi2PacketValid
```

Python과 MATLAB 디코더가 같은 바이트열을 해석하는지 확인하려면 다음을 실행합니다.

```matlab
setupRCCarProject
testRpi2Contract
testRpi2LinkMonitor
```
