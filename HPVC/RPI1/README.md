# Raspberry Pi #1 HPVC

이 폴더는 RPi #1에서 실행되는 수신, 통신 감시 및 제어 코드 생성 파일만
관리한다. RPi #2의 카메라·초음파 Python 코드는 `RPI2` 폴더에 분리되어 있다.

## 현재 파일

```text
buildRpi1AlgorithmCode.m   하드웨어 독립 제어 코어 ERT 코드 생성
buildRpi2UdpReceiveModel.m RPi #1/PC 수신 시험 모델 생성
buildRpi1DesktopValidationModel.m 전체 데스크톱 UDP 검증 모델 생성
buildRpi1DeploymentModel.m Raspberry Pi용 최종 수신 모델 생성
configureRpi1Target.m     RPi #1 전용 IP와 사용자 설정
rpi2DecodePacket.m        RPi #2의 80바이트 UDP 패킷 검증·해석
rpi2LinkMonitor.m         Sequence 및 RPi #1 로컬 시간 기반 watchdog
tc375EncodeSteeringPacket.m Front TC375 40바이트 조향 패킷 인코더
diagnostic_receiver.py   R1DG 진단 수신 및 선택적 echo-back 서버
Models/                   RPi #1 수신 및 향후 배포용 Simulink 모델
tests/                    Python 송신기와 MATLAB 수신기의 계약 시험
```

공유 통신 규격은 `../Interfaces/RPI1_RPI2_PROTOCOL.md`와
`../Interfaces/RPI1_TC375_FRONT_PROTOCOL.md`, 공유 Simulink Bus는
`../Interfaces/createRCCarBusObjects.m`에 있다.

## 폴더 경계

```text
RPI1/       MATLAB/Simulink, HPVC 수신과 제어
RPI2/       Python, 카메라·측면 초음파와 UDP 송신
Interfaces/ 양쪽이 공유하는 고정 통신 규격과 Simulink Bus
Components/ 하드웨어 독립 LKAS/LCA 참조 모델
```

RPi #1의 최종 배포 모델은 다음 구조로 추가한다.

```text
RPi UDP Receive
-> rpi2DecodePacket
-> rpi2LinkMonitor
-> RCCarAutonomousSystem
-> TC375 UDP Send
```

`left.py` / `right.py`는 `--verify-roundtrip` 옵션과 `diagnostic_receiver.py
--echo-back` 조합으로 UDP 왕복 경로를 먼저 검증할 수 있다. 이 경로는
`R1DG` 진단 패킷을 재사용한다.

실제 조향 게이팅은 JSON 센서 소스의 초음파 거리값을 사용한다. 우선 순위는
`rear_left/middle_left/front_left` 또는 `rear_right/middle_right/front_right`
키이고, 이 키가 없으면 기존 호환용 `left_ultrasonic_m` / `right_ultrasonic_m`
값으로 대체한다. 현재는 실제 장비가 가진 값이 하나뿐이면 그 값으로 동작하고,
새 센서 집계기가 들어오면 3개 게이트를 모두 검사한다.

Raspberry Pi Blockset 26.1이 설치되어 있으며 배포 모델도 생성되어 있다.
제어 코어의 portable C 코드는 다음과 같이 별도로 생성할 수 있다.

```matlab
cd('/Users/taegon/Documents/MATLAB/Examples/R2026a/autonomous_control/RCCarAutonomousSystem')
setupRCCarProject
buildRpi1AlgorithmCode
```

## 통신 계약 시험

```matlab
setupRCCarProject
testRpi2Contract
testRpi2LinkMonitor
testTc375SteeringContract
```

수신 시험 모델은 다음과 같이 다시 생성한다.

```matlab
setupRCCarProject
buildRpi2UdpReceiveModel(true)
open_system('RPI2UdpReceiveTest')
```

전체 제어 경로의 권장 검증 순서는 다음과 같다.

```matlab
setupRCCarProject
buildRpi1DesktopValidationModel(true)
open_system('RPI1DesktopValidation')
sim('RPI1DesktopValidation')
```

이 모델은 실제 UDP 수신, 80바이트/CRC 검사, Sequence watchdog, LKAS/LCA 제어
코어까지 실행한다. 이 검증이 통과한 뒤 Raspberry Pi Blockset을 설치하고 다음을
실행한다.

```matlab
buildRpi1DeploymentModel(true)
open_system('RPI1Deployment')
```

생성 직후 대상 주소는 오배포 방지를 위해 `0.0.0.0`이다. 실제 RPi #1 주소를
설정한 뒤에만 Build, Deploy & Start를 실행한다. 아래 주소는 예시다.

```matlab
configureRpi1Target('192.168.1.20', 'pi')
```

RPi #2로 등록된 주소와 동일한 주소는 설정 함수가 거부한다. 비밀번호 인수는
모델 파일에 저장되지 않고 MATLAB 원격 빌드 설정으로 전달된다.

배포 모델은 부팅 시 자동 조향을 막기 위해 `LKAS Enable=false`로 생성된다.
이 상태에서 Front TC375 UDP에는 `SteeringValid=0`, `EmergencyCenter=1`,
`SteeringAngleRad=0`만 송신된다. TC375 주소는 실제 장치 설정 후
`rccar.Tc375FrontAddress`에서 확정해야 한다.
