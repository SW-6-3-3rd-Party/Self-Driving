<div align="center">

# Self-Driving

### 센서 융합 기반 RC카 자율주행 통합 제어 시스템

카메라 기반 차선·보행자 인식과 초음파·ToF 거리 센서를 결합하여  
차선 유지, 측면 안전 판단, 자동 긴급 제동, 조향·구동 제어를 수행하는 프로젝트

<p>
  <img src="https://img.shields.io/badge/Infineon-TC375-005B95?style=for-the-badge" alt="Infineon TC375" />
  <img src="https://img.shields.io/badge/Raspberry%20Pi-Perception%20ECU-C51A4A?style=for-the-badge" alt="Raspberry Pi" />
  <img src="https://img.shields.io/badge/MATLAB-Simulink-0076A8?style=for-the-badge" alt="MATLAB Simulink" />
  <img src="https://img.shields.io/badge/Python-3-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3" />
  <img src="https://img.shields.io/badge/C-Embedded-00599C?style=for-the-badge&logo=c&logoColor=white" alt="Embedded C" />
</p>
<p>
  <img src="https://img.shields.io/badge/Control-LKAS%20%7C%20LCA%20%7C%20AEB%20%7C%20ACC-111827?style=for-the-badge" alt="Control Features" />
  <img src="https://img.shields.io/badge/Protocol-UDP%20%2B%20CRC32-0F766E?style=for-the-badge" alt="UDP CRC32" />
  <img src="https://img.shields.io/badge/Vision-OpenCV%20%2B%20YOLO-15803D?style=for-the-badge" alt="OpenCV YOLO" />
</p>

</div>

---

## Contributors

<div align="center">
<table>
  <tr>
    <td align="center">
      <a href="https://github.com/Gon0304">
        <img src="https://github.com/Gon0304.png" width="100px;" alt="김태곤"/>
        <br />
        <sub><b>김태곤</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/kookjd7759">
        <img src="https://github.com/kookjd7759.png" width="100px;" alt="국동균"/>
        <br />
        <sub><b>국동균</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/LSA31">
        <img src="https://github.com/LSA31.png" width="100px;" alt="이승아"/>
        <br />
        <sub><b>이승아</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/cenway">
        <img src="https://github.com/cenway.png" width="100px;" alt="윤한준"/>
        <br />
        <sub><b>윤한준</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/chohabin">
        <img src="https://github.com/chohabin.png" width="100px;" alt="조하빈"/>
        <br />
        <sub><b>조하빈</b></sub>
      </a>
    </td>
  </tr>
</table>
</div>

<sub><em>
본 프로젝트는 현대오토에버 모빌리티 임베디드 SW 스쿨 6기 교육 과정에서 학습과 실습을 바탕으로 수행한 팀 프로젝트입니다.
</em></sub>

## 1. 프로젝트 소개

> Self-Driving은 **RC카 플랫폼에서 인지, 판단, 제어, 액추에이터 구동을 분리한 자율주행 통합 제어 시스템**입니다.  
> Raspberry Pi 기반 MIDDLE 인지 ECU가 차선·보행자·측면 거리 정보를 생성하고, HPVC가 이를 검증·융합하여 Front/Rear TC375 제어기로 조향과 구동 명령을 전달합니다.

<table>
  <tr>
    <td>
      본 프로젝트는 다음 흐름을 하나의 차량 제어 구조로 연결하는 것을 목표로 합니다.
      <br /><br />
      - 카메라 영상 기반 차선 경계 및 보행자 인식<br />
      - 초음파·ToF 센서를 이용한 전방·측면·후방 거리 감지<br />
      - LKAS, LCA, AEB, ACC 기능을 위한 HPVC 판단 로직<br />
      - TC375 기반 조향 서보, 구동 모터, 브레이크 제어<br />
      - UDP, CRC32, Sequence, Watchdog 기반 ECU 간 통신 안정성 확보
    </td>
  </tr>
</table>

## 2. 프로젝트 목표

- RC카 플랫폼에서 동작 가능한 저속 자율주행 제어 구조 구현
- 인지 ECU, 판단 ECU, 조향 ECU, 구동 ECU의 역할 분리
- 차선 유지와 차로 변경 보조를 위한 조향 명령 생성 및 검증
- 보행자·거리 센서 융합 기반 자동 긴급 제동 판단
- CRC, Sequence, Watchdog 기반 통신 오류 및 입력 stale 상황 대응
- 실제 센서 없이도 통합 검증이 가능한 synthetic camera, mock sensor, contract test 환경 구성

## 3. 주요 기능

| 구분 | 내용 |
| --- | --- |
| **3-1. 카메라 기반 인지** | **OpenCV 차선 검출**: ROI, 색상/에지 필터, Bird's-eye 변환, Sliding Window 기반 좌우 차선 경계 추정<br />**차선 형상 추출**: 곡률, 곡률 변화율, 헤딩, 횡방향 오프셋, 신뢰도 산출<br />**YOLO 보행자 검출**: 사람 객체를 confidence, 중심 좌표, bounding box 크기로 정규화하여 HPVC로 전달 |
| **3-2. 거리 센서 수집** | **MIDDLE 측면 초음파**: 좌/우 측면 거리 수집 및 mock sensor 모드 지원<br />**FRONT 전방 센서**: TC375에서 ToF와 좌/우 초음파 거리 측정 후 `AEB1` 패킷 송신<br />**REAR 후방 센서**: 후방/측면 초음파와 차량 속도 상태를 HPVC/PC로 송신 |
| **3-3. HPVC 판단 및 제어** | **LKAS**: MIDDLE 차선 링크와 차선 유효성을 확인한 뒤 Front TC375로 조향 명령 송신<br />**LCA**: 방향 요청과 측면 초음파 clearance를 확인해 차로 변경 조향 테스트 수행<br />**AEB**: 보행자 인식, 전방 ToF, 전방 초음파를 융합해 FCW, 부분 제동, 완전 제동, 정지 유지 상태 판단<br />**ACC/주행 요청**: Runtime API와 Remote Controller를 통해 Rear TC375 구동 명령 생성 |
| **3-4. Front TC375 조향 제어** | **HPSC 조향 패킷 검증**: 길이, Magic, Version, Header, CRC32, Sequence, 각도 제한 확인<br />**서보 PWM 제어**: P02.3 기반 50 Hz 소프트웨어 PWM으로 조향각 반영<br />**Fail-safe Center**: 유효하지 않은 패킷, EmergencyCenter, Watchdog timeout 시 중앙 복귀 |
| **3-5. Rear TC375 구동·제동 제어** | **MANUAL 모드**: 원격 target speed와 drive direction 기반 오픈루프 듀티 제어<br />**ACC 모드**: 가속도 명령 적분, 속도 추종 PI + Feed-forward 제어<br />**AEB/E-Stop 우선 처리**: 모드와 무관하게 최우선 단락 제동 수행<br />**텔레메트리 송신**: 속도, 제어 입력, 후방 센서 상태를 UDP로 송신 |
| **3-6. 통신 안전성 및 검증** | **고정 UDP 계약**: `MID2`, `AEB1`, `HPSC`, `HPAB` 패킷 명세 관리<br />**CRC32 / Sequence / Watchdog**: 손상 패킷, 중복·역순 패킷, stale 링크 방어<br />**Contract Test**: Python과 MATLAB/Simulink가 동일한 바이너리 패킷을 해석하는지 검증 |

## 4. 시스템 구성

본 시스템은 4개의 주요 영역으로 구성됩니다.

- **MIDDLE (Raspberry Pi #2 / Perception ECU)** - 카메라 차선 인식, YOLO 보행자 검출, 측면 초음파 수집, 웹 미리보기, `MID2` UDP 송신
- **HPVC (Raspberry Pi #1 / Vehicle Controller)** - MIDDLE/FRONT/REAR 데이터 수신, LKAS/LCA/AEB/ACC 판단, Simulink 모델 검증 및 배포, 조향·구동 명령 송신
- **FRONT (TC375 Front Zone)** - 전방 ToF/초음파 센서 수집, `AEB1` 센서 패킷 송신, `HPSC` 조향 명령 수신, 서보 PWM 제어
- **REAR (TC375 Rear Zone)** - 구동 모터 및 브레이크 제어, 엔코더 기반 속도 피드백, 후방/측면 초음파 상태 송신

### 전체 아키텍처  
<img width="994" height="533" alt="스크린샷 2026-07-01 185417" src="https://github.com/user-attachments/assets/78bb33b7-6fcd-443e-b6e3-59d13fb6516f" />

### 네트워크 및 프로토콜

| 송신 | 수신 | 포트 | 패킷 | 주요 데이터 |
| --- | --- | ---: | --- | --- |
| MIDDLE | HPVC | 5005 | `MID2` v2 | 차선 경계, 측면 거리, 보행자 검출 |
| FRONT TC375 | HPVC | 5011 | `AEB1` v1 | 전방 ToF, 좌/우 초음파, 센서 유효 상태 |
| HPVC | FRONT TC375 | 5100 | `HPSC` v1 | 조향각, 조향 속도 제한, EmergencyCenter |
| FRONT TC375 | HPVC | 5102 | Steering Status | 조향 수신 상태 및 fail-safe 상태 |
| HPVC | Brake Controller | 5013 | `HPAB` v1 | AEB 상태, 제동률, 판단 이유 |
| HPVC | REAR TC375 | 5110 | Rear Drive Command | 주행 모드, 목표 속도, 가속도, E-Stop |
| REAR TC375 | HPVC/PC | 5012 | Rear Status | 후방 거리, 차량 속도, 상태 플래그 |

## 5. 저장소 구조

| 디렉터리 | 설명 |
| --- | --- |
| **FRONT/** | TC375 Front Zone 코드입니다. 전방 ToF/초음파 센서 노드, HPVC 조향 명령 수신기, 서보 PWM 제어, 센서/조향 벤치 테스트 스크립트가 포함되어 있습니다. |
| **MIDDLE/** | Raspberry Pi #2 인지 ECU 코드입니다. 카메라 차선 검출, YOLO 보행자 검출, 측면 초음파 수집, UDP 패킷 송신, 웹 미리보기, Python 단위 테스트가 포함되어 있습니다. |
| **MIDDLE/RPI2/** | RPI2 실행 환경용 복사본입니다. 실제 Raspberry Pi에서 카메라·초음파·UDP 송신을 실행하기 위한 README, 실행 스크립트, 테스트가 정리되어 있습니다. |
| **HPVC/** | HPVC 판단/제어 계층입니다. AEB fusion runtime, Simulink 모델 생성 스크립트, 조향 패킷 인코더, LCA 테스트, 원격 제어 서버, 프로토콜 문서, MATLAB/Python 테스트가 포함되어 있습니다. |
| **HPVC/Interfaces/** | MIDDLE, FRONT, HPVC 사이의 고정 UDP wire contract 문서입니다. 패킷 크기, 필드 순서, CRC, receiver rule을 정의합니다. |
| **HPVC/Models/** | Middle 수신, Desktop Validation, Deployment용 Simulink 모델 파일입니다. |
| **HPVC/RPI1/** | Raspberry Pi #1 HPVC sidecar 코드입니다. LKAS/LCA/ACC/AEB 요청 상태 관리, JSON status API, Front/Rear UDP 송신 및 상태 수신 기능을 제공합니다. |
| **REAR/** | TC375 Rear Zone 코드입니다. 모터 PWM, 브레이크, 엔코더 속도 추정, ACC/Manual/AEB 모드 제어, 후방 초음파 수집, UDP 텔레메트리 송신 코드가 포함되어 있습니다. |

## 6. 실행 및 검증

### MIDDLE 인지 ECU 실행

실제 카메라와 초음파 센서를 사용하는 경우:

```bash
python3 -m MIDDLE.app \
  --camera /dev/video0 \
  --udp-host <hpvc-ip> \
  --udp-port 5005 \
  --udp-source-port 5006
```

센서 없이 벤치 테스트를 수행하는 경우:

```bash
python3 -m MIDDLE.app \
  --synthetic-camera \
  --udp-host <hpvc-ip> \
  --mock-ultrasonic 1.0 1.0
```

웹 미리보기는 다음 주소에서 확인할 수 있습니다.

```text
http://<middle-ip>:8000
```

### HPVC AEB Runtime

```bash
python3 -m HPVC.hpvc_aeb
python3 -m HPVC.hpvc_aeb --brake-host 192.168.10.12 --brake-port 5013
```

### FRONT 센서 및 조향 벤치 테스트

```bash
python FRONT/sensor_test.py sensor --bind-ip 0.0.0.0
python FRONT/sensor_test.py steer --front-host 192.168.10.11 --source-ip 192.168.10.1 --angle-rad 0.15 --duration 2 --arm
python FRONT/servo.py --front-host 192.168.10.11 --source-ip 192.168.10.1
```

### Python 테스트

```bash
python3 -m unittest discover -s MIDDLE/tests -v
python3 -m unittest discover -s HPVC/tests -v
python3 -m compileall -q MIDDLE HPVC
```

### MATLAB/Simulink 계약 테스트

```matlab
setupRCCarProject
testMiddleContract
testMiddleLinkMonitor
testFrontSteeringContract
buildMiddleUdpReceiveModel(true)
buildHpvcDesktopValidationModel(true)
buildHpvcDeploymentModel(true)
```

## 7. 담당 역할  
<img width="981" height="418" alt="스크린샷 2026-07-01 185718" src="https://github.com/user-attachments/assets/3b8beb2f-1c9b-42bb-8f38-cf41c1865925" />

## 8. 개발 포인트

- **Zonal Architecture**: FRONT, MIDDLE, HPVC, REAR로 기능을 분리해 실제 차량 E/E 구조와 유사한 제어 흐름 구성
- **고정 바이너리 프로토콜**: Python, C, MATLAB/Simulink가 공유하는 UDP wire contract 정의
- **Fail-safe 우선 설계**: 조향 watchdog, EmergencyCenter, AEB/E-Stop 우선 제동, stale link 차단 로직 구현
- **센서 융합 AEB**: 보행자 인식과 전방 거리 센서를 함께 사용해 confidence와 거리 기반 제동 단계 판단
- **실차/벤치 겸용 구조**: 실제 카메라·초음파 센서뿐 아니라 synthetic camera, mock ultrasonic, virtual LKAS curve로 통합 검증 가능
- **계약 기반 검증**: CRC32, Sequence, payload layout을 Python/MATLAB 테스트로 반복 검증

## 9. 기술 스택

| Hardware | Software | Tools |
| --- | --- | --- |
| Infineon **TC375**<br />**Raspberry Pi**<br />USB Camera<br />ToF Sensor<br />Ultrasonic Sensor<br />Servo Motor<br />DC Motor / Encoder<br />Ethernet Switch | **Embedded C**<br />**Python 3**<br />**MATLAB / Simulink**<br />**OpenCV**<br />**Ultralytics YOLO**<br />**Flask**<br />**lwIP**<br />**UDP / CRC32** | **AURIX Development Studio**<br />**VS Code**<br />**GitHub**<br />**MATLAB / Simulink**<br />**Raspberry Pi Blockset**<br />**Python unittest** |

## 10. 프로젝트 의의

Self-Driving은 단일 기능 데모가 아니라,  
**인지 ECU → HPVC 판단 → Front/Rear 제어기 → 액추에이터 구동 → 상태 피드백**으로 이어지는 전체 자율주행 제어 흐름을 구성한 프로젝트입니다.

특히 카메라 인식, 거리 센서, UDP 통신, Simulink 제어 모델, TC375 액추에이터 제어를 하나의 RC카 플랫폼 안에서 연결하며 임베디드 자율주행 시스템의 데이터 흐름과 fail-safe 구조를 직접 구현했다는 점에 의미가 있습니다.

## 11. 개선 방향

- 실제 주행 환경에서 차선 인식 파라미터와 카메라 보정값 추가 튜닝
- AEB 판단에 차량 속도, 제동 거리, TTC 기반 위험도 계산 추가
- Rear 구동 명령 프로토콜의 문서화 및 contract test 강화
- HPVC Runtime API와 Simulink Deployment 모델 간 상태 동기화 고도화
- 장시간 주행 시 UDP packet loss, sensor stale, actuator watchdog 상황의 통합 테스트 확대
