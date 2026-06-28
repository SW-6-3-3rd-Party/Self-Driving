"""
PI 튜닝용 텔레메트리 수신/로깅
- TC375(CPU0)가 20바이트(float 5개) 프레임을 5001 포트로 송신
- 순서: v_ref, v_meas, u, a_cmd, i_term  (CPU1 TelemFrame 과 동일)
- 실시간 출력 + telemetry_log.csv 저장

가속도 명령은 별도 송신 스크립트(W/S 키 등)로 5000 포트에 보낼 것.
이 스크립트는 수신/로깅 전용.
"""

import socket
import struct
import csv
import time

RX_PORT = 5001
FMT = '<5f'                 # little-endian float 5개
SIZE = struct.calcsize(FMT) # 20 bytes

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", RX_PORT))

print(f"텔레메트리 수신 :{RX_PORT}  (프레임 {SIZE}바이트)")
print(f"{'t':>8} {'v_ref':>8} {'v_meas':>8} {'u':>7} {'a_cmd':>7} {'i_term':>8}")

t0 = time.perf_counter()

with open("telemetry_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_sec", "v_ref", "v_meas", "u", "a_cmd", "i_term"])

    try:
        while True:
            data, _ = sock.recvfrom(1024)

            if len(data) != SIZE:
                print(f"  (크기 이상: {len(data)}바이트 무시)")
                continue

            v_ref, v_meas, u, a_cmd, i_term = struct.unpack(FMT, data)
            t = time.perf_counter() - t0

            w.writerow([f"{t:.4f}", f"{v_ref:.5f}", f"{v_meas:.5f}",
                        f"{u:.5f}", f"{a_cmd:.5f}", f"{i_term:.5f}"])
            f.flush()

            print(f"{t:8.2f} {v_ref:8.4f} {v_meas:8.4f} {u:7.3f} "
                  f"{a_cmd:7.3f} {i_term:8.4f}")

    except KeyboardInterrupt:
        print("\n저장 완료: telemetry_log.csv")
    finally:
        sock.close()
