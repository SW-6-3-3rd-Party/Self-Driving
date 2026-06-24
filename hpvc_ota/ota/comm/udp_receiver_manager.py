import socket
import threading

from comm.packet import parse_header, extract_payload, validate_packet
from comm.payload_parser import (
    MSG_ID_CENTER_SENSOR_SUMMARY,
    MSG_ID_FRONT_SENSOR_DATA_V1,
    MSG_ID_FRONT_SENSOR_DATA_V2,
    MSG_ID_REAR_STATUS_DATA_V1,
    MSG_ID_REAR_STATUS_DATA_V2,
    MSG_ID_HEARTBEAT,
    parse_payload,
)
from comm.ecu_data_store import ECUDataStore


class UDPReceiverManager:
    def __init__(self):
        self.store = ECUDataStore()
        self.running = False
        self.threads = []

        self.ports = {
            5002: "center",
            5011: "front",
            5012: "rear",
            5200: "heartbeat",
        }

    def start(self):
        if self.running:
            return

        self.running = True

        for port, name in self.ports.items():
            thread = threading.Thread(
                target=self._receive_loop,
                args=(port, name),
                daemon=True,
            )
            thread.start()
            self.threads.append(thread)

        print("[UDP RECEIVER] started: 5002(center), 5011(front), 5012(rear), 5200(heartbeat)")

    def stop(self):
        self.running = False

    def get_status(self) -> dict:
        return self.store.get_snapshot()

    def _receive_loop(self, port: int, name: str):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.5)

        print(f"[UDP RECEIVER] {name} listening on 0.0.0.0:{port}")

        while self.running:
            try:
                packet, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                print(f"[UDP RECEIVER ERROR] port={port}, error={exc}")
                continue

            self._handle_packet(packet, addr, port, name)

    def _handle_packet(self, packet: bytes, addr, port: int, name: str):
        valid, reason = validate_packet(packet)

        if not valid:
            print(f"[UDP RX DROP] port={port}, from={addr}, reason={reason}")
            self.store.increment_fault("crc_error")
            return

        try:
            header = parse_header(packet)
            payload_bytes = extract_payload(packet)
            payload = parse_payload(header["msg_id"], payload_bytes)
        except Exception as exc:
            print(f"[UDP RX PARSE ERROR] port={port}, from={addr}, error={exc}")
            self.store.increment_fault("parse_error")
            return

        msg_id = header["msg_id"]

        if msg_id == MSG_ID_CENTER_SENSOR_SUMMARY:
            self.store.update_center(header, payload)

        elif msg_id in (MSG_ID_FRONT_SENSOR_DATA_V1, MSG_ID_FRONT_SENSOR_DATA_V2):
            self.store.update_front(header, payload)

        elif msg_id in (MSG_ID_REAR_STATUS_DATA_V1, MSG_ID_REAR_STATUS_DATA_V2):
            self.store.update_rear(header, payload)

        elif msg_id == MSG_ID_HEARTBEAT:
            self.store.update_heartbeat(header, payload)

        else:
            self.store.increment_fault("unknown_msg")

        print(
            "[UDP RX] "
            f"port={port}, from={addr}, "
            f"msg_id=0x{msg_id:02X}, "
            f"type={payload.get('type')}, "
            f"seq={header['seq']}"
        )
