import json
import socket
import struct
from pathlib import Path


class TCPFOTAClient:
    def send_package(
        self,
        ip: str,
        port: int,
        artifact_path: Path,
        metadata: dict,
        timeout_sec: float = 5.0,
    ) -> dict:
        artifact_path = Path(artifact_path)

        if not artifact_path.exists():
            raise FileNotFoundError(f"artifact not found: {artifact_path}")

        metadata = dict(metadata)
        metadata["filename"] = artifact_path.name
        metadata["size"] = artifact_path.stat().st_size

        metadata_bytes = json.dumps(metadata).encode("utf-8")

        with socket.create_connection((ip, port), timeout=timeout_sec) as sock:
            sock.settimeout(timeout_sec)

            # 1. metadata length
            sock.sendall(struct.pack("<I", len(metadata_bytes)))

            # 2. metadata body
            sock.sendall(metadata_bytes)

            # 3. artifact binary
            with open(artifact_path, "rb") as f:
                for chunk in iter(lambda: f.read(1024 * 64), b""):
                    sock.sendall(chunk)

            # 4. optional response
            try:
                response_len_data = sock.recv(4)

                if len(response_len_data) < 4:
                    return {
                        "result": "UNKNOWN",
                        "reason": "no response length",
                    }

                response_len = struct.unpack("<I", response_len_data)[0]
                response = sock.recv(response_len).decode("utf-8")

                return json.loads(response)

            except socket.timeout:
                return {
                    "result": "SENT",
                    "reason": "no response before timeout",
                }
