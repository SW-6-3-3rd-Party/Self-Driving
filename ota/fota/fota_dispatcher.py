import json
from pathlib import Path

from config_paths import CONFIG_DIR
from fota.tcp_fota_client import TCPFOTAClient


class FOTADispatcher:
    def __init__(self):
        self.targets = self._load_targets()
        self.tcp_client = TCPFOTAClient()

    def _load_targets(self) -> dict:
        config_path = CONFIG_DIR / "fota_targets.json"

        if not config_path.exists():
            return {}

        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def dispatch(
        self,
        target: str,
        artifact_path: Path,
        target_version: str,
        job_id: str,
    ) -> dict:
        if target not in self.targets:
            raise ValueError(f"FOTA target not configured: {target}")

        target_config = self.targets[target]
        mode = target_config.get("mode", "tcp")

        if mode != "tcp":
            raise ValueError(f"unsupported FOTA mode: {mode}")

        ip = target_config["ip"]
        port = int(target_config["port"])

        metadata = {
            "job_id": job_id,
            "target": target,
            "target_version": target_version,
            "protocol": "TCP_FOTA_PROJECT",
        }

        print(
            f"[FOTA DISPATCH] target={target}, ip={ip}, port={port}, "
            f"artifact={artifact_path}"
        )

        return self.tcp_client.send_package(
            ip=ip,
            port=port,
            artifact_path=artifact_path,
            metadata=metadata,
        )
