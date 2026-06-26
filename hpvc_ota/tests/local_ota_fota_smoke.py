"""Local OTA/FOTA smoke test without real TC375 boards.

This test starts virtual Front and Rear FOTA TCP receivers on localhost, creates
temporary OTA artifacts, and drives the HPVC OTAManager against them. It avoids
MQTT and HTTP so it can run on a development PC with only the Python standard
library.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any


THIS_FILE = Path(__file__).resolve()
HPVC_ROOT = THIS_FILE.parents[1]
WORKSPACE_ROOT = HPVC_ROOT.parents[1]

sys.path.insert(0, str(HPVC_ROOT / "ota"))
sys.path.insert(0, str(WORKSPACE_ROOT / "TC375_front"))
sys.path.insert(0, str(WORKSPACE_ROOT / "rear_src"))

from ota_manager import OTAManager  # noqa: E402
from TC375_front.front_OTA.virtual_fota_receiver import (  # noqa: E402
    VirtualFrontFotaReceiver,
)
from rear_OTA.virtual_fota_receiver import (  # noqa: E402
    VirtualRearFotaReceiver,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def make_artifact(directory: Path, filename: str, content: bytes) -> Path:
    path = directory / filename
    path.write_bytes(content)
    return path


def run_target_smoke(
    manager: OTAManager,
    *,
    target: str,
    target_version: str,
    artifact_path: Path,
    receiver: Any,
) -> dict:
    receiver_thread = receiver.start_background()
    if receiver.bound_address is None:
        raise RuntimeError("virtual FOTA receiver did not bind")

    manager.fota_dispatcher.targets[target] = {
        "mode": "tcp",
        "ip": "127.0.0.1",
        "port": receiver.bound_address[1],
    }

    statuses = []
    job = {
        "job_id": f"local-smoke-{target.lower()}",
        "target": target,
        "target_version": target_version,
        "artifact_url": artifact_path.resolve().as_uri(),
        "sha256": sha256_file(artifact_path),
        "size": artifact_path.stat().st_size,
        "rollback_enabled": True,
    }

    result = manager.start_update(job, status_callback=statuses.append)
    receiver_thread.join(timeout=2.0)

    if receiver_thread.is_alive():
        raise RuntimeError(f"{target} virtual receiver did not finish")

    if result.get("result") != "SUCCESS":
        raise AssertionError(f"{target} OTA failed: {result}")

    if not receiver.last_response or receiver.last_response.get("result") != "SUCCESS":
        raise AssertionError(f"{target} FOTA response failed: {receiver.last_response}")

    return {
        "ota_result": result,
        "fota_response": receiver.last_response,
        "states": [item.get("state") for item in statuses if "state" in item],
    }


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="hpvc_ota_smoke_") as tmp:
        tmp_dir = Path(tmp)
        runtime_dir = tmp_dir / "runtime"
        artifact_dir = tmp_dir / "artifacts"
        config_dir = tmp_dir / "config"

        artifact_dir.mkdir(parents=True, exist_ok=True)
        config_dir.mkdir(parents=True, exist_ok=True)

        os.environ["HPVC_RUNTIME_DIR"] = str(runtime_dir)

        manager = OTAManager()
        manager.version_path = config_dir / "device_versions.json"
        manager.version_path.write_text(
            json.dumps(
                {
                    "HPVC": "1.0.0",
                    "FRONT_ZONE": "1.0.0",
                    "REAR_ZONE": "1.0.0",
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        front_artifact = make_artifact(
            artifact_dir,
            "front_zone_2.0.0.bin",
            b"front-zone-local-smoke-test" * 128,
        )
        rear_artifact = make_artifact(
            artifact_dir,
            "rear_zone_2.0.0.bin",
            b"rear-zone-local-smoke-test" * 128,
        )

        front_result = run_target_smoke(
            manager,
            target="FRONT_ZONE",
            target_version="2.0.0",
            artifact_path=front_artifact,
            receiver=VirtualFrontFotaReceiver("127.0.0.1", 0),
        )

        rear_result = run_target_smoke(
            manager,
            target="REAR_ZONE",
            target_version="2.0.0",
            artifact_path=rear_artifact,
            receiver=VirtualRearFotaReceiver("127.0.0.1", 0),
        )

        versions = json.loads(manager.version_path.read_text(encoding="utf-8"))
        if versions["FRONT_ZONE"] != "2.0.0" or versions["REAR_ZONE"] != "2.0.0":
            raise AssertionError(f"version update failed: {versions}")

        print(json.dumps(
            {
                "result": "PASS",
                "front": front_result,
                "rear": rear_result,
                "versions": versions,
            },
            indent=2,
        ))


if __name__ == "__main__":
    main()
