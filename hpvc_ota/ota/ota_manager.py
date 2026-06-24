import hashlib
import json
import shutil
import time
import zipfile
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

from artifact_downloader import ArtifactDownloader
from config_paths import BASE_DIR, CONFIG_DIR
from fota.fota_dispatcher import FOTADispatcher
from ota_job import OTAJob


class OTAState(Enum):
    IDLE = "IDLE"
    JOB_RECEIVED = "JOB_RECEIVED"
    DOWNLOADING = "DOWNLOADING"
    VERIFYING = "VERIFYING"
    STAGING = "STAGING"
    APPLYING = "APPLYING"
    FOTA_DISPATCHING = "FOTA_DISPATCHING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    ROLLBACK = "ROLLBACK"


class OTAManager:
    def __init__(self):
        self.state = OTAState.IDLE
        self.progress = 0
        self.error_code = None
        self.error_message = None

        self.running = False
        self.current_job = None
        self.last_result = None

        self.downloader = ArtifactDownloader()
        self.fota_dispatcher = FOTADispatcher()

        self.updates_dir = BASE_DIR / "updates"
        self.download_dir = self.updates_dir / "downloaded"
        self.staging_dir = self.updates_dir / "staging"
        self.backup_dir = self.updates_dir / "backup"
        self.log_dir = self.updates_dir / "logs"

        for directory in (
            self.download_dir,
            self.staging_dir,
            self.backup_dir,
            self.log_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)

        self.version_path = CONFIG_DIR / "device_versions.json"

    def get_status(self) -> dict:
        return {
            "state": self.state.value,
            "progress": self.progress,
            "running": self.running,
            "error_code": self.error_code,
            "error_message": self.error_message,
            "current_job": self.current_job.to_dict() if self.current_job else None,
            "last_result": self.last_result,
        }

    def start_update(
        self,
        job,
        status_callback: Optional[Callable[[dict], None]] = None,
    ) -> dict:
        if self.running:
            raise RuntimeError("OTA already running")

        if isinstance(job, dict):
            ota_job = OTAJob.from_dict(job)
        elif isinstance(job, OTAJob):
            ota_job = job
        else:
            raise TypeError("job must be dict or OTAJob")

        self.running = True
        self.current_job = ota_job
        self.last_result = None
        self.error_code = None
        self.error_message = None

        version_backup = None

        try:
            self._set_state(OTAState.JOB_RECEIVED, 5, status_callback)

            self._validate_job(ota_job)

            version_backup = self._backup_versions(ota_job.job_id)

            artifact_path = self._download_artifact(
                ota_job,
                status_callback,
            )

            self._verify_artifact(
                ota_job,
                artifact_path,
                status_callback,
            )

            staged_path = self._stage_artifact(
                ota_job,
                artifact_path,
                status_callback,
            )

            self._apply_update(
                ota_job,
                staged_path,
                status_callback,
            )

            self._update_version(
                ota_job.target,
                ota_job.target_version,
            )

            self._set_state(OTAState.SUCCESS, 100, status_callback)

            self.last_result = {
                "result": "SUCCESS",
                "job_id": ota_job.job_id,
                "target": ota_job.target,
                "target_version": ota_job.target_version,
            }

            self._publish(status_callback, self.last_result)
            return self.last_result

        except Exception as exc:
            self.error_code = "OTA_FAILED"
            self.error_message = str(exc)

            if ota_job.rollback_enabled:
                self._set_state(
                    OTAState.ROLLBACK,
                    self.progress,
                    status_callback,
                )
                self._rollback_versions(version_backup)

            self._set_state(
                OTAState.FAILED,
                self.progress,
                status_callback,
            )

            self.last_result = {
                "result": "FAILED",
                "job_id": ota_job.job_id,
                "target": ota_job.target,
                "target_version": ota_job.target_version,
                "error_code": self.error_code,
                "error_message": self.error_message,
            }

            self._publish(status_callback, self.last_result)
            return self.last_result

        finally:
            self.running = False
            self.current_job = None

    def _validate_job(self, job: OTAJob):
        allowed_targets = {
            "HPVC",
            "CENTER_RPI",
            "FRONT_ZONE",
            "REAR_ZONE",
        }

        if job.target not in allowed_targets:
            raise ValueError(f"unsupported OTA target: {job.target}")

        if not job.artifact_url:
            raise ValueError("artifact_url is required")

    def _download_artifact(
        self,
        job: OTAJob,
        status_callback,
    ) -> Path:
        self._set_state(OTAState.DOWNLOADING, 15, status_callback)

        filename = Path(job.artifact_url.split("?")[0]).name

        if not filename:
            filename = f"{job.target}_{job.target_version}.bin"

        output_path = self.download_dir / f"{job.job_id}_{filename}"

        artifact_path = self.downloader.download(
            artifact_url=job.artifact_url,
            output_path=output_path,
        )

        self._set_state(OTAState.DOWNLOADING, 35, status_callback)
        return artifact_path

    def _verify_artifact(
        self,
        job: OTAJob,
        artifact_path: Path,
        status_callback,
    ):
        self._set_state(OTAState.VERIFYING, 45, status_callback)

        if job.sha256:
            calculated = self._sha256_file(artifact_path)

            if calculated.lower() != job.sha256.lower():
                raise ValueError(
                    f"sha256 mismatch: expected={job.sha256}, calculated={calculated}"
                )

        if artifact_path.suffix.lower() == ".zip":
            with zipfile.ZipFile(artifact_path, "r") as zf:
                names = set(zf.namelist())

                if "manifest.json" in names:
                    manifest = json.loads(
                        zf.read("manifest.json").decode("utf-8-sig")
                    )

                    manifest_target = manifest.get("target")
                    manifest_version = manifest.get("version")

                    if manifest_target and manifest_target != job.target:
                        raise ValueError(
                            f"manifest target mismatch: {manifest_target} != {job.target}"
                        )

                    if manifest_version and manifest_version != job.target_version:
                        raise ValueError(
                            f"manifest version mismatch: {manifest_version} != {job.target_version}"
                        )

        self._set_state(OTAState.VERIFYING, 60, status_callback)

    def _stage_artifact(
        self,
        job: OTAJob,
        artifact_path: Path,
        status_callback,
    ) -> Path:
        self._set_state(OTAState.STAGING, 70, status_callback)

        job_stage_dir = self.staging_dir / job.job_id

        if job_stage_dir.exists():
            shutil.rmtree(job_stage_dir)

        job_stage_dir.mkdir(parents=True, exist_ok=True)

        staged_path = job_stage_dir / artifact_path.name
        shutil.copy2(artifact_path, staged_path)

        if staged_path.suffix.lower() == ".zip":
            extract_dir = job_stage_dir / "extracted"
            extract_dir.mkdir(parents=True, exist_ok=True)

            with zipfile.ZipFile(staged_path, "r") as zf:
                zf.extractall(extract_dir)

        self._set_state(OTAState.STAGING, 80, status_callback)
        return staged_path

    def _apply_update(
        self,
        job: OTAJob,
        staged_path: Path,
        status_callback,
    ):
        if job.target == "HPVC":
            self._set_state(OTAState.APPLYING, 90, status_callback)

            # 프로젝트 시연용:
            # 실제 HPVC 재부팅/서비스 교체 대신 검증된 패키지를 staging까지만 수행하고
            # version file을 갱신한다.
            time.sleep(0.5)
            return

        self._set_state(OTAState.FOTA_DISPATCHING, 85, status_callback)

        self.fota_dispatcher.dispatch(
            target=job.target,
            artifact_path=staged_path,
            target_version=job.target_version,
            job_id=job.job_id,
        )

        self._set_state(OTAState.FOTA_DISPATCHING, 95, status_callback)

    def _backup_versions(self, job_id: str) -> Optional[Path]:
        if not self.version_path.exists():
            return None

        backup_path = self.backup_dir / f"{job_id}_device_versions.json"
        shutil.copy2(self.version_path, backup_path)
        return backup_path

    def _rollback_versions(self, backup_path: Optional[Path]):
        if backup_path and backup_path.exists():
            shutil.copy2(backup_path, self.version_path)

    def _update_version(self, target: str, target_version: str):
        versions = {}

        if self.version_path.exists():
            with open(self.version_path, "r", encoding="utf-8") as f:
                versions = json.load(f)

        versions[target] = target_version

        with open(self.version_path, "w", encoding="utf-8") as f:
            json.dump(versions, f, indent=2)

    def _sha256_file(self, path: Path) -> str:
        digest = hashlib.sha256()

        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                digest.update(chunk)

        return digest.hexdigest()

    def _set_state(
        self,
        state: OTAState,
        progress: int,
        status_callback,
    ):
        self.state = state
        self.progress = progress
        self._publish(status_callback, self.get_status())

    def _publish(self, status_callback, payload: dict):
        if status_callback:
            status_callback(payload)
