import hashlib
import json
import os
import shutil
import subprocess
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

        # 실제 HPVC 서비스 업데이트용 runtime 경로
        self.runtime_dir = Path(
            os.environ.get("HPVC_RUNTIME_DIR", "/home/pi/hpvc_runtime")
        )
        self.releases_dir = self.runtime_dir / "releases"
        self.current_link = self.runtime_dir / "current"
        self.service_name = os.environ.get("HPVC_SERVICE_NAME", "hpvc-ota.service")

        # 업데이트 작업 결과물은 release 내부가 아니라 runtime 공용 updates에 저장
        self.updates_dir = self.runtime_dir / "updates"
        self.download_dir = self.updates_dir / "downloaded"
        self.staging_dir = self.updates_dir / "staging"
        self.backup_dir = self.updates_dir / "backup"
        self.log_dir = self.updates_dir / "logs"

        for directory in (
            self.releases_dir,
            self.download_dir,
            self.staging_dir,
            self.backup_dir,
            self.log_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)

        self.version_path = CONFIG_DIR / "device_versions.json"

        # HPVC 실제 apply/rollback 상태 관리
        self._hpvc_previous_release = None
        self._hpvc_new_release = None
        self._hpvc_restart_required = False

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

        self._hpvc_previous_release = None
        self._hpvc_new_release = None
        self._hpvc_restart_required = False

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

            if ota_job.target == "HPVC" and self._hpvc_restart_required:
                self._schedule_service_restart()

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

                if ota_job.target == "HPVC":
                    self._rollback_hpvc_release()

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

        if not job.target_version:
            raise ValueError("target_version is required")

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

            self._apply_hpvc_release_update(
                job=job,
                staged_path=staged_path,
            )

            self._set_state(OTAState.APPLYING, 95, status_callback)
            return

        self._set_state(OTAState.FOTA_DISPATCHING, 85, status_callback)

        self.fota_dispatcher.dispatch(
            target=job.target,
            artifact_path=staged_path,
            target_version=job.target_version,
            job_id=job.job_id,
        )

        self._set_state(OTAState.FOTA_DISPATCHING, 95, status_callback)

    def _apply_hpvc_release_update(
        self,
        job: OTAJob,
        staged_path: Path,
    ):
        if not self.current_link.exists():
            raise RuntimeError(f"current link does not exist: {self.current_link}")

        if not self.current_link.is_symlink():
            raise RuntimeError(
                f"current path must be symlink for safe OTA: {self.current_link}"
            )

        previous_release = self.current_link.resolve()
        self._hpvc_previous_release = previous_release

        source_root = self._find_hpvc_update_source(staged_path)

        target_release_dir = self.releases_dir / job.target_version

        if target_release_dir.exists():
            shutil.rmtree(target_release_dir)

        target_release_dir.mkdir(parents=True, exist_ok=True)

        self._copy_release_source(
            source_root=source_root,
            target_release_dir=target_release_dir,
        )

        self._update_release_version_file(
            release_dir=target_release_dir,
            target=job.target,
            target_version=job.target_version,
        )

        self._switch_current_link(target_release_dir)

        self._hpvc_new_release = target_release_dir
        self._hpvc_restart_required = True

        print(
            "[HPVC OTA] release applied:",
            f"previous={previous_release}",
            f"new={target_release_dir}",
        )

    def _find_hpvc_update_source(self, staged_path: Path) -> Path:
        stage_dir = staged_path.parent
        extract_dir = stage_dir / "extracted"

        if staged_path.suffix.lower() != ".zip":
            raise ValueError("HPVC OTA package must be a zip file")

        if not extract_dir.exists():
            raise ValueError(f"extracted directory not found: {extract_dir}")

        # 권장 패키지 구조:
        # hpvc_2.0.1.zip
        # ├─ manifest.json
        # └─ hpvc_ota/
        #    ├─ config/
        #    └─ ota/
        hpvc_dir = extract_dir / "hpvc_ota"

        if hpvc_dir.exists() and hpvc_dir.is_dir():
            self._validate_hpvc_source_dir(hpvc_dir)
            return hpvc_dir

        # 예외적으로 zip 최상위에 config/ota가 바로 있는 구조도 허용
        self._validate_hpvc_source_dir(extract_dir)
        return extract_dir

    def _validate_hpvc_source_dir(self, source_dir: Path):
        config_dir = source_dir / "config"
        ota_dir = source_dir / "ota"
        ota_server = ota_dir / "ota_server.py"

        if not config_dir.exists():
            raise ValueError(f"HPVC package missing config directory: {config_dir}")

        if not ota_dir.exists():
            raise ValueError(f"HPVC package missing ota directory: {ota_dir}")

        if not ota_server.exists():
            raise ValueError(f"HPVC package missing ota/ota_server.py: {ota_server}")

    def _copy_release_source(
        self,
        source_root: Path,
        target_release_dir: Path,
    ):
        ignore_patterns = shutil.ignore_patterns(
            ".git",
            "__pycache__",
            "*.pyc",
            "*.zip",
            "*.tar.gz",
            "*.bin",
            "updates",
            "test_artifacts",
        )

        for item in source_root.iterdir():
            src = item
            dst = target_release_dir / item.name

            if item.name in {".git", "__pycache__", "updates", "test_artifacts"}:
                continue

            if item.is_dir():
                shutil.copytree(src, dst, ignore=ignore_patterns)
            else:
                if item.suffix in {".zip", ".bin", ".gz"}:
                    continue
                shutil.copy2(src, dst)

        # 새 release에도 updates 기본 구조 생성
        for subdir in ("downloaded", "staging", "backup", "logs"):
            (target_release_dir / "updates" / subdir).mkdir(parents=True, exist_ok=True)

    def _switch_current_link(self, target_release_dir: Path):
        target_release_dir = target_release_dir.resolve()

        temp_link = self.current_link.with_name("current.tmp")

        if temp_link.exists() or temp_link.is_symlink():
            temp_link.unlink()

        os.symlink(target_release_dir, temp_link, target_is_directory=True)
        os.replace(temp_link, self.current_link)

    def _rollback_hpvc_release(self):
        if not self._hpvc_previous_release:
            return

        if not self._hpvc_previous_release.exists():
            print(
                "[HPVC OTA] rollback skipped:",
                f"previous release missing: {self._hpvc_previous_release}",
            )
            return

        try:
            self._switch_current_link(self._hpvc_previous_release)
            print(
                "[HPVC OTA] rollback current link:",
                f"current -> {self._hpvc_previous_release}",
            )
        except Exception as exc:
            print(f"[HPVC OTA] rollback failed: {exc}")

    def _schedule_service_restart(self):
        print(f"[HPVC OTA] scheduling service restart: {self.service_name}")

        # SUCCESS/result publish 이후 서비스가 재시작되도록 약간 지연한다.
        command = (
            "sleep 2; "
            f"sudo -n /usr/bin/systemctl restart {self.service_name}"
        )

        subprocess.Popen(
            ["/bin/sh", "-c", command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

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

    def _update_release_version_file(
        self,
        release_dir: Path,
        target: str,
        target_version: str,
    ):
        config_dir = release_dir / "config"
        config_dir.mkdir(parents=True, exist_ok=True)

        version_path = config_dir / "device_versions.json"

        versions = {}

        if version_path.exists():
            with open(version_path, "r", encoding="utf-8") as f:
                versions = json.load(f)

        versions[target] = target_version

        with open(version_path, "w", encoding="utf-8") as f:
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
