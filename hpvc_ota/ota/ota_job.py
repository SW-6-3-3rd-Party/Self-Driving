from dataclasses import dataclass
from typing import Optional


@dataclass
class OTAJob:
    job_id: str
    target: str
    target_version: str
    artifact_url: str
    sha256: str = ""
    size: Optional[int] = None
    rollback_enabled: bool = True

    @classmethod
    def from_dict(cls, data: dict):
        required_fields = [
            "job_id",
            "target",
            "target_version",
            "artifact_url",
        ]

        for field in required_fields:
            if field not in data or data[field] in (None, ""):
                raise ValueError(f"missing required field: {field}")

        return cls(
            job_id=str(data["job_id"]),
            target=str(data["target"]),
            target_version=str(data["target_version"]),
            artifact_url=str(data["artifact_url"]),
            sha256=str(data.get("sha256", "")),
            size=data.get("size"),
            rollback_enabled=bool(data.get("rollback_enabled", True)),
        )

    def to_dict(self) -> dict:
        return {
            "job_id": self.job_id,
            "target": self.target,
            "target_version": self.target_version,
            "artifact_url": self.artifact_url,
            "sha256": self.sha256,
            "size": self.size,
            "rollback_enabled": self.rollback_enabled,
        }
