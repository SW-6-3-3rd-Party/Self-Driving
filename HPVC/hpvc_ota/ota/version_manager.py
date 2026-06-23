import json
from pathlib import Path


class VersionManager:
    def __init__(self, version_path: str, manifest_path: str):
        self.version_path = Path(version_path)
        self.manifest_path = Path(manifest_path)

    def load_versions(self):
        if not self.version_path.exists():
            raise FileNotFoundError(f"Version file not found: {self.version_path}")

        with open(self.version_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def save_versions(self, versions):
        with open(self.version_path, "w", encoding="utf-8") as f:
            json.dump(versions, f, indent=2)

    def load_manifest(self):
        if not self.manifest_path.exists():
            raise FileNotFoundError(f"Manifest not found: {self.manifest_path}")

        with open(self.manifest_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def parse_version(self, version: str):
        return tuple(int(part) for part in version.split("."))

    def check_update_required(self):
        current_versions = self.load_versions()
        manifest = self.load_manifest()

        for target in manifest["targets"]:
            device_id = target["device_id"]
            manifest_current = target["current_version"]
            target_version = target["target_version"]

            if device_id not in current_versions:
                raise ValueError(f"Unknown device_id in version file: {device_id}")

            actual_current = current_versions[device_id]

            if actual_current != manifest_current:
                raise ValueError(
                    f"Current version mismatch: {device_id}, "
                    f"device={actual_current}, manifest={manifest_current}"
                )

            if self.parse_version(target_version) <= self.parse_version(actual_current):
                raise ValueError(
                    f"No valid update required: {device_id}, "
                    f"current={actual_current}, target={target_version}"
                )

            print(f"[VERSION OK] {device_id}: {actual_current} -> {target_version}")

        print("[VERSION CHECK] SUCCESS")
        return True

    def apply_target_versions_mock(self):
        versions = self.load_versions()
        manifest = self.load_manifest()

        for target in manifest["targets"]:
            device_id = target["device_id"]
            target_version = target["target_version"]

            versions[device_id] = target_version
            print(f"[VERSION APPLY MOCK] {device_id} -> {target_version}")

        self.save_versions(versions)
        print("[VERSION APPLY MOCK] SUCCESS")

    def validate_updated_versions(self):
        versions = self.load_versions()
        manifest = self.load_manifest()

        for target in manifest["targets"]:
            device_id = target["device_id"]
            target_version = target["target_version"]
            actual_version = versions.get(device_id)

            if actual_version != target_version:
                raise ValueError(
                    f"Version validation failed: {device_id}, "
                    f"expected={target_version}, actual={actual_version}"
                )

            print(f"[VERSION VALIDATED] {device_id}: {actual_version}")

        print("[VERSION VALIDATION] SUCCESS")
        return True


if __name__ == "__main__":
    manager = VersionManager(
        version_path="config/device_versions.json",
        manifest_path="updates/incoming/manifest.json"
    )
    manager.check_update_required()
