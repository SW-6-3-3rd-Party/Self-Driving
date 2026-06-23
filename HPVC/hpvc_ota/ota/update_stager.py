import json
import shutil
from pathlib import Path


class UpdateStager:
    def __init__(self, manifest_path: str, package_dir: str, verified_root: str):
        self.manifest_path = Path(manifest_path)
        self.package_dir = Path(package_dir)
        self.verified_root = Path(verified_root)

    def stage(self):
        if not self.manifest_path.exists():
            raise FileNotFoundError(f"Manifest not found: {self.manifest_path}")

        with open(self.manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        package_version = manifest["package_version"]
        verified_dir = self.verified_root / package_version
        verified_dir.mkdir(parents=True, exist_ok=True)

        shutil.copy2(self.manifest_path, verified_dir / "manifest.json")
        print(f"[STAGE] manifest -> {verified_dir / 'manifest.json'}")

        for target in manifest["targets"]:
            file_name = target["file_name"]
            src = self.package_dir / file_name
            dst = verified_dir / file_name

            if not src.exists():
                raise FileNotFoundError(f"Update file not found: {src}")

            shutil.copy2(src, dst)
            print(f"[STAGE] {target['device_id']} -> {dst}")

        print(f"[STAGE] package_version={package_version} staged successfully")
        return verified_dir


if __name__ == "__main__":
    stager = UpdateStager(
        manifest_path="updates/incoming/manifest.json",
        package_dir="updates/incoming",
        verified_root="updates/verified"
    )
    stager.stage()
