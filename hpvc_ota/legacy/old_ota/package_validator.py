from pathlib import Path
from manifest_parser import ManifestParser
from checksum import calculate_sha256


class PackageValidator:
    def __init__(self, manifest_path: str, package_dir: str):
        self.manifest_path = manifest_path
        self.package_dir = Path(package_dir)
        self.manifest = None

    def load_manifest(self):
        parser = ManifestParser(self.manifest_path)
        self.manifest = parser.load()
        parser.validate()
        return self.manifest

    def check_target_files(self):
        if self.manifest is None:
            raise ValueError("Manifest is not loaded")

        for target in self.manifest["targets"]:
            file_name = target["file_name"]
            file_path = self.package_dir / file_name

            if not file_path.exists():
                raise FileNotFoundError(f"Update file not found: {file_path}")

            print(f"[FILE OK] {target['device_id']} -> {file_path}")

        return True

    def check_checksum(self):
        if self.manifest is None:
            raise ValueError("Manifest is not loaded")

        for target in self.manifest["targets"]:
            device_id = target["device_id"]
            file_name = target["file_name"]
            expected_checksum = target["checksum"]

            file_path = self.package_dir / file_name
            actual_checksum = calculate_sha256(str(file_path))

            if actual_checksum != expected_checksum:
                raise ValueError(
                    f"Checksum mismatch: {device_id}, "
                    f"expected={expected_checksum}, actual={actual_checksum}"
                )

            print(f"[CHECKSUM OK] {device_id}")

        return True

    def validate_package(self):
        self.load_manifest()
        self.check_target_files()
        self.check_checksum()
        print("[PACKAGE VALIDATION] SUCCESS")


if __name__ == "__main__":
    validator = PackageValidator(
        manifest_path="updates/incoming/manifest.json",
        package_dir="updates/incoming"
    )
    validator.validate_package()
