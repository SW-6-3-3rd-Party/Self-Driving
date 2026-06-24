import json
from pathlib import Path


class ManifestParser:
    def __init__(self, manifest_path: str):
        self.manifest_path = Path(manifest_path)
        self.manifest = None

    def load(self):
        if not self.manifest_path.exists():
            raise FileNotFoundError(f"Manifest not found: {self.manifest_path}")

        with open(self.manifest_path, "r", encoding="utf-8") as f:
            self.manifest = json.load(f)

        return self.manifest

    def validate(self):
        if self.manifest is None:
            raise ValueError("Manifest is not loaded")

        required_top_keys = ["package_version", "targets"]
        for key in required_top_keys:
            if key not in self.manifest:
                raise ValueError(f"Missing manifest key: {key}")

        if not isinstance(self.manifest["targets"], list):
            raise ValueError("targets must be a list")

        required_target_keys = [
            "device_id",
            "current_version",
            "target_version",
            "file_name",
            "checksum"
        ]

        for target in self.manifest["targets"]:
            for key in required_target_keys:
                if key not in target:
                    raise ValueError(f"Missing target key: {key}")

        return True

    def print_summary(self):
        print(f"[MANIFEST] package_version={self.manifest['package_version']}")

        for target in self.manifest["targets"]:
            print(
                f"[TARGET] device={target['device_id']}, "
                f"{target['current_version']} -> {target['target_version']}, "
                f"file={target['file_name']}"
            )


if __name__ == "__main__":
    parser = ManifestParser("updates/incoming/manifest.json")
    parser.load()
    parser.validate()
    parser.print_summary()
