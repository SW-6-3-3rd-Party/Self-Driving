import json
import hashlib
from pathlib import Path


PACKAGE_DIR = Path("updates/incoming")
MANIFEST_PATH = PACKAGE_DIR / "manifest.json"


def calculate_sha256(file_path: Path) -> str:
    sha256 = hashlib.sha256()

    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            sha256.update(chunk)

    return sha256.hexdigest()


with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
    manifest = json.load(f)

for target in manifest["targets"]:
    file_path = PACKAGE_DIR / target["file_name"]

    if not file_path.exists():
        raise FileNotFoundError(f"Update file not found: {file_path}")

    target["checksum"] = calculate_sha256(file_path)
    print(f"[CHECKSUM UPDATED] {target['device_id']} -> {target['checksum']}")

with open(MANIFEST_PATH, "w", encoding="utf-8") as f:
    json.dump(manifest, f, indent=2)

print("[MANIFEST UPDATED] updates/incoming/manifest.json")
