import shutil
from pathlib import Path
from urllib.parse import unquote, urlparse

import requests


class ArtifactDownloader:
    def download(self, artifact_url: str, output_path: Path) -> Path:
        output_path.parent.mkdir(parents=True, exist_ok=True)

        parsed = urlparse(artifact_url)

        if parsed.scheme == "file":
            src = Path(unquote(parsed.path))

            if not src.exists():
                raise FileNotFoundError(f"artifact file not found: {src}")

            shutil.copy2(src, output_path)
            return output_path

        if parsed.scheme in ("http", "https"):
            with requests.get(artifact_url, stream=True, timeout=20) as response:
                response.raise_for_status()

                with open(output_path, "wb") as f:
                    for chunk in response.iter_content(chunk_size=1024 * 64):
                        if chunk:
                            f.write(chunk)

            return output_path

        raise ValueError(f"unsupported artifact_url scheme: {artifact_url}")
