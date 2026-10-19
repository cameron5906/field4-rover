#!/usr/bin/env python3
"""Fetch the two pinned official model artifacts for developer builds only."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import urllib.request
from urllib.parse import urlparse


def main() -> None:
    root = Path(__file__).resolve().parent
    for provenance, filename in [
        ("model_provenance.json", "efficientdet_lite0.tflite"),
        ("pose_model_provenance.json", "pose_landmarker_lite.task"),
    ]:
        metadata = json.loads((root / provenance).read_text(encoding="utf-8"))
        target = root / "models" / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            data = target.read_bytes()
        else:
            parsed = urlparse(metadata["source"])
            if parsed.scheme != "https" or parsed.hostname != "storage.googleapis.com" or not parsed.path.startswith("/mediapipe-models/"):
                raise ValueError("Unexpected model origin")
            with urllib.request.urlopen(metadata["source"], timeout=120) as response:
                data = response.read()
        if len(data) != metadata["bytes"] or hashlib.sha256(data).hexdigest() != metadata["sha256"]:
            raise ValueError(f"Model hash/size mismatch: {filename}")
        if not target.exists():
            temporary = target.with_suffix(target.suffix + ".partial")
            temporary.write_bytes(data)
            temporary.replace(target)
        print(f"Verified {filename} ({len(data):,} bytes)")


if __name__ == "__main__":
    main()
