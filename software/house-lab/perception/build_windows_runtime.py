#!/usr/bin/env python3
"""Assemble the pinned offline Windows x64 runtime; run this on Linux or Windows.

Downloads only the exact artifacts recorded in windows_runtime_manifest.json.
It never installs Python system-wide, runs an installer, or binds a network port.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import urllib.request
from urllib.parse import urlparse
import zipfile

ROOT = Path(__file__).resolve().parent


def checked_artifact(entry: dict, cache: Path, offline: bool) -> Path:
    path = cache / entry["filename"]
    if not path.exists():
        if offline:
            raise FileNotFoundError(f"Offline cache lacks {path.name}")
        url = entry["url"]
        host = urlparse(url).hostname
        if urlparse(url).scheme != "https" or host not in {"www.python.org", "files.pythonhosted.org"}:
            raise ValueError(f"Unexpected artifact source: {url}")
        temporary = path.with_suffix(path.suffix + ".partial")
        with urllib.request.urlopen(url, timeout=120) as response, temporary.open("wb") as output:
            shutil.copyfileobj(response, output)
        temporary.replace(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != entry["sha256"] or path.stat().st_size != entry["bytes"]:
        raise ValueError(f"Artifact hash/size mismatch: {path.name}")
    return path


def extract_archive(archive: Path, destination: Path, wheel: bool = False) -> None:
    with zipfile.ZipFile(archive) as zipped:
        for info in zipped.infolist():
            relative = PurePosixPath(info.filename)
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError(f"Unsafe archive member: {info.filename}")
            if info.is_dir():
                continue
            if wheel and relative.parts[0].endswith(".data"):
                # Wheel relocation scheme: only importable package data is needed.
                if len(relative.parts) < 3 or relative.parts[1] not in {"purelib", "platlib"}:
                    continue
                relative = PurePosixPath(*relative.parts[2:])
            target = destination.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            with zipped.open(info) as source, target.open("wb") as output:
                shutil.copyfileobj(source, output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime_windows")
    parser.add_argument("--cache", type=Path, default=ROOT / "vendor_cache")
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "windows_runtime_manifest.json").read_text(encoding="utf-8"))
    args.cache.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing runtime: {output}")
    staging = output.with_name(output.name + ".staging")
    if staging.exists():
        raise FileExistsError(f"Remove or inspect incomplete staging first: {staging}")
    staging.mkdir(parents=True)
    extract_archive(checked_artifact(manifest["python"], args.cache, args.offline), staging)
    site_packages = staging / "Lib" / "site-packages"
    for entry in manifest["wheels"]:
        print(f"Extracting {entry['filename']}", flush=True)
        extract_archive(checked_artifact(entry, args.cache, args.offline), site_packages, wheel=True)
    # Isolated, app-private Python. The parent perception directory contains worker
    # modules; no system Python, PYTHONPATH or user-site packages are used.
    (staging / "python312._pth").write_text(
        "python312.zip\n.\n..\nLib/site-packages\nimport site\n", encoding="utf-8"
    )
    (staging / "FIELD4_RUNTIME_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    staging.replace(output)
    total = sum(p.stat().st_size for p in output.rglob("*") if p.is_file())
    print(json.dumps({"runtime": str(output), "bytes": total, "windows_execution_verified": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
