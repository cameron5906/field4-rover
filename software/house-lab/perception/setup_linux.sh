#!/usr/bin/env bash
# Optional developer setup; downloads pinned dependencies and model artifacts.
set -euo pipefail
cd "$(dirname "$0")"
PYTHON="${PYTHON:-python3}"
"$PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 12), "The pinned Linux wheel lock requires Python 3.12"'
"$PYTHON" -m venv .venv
.venv/bin/python -m pip install --index-url https://pypi.org/simple --only-binary=:all: --require-hashes -r requirements-linux.lock
.venv/bin/python download_models.py
.venv/bin/python -m pip check
.venv/bin/python probe_runtime.py --pose
