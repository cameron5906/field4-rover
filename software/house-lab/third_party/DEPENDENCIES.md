# Dependency provenance

Upstream payloads are not bundled. The files below retain source-archive version, size and hash metadata; no claim is made that their links were re-fetched during publication review.

- `perception/requirements.txt`: pinned Python package list.
- `perception/requirements-linux.lock` and `linux_artifacts.json`: hashed Linux CPython 3.12 x86-64 artifacts from PyPI.
- `perception/requirements-windows.lock` and `windows_runtime_manifest.json`: hashed Windows CPython 3.12 x64 artifacts from PyPI and the official Python distribution.
- `perception/model_provenance.json`: EfficientDet-Lite0 model URL, SHA-256 and associated documentation.
- `perception/pose_model_provenance.json`: MediaPipe Pose Landmarker Lite URL, SHA-256 and associated documentation.

Official reference pages:

- [Godot](https://godotengine.org/) and [archived 4.6.3 release page](https://godotengine.org/download/archive/4.6.3-stable/)
- [Python Windows embeddable distribution](https://docs.python.org/3.12/using/windows.html#the-embeddable-package)
- [MediaPipe object detector](https://ai.google.dev/edge/mediapipe/solutions/vision/object_detector)
- [MediaPipe pose landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [PyPI](https://pypi.org/)

Review the licenses and notices applicable to the exact versions you acquire before redistribution. A source URL or checksum is provenance, not a redistribution permission. The combined CAD/GLB and embedded external sensor mesh were excluded because a sufficient license chain was not established.

The archived acquisition helpers validate exact hashes and reject unexpected direct download hosts. They are optional developer utilities; do not run them merely to inspect source or execute the pure synthetic tests.
