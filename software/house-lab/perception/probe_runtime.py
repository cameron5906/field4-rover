#!/usr/bin/env python3
"""Offline real-inference probe. Does not bind a port or call a model service."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import tempfile
import time

MODEL_SHA256 = "0720bf247bd76e6594ea28fa9c6f7c5242be774818997dbbeffc4da460c723bb"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", nargs="*", type=Path)
    parser.add_argument("--model", type=Path, default=Path(__file__).parent / "models" / "efficientdet_lite0.tflite")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--threshold", type=float, default=0.35)
    parser.add_argument("--pose", action="store_true", help="Also execute Pose Landmarker Lite on each image")
    args = parser.parse_args()
    # MediaPipe imports Matplotlib even when no plot is requested. Keep its
    # caches out of the app folder and avoid depending on a writable home.
    cache = Path(tempfile.gettempdir()) / "field4-perception-cache"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache / "matplotlib"))
    os.environ.setdefault("XDG_CACHE_HOME", str(cache))
    os.environ.setdefault("MPLBACKEND", "Agg")
    started = time.perf_counter()
    import mediapipe as mp
    import numpy as np

    digest = hashlib.sha256(args.model.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        raise RuntimeError(f"Model hash mismatch: {digest}")
    options = mp.tasks.vision.ObjectDetectorOptions(
        base_options=mp.tasks.BaseOptions(
            model_asset_path=str(args.model.resolve()),
            delegate=mp.tasks.BaseOptions.Delegate.CPU,
        ),
        running_mode=mp.tasks.vision.RunningMode.IMAGE,
        max_results=20,
        score_threshold=args.threshold,
    )
    records = []
    with mp.tasks.vision.ObjectDetector.create_from_options(options) as detector:
        initialized_ms = round((time.perf_counter() - started) * 1000, 2)
        for image_path in args.images or [None]:
            if image_path is None:
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.zeros((360, 640, 3), dtype=np.uint8))
                source = "synthetic_black_frame_runtime_smoke_only"
            else:
                image = mp.Image.create_from_file(str(image_path.resolve()))
                source = str(image_path)
            begin = time.perf_counter()
            result = detector.detect(image)
            inference_ms = round((time.perf_counter() - begin) * 1000, 2)
            records.append({
                "image": source,
                "image_width": image.width,
                "image_height": image.height,
                "inference_ms": inference_ms,
                "detections": [{
                    "box_xywh": [d.bounding_box.origin_x, d.bounding_box.origin_y, d.bounding_box.width, d.bounding_box.height],
                    "categories": [{"label": c.category_name, "score": c.score, "index": c.index} for c in d.categories],
                } for d in result.detections],
            })
    pose_records = []
    if args.pose:
        pose_model = Path(__file__).parent / "models" / "pose_landmarker_lite.task"
        pose_sha = hashlib.sha256(pose_model.read_bytes()).hexdigest()
        if pose_sha != "59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a":
            raise RuntimeError("Pose model hash mismatch")
        pose_options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(pose_model.resolve()), delegate=mp.tasks.BaseOptions.Delegate.CPU),
            running_mode=mp.tasks.vision.RunningMode.IMAGE,
            num_poses=1,
        )
        with mp.tasks.vision.PoseLandmarker.create_from_options(pose_options) as landmarker:
            for image_path in args.images or [None]:
                image = mp.Image.create_from_file(str(image_path.resolve())) if image_path else mp.Image(image_format=mp.ImageFormat.SRGB, data=np.zeros((360, 640, 3), dtype=np.uint8))
                begin = time.perf_counter()
                result = landmarker.detect(image)
                pose_records.append({
                    "image": str(image_path) if image_path else "synthetic_black_frame_runtime_smoke_only",
                    "inference_ms": round((time.perf_counter() - begin) * 1000, 2),
                    "poses": [[{"x": p.x, "y": p.y, "z": p.z, "visibility": p.visibility, "presence": p.presence} for p in pose] for pose in result.pose_landmarks],
                })
    report = {
        "success": True,
        "platform": platform.platform(),
        "python": sys.version,
        "mediapipe": mp.__version__,
        "model_sha256": digest,
        "threshold": args.threshold,
        "initialization_ms": initialized_ms,
        "records": records,
        "pose_records": pose_records,
        "limits": "Runtime/inference evidence only. Synthetic scenes are out of domain; a smoke test is not detection accuracy or Windows execution validation.",
    }
    rendered = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
