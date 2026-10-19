#!/usr/bin/env python3
"""Run the actual worker on independent captured frames and draw its actual boxes.

Capture JSON supplies only the worker's frame/sensor contract. No object oracle,
expected labels or scene geometry is passed to either model. This is detection
evidence and timing, not an IoU/precision/recall evaluation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--montage", type=Path, required=True)
    args = parser.parse_args()
    cache = Path(tempfile.gettempdir()) / "field4-perception-cache"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache / "matplotlib"))
    os.environ.setdefault("XDG_CACHE_HOME", str(cache))
    os.environ.setdefault("MPLBACKEND", "Agg")
    from PIL import Image, ImageDraw, ImageFont
    from worker import Perception

    paths = sorted(args.frames.glob("*.png"))
    if not paths:
        raise ValueError(f"No PNG captures in {args.frames}")
    start = time.perf_counter()
    engine = Perception()
    initialization_ms = (time.perf_counter() - start) * 1000
    records = []
    try:
        for path in paths:
            frame = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            # Deliberately reset association between camera placements. Models use
            # IMAGE mode and no temporally accumulated landmark is being evaluated.
            engine.fusion.reset()
            engine.epoch = None
            result = engine.infer(path, frame)
            record = {"image": str(path), "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "result": result}
            records.append(record)
            print(json.dumps({"image": path.name, "labels": [{"label": d["label"], "score": round(d["score"], 4), "bbox": d["bbox"]} for d in result.get("detections", [])], "poses": len(result.get("poses", [])), "timings_ms": result["timings_ms"]}), flush=True)
    finally:
        engine.close()
    totals = [r["result"]["timings_ms"]["total"] for r in records]
    report = {
        "mode": "actual_worker_independent_single_frames",
        "worker_sha256": hashlib.sha256((Path(__file__).parent / "worker.py").read_bytes()).hexdigest(),
        "frame_count": len(records),
        "initialization_ms": initialization_ms,
        "latency_total_ms": {"mean": statistics.mean(totals), "median": statistics.median(totals), "min": min(totals), "max": max(totals)},
        "frames_with_detections": sum(bool(r["result"].get("detections")) for r in records),
        "total_detections": sum(len(r["result"].get("detections", [])) for r in records),
        "records": records,
        "limits": "No oracle comparison, IoU, precision or recall claim. CPU timing measured on cloud x86-64, not Raspberry Pi or Windows. Image pixels preserved except detection overlays; original captures remain unchanged.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
    except OSError:
        font = ImageFont.load_default()
    cols = 2
    width, height = Image.open(paths[0]).size
    header = 48
    montage = Image.new("RGB", (cols * width, math.ceil(len(records) / cols) * (height + header)), "#121b24")
    md = ImageDraw.Draw(montage)
    for index, record in enumerate(records):
        source = Image.open(record["image"]).convert("RGB")
        if source.size != (width, height):
            raise ValueError("Mixed image sizes would require changing source pixels")
        draw = ImageDraw.Draw(source)
        result = record["result"]
        for detection in result.get("detections", []):
            x, y, w, h = detection["bbox"]
            label = f"{detection['label']} {detection['score']:.2f}"
            draw.rectangle((x, y, x + w, y + h), outline="#ffcc00", width=2)
            text_box = draw.textbbox((x + 3, max(1, y - 20)), label, font=font)
            draw.rectangle(text_box, fill="#121b24")
            draw.text((x + 3, max(1, y - 20)), label, font=font, fill="#ffdd66")
        left, top = (index % cols) * width, (index // cols) * (height + header)
        montage.paste(source, (left, top + header))
        labels = ", ".join(f"{d['label']} {d['score']:.2f}" for d in result.get("detections", [])) or "no detections >= 0.40"
        md.text((left + 8, top + 4), f"{Path(record['image']).stem} | {result['timings_ms']['total']:.1f} ms | {len(result.get('poses', []))} poses", font=font, fill="white")
        md.text((left + 8, top + 25), labels, font=font, fill="#c5d1dc")
    args.montage.parent.mkdir(parents=True, exist_ok=True)
    montage.save(args.montage)
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
