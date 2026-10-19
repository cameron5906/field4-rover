#!/usr/bin/env python3
"""Reproducible offline worker timing on this host, including future real Pi4 runs.

Uses local images/models only. No installs, network, motors or OS policy changes.
No images means an explicitly labelled blank-frame runtime smoke, not accuracy.
"""
from __future__ import annotations
import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time

from compute_profile import Scheduler, configure_worker_limits, fit_input_size, load_profile

ROOT = Path(__file__).resolve().parent


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_if_present(path):
    try:
        return Path(path).read_text(encoding="utf-8").strip().strip("\0")
    except (OSError, UnicodeError):
        return None


def hardware_sample():
    result = {"monotonic_s": time.monotonic(), "temperature_c": None, "arm_clock_hz": None, "throttled_bits": None}
    executable = shutil.which("vcgencmd")
    if executable:
        for key, command in (("temperature_raw", ["measure_temp"]), ("arm_clock_raw", ["measure_clock", "arm"]), ("throttled_raw", ["get_throttled"])):
            try:
                reply = subprocess.run([executable, *command], capture_output=True, text=True, timeout=2)
                result[key] = reply.stdout.strip() if reply.returncode == 0 else "unavailable"
            except (OSError, subprocess.TimeoutExpired):
                result[key] = "unavailable"
        try: result["temperature_c"] = float(result["temperature_raw"].split("=")[1].split("'")[0])
        except (KeyError, ValueError, IndexError): pass
        try: result["arm_clock_hz"] = int(result["arm_clock_raw"].split("=")[1])
        except (KeyError, ValueError, IndexError): pass
        try: result["throttled_bits"] = int(result["throttled_raw"].split("=")[1], 16)
        except (KeyError, ValueError, IndexError): pass
    if result["temperature_c"] is None:
        value = read_if_present("/sys/class/thermal/thermal_zone0/temp")
        try: result["temperature_c"] = float(value) / 1000
        except (TypeError, ValueError): pass
    return result


def host_info():
    board = read_if_present("/proc/device-tree/model")
    info = {"platform": platform.platform(), "machine": platform.machine(), "python": sys.version,
            "cpu_count": os.cpu_count(), "board_model": board, "is_raspberry_pi4": bool(board and "Raspberry Pi 4" in board),
            "memory_total_kib": None, "package_versions": {}}
    mem = read_if_present("/proc/meminfo")
    if mem:
        for line in mem.splitlines():
            if line.startswith("MemTotal:"):
                info["memory_total_kib"] = int(line.split()[1])
    for package in ("mediapipe", "numpy", "pillow"):
        try: info["package_versions"][package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: info["package_versions"][package] = None
    return info


def distribution(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values: return None
    percentile = lambda p: values[max(0, min(len(values) - 1, math.ceil(p * len(values)) - 1))]
    return {"n": len(values), "mean": sum(values) / len(values), "p50": percentile(.50),
            "p95": percentile(.95), "p99": percentile(.99), "max": values[-1]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", nargs="*", type=Path)
    parser.add_argument("--profile", default="native", help="native, pi4_budget, pi4_stress, or a JSON path")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threads", type=int)
    parser.add_argument("--cores", type=int)
    parser.add_argument("--navigation-log", type=Path, help="Attach an existing navigation log's hash; its co-execution remains unverified")
    parser.add_argument("--workload-label", default="isolated_vision", help="Record what else actually ran concurrently")
    args = parser.parse_args(argv)
    if not 1 <= args.frames <= 100000 or not 0 <= args.warmup <= 1000:
        parser.error("frames must be 1..100000 and warmup 0..1000")
    profile = load_profile(args.profile)
    if args.threads is not None: profile = replace(profile, threads=args.threads)
    if args.cores is not None: profile = replace(profile, affinity_cores=args.cores)
    limits = configure_worker_limits(profile)  # BEFORE importing actual worker/ML.
    started = time.monotonic()
    from PIL import Image
    from worker import Perception

    report = {"schema_version": 1, "started_utc": datetime.now(timezone.utc).isoformat(),
              "host": host_info(), "profile": profile.as_dict(), "enforcement": limits,
              "workload_label": args.workload_label, "models": {}, "inputs": [], "records": [],
              "hardware_samples": [hardware_sample()], "calibrated": False,
              "accuracy_evaluated": False, "navigation_coexecution_verified": False,
              "limits": "Host measurements only. Injected delays are separate. Blank frames and still-image replay are not accuracy, camera latency, gesture latency, thermal soak, or simultaneous-navigation validation. Pi4 calibration requires representative inputs on the actual board with its navigation workload and cooling/power conditions."}
    source_files = ["worker.py", "compute_profile.py", "benchmark_compute.py", "landmarks.py", "gesture.py"]
    report["source_sha256_before"] = {name: sha256(ROOT / name) for name in source_files}
    for filename, expected_file in (("efficientdet_lite0.tflite", "model_provenance.json"), ("pose_landmarker_lite.task", "pose_model_provenance.json")):
        digest = sha256(ROOT / "models" / filename)
        expected = json.loads((ROOT / expected_file).read_text())["sha256"]
        if digest != expected:
            raise ValueError(f"model hash mismatch for {filename}")
        report["models"][filename] = {"sha256": digest, "bytes": (ROOT / "models" / filename).stat().st_size}
    if args.navigation_log:
        report["navigation_evidence"] = {"filename": args.navigation_log.name, "sha256": sha256(args.navigation_log), "status": "attached_by_operator_not_verified_concurrent"}
    with tempfile.TemporaryDirectory(prefix="field4-benchmark-") as directory:
        prepared = []
        for index, source in enumerate(args.images or [None]):
            if source is None:
                image = Image.new("RGB", (640, 360))
                provenance = {"source": "synthetic_black_frame_runtime_smoke_only", "source_sha256": None}
            else:
                with Image.open(source) as loaded: image = loaded.convert("RGB")
                provenance = {"source": source.name, "source_sha256": sha256(source)}
            provenance["original_size"] = list(image.size)
            size = fit_input_size(*image.size, profile)
            if size != image.size: image = image.resize(size, Image.Resampling.BILINEAR)
            path = Path(directory) / f"frame{index}.png"
            image.save(path)
            provenance.update(input_size=list(size), processed_sha256=sha256(path))
            report["inputs"].append(provenance)
            prepared.append((path, size))
        engine = Perception()
        report["initialization_ms"] = (time.monotonic() - started) * 1000
        try:
            sequence = 0
            def infer(index, run_objects=True, run_pose=True):
                nonlocal sequence
                path, (width, height) = prepared[index % len(prepared)]
                frame = {"frame_id": sequence, "epoch": 0, "timestamp_s": time.monotonic(),
                         "capture_monotonic_ms": time.monotonic() * 1000,
                         "width": width, "height": height, "hfov_deg": 102.0,
                         "pose": [0.0, 0.0, 0.0], "camera_mount": [0.0, .133, -.1078]}
                sequence += 1
                return engine.infer(path, frame, run_objects=run_objects, run_pose=run_pose)
            warmup_start = time.monotonic()
            for i in range(args.warmup): infer(i)
            report["warmup"] = {"frames": args.warmup, "elapsed_s": time.monotonic() - warmup_start, "excluded_from_timing_distributions": True}
            scheduler = Scheduler(profile)
            for i in range(args.frames):
                delay = scheduler.wait_seconds()
                if delay: time.sleep(delay)
                selection = scheduler.start(frame_id=sequence, capture_age_at_start_ms=0)
                if selection is None:
                    raise RuntimeError("scheduler unexpectedly refused a serial benchmark frame")
                result = infer(i, selection["run_objects"], selection["run_pose"])
                timings = result["timings_ms"]
                telemetry = scheduler.finish({"inference_ms": timings["object"] + timings["pose"],
                    "object_inference_ms": timings["object"], "pose_inference_ms": timings["pose"],
                    "object_ran": selection["run_objects"], "pose_ran": selection["run_pose"]})
                record = {"index": i, "input_index": i % len(prepared), **telemetry["last"],
                          "detection_count": len(result.get("detections", [])), "pose_count": len(result.get("poses", []))}
                report["records"].append(record)
                if i % 30 == 29: report["hardware_samples"].append(hardware_sample())
            report["scheduler"] = scheduler.snapshot()
        finally:
            engine.close()
    report["hardware_samples"].append(hardware_sample())
    report["elapsed_s"] = time.monotonic() - started
    report["source_sha256_after"] = {name: sha256(ROOT / name) for name in source_files}
    report["source_stable_during_run"] = report["source_sha256_before"] == report["source_sha256_after"]
    report["summary_ms"] = {key: distribution([r[key] for r in report["records"]
                            if (not key.startswith("object_") or r["object_ran"]) and (not key.startswith("pose_") or r["pose_ran"])])
                            for key in ("object_inference_ms", "pose_inference_ms", "inference_ms", "real_service_ms", "injected_delay_ms", "service_ms")}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "host_is_pi4": report["host"]["is_raspberry_pi4"],
                      "calibrated": False, "summary_ms": report["summary_ms"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
