"""Bounded, measurable compute-envelope emulation for the isolated vision worker.

These profiles are engineering stress scenarios, NOT Raspberry Pi benchmarks.
Never call configure_worker_limits or finish from the simulation/control process.
Only standard-library imports are used, so limits can be set before ML imports.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, fields
import json
import math
import os
from pathlib import Path
import random
import sys
import threading
import time


PROFILE_DIR = Path(__file__).with_name("profiles")


@dataclass(frozen=True)
class ComputeProfile:
    name: str = "pi4_budget"
    description: str = "Uncalibrated Pi4-oriented scheduling envelope"
    inference_rate_hz: float = 3.0
    object_every_n_frames: int = 1
    pose_every_n_frames: int = 1
    minimum_processing_ms: float = 220.0
    delay_jitter_ms: float = 40.0
    contention_every_n_frames: int = 0
    contention_delay_ms: float = 0.0
    threads: int = 2
    affinity_cores: int = 2
    max_input_width: int = 640
    max_input_height: int = 360
    random_seed: int = 4
    calibration_evidence: str | None = None

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("profile name must be a nonempty string")
        if not isinstance(self.description, str):
            raise ValueError("profile description must be a string")
        for name in ("inference_rate_hz", "minimum_processing_ms", "delay_jitter_ms", "contention_delay_ms"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if not 0 < self.inference_rate_hz <= 60:
            raise ValueError("inference_rate_hz must be in (0, 60]")
        for name in ("minimum_processing_ms", "delay_jitter_ms", "contention_delay_ms"):
            if not 0 <= getattr(self, name) <= 60_000:
                raise ValueError(f"{name} must be in [0, 60000]")
        for name in ("object_every_n_frames", "pose_every_n_frames", "threads", "max_input_width", "max_input_height"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("affinity_cores", "contention_every_n_frames"):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if type(self.random_seed) is not int:
            raise ValueError("random_seed must be an integer")
        if self.calibration_evidence is not None and (not isinstance(self.calibration_evidence, str) or not self.calibration_evidence):
            raise ValueError("calibration_evidence must be null or a nonempty evidence reference")
        if self.threads > 64 or self.affinity_cores > 64 or self.max_input_width > 4096 or self.max_input_height > 4096:
            raise ValueError("unreasonable thread/core/image limit")

    def as_dict(self):
        return asdict(self)


def load_profile(name_or_path: str | Path = "pi4_budget") -> ComputeProfile:
    path = Path(name_or_path)
    if not path.is_file():
        path = PROFILE_DIR / (str(name_or_path) + ".json")
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("compute profile must be an object")
    unknown = set(raw) - {f.name for f in fields(ComputeProfile)}
    if unknown:
        raise ValueError(f"unknown compute profile fields: {sorted(unknown)}")
    return ComputeProfile(**raw)


def fit_input_size(width: int, height: int, profile: ComputeProfile) -> tuple[int, int]:
    """Bound pixels while preserving aspect ratio; never upscale or crop."""
    if type(width) is not int or type(height) is not int or min(width, height) < 1:
        raise ValueError("input dimensions must be positive integers")
    scale = min(1.0, profile.max_input_width / width, profile.max_input_height / height)
    return max(1, int(width * scale)), max(1, int(height * scale))


def configure_worker_limits(profile: ComputeProfile) -> dict:
    """Call once in an isolated worker BEFORE importing MediaPipe/numpy.

    Environment caps are library requests, not guaranteed TFLite thread caps.
    Affinity bounds cores available for simultaneous execution, not thread count.
    Failure is reported rather than disguised as successful enforcement.
    This modifies only the current worker, never OS policy or another process.
    """
    keys = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS")
    for key in keys:
        os.environ[key] = str(profile.threads)
    status = {
        "library_threads_requested": profile.threads,
        "environment_requests": {key: os.environ[key] for key in keys},
        "hard_native_thread_cap_enforced": False,
        "affinity_cores_requested": profile.affinity_cores,
        "affinity_enforced": False,
        "effective_affinity_cpus": None,
        "late_ml_import": any(name in sys.modules for name in ("mediapipe", "numpy")),
        "note": "MediaPipe public Python BaseOptions does not expose a native thread count; affinity is a core ceiling, not Pi4 equivalence.",
    }
    if profile.affinity_cores == 0:
        status["affinity_status"] = "not_requested"
        return status
    try:
        if hasattr(os, "sched_getaffinity") and hasattr(os, "sched_setaffinity"):
            cpus = sorted(os.sched_getaffinity(0))[:profile.affinity_cores]
            # Existing worker threads also need the restriction. Future threads
            # inherit their creating thread's affinity. No other PID is touched.
            tids = [int(p.name) for p in Path("/proc/self/task").iterdir()] if Path("/proc/self/task").is_dir() else [0]
            for tid in tids:
                try:
                    os.sched_setaffinity(tid, cpus)
                except ProcessLookupError:
                    pass  # A worker thread may have exited during enumeration.
            effective = sorted(os.sched_getaffinity(0))
            status.update(affinity_enforced=len(effective) <= profile.affinity_cores,
                          effective_affinity_cpus=effective, affinity_status="applied")
        elif sys.platform == "win32":
            import ctypes
            from ctypes import wintypes
            kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            kernel.GetCurrentProcess.restype = wintypes.HANDLE
            kernel.GetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
            kernel.GetProcessAffinityMask.restype = wintypes.BOOL
            kernel.SetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
            kernel.SetProcessAffinityMask.restype = wintypes.BOOL
            handle = kernel.GetCurrentProcess()
            allowed, system = ctypes.c_size_t(), ctypes.c_size_t()
            if not kernel.GetProcessAffinityMask(handle, ctypes.byref(allowed), ctypes.byref(system)):
                raise ctypes.WinError(ctypes.get_last_error())
            cpus = [bit for bit in range(ctypes.sizeof(ctypes.c_size_t) * 8) if allowed.value & (1 << bit)][:profile.affinity_cores]
            if not cpus or not kernel.SetProcessAffinityMask(handle, sum(1 << bit for bit in cpus)):
                raise ctypes.WinError(ctypes.get_last_error())
            status.update(affinity_enforced=True, effective_affinity_cpus=cpus, affinity_status="applied")
        else:
            status["affinity_status"] = "unsupported_platform"
    except (OSError, ValueError) as exc:
        status.update(affinity_status="failed", affinity_error=str(exc))
    return status


class LatestFrameSlot:
    """Optional one-slot mailbox. Replaces waiting work; never queues history.

    A single-in-flight adapter can instead drop frames before capture and keep
    queue_length=0. Do not add this mailbox on top of an unbounded transport.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._value = None
        self.offered = self.replaced = 0

    def offer(self, frame):
        if frame is None:
            raise ValueError("None is the empty-slot sentinel")
        with self._lock:
            self.offered += 1
            self.replaced += self._value is not None
            self._value = frame

    def take(self):
        with self._lock:
            value, self._value = self._value, None
            return value

    @property
    def queue_length(self):
        with self._lock:
            return int(self._value is not None)


class Scheduler:
    """Single-worker cadence and result-availability delays, in monotonic time.

    can_start is a pure query; start reserves one job and returns model selection.
    finish sleeps ONLY this worker and returns honest timing/counter telemetry.
    There is no simulation-time multiplier, busy-loop load or control dependency.
    """
    def __init__(self, profile: ComputeProfile, *, clock=time.monotonic, sleeper=time.sleep, cpu_clock=time.process_time):
        self.profile = profile
        self.clock, self.sleeper, self.cpu_clock = clock, sleeper, cpu_clock
        self.rng = random.Random(profile.random_seed)
        self.created_s, self.created_cpu_s = clock(), cpu_clock()
        self.next_start_s = self.created_s
        self.active = None
        self.counts = Counter()
        self.inference_total_ms = self.service_total_ms = self.injected_total_ms = 0.0
        self.last = {}

    def can_start(self, monotonic_s: float | None = None) -> bool:
        now = self.clock() if monotonic_s is None else monotonic_s
        return self.active is None and now + 1e-9 >= self.next_start_s

    def wait_seconds(self, monotonic_s: float | None = None) -> float:
        now = self.clock() if monotonic_s is None else monotonic_s
        return max(0.0, self.next_start_s - now)

    def start(self, monotonic_s: float | None = None, *, frame_id=None, capture_age_at_start_ms: float | None = None) -> dict | None:
        now = self.clock() if monotonic_s is None else monotonic_s
        if not math.isfinite(now):
            raise ValueError("start time must be finite")
        if capture_age_at_start_ms is not None and (not math.isfinite(capture_age_at_start_ms) or capture_age_at_start_ms < 0):
            raise ValueError("capture age must be nonnegative, measured in one clock domain")
        if not self.can_start(now):
            self.counts["busy_rejected" if self.active else "cadence_rejected"] += 1
            return None
        index = self.counts["frames_started"]
        self.counts["frames_started"] += 1
        self.active = {
            "frame_id": frame_id, "start_s": now,
            "capture_age_at_start_ms": capture_age_at_start_ms,
            "run_objects": index % self.profile.object_every_n_frames == 0,
            "run_pose": index % self.profile.pose_every_n_frames == 0,
        }
        self.next_start_s = now + 1 / self.profile.inference_rate_hz
        return dict(self.active)

    def record_dropped(self, count: int = 1, reason: str = "producer_busy"):
        if type(count) is not int or count < 0:
            raise ValueError("drop count must be a nonnegative integer")
        self.counts["frames_dropped"] += count
        self.counts["dropped_" + reason] += count

    def finish(self, timing: dict) -> dict:
        """timing: measured inference_ms plus optional object/pose_inference_ms.

        Optional object_ran/pose_ran override planned counts after backend failure.
        capture-to-result age is null unless a same-clock capture age was supplied.
        Results must retain their original frame/epoch/capture identity in adapter.
        """
        if self.active is None:
            raise RuntimeError("finish requires an active job")
        job = self.active
        inference_ms = timing.get("inference_ms", 0.0)
        for key in ("inference_ms", "object_inference_ms", "pose_inference_ms"):
            value = timing.get(key, 0.0)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be a measured nonnegative finite duration")
        raw_done = self.clock()
        real_service_ms = max(0.0, (raw_done - job["start_s"]) * 1000)
        jitter_ms = self.rng.uniform(0.0, self.profile.delay_jitter_ms)
        contention_ms = 0.0
        if self.profile.contention_every_n_frames and self.counts["frames_started"] % self.profile.contention_every_n_frames == 0:
            contention_ms = self.profile.contention_delay_ms
        # Never shorten a genuinely slower host/device result. The floor models
        # total service; separate positive jitter/contention model delivery load.
        requested_ms = max(0.0, self.profile.minimum_processing_ms - real_service_ms) + jitter_ms + contention_ms
        if requested_ms > 0:
            self.sleeper(requested_ms / 1000)
        done = self.clock()
        injected_ms = max(0.0, (done - raw_done) * 1000) if requested_ms > 0 else 0.0
        service_ms = max(0.0, (done - job["start_s"]) * 1000)
        self.counts["frames_completed"] += 1
        objects_ran = bool(timing.get("object_ran", job["run_objects"]))
        pose_ran = bool(timing.get("pose_ran", job["run_pose"]))
        self.counts["object_runs"] += objects_ran
        self.counts["pose_runs"] += pose_ran
        self.counts["object_skips"] += not objects_ran
        self.counts["pose_skips"] += not pose_ran
        self.inference_total_ms += inference_ms
        self.service_total_ms += service_ms
        self.injected_total_ms += injected_ms
        self.last = {
            "frame_id": job["frame_id"], "inference_ms": inference_ms,
            "object_inference_ms": timing.get("object_inference_ms"),
            "pose_inference_ms": timing.get("pose_inference_ms"),
            "real_service_ms": real_service_ms, "injected_delay_ms": injected_ms,
            "injected_delay_requested_ms": requested_ms, "service_ms": service_ms,
            "emulated_jitter_ms": jitter_ms, "emulated_contention_ms": contention_ms,
            "object_ran": objects_ran, "pose_ran": pose_ran,
            "capture_to_result_age_ms": None if job["capture_age_at_start_ms"] is None else job["capture_age_at_start_ms"] + service_ms,
            "age_clock_note": "End-to-end capture-to-consumption age must also be measured by the producer in its own monotonic clock.",
        }
        self.active = None
        # Never replay missed deadlines as a burst. The next start can occur now
        # after an overrun, but its own subsequent period starts at that time.
        self.next_start_s = max(self.next_start_s, done)
        return self.snapshot(done)

    def snapshot(self, monotonic_s: float | None = None) -> dict:
        now = self.clock() if monotonic_s is None else monotonic_s
        elapsed = max(0.0, now - self.created_s)
        rate = lambda count: count / elapsed if elapsed > 0 else 0.0
        return {
            "profile": self.profile.name,
            "calibration_evidence": self.profile.calibration_evidence,
            "pi4_hardware_equivalence": False,
            "target_frame_rate_hz": self.profile.inference_rate_hz,
            "actual_frame_rate_hz": rate(self.counts["frames_completed"]),
            "actual_object_rate_hz": rate(self.counts["object_runs"]),
            "actual_pose_rate_hz": rate(self.counts["pose_runs"]),
            "max_in_flight": 1, "in_flight": int(self.active is not None),
            "queue_length": 0,
            "counts": dict(self.counts), "last": dict(self.last),
            "worker_elapsed_s": elapsed,
            "inference_wall_busy_fraction": self.inference_total_ms / (1000 * elapsed) if elapsed > 0 else 0.0,
            "worker_service_busy_fraction": self.service_total_ms / (1000 * elapsed) if elapsed > 0 else 0.0,
            "process_cpu_core_equivalents": max(0.0, self.cpu_clock() - self.created_cpu_s) / elapsed if elapsed > 0 else 0.0,
            "utilization_note": "Measured worker CPU time / wall time in core equivalents; service busy includes injected sleep. Neither is whole-Pi CPU utilization.",
        }
