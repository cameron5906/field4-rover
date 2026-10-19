# Reusable interfaces

This extraction retains algorithms, not the original integrated application.

## Observation pipeline

A camera frame is accompanied by an epoch, monotonic frame ID, capture timestamp, estimated pose, camera projection/mount metadata and synchronized measured lidar returns. `perception/worker.py` uses a one-request/one-result file mailbox. The client publishes an image and request atomically and rejects stale, mismatched or reordered replies. Starting that worker requires the optional local vision dependencies and model weights.

`landmarks.py` builds conservative object hypotheses from image bearings and measured motion. Lidar supports a range hypothesis only when geometry and timing are consistent. `person_evidence.py` associates visible body evidence with measured scan structure. `person_tracking.py` creates short-lived anonymous continuity IDs; these are not a biometric identity or a claim to know who someone is. Prediction supports association, not fabricated current range.

`person_detector_bridge.py` permits bounded continuation from an actual detector box and current measured foreground range after a previously confirmed pose. Missing poses do not become synthesized landmarks, and detector-only continuation cannot release a raised-hand hold.

## Estimation and control

- `mapping.gd`: empty-grid occupancy evidence and estimated odometry.
- `scan_matcher.gd`: bounded local scan corrections against already observed structure.
- `grid_motion.gd`: conservative grid traversal shared by navigation modules.
- `exploration_memory.gd`: observed progress, tried viewpoint regions and finite no-information inspection.
- `semantic_planner.gd`: reachable candidate selection, room-evidence handling and fresh-view mission-completion contracts.
- `landmark_task.gd`: observed-map routes toward a selected landmark with stand-off checks.
- `person_follow.gd` / `gesture_guard.gd`: target continuity, fresh-evidence gates and gesture holds.
- `decisions_adapter.gd`: bounded action-envelope parsing, matching identity and expiry validation. No transport is included.
- `physics/attitude_estimator.gd`: gyro/gravity filtering with explicit invalid/stale/uncertain states.

These modules still require a caller to provide observations and apply their outputs. Final application-time motor guards from the omitted orchestration are not a complete part of this extraction. A caller must not interpret a module's proposed velocity as permission to energize hardware.

## Simulation assumptions

The IMU and wheel/contact modules generate or approximate simulated measurements and forces. The configuration records robot-relative component geometry and estimated mass properties; it is not a home model. Camera profiles are uncalibrated projection assumptions. Uniform instantaneous lidar scans do not reproduce a rotating sensor's per-beam time skew, reflectivity errors or dropouts.

Global loop closure, persistent cross-launch semantic maps, room/door topology and physical sensor/motor adapters are not completed by this subset.
