"""Build explicit SYNTHETIC tracker→Godot contract data; no model inference."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "perception"))
sys.path.insert(0, str(ROOT / "perception" / "tests"))
from person_tracking import PersonTracker
from test_person_tracking import frame, person, synthetic_body, synthetic_scan_frame, color
from person_evidence import make_person_evidence
from person_detector_bridge import PersonDetectorBridge

tracker = PersonTracker()
steps = []
spec = [(1., 'lowered', False), (1.4, 'lowered', False), (1.8, 'lowered', True),
        (2.2, 'raised', False), (2.6, 'lowered', False), (3., 'lowered', False),
        (3.4, 'lowered', True), (3.8, None, False), (4.2, 'lowered', False),
        (4.6, 'lowered', False), (5., 'lowered', True), (6.95, None, False),
        (7.35, 'lowered', False), (7.75, 'lowered', False), (8.15, 'lowered', False),
        (8.55, 'lowered', False), (8.95, 'lowered', False), (9.35, 'lowered', True)]
for index, (t, gesture, moving) in enumerate(spec, 1):
    metadata = frame(index, t, epoch=4)
    poses = [] if gesture is None else [person(gesture=gesture)]
    for pose in poses:
        pose.update(center=[.5, .5], scale=.25)
    tracks = tracker.update(metadata, poses)
    result = {"ok": True, "epoch": 4, "frame_id": index,
              "capture_monotonic_ms": metadata["capture_monotonic_ms"],
              "poses": poses, "person_tracks": tracks}
    steps.append({"metadata": metadata, "result": result, "expected_moving": moving,
                  "expected_identity": 2 if index >= 16 else 1,
                  "explicit_restart": index == 16})
# Cross-layer semantics: current range readiness is distinct from releasing a
# positively raised-arm latch. Both unresolved same-position appearance
# conflicts and a geometrically distinct confirmed identity must remain held.
gesture_spec = [
    (1., 'unknown', 0, False, ''), (1.4, 'unknown', 0, False, ''),
    (1.8, 'unknown', 0, True, ''), (2.2, 'raised', 0, False, ''),
    (2.6, 'unknown', 0, False, ''), (3., 'lowered', 1, False, ''),
    (3.4, 'lowered', 1, False, ''), (3.8, 'lowered', 1, False, ''),
    (4.2, 'unknown', 0, False, ''), (4.6, 'lowered', 0, False, ''),
    (5., None, 0, False, ''), (5.4, 'lowered', 0, False, ''),
    (5.8, 'lowered', 0, False, ''), (6.2, 'lowered', 0, True, ''),
    (6.6, 'unknown', 0, False, 'stop'), (7., 'unknown', 0, False, ''),
    (7.4, 'unknown', 0, False, 'restart'), (7.8, 'unknown', 0, False, ''),
    (8.2, 'unknown', 0, True, ''),
]
gesture_scenarios = []
for distinct_identity in (False, True):
    gesture_tracker = PersonTracker()
    gesture_steps = []
    for index, (t, gesture, color_bin, moving, action) in enumerate(gesture_spec, 1):
        metadata = frame(index, t, epoch=4)
        depth = 5.3 if distinct_identity and color_bin else 3.0
        poses = [] if gesture is None else [person(gesture=gesture, bin_id=color_bin, depth=depth)]
        for pose in poses:
            pose.update(center=[.5, .5], scale=.25 * 3.0 / depth)
            pose['follow_evidence']['torso_box'][3] *= 3.0 / depth
            if gesture == 'unknown':
                pose['confidence'] = .4
        tracks = gesture_tracker.update(metadata, poses)
        result = {'ok': True, 'epoch': 4, 'frame_id': index,
                  'capture_monotonic_ms': metadata['capture_monotonic_ms'],
                  'poses': poses, 'person_tracks': tracks}
        if distinct_identity and color_bin:
            assert poses[0]['follow_evidence']['track_id'] == 2
            if index == 8:
                assert poses[0]['follow_evidence']['track_state'] == 'confirmed'
                assert poses[0]['follow_evidence']['gesture_release_eligible']
        elif color_bin:
            assert poses[0]['follow_evidence']['track_id'] is None
        gesture_steps.append({'metadata': metadata, 'result': result,
                              'expected_moving': moving, 'expected_identity': -1 if index in (15, 16) else 1,
                              'expect_observed_other_identity': distinct_identity and bool(color_bin),
                              'expect_confirmed_other_identity': distinct_identity and index == 8,
                              'explicit_restart': False, 'action': action})
    gesture_scenarios.append({'name': 'different_identity_gesture_latch' if distinct_identity else 'unresolved_identity_gesture_latch',
                              'steps': gesture_steps})

# Production bridge + tracker integration: no fabricated pose landmarks on the
# detector lane, no box-only acquisition, and genuine same-ID hand proof only.
bridge_tracker = PersonTracker()
bridge = PersonDetectorBridge()
bridge_steps = []
bridge_spec = [
    ('box', False, -1, ''), ('box', False, -1, ''), ('box', False, -1, ''),
    ('lowered', False, 1, ''), ('lowered', False, 1, ''), ('lowered', True, 1, ''),
    ('box', True, 1, ''), ('box', True, 1, ''), ('box', False, -1, 'stop'),
    ('box', False, -1, 'restart'), ('lowered', False, 1, ''),
    ('lowered', False, 1, ''), ('lowered', True, 1, ''),
    ('raised', False, 1, ''), ('box', False, 1, ''), ('box', False, 1, ''),
    ('box', False, 1, ''), ('lowered', False, 1, ''),
    ('lowered', False, 1, ''), ('lowered', True, 1, ''),
]
for index, (kind, moving, identity, action) in enumerate(bridge_spec, 1):
    t = 1. + (index-1)*.4
    metadata = synthetic_scan_frame(depth=4., beam_count=720, index=index, t=t)
    metadata['epoch'] = 4
    points = synthetic_body(depth=4.)
    xx, yy = [p['x']*640 for p in points], [p['y']*360 for p in points]
    box = [min(xx)-5, min(yy)-5, max(xx)-min(xx)+10, max(yy)-min(yy)+10]
    detections = [dict(label='person', score=.9, bbox=box)]
    poses = [] if kind == 'box' else [dict(
        gesture=kind, confidence=.95, center=[.5, .4], scale=.25,
        points=points, observation_source='pose_landmarks',
        follow_evidence=make_person_evidence(points, metadata, None, lambda *_: color()))]
    observations = poses[:] if poses else bridge.continuations(metadata, detections, bridge_tracker, None, lambda *_: color())
    tracks = bridge_tracker.update(metadata, observations)
    bridge.remember(metadata, poses, detections, bridge_tracker)
    if kind == 'box':
        assert not poses
        if index <= 3:
            assert not observations and not bridge_tracker.tracks
        else:
            assert len(observations) == 1 and 'points' not in observations[0]
            evidence = observations[0]['follow_evidence']
            assert evidence['track_id'] == 1 and evidence['motion_eligible']
            assert evidence['range_observed'] and not evidence['gesture_release_eligible']
    result = {'ok': True, 'epoch': 4, 'frame_id': index,
              'capture_monotonic_ms': metadata['capture_monotonic_ms'],
              'poses': poses, 'person_observations': observations, 'person_tracks': tracks}
    bridge_steps.append({'metadata': metadata, 'result': result,
                         'expected_moving': moving, 'expected_identity': identity,
                         'expect_non_pose_continuation': kind == 'box' and index > 3,
                         'explicit_restart': False, 'action': action})
gesture_scenarios.append({'name': 'detector_lidar_continuation_contract', 'steps': bridge_steps})

# Reproduce the independent-review overlapping-box appearance conflict through
# the production bridge in both detector output orders.
for reverse in (False, True):
    overlap_tracker, overlap_bridge, overlap_steps = PersonTracker(), PersonDetectorBridge(), []
    for index in range(1, 5):
        metadata = synthetic_scan_frame(depth=4., beam_count=720, index=index, t=1.+(index-1)*.4)
        metadata['epoch'] = 4
        points = synthetic_body(depth=4.)
        xx, yy = [p['x']*640 for p in points], [p['y']*360 for p in points]
        box = [min(xx)-5, min(yy)-5, max(xx)-min(xx)+10, max(yy)-min(yy)+10]
        detections = [dict(label='person', score=.9, bbox=box)]
        poses = [] if index == 4 else [dict(gesture='lowered', confidence=.95,
            center=[.5, .4], scale=.25, points=points, observation_source='pose_landmarks',
            follow_evidence=make_person_evidence(points, metadata, None, lambda *_: color()))]
        if index == 4:
            anchor = overlap_bridge.anchors[1]
            first_crop_x = int(box[0]+anchor['relative_torso'][0]*box[2])
            shifted = box[:]; shifted[0] += 2.
            detections.append(dict(label='person', score=.9, bbox=shifted))
            if reverse: detections.reverse()
            observations = overlap_bridge.continuations(metadata, detections, overlap_tracker, None,
                lambda _rgb, crop: color(0 if crop[0] == first_crop_x else 7))
            assert not observations, 'Appearance-conflicting overlapping box must veto all candidates'
        else:
            observations = poses[:]
        tracks = overlap_tracker.update(metadata, observations)
        overlap_bridge.remember(metadata, poses, detections, overlap_tracker)
        result = {'ok': True, 'epoch': 4, 'frame_id': index,
                  'capture_monotonic_ms': metadata['capture_monotonic_ms'],
                  'poses': poses, 'person_observations': observations, 'person_tracks': tracks}
        overlap_steps.append({'metadata': metadata, 'result': result,
                             'expected_moving': index == 3, 'expected_identity': 1,
                             'explicit_restart': False})
    gesture_scenarios.append({'name': 'overlapping_box_conflict_' + str(reverse), 'steps': overlap_steps})

path = ROOT / "test_results" / "follow_tracker_contract_synthetic_v073.json"
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps({"source": "synthetic_observations_through_production_PersonTracker",
                            "not_rendered_vision": True, "steps": steps,
                            "scenarios": [{"name": "base_track_contract", "steps": steps}] + gesture_scenarios}, indent=2)+"\n")
print(path)
