#!/usr/bin/env python3
"""Replay genuine saved model outputs/RGB/lidar through current person fusion.

No model inference is rerun and no commands are generated. This measures whether
detector-backed continuation has genuine current evidence, not closed-loop motion.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_rendered_capture import convert_records, read_complete_records
from person_detector_bridge import PersonDetectorBridge
from person_evidence import make_person_evidence
from person_tracking import PersonTracker
from gesture import classify_pose


def evaluate(capture, output_prefix, last_frame=None):
    # Imports the production pixel histogram, but does not instantiate or run
    # Perception's ML models. The pinned local runtime supplies Pillow/numpy.
    from worker import appearance, load_frame_image
    records = convert_records(read_complete_records(capture/'frames.jsonl'),
                              read_complete_records(capture/'events.jsonl'), capture)
    tracker, bridge = PersonTracker(), PersonDetectorBridge()
    totals, scenarios, rows, normalized = Counter(), {}, [], []
    for record in records:
        frame = record['frame']
        if last_frame is not None and frame['frame_id'] > last_frame:
            break
        source = record['result']
        rgb = load_frame_image(record['image']).numpy_view()
        poses = []
        for raw in source.get('poses', []):
            points = raw['points']
            pose = classify_pose(points)
            pose.update(points=points, observation_source='pose_landmarks')
            pose['follow_evidence'] = make_person_evidence(points, frame, rgb, appearance)
            poses.append(pose)
        detections = source.get('detections', [])
        observations = poses[:]
        if source.get('pose_ran', True) and not poses:
            observations = bridge.continuations(frame, detections, tracker, rgb, appearance)
        raw_ranges = sum(bool(obs['follow_evidence'].get('range_valid')) for obs in observations)
        tracks = tracker.update(frame, observations, pose_ran=source.get('pose_ran', True))
        bridge.remember(frame, poses, detections, tracker)
        counts = scenarios.setdefault(record['scenario'], Counter())
        for count in (totals, counts):
            count['frames'] += 1
            count['actual_pose_frames'] += bool(poses)
            count['person_box_frames'] += any(d.get('label') == 'person' for d in detections)
            count['detector_continuation_frames'] += bool(observations and not poses)
            count['raw_range_observations'] += raw_ranges
            count['measurement_eligible_observations'] += sum(bool(obs['follow_evidence'].get('motion_eligible')) for obs in observations)
            count['detector_measurement_eligible'] += sum(bool(obs['follow_evidence'].get('motion_eligible')) for obs in observations if obs['observation_source'] == 'detector_lidar_continuation')
        rows.append({'frame_id': frame['frame_id'], 'scenario': record['scenario'],
                     'actual_poses': len(poses), 'anchor_ids': sorted(bridge.anchors),
                     'observations': [{'source': obs['observation_source'], 'gesture': obs['gesture'],
                                       **{key: obs['follow_evidence'].get(key) for key in
                                          ('track_id', 'track_state', 'reason', 'range_m', 'range_observed',
                                           'range_valid', 'motion_eligible', 'gesture_release_eligible',
                                           'pose_anchor_frame_id', 'pose_anchor_age_s')}} for obs in observations]})
        result = {'poses': poses, 'person_observations': observations, 'person_tracks': tracks,
                  'pose_ran': source.get('pose_ran', True), 'detections': detections}
        normalized.append(dict(record, result=result))
    path = capture/(output_prefix+'.jsonl')
    path.write_text(''.join(json.dumps(record, allow_nan=False)+'\n' for record in normalized))
    report = {'capture': str(capture), 'scope': 'Offline fusion replay of genuine saved model outputs, RGB, and lidar. No new model inference or closed-loop motion.',
              'totals': dict(totals), 'scenarios': {key: dict(value) for key, value in scenarios.items()}, 'frames': rows}
    (capture/(output_prefix+'.report.json')).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output-prefix', default='detector_bridge_replay')
    parser.add_argument('--last-frame', type=int)
    args = parser.parse_args()
    report = evaluate(args.capture, args.output_prefix, args.last_frame)
    print(json.dumps({'totals': report['totals'], 'scenarios': report['scenarios']}, indent=2))
