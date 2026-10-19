#!/usr/bin/env python3
"""Offline old/new range and identity replay from locally captured observations.

Input JSONL, one record per frame:
 {"fixture_kind":"rendered"|"synthetic", "scenario":"walking", "frame":{...},
  "result":{"poses":[{"points":[...], "gesture":..., "confidence":...,
                       "follow_evidence":{"appearance":[24 bins]}}]}}
The histogram is the captured RGB-derived observation, not a simulator label.
Actor/truth fields, if present for evaluation, are never passed to perception.
No image inference or API calls occur in this replay. Report its fixture origin.
"""
import argparse
from collections import Counter
import copy
import importlib.util
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from person_evidence import make_person_evidence
from person_tracking import PersonTracker

FRAME_KEYS = ('frame_id', 'epoch', 'timestamp_s', 'capture_monotonic_ms', 'width',
              'height', 'hfov_deg', 'pose', 'camera_mount', 'pitch_rad', 'roll_rad', 'scan')
SCAN_KEYS = ('timestamp_s', 'pose', 'mount', 'ranges_m', 'pitch_rad', 'roll_rad',
             'scan_id', 'profile', 'ray_count', 'angle_increment_rad',
             'field_of_view_rad', 'scan_hz', 'measurements_per_second',
             'calibrated', 'scan_model', 'sampling_status')


def replay(path, baseline=None):
    old = None
    if baseline:
        spec = importlib.util.spec_from_file_location('baseline_person_evidence', baseline)
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
    tracker = PersonTracker()
    totals, scenarios, rows = Counter(), {}, []
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get('fixture_kind') not in ('rendered', 'synthetic'):
            raise ValueError('Every fixture must explicitly declare rendered or synthetic origin')
        scenario = record.get('scenario', 'unspecified')
        counts = scenarios.setdefault(scenario, Counter())
        frame = {k: v for k, v in record['frame'].items() if k in FRAME_KEYS}
        if isinstance(frame.get('scan'), dict):
            frame['scan'] = {k: v for k, v in frame['scan'].items() if k in SCAN_KEYS}
        poses, ranges, old_ranges, reasons = [], 0, 0, []
        for source in record['result'].get('poses', []):
            appearance = source.get('follow_evidence', {}).get('appearance', [])
            histogram = lambda _rgb, _box, a=appearance: a[:]
            points = source.get('points', [])
            evidence = make_person_evidence(points, frame, None, histogram)
            ranges += bool(evidence.get('range_valid'))
            reasons.append(evidence.get('reason'))
            if old:
                old_ranges += bool(old.make_person_evidence(points, frame, None, histogram).get('range_valid'))
            poses.append({'gesture': source.get('gesture'), 'confidence': source.get('confidence'),
                          'follow_evidence': evidence})
        status = tracker.update(frame, poses, pose_ran=record['result'].get('pose_ran', True))
        moving = sum(bool(p['follow_evidence'].get('motion_eligible')) for p in poses)
        for counts in (totals, counts):
            counts['frames'] += 1
            counts[record['fixture_kind']+'_frames'] += 1
            counts['person_observations'] += len(poses)
            counts['range_observations_accepted'] += ranges
            counts['motion_eligible_observations'] += moving
            if old:
                counts['baseline_range_observations_accepted'] += old_ranges
            for reason in reasons:
                counts['reason:'+str(reason)] += 1
            for pose in poses:
                counts['track_state:'+str(pose['follow_evidence'].get('track_state'))] += 1
        rows.append({'frame_id': frame['frame_id'], 'scenario': scenario, 'range_accepted': ranges,
                     'baseline_range_accepted': old_ranges if old else None,
                     'motion_eligible': moving, 'reasons': reasons,
                     'poses': [{'gesture': p.get('gesture'),
                                'track_id': p['follow_evidence'].get('track_id'),
                                'track_state': p['follow_evidence'].get('track_state'),
                                'range_valid': p['follow_evidence'].get('range_valid', False),
                                'motion_eligible': p['follow_evidence'].get('motion_eligible', False),
                                'lowered_fresh_count': p['follow_evidence'].get('lowered_fresh_count', 0)}
                               for p in poses],
                     'tracks': status['tracks']})
    return {'fixture_path': str(path), 'baseline_path': str(baseline) if baseline else None,
            'totals': dict(totals), 'scenarios': {k: dict(v) for k,v in scenarios.items()}, 'frames': rows}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture')
    parser.add_argument('--baseline')
    parser.add_argument('--output')
    args = parser.parse_args()
    result = replay(args.fixture, args.baseline)
    output = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        Path(args.output).write_text(output+'\n')
    else:
        print(output)
