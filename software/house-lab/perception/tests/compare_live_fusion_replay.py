#!/usr/bin/env python3
"""Compare saved live fusion decisions with a separately regenerated replay."""
import argparse
import json
import math
from pathlib import Path
from replay_rendered_capture import read_complete_records

FIELDS = ('track_id', 'track_state', 'range_observed', 'range_valid',
          'motion_eligible', 'gesture_release_eligible', 'pose_anchor_frame_id')
NUMERIC_FIELDS = ('range_m', 'range_lower_m', 'range_uncertainty_m', 'pose_anchor_age_s')


def observations(record):
    result = record['result']
    return result.get('person_observations', result.get('poses', []))


def compare(capture, replay_name):
    live = read_complete_records(capture/'frames.jsonl')
    replay = read_complete_records(capture/replay_name)
    differences = []
    if len(live) != len(replay):
        differences.append({'reason': 'frame_count_mismatch', 'live': len(live), 'replay': len(replay)})
    for a, b in zip(live, replay):
        frame = a['metadata']['frame_id']
        if frame != b['frame']['frame_id']:
            differences.append({'frame': frame, 'reason': 'frame_order_mismatch'})
            continue
        aa, bb = observations(a), observations(b)
        if len(aa) != len(bb):
            differences.append({'frame': frame, 'reason': 'observation_count_mismatch', 'live': len(aa), 'replay': len(bb)})
            continue
        for index, (x, y) in enumerate(zip(aa, bb)):
            ex, ey = x.get('follow_evidence', {}), y.get('follow_evidence', {})
            for key in ('observation_source', 'gesture'):
                if x.get(key) != y.get(key):
                    differences.append({'frame': frame, 'observation': index, 'field': key, 'live': x.get(key), 'replay': y.get(key)})
            for key in FIELDS:
                if ex.get(key) != ey.get(key):
                    differences.append({'frame': frame, 'observation': index, 'field': key, 'live': ex.get(key), 'replay': ey.get(key)})
            for key in NUMERIC_FIELDS:
                p, q = ex.get(key), ey.get(key)
                equal = p == q
                if isinstance(p, (int, float)) and isinstance(q, (int, float)):
                    equal = math.isfinite(p) and math.isfinite(q) and math.isclose(p, q, rel_tol=0, abs_tol=1e-7)
                if not equal:
                    differences.append({'frame': frame, 'observation': index, 'field': key, 'live': p, 'replay': q})
    return {'live_frames': len(live), 'replayed_frames': len(replay),
            'decision_fields': list(FIELDS) + ['observation_source', 'gesture'],
            'numeric_fields': list(NUMERIC_FIELDS), 'numeric_absolute_tolerance': 1e-7,
            'mismatched_frames': len({d['frame'] for d in differences if 'frame' in d}) + abs(len(live)-len(replay)),
            'mismatches': len(differences), 'differences': differences}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--replay-name', default='detector_bridge_replay.jsonl')
    args = parser.parse_args()
    report = compare(args.capture, args.replay_name)
    (args.capture/'replay_live_decision_consistency.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'differences'}, indent=2))
    raise SystemExit(1 if report['mismatches'] else 0)
