#!/usr/bin/env python3
"""Run real local MediaPipe chronologically on captured rendered RGB and sensors.

Every PNG needs same-stem JSON sensor metadata. Model inference never receives
scene/actor truth or scenario labels. Captures and raw output are kept intact.
This is cloud CPU evidence, not a Windows/Pi4 or physical-robot performance test.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_person_tracking import FRAME_KEYS, SCAN_KEYS, replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--scenario', default='rendered_sequence')
    args = parser.parse_args()
    paths = sorted(args.frames.glob('*.png'))
    if not paths:
        raise ValueError('No captured PNG frames')
    from worker import Perception
    started = time.monotonic()
    engine = Perception()
    initialization_ms = (time.monotonic()-started)*1000
    args.output.parent.mkdir(parents=True, exist_ok=True)
    timing = []
    try:
        with args.output.open('w') as output:
            for path in paths:
                raw = json.loads(path.with_suffix('.json').read_text())
                frame = {k: v for k, v in raw.items() if k in FRAME_KEYS}
                if isinstance(frame.get('scan'), dict):
                    frame['scan'] = {k: v for k, v in frame['scan'].items() if k in SCAN_KEYS}
                result = engine.infer(path, frame)
                timing.append(result['timings_ms']['total'])
                record = {'fixture_kind': 'rendered', 'scenario': args.scenario,
                          'image': str(path), 'image_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                          'frame': frame, 'result': result}
                output.write(json.dumps(record, allow_nan=False)+'\n')
                output.flush()
                print(json.dumps({'image': path.name, 'ok': result['ok'],
                    'poses': [{'gesture': p.get('gesture'), 'reason': p.get('follow_evidence', {}).get('reason'),
                               'range_observed': p.get('follow_evidence', {}).get('range_observed'),
                               'track_id': p.get('follow_evidence', {}).get('track_id'),
                               'state': p.get('follow_evidence', {}).get('track_state'),
                               'motion_eligible': p.get('follow_evidence', {}).get('motion_eligible')}
                              for p in result.get('poses', [])]}), flush=True)
    finally:
        engine.close()
    report = replay(args.output, args.baseline)
    report['runtime'] = {'initialization_ms': initialization_ms,
                         'inference_total_ms': timing,
                         'limits': 'Real MediaPipe inference on rendered captures; cloud CPU only, not closed-loop robot-motion verification.'}
    report_path = args.output.with_suffix('.report.json')
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'report': str(report_path), 'totals': report['totals']}))


if __name__ == '__main__':
    main()
