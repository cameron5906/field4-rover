#!/usr/bin/env python3
"""Replay a snapshot of the actual rendered test's captured inference/sensors.

No model rerun, simulator truth input, fabricated pose, or movement commands.
For an active capture, the report covers only complete records available when
read. Phase names label evaluation only and are never passed to perception.
"""
import argparse
import json
from pathlib import Path

from replay_person_tracking import FRAME_KEYS, SCAN_KEYS, replay


PHASE_EVENTS = {
    'explicit_follow_start', 'walking_enabled', 'test_actor_hands_raised',
    'test_actor_hands_lowered', 'brief_rgb_occlusion_start',
    'brief_rgb_occlusion_end', 'long_rgb_occlusion_start',
    'long_rgb_occlusion_end', 'explicit_restart_after_track_expiry',
    'manual_stop', 'explicit_restart_after_manual_stop', 'default_walk_button',
}


def read_complete_records(path):
    """Do not mistake an unfinished writer's last JSON line for a full frame."""
    lines = Path(path).read_text().splitlines(keepends=True)
    return [json.loads(line) for line in lines if line.endswith('\n') and line.strip()]


def convert_records(records, events, capture):
    output = []
    for record in records:
        raw = record['metadata']
        frame = {key: value for key, value in raw.items() if key in FRAME_KEYS}
        if isinstance(frame.get('scan'), dict):
            frame['scan'] = {key: value for key, value in frame['scan'].items() if key in SCAN_KEYS}
        phase = 'initial'
        for event in events:
            if event['wall_ms'] <= frame['capture_monotonic_ms'] and event['type'] in PHASE_EVENTS:
                phase = event['type']
        output.append({'fixture_kind': 'rendered', 'scenario': phase,
                       'image': str(capture/record['image']), 'frame': frame,
                       'result': record['result']})
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--output-prefix', default='range_replay_current')
    args = parser.parse_args()
    records = read_complete_records(args.capture/'frames.jsonl')
    events = read_complete_records(args.capture/'events.jsonl')
    normalized = convert_records(records, events, args.capture)
    path = args.capture/(args.output_prefix+'.jsonl')
    path.write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in normalized))
    report = replay(path, args.baseline)
    report['capture_complete'] = any(event['type'] == 'finished' for event in events)
    report['scope'] = ('Actual rendered RGB model outputs and recorded lidar replayed offline. '
                       'No new inference or closed-loop motion is performed by this replay.')
    report_path = args.capture/(args.output_prefix+'.report.json')
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'report': str(report_path), 'capture_complete': report['capture_complete'],
                      'totals': report['totals'], 'scenarios': report['scenarios']}, indent=2))


if __name__ == '__main__':
    main()
