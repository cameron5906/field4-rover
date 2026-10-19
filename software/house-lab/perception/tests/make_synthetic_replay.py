#!/usr/bin/env python3
"""Generate clearly labelled deterministic sensor/pose doubles for replay tests."""
import argparse
import json
from pathlib import Path
from test_person_tracking import color, synthetic_body, synthetic_scan_frame


def records():
    index = 0
    for scenario, count, gesture in [('walking_two_legs', 10, 'lowered'),
                                      ('one_leg_visible', 6, 'lowered'),
                                      ('occluded', 3, 'unknown'),
                                      ('reacquisition', 5, 'lowered'),
                                      ('stationary_raised', 3, 'raised'),
                                      ('stationary_lowered', 3, 'lowered')]:
        for _ in range(count):
            index += 1
            frame = synthetic_scan_frame(index=index, t=1.+index*.4)
            if scenario == 'one_leg_visible':
                wall = synthetic_scan_frame(legs=False)['scan']['ranges_m']
                for beam in range(1, 180):
                    frame['scan']['ranges_m'][beam] = wall[beam]
            pose = {'gesture': gesture, 'confidence': .95,
                    'points': synthetic_body(raised=gesture == 'raised'),
                    'follow_evidence': {'appearance': color()}}
            yield {'fixture_kind': 'synthetic', 'scenario': scenario, 'frame': frame,
                   'result': {'poses': [] if scenario == 'occluded' else [pose], 'pose_ran': True}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    Path(args.output).write_text(''.join(json.dumps(record, allow_nan=False)+'\n' for record in records()))
