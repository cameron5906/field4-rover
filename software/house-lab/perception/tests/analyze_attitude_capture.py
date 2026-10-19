#!/usr/bin/env python3
"""Compare recorded raw-specific-force tilt and sensor-only attitude decisions.

Read-only with respect to production sources. It never supplies replacement
orientation, inferred range or commands, and does not use evaluation truth.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def complete_rows(path):
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            break
    return rows


def raw_tilt(raw):
    values = raw.get('accel_m_s2', [])
    if not raw.get('valid') or raw.get('stale') or len(values) != 3:
        return 1.0
    if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return 1.0
    if abs(math.sqrt(sum(v*v for v in values)) - 9.80665) > .55:
        return 1.0
    return max(abs(math.atan2(values[0], values[2])), abs(math.atan2(values[1], values[2])))


def analyze(capture):
    records = complete_rows(capture/'frames.jsonl')
    telemetry = complete_rows(capture/'telemetry.jsonl')
    start = telemetry[0]['sim_time'] - telemetry[0]['episode_s'] if telemetry else 0.0
    totals, first45, walking, reasons = Counter(), Counter(), Counter(), Counter()
    observations = []
    for record in records:
        frame = record['metadata']
        estimate = frame.get('attitude_estimate')
        scan_estimate = frame.get('scan', {}).get('attitude_estimate')
        if not estimate or not scan_estimate:
            totals['frames_without_attitude_diagnostics'] += 1
            continue
        episode = frame['timestamp_s'] - start
        old_angle = max(raw_tilt(frame.get('raw_imu', {})), raw_tilt(frame['scan'].get('raw_imu', {})))
        old_reject = old_angle > .025
        new_reject = not (estimate.get('projection_safe') and scan_estimate.get('projection_safe'))
        actual_reject = max(abs(frame.get('roll_rad', 1)), abs(frame.get('pitch_rad', 1)), abs(frame['scan'].get('roll_rad', 1)), abs(frame['scan'].get('pitch_rad', 1))) > .025
        people = record['result'].get('person_observations', [])
        counts = [totals]
        if 0 <= episode < 45:
            counts.append(first45)
        if 10 <= episode < 45:
            counts.append(walking)
        for count in counts:
            count['frames'] += 1
            count['raw_specific_force_tilt_rejected'] += old_reject
            count['estimator_projection_rejected'] += new_reject
            count['raw_rejected_estimator_safe'] += old_reject and not new_reject
            count['raw_safe_estimator_rejected'] += not old_reject and new_reject
            count['recorded_gate_estimator_mismatch'] += actual_reject != new_reject
            count['person_observation_frames'] += bool(people)
            count['current_range_observations'] += sum(bool(p.get('follow_evidence', {}).get('range_observed')) for p in people)
            count['measurement_ready_observations'] += sum(bool(p.get('follow_evidence', {}).get('motion_eligible')) for p in people)
        if new_reject:
            if not estimate.get('projection_safe'):
                reasons['capture:' + estimate.get('reason', 'missing')] += 1
            if not scan_estimate.get('projection_safe'):
                reasons['scan:' + scan_estimate.get('reason', 'missing')] += 1
        observations.append({'frame_id': frame['frame_id'], 'episode_s': episode,
            'raw_angle_or_invalid_sentinel_rad': old_angle,
            'estimated_roll_rad': estimate.get('roll_rad'), 'estimated_pitch_rad': estimate.get('pitch_rad'),
            'uncertainty_rad': estimate.get('uncertainty_rad'),
            'estimate_valid': estimate.get('valid'), 'projection_safe': not new_reject,
            'reason': estimate.get('reason'), 'scan_reason': scan_estimate.get('reason'),
            'scan_projection_safe': scan_estimate.get('projection_safe')})
    return {'scope': 'Actual saved sensor diagnostics. Raw-specific-force comparison diagnoses the previous formula, not true attitude. Capture snapshots cannot reconstruct full-rate gyro history.',
        'capture_complete': (capture/'summary.json').exists(), 'total_saved_frames': len(records),
        'totals': totals, 'first45_sim_seconds': first45, 'walking10_to45_sim_seconds': walking,
        'projection_rejection_reasons': reasons, 'frames': observations}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output-name', default='attitude_capture_audit.json')
    args = parser.parse_args()
    report = analyze(args.capture)
    (args.capture/args.output_name).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'frames'}, indent=2))
