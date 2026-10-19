"""Capture conversion tests; these are not model or robot acceptance evidence."""
import json
from pathlib import Path
import tempfile
import unittest

from replay_rendered_capture import convert_records, read_complete_records


class CaptureReplayTests(unittest.TestCase):
    def test_partial_last_line_is_not_a_captured_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'frames.jsonl'
            path.write_text('{"frame":1}\n{"frame":')
            self.assertEqual(read_complete_records(path), [{'frame': 1}])

    def test_truth_fields_are_not_passed_to_perception(self):
        record = {'image': 'frames/1.png', 'result': {'poses': []},
                  'metadata': {'frame_id': 1, 'capture_monotonic_ms': 100,
                               'actor_position': [1, 2, 3],
                               'scan': {'timestamp_s': 1, 'actor_id': 'person'}}}
        row = convert_records([record], [], Path('capture'))[0]
        self.assertNotIn('actor_position', row['frame'])
        self.assertNotIn('actor_id', row['frame']['scan'])
        self.assertEqual(row['fixture_kind'], 'rendered')

    def test_phase_uses_capture_time_not_later_delivery(self):
        record = {'image': 'frames/1.png', 'result': {'poses': []},
                  'delivery_monotonic_ms': 250,
                  'metadata': {'frame_id': 1, 'capture_monotonic_ms': 150}}
        events = [{'type': 'explicit_follow_start', 'wall_ms': 100},
                  {'type': 'walking_enabled', 'wall_ms': 200}]
        row = convert_records([record], events, Path('capture'))[0]
        self.assertEqual(row['scenario'], 'explicit_follow_start')

    def test_sensor_profile_metadata_is_preserved_for_validation(self):
        scan = {'ray_count': 720, 'angle_increment_rad': .01,
                'field_of_view_rad': 6.28, 'scan_hz': 5, 'calibrated': False,
                'scan_model': 'instantaneous_uniform_360deg', 'scan_id': 42}
        record = {'image': 'frames/1.png', 'result': {'poses': []},
                  'metadata': {'frame_id': 1, 'capture_monotonic_ms': 100, 'scan': scan}}
        row = convert_records([record], [], Path('capture'))[0]
        self.assertEqual(row['frame']['scan'], scan)


if __name__ == '__main__':
    unittest.main()
