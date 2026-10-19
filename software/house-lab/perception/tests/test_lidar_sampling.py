"""Uniform scan count/angle regression tests; synthetic geometry, no hardware claim."""
import copy
import math
import unittest
from test_person_tracking import synthetic_scan_frame, synthetic_body, color
from landmarks import project_scan
from person_evidence import make_person_evidence


class LidarSamplingTests(unittest.TestCase):
    def frame(self, count):
        frame = synthetic_scan_frame(beam_count=count)
        frame['scan'].update(ray_count=count, angle_increment_rad=2*math.pi/count,
                             field_of_view_rad=2*math.pi, scan_hz=5.,
                             profile=f'uniform_instantaneous_{count}', calibrated=False)
        return frame

    def test_counts_preserve_projection_and_person_range(self):
        for count in (180, 360, 720):
            with self.subTest(count=count):
                frame = self.frame(count)
                rays = project_scan(frame)
                self.assertTrue(rays)
                self.assertEqual(len({r['i'] for r in rays}), len(rays))
                self.assertTrue(all(0 <= r['i'] < count for r in rays))
                self.assertAlmostEqual(next(r['u'] for r in rays if r['i'] == 0), 320.)
                evidence = make_person_evidence(synthetic_body(), frame, None, lambda *_: color())
                self.assertTrue(evidence['range_valid'], evidence)
                self.assertAlmostEqual(evidence['range_m'], 3.)

    def test_metadata_mismatch_never_reinterprets_scan(self):
        for count in (180, 360, 720):
            for field, value in [('ray_count', count-1), ('ray_count', True),
                                 ('profile', 'uniform_instantaneous_90'),
                                 ('angle_increment_rad', 2*math.pi/(count*2)),
                                 ('angle_increment_rad', float('nan')),
                                 ('field_of_view_rad', math.pi),
                                 ('field_of_view_rad', 'full')]:
                with self.subTest(count=count, field=field, value=value):
                    frame = self.frame(count)
                    frame['scan'][field] = value
                    self.assertEqual(project_scan(frame), [])
                    evidence = make_person_evidence(synthetic_body(), frame, None, lambda *_: color())
                    self.assertFalse(evidence['range_valid'])

    def test_legacy_metadata_free_and_stale_scans(self):
        for count in (180, 360, 720):
            frame = synthetic_scan_frame(beam_count=count)
            self.assertTrue(project_scan(frame))
            frame['scan']['timestamp_s'] -= .121
            self.assertEqual(project_scan(frame), [])

    def test_nonfinite_and_negative_times_or_tilt_fail_closed(self):
        for count in (180, 360, 720):
            for scan_side in (False, True):
                for key in ('timestamp_s', 'roll_rad', 'pitch_rad'):
                    for value in (float('nan'), float('inf'), -1., True):
                        with self.subTest(count=count, scan=scan_side, key=key, value=value):
                            frame = self.frame(count)
                            target = frame['scan'] if scan_side else frame
                            target[key] = value
                            self.assertEqual(project_scan(frame), [])

    def test_boolean_distance_is_not_a_measured_meter(self):
        frame = self.frame(720)
        frame['scan']['ranges_m'] = [True] * 720
        self.assertEqual(project_scan(frame), [])

    def test_oversized_scan_rejected(self):
        frame = synthetic_scan_frame(beam_count=1440)
        self.assertEqual(project_scan(frame), [])

if __name__ == '__main__':
    unittest.main()
