"""Worker glue tests with explicitly stubbed inference, no MediaPipe claim.

Compile the actual Perception class body while supplying model/image doubles;
exercise its real infer() glue with actual range/tracker/gesture/fusion modules.
This keeps core regression tests independent of optional native ML dependencies.
"""
import ast
import copy
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gesture import classify_pose
from landmarks import LandmarkMap
from person_evidence import make_person_evidence
from person_tracking import PersonTracker
from person_detector_bridge import PersonDetectorBridge
from test_person_tracking import color, synthetic_body, synthetic_scan_frame


class WorkerGlueTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((ROOT/'worker.py').read_text())
        body = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Perception')
        image = SimpleNamespace(numpy_view=lambda: SimpleNamespace(shape=(360, 640, 3)))
        namespace = {'time': time, 'math': math, 'trace_event': lambda *a, **kw: None,
                     'load_frame_image': lambda _: image,
                     'appearance': lambda *_: color(), 'classify_pose': classify_pose,
                     'make_person_evidence': make_person_evidence,
                     'PersonDetectorBridge': PersonDetectorBridge}
        exec(compile(ast.Module(body=[body], type_ignores=[]), str(ROOT/'worker.py'), 'exec'), namespace)
        self.engine = namespace['Perception'].__new__(namespace['Perception'])
        self.engine.fusion, self.engine.person_tracker, self.engine.epoch = LandmarkMap(), PersonTracker(), None
        self.engine.person_detector_bridge = PersonDetectorBridge()
        self.engine.detector = SimpleNamespace(detect=lambda _: SimpleNamespace(detections=[]))
        self.points = synthetic_body()
        self.engine.pose = SimpleNamespace(detect=lambda _: SimpleNamespace(pose_landmarks=[[SimpleNamespace(**p) for p in self.points]]))

    def test_infer_reuses_tracker_across_frames(self):
        identities = []
        for i, t in enumerate((1., 1.4, 1.8), 1):
            result = self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=i, t=t))
            e = result['poses'][0]['follow_evidence']
            identities.append(e['track_id'])
        self.assertEqual(identities, [1, 1, 1])
        self.assertTrue(e['motion_eligible'])
        self.assertEqual(result['person_tracks']['status'], 'updated')
        self.assertEqual(result['capture_monotonic_ms'], 1800)

    def test_old_epoch_and_duplicate_rejected_before_models_or_memory(self):
        f = synthetic_scan_frame(); f['epoch'] = 2
        self.engine.infer('synthetic-image-double', f)
        tracker_before = copy.deepcopy(self.engine.person_tracker.__dict__)
        map_before = copy.deepcopy(self.engine.fusion.__dict__)
        self.engine.detector.detect = lambda _: self.fail('stale frame reached model')
        for bad, reason in [(dict(f, epoch=1), 'wrong_epoch'), (f, 'out_of_order'),
                            (dict(f, frame_id=2, timestamp_s=.5), 'out_of_order')]:
            result = self.engine.infer('synthetic-image-double', bad)
            self.assertFalse(result['ok'])
            self.assertEqual(result['error'], reason)
            self.assertEqual(result['poses'], [])
            self.assertEqual(self.engine.person_tracker.__dict__, tracker_before)
            self.assertEqual(self.engine.fusion.__dict__, map_before)
            self.assertEqual(self.engine.epoch, 2)

    def test_later_epoch_restarts_confirmation(self):
        self.engine.infer('synthetic-image-double', synthetic_scan_frame())
        f = synthetic_scan_frame(); f['epoch'] = 1
        result = self.engine.infer('synthetic-image-double', f)
        self.assertTrue(result['ok'])
        e = result['poses'][0]['follow_evidence']
        self.assertEqual(e['track_epoch'], 1)
        self.assertEqual(e['track_state'], 'tentative')
        self.assertFalse(e['motion_eligible'])

    def test_invalid_timestamp_is_json_serializable_failure(self):
        import json
        f = synthetic_scan_frame(); f['timestamp_s'] = float('nan')
        result = self.engine.infer('synthetic-image-double', f)
        self.assertEqual(result['error'], 'invalid_frame')
        json.dumps(result, allow_nan=False)

    def test_raised_gesture_not_suppressed_by_tracking(self):
        self.points = synthetic_body(raised=True)
        result = self.engine.infer('synthetic-image-double', synthetic_scan_frame())
        pose = result['poses'][0]
        self.assertEqual(pose['gesture'], 'raised')
        self.assertFalse(pose['follow_evidence']['motion_eligible'])

    def test_unknown_wrists_do_not_remove_valid_person_measurement(self):
        self.points[15]['visibility'] = .2
        self.points[16]['visibility'] = .2
        for i, t in enumerate((1., 1.4, 1.8), 1):
            result = self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=i, t=t))
        pose = result['poses'][0]
        self.assertEqual(pose['gesture'], 'unknown')
        self.assertTrue(pose['follow_evidence']['motion_eligible'])
        self.assertFalse(pose['follow_evidence']['gesture_release_eligible'])

    def seed_detector_anchor(self):
        detected = SimpleNamespace(
            categories=[SimpleNamespace(category_name='person', score=.8)],
            bounding_box=SimpleNamespace(origin_x=298, origin_y=43, width=44, height=153))
        self.engine.detector.detect = lambda _: SimpleNamespace(detections=[detected])
        for i, t in enumerate((1., 1.4, 1.8), 1):
            self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=i, t=t, beam_count=720))
        self.assertIn(1, self.engine.person_detector_bridge.anchors)
        self.engine.pose.detect = lambda _: SimpleNamespace(pose_landmarks=[])

    def test_real_worker_glue_routes_detector_continuation_outside_pose_lane(self):
        self.seed_detector_anchor()
        result = self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=4, t=2.2, beam_count=720))
        self.assertEqual(result['poses'], [])
        self.assertEqual(len(result['person_observations']), 1)
        obs = result['person_observations'][0]
        self.assertEqual(obs['observation_source'], 'detector_lidar_continuation')
        self.assertNotIn('points', obs)
        self.assertEqual(obs['follow_evidence']['track_id'], 1)
        self.assertTrue(obs['follow_evidence']['motion_eligible'])
        self.assertTrue(obs['follow_evidence']['range_observed'])
        self.assertFalse(obs['follow_evidence']['gesture_release_eligible'])

    def test_skipped_pose_model_does_not_activate_detector_continuation(self):
        self.seed_detector_anchor()
        result = self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=4, t=2.2, beam_count=720), run_pose=False)
        self.assertEqual(result['poses'], [])
        self.assertEqual(result['person_observations'], [])
        self.assertEqual(result['person_tracks']['status'], 'pose_not_run')

    def test_pose_not_run_does_not_emit_cached_pose(self):
        self.engine.infer('synthetic-image-double', synthetic_scan_frame())
        result = self.engine.infer('synthetic-image-double', synthetic_scan_frame(index=2, t=1.4), run_pose=False)
        self.assertEqual(result['poses'], [])
        self.assertEqual(result['person_tracks']['status'], 'pose_not_run')


if __name__ == '__main__':
    unittest.main()
