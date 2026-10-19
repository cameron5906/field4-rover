"""Synthetic contract tests for detector continuation; no model accuracy claim."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from person_detector_bridge import PersonDetectorBridge
from person_evidence import make_person_evidence
from person_tracking import PersonTracker
from test_person_tracking import color, frame, person, synthetic_body, synthetic_scan_frame


def detection(**changes):
    return dict({'label': 'person', 'score': .8, 'bbox': [298., 43., 44., 153.]}, **changes)


class DetectorBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tracker = PersonTracker()
        self.bridge = PersonDetectorBridge()
        self.histogram = lambda *_: color()

    def seed(self):
        for index, t in enumerate((1., 1.4, 1.8), 1):
            f = synthetic_scan_frame(index=index, t=t, beam_count=720)
            points = synthetic_body()
            poses = [{'gesture': 'lowered', 'confidence': .95, 'points': points,
                      'observation_source': 'pose_landmarks',
                      'follow_evidence': make_person_evidence(points, f, None, self.histogram)}]
            self.tracker.update(f, poses)
            self.bridge.remember(f, poses, [detection()], self.tracker)
        self.assertIn(1, self.bridge.anchors)
        self.assertEqual(self.tracker.tracks[0].state, 'confirmed')
        return 1

    def step(self, index=4, t=2.2, detections=None, scan=None):
        f = scan or synthetic_scan_frame(index=index, t=t, beam_count=720)
        obs = self.bridge.continuations(f, [detection()] if detections is None else detections,
                                        self.tracker, None, self.histogram)
        self.tracker.update(f, obs)
        return obs

    def test_detector_alone_cannot_seed_identity(self):
        self.assertEqual(self.step(), [])
        self.assertEqual(self.tracker.tracks, [])

    def test_continues_current_measured_id_without_fake_pose_or_release(self):
        identity = self.seed()
        obs = self.step()
        self.assertEqual(len(obs), 1)
        self.assertNotIn('points', obs[0])
        self.assertEqual(obs[0]['observation_source'], 'detector_lidar_continuation')
        self.assertEqual(obs[0]['gesture'], 'unknown')
        evidence = obs[0]['follow_evidence']
        self.assertEqual(evidence['track_id'], identity)
        self.assertTrue(evidence['range_observed'])
        self.assertTrue(evidence['motion_eligible'])
        self.assertFalse(evidence['gesture_release_eligible'])
        self.assertGreaterEqual(evidence['support_rays'], 3)
        self.assertAlmostEqual(evidence['range_m'], 3.)

    def test_uncorroborated_box_retains_no_current_range(self):
        self.seed()
        scan = synthetic_scan_frame(index=4, t=2.2, beam_count=720, legs=False)
        obs = self.step(scan=scan)
        self.assertEqual(len(obs), 1)
        self.assertFalse(obs[0]['follow_evidence']['range_observed'])
        self.assertFalse(obs[0]['follow_evidence']['motion_eligible'])

    def test_one_current_ray_cannot_continue_range(self):
        self.seed()
        scan = synthetic_scan_frame(index=4, t=2.2, beam_count=720, legs=False)
        foreground = synthetic_scan_frame(beam_count=720)
        scan['scan']['ranges_m'][4] = foreground['scan']['ranges_m'][4]
        obs = self.step(scan=scan)
        self.assertFalse(obs[0]['follow_evidence']['motion_eligible'])

    def test_wall_at_prior_depth_is_not_an_isolated_person_cluster(self):
        self.seed()
        scan = synthetic_scan_frame(index=4, t=2.2, beam_count=720, background=3., legs=False)
        obs = self.step(scan=scan)
        self.assertFalse(obs[0]['follow_evidence']['range_observed'])
        self.assertFalse(obs[0]['follow_evidence']['motion_eligible'])

    def test_two_plausible_current_range_clusters_are_ambiguous(self):
        self.seed()
        scan = synthetic_scan_frame(index=4, t=2.2, beam_count=720)
        import math
        for index, distance in enumerate(scan['scan']['ranges_m']):
            if distance < 4:
                depth = 2.7 if index < 360 else 3.25
                scan['scan']['ranges_m'][index] = (depth+.1698)/math.cos(index*math.tau/720)
        obs = self.step(scan=scan)
        self.assertEqual(obs[0]['follow_evidence']['reason'], 'ambiguous_detector_range_clusters')
        self.assertFalse(obs[0]['follow_evidence']['motion_eligible'])

    def test_duplicate_boxes_are_ambiguous(self):
        self.seed()
        obs = self.step(detections=[detection(), detection()])
        self.assertEqual(len(obs), 2)
        self.assertTrue(all(p['follow_evidence']['track_state'] == 'ambiguous' for p in obs))
        self.assertTrue(all(not p['follow_evidence']['motion_eligible'] for p in obs))

    def test_one_box_cannot_choose_between_two_existing_identities(self):
        self.seed()
        other = copy.deepcopy(self.tracker.tracks[0])
        other.id = 2
        self.tracker.tracks.append(other)
        self.bridge.anchors[2] = copy.deepcopy(self.bridge.anchors[1])
        self.assertEqual(self.step(), [])

    def test_appearance_mismatch_cannot_continue(self):
        self.seed()
        self.histogram = lambda *_: color(7)
        self.assertEqual(self.step(), [])

    def test_conflicting_box_cannot_disappear_before_ambiguity_check(self):
        for reverse in (False, True):
            with self.subTest(reverse=reverse):
                self.setUp(); self.seed()
                boxes = [detection(), detection(bbox=[300., 43., 44., 153.])]
                if reverse:
                    boxes.reverse()
                self.histogram = lambda _rgb, box: color(0 if box[0] == 300 else 7)
                self.assertEqual(self.step(detections=boxes), [])
                self.assertEqual(self.tracker.tracks[0].state, 'occluded')

    def test_malformed_sensor_geometry_returns_no_observation(self):
        self.seed()
        for key, value in [('width', 0), ('pose', None), ('hfov_deg', float('nan'))]:
            f = synthetic_scan_frame(index=4, t=2.2, beam_count=720)
            f[key] = value
            self.assertEqual(self.bridge.continuations(f, [detection()], self.tracker, None, self.histogram), [])

    def test_low_confidence_or_nonperson_box_cannot_continue(self):
        for changes in ({'score': .49}, {'label': 'chair'}):
            with self.subTest(changes=changes):
                self.setUp(); self.seed()
                self.assertEqual(self.step(detections=[detection(**changes)]), [])

    def test_pose_anchor_expires_despite_current_detector_observations(self):
        self.seed()
        for index in range(4, 16):
            self.assertTrue(self.step(index, 1.+(index-1)*.4))
        self.assertEqual(self.step(16, 7.), [])

    def test_current_range_and_identity_gaps_remain_bounded(self):
        self.seed()
        self.assertEqual(self.step(4, 2.8), [])
        self.setUp(); self.seed()
        for index, t in enumerate((2.2, 2.6, 3.), 4):
            scan = synthetic_scan_frame(index=index, t=t, beam_count=720, legs=False)
            self.step(index, t, scan=scan)
        obs = self.step(7, 3.4)
        self.assertFalse(obs[0]['follow_evidence']['motion_eligible'])
        self.assertEqual(obs[0]['follow_evidence']['reason'], 'detector_prior_range_stale')

    def test_continuation_marker_can_never_create_missing_track(self):
        p = person()
        p['follow_evidence']['continuation_track_id'] = 999
        self.tracker.update(frame(), [p])
        self.assertEqual(self.tracker.tracks, [])
        self.assertEqual(p['follow_evidence']['track_state'], 'detector_continuation_unmatched')


if __name__ == '__main__':
    unittest.main()
