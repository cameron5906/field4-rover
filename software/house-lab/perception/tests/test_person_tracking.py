"""Synthetic regression fixtures, not captured MediaPipe/robot performance data.

Run: python -m unittest discover -s perception/tests -v
No API calls, models, keys, network or third-party packages are used.
"""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from landmarks import project_scan
from person_evidence import make_person_evidence
from person_tracking import PersonTracker


def color(bin_id=0):
    result = [0.]*24
    for channel in range(3):
        result[8*channel+bin_id] = 1/3
    return result


def frame(index=1, t=1., epoch=0, pose=None, mount=None):
    return {'frame_id': index, 'epoch': epoch, 'timestamp_s': t,
            'capture_monotonic_ms': int(t*1000), 'width': 640, 'height': 360,
            'hfov_deg': 102, 'pose': pose or [0., 0., 0.],
            'camera_mount': mount or [0., .133, -.1078]}


def synthetic_body(depth=3., raised=False, lateral=0.):
    f = 320/math.tan(math.radians(51))
    p = [{'x': .5, 'y': .3, 'visibility': .95, 'presence': .95} for _ in range(33)]
    coords = {0:(0., 1.65), 11:(-.22, 1.4), 12:(.22, 1.4),
              23:(-.14, .87), 24:(.14, .87), 25:(-.13, .45),
              26:(.13, .45), 27:(-.12, .02), 28:(.12, .02),
              13:(-.25, 1.0), 14:(.25, 1.0),
              15:(-.25, 1.8 if raised else .8), 16:(.25, 1.8 if raised else .8)}
    for index, (x, y) in coords.items():
        p[index]['x'] = (.5*640+f*(x+lateral)/depth)/640
        p[index]['y'] = (.5*360-f*(y-.133)/depth)/360
    return p


def synthetic_scan_frame(depth=3., background=5.6, legs=True, index=1, t=1., beam_count=360, lateral=0.):
    result = frame(index, t)
    ranges = []
    for i in range(beam_count):
        angle = i*2*math.pi/beam_count
        c = math.cos(angle)
        if c <= 0:
            ranges.append(6.)
            continue
        leg_r = (depth+.1698)/c
        x = -leg_r*math.sin(angle)
        is_leg = legs and (.08 <= abs(x-lateral) <= .18)
        distance = leg_r if is_leg else (background+.1698)/c
        ranges.append(min(6., distance))
    result['scan'] = {'timestamp_s': t, 'pose': result['pose'][:],
                      'mount': [0., .164, .062], 'ranges_m': ranges}
    return result


def person(bearing=0., depth=3., bin_id=0, gesture='lowered', range_valid=True):
    return {'gesture': gesture, 'confidence': .95,
            'follow_evidence': {'valid': True, 'identity_valid': True,
                'appearance': color(bin_id), 'torso_box': [.45, .2, .1, .15],
                'bearing_rad': bearing, 'range_valid': range_valid,
                'range_m': depth, 'range_lower_m': depth-.4}}


class RangeTests(unittest.TestCase):
    def evidence(self, scan=None, points=None):
        return make_person_evidence(points or synthetic_body(), scan or synthetic_scan_frame(), None, lambda *_: color())

    def test_foreground_legs_and_background_gap(self):
        scan = synthetic_scan_frame()
        # Confirm this explicitly contains the old failure: same body interval
        # contains both 3m legs and 5.6m wall through the leg gap.
        rays = [r['depth'] for r in project_scan(scan) if 300 < r['u'] < 340]
        self.assertGreater(max(rays)-min(rays), .38)
        e = self.evidence(scan)
        self.assertTrue(e['range_valid'], e)
        self.assertAlmostEqual(e['range_m'], 3.)
        self.assertGreaterEqual(e['support_rays'], 2)
        self.assertLess(e['range_lower_m'], e['range_m'])

    def test_walking_legs_at_different_depths_are_clustered(self):
        scan = synthetic_scan_frame()
        for i in (2, 3, 357, 358):
            depth = 3. + (.14 if i < 180 else -.14)
            scan['scan']['ranges_m'][i] = (depth+.1698)/math.cos(i*2*math.pi/360)
        e = self.evidence(scan)
        self.assertTrue(e['range_valid'], e)
        self.assertAlmostEqual(e['range_m'], 3.)

    def test_production_180_beams_can_corroborate_two_legs(self):
        self.assertTrue(self.evidence(synthetic_scan_frame(beam_count=180))['range_valid'])

    def test_coherent_720_ray_profile_preserves_actual_leg_support(self):
        scan = synthetic_scan_frame(beam_count=720, lateral=.04)
        scan['scan'].update(ray_count=720, angle_increment_rad=math.tau/720,
                            field_of_view_rad=math.tau, scan_hz=5.,
                            calibrated=False, profile='uniform_instantaneous_720')
        e = self.evidence(scan, synthetic_body(lateral=.04))
        self.assertTrue(e['range_valid'], e)
        self.assertAlmostEqual(e['range_m'], 3.)

    def test_incoherent_lidar_profile_metadata_never_authorizes_range(self):
        for key, value in [('ray_count', 720), ('angle_increment_rad', math.tau/720),
                           ('field_of_view_rad', math.pi)]:
            with self.subTest(key=key):
                scan = synthetic_scan_frame(beam_count=180)
                scan['scan'][key] = value
                self.assertFalse(self.evidence(scan)['range_valid'])

    def test_sparse_beam_miss_does_not_invent_second_leg(self):
        # At the production 2-degree angular spacing, a small lateral shift
        # makes one leg fall between beams. RGB visibility does not fill in the
        # missing lidar evidence, even when a denser synthetic scan would hit.
        points = synthetic_body(lateral=.04)
        sparse = synthetic_scan_frame(beam_count=180, lateral=.04)
        dense = synthetic_scan_frame(beam_count=360, lateral=.04)
        self.assertFalse(self.evidence(sparse, points)['range_valid'])
        self.assertTrue(self.evidence(dense, points)['range_valid'])

    def test_overlapping_legs_do_not_double_count_one_leg(self):
        p = synthetic_body()
        for left, right in ((23,24), (25,26), (27,28)):
            p[right]['x'] = p[left]['x']
        self.assertFalse(self.evidence(points=p)['range_valid'])

    def test_malformed_scan_is_rejected(self):
        for key, value in [('pose', None), ('mount', []), ('ranges_m', None)]:
            scan = synthetic_scan_frame(); scan['scan'][key] = value
            self.assertFalse(self.evidence(scan)['range_valid'])

    def test_wall_is_not_person_range(self):
        self.assertFalse(self.evidence(synthetic_scan_frame(legs=False))['range_valid'])

    def test_no_return_only_is_not_person_range(self):
        scan = synthetic_scan_frame()
        scan['scan']['ranges_m'] = [6.]*360
        self.assertFalse(self.evidence(scan)['range_valid'])

    def test_single_ray_and_one_leg_are_not_a_person(self):
        for keep in ([2], [2, 3]):
            scan = synthetic_scan_frame(legs=False)
            foreground = synthetic_scan_frame()['scan']['ranges_m']
            for i in keep:
                scan['scan']['ranges_m'][i] = foreground[i]
            self.assertFalse(self.evidence(scan)['range_valid'])

    def test_stale_scan_and_tilt_reject(self):
        for update in ('time', 'pitch'):
            scan = synthetic_scan_frame()
            if update == 'time': scan['scan']['timestamp_s'] -= .13
            else: scan['pitch_rad'] = .03
            self.assertFalse(self.evidence(scan)['range_valid'])

    def test_occluded_ankle_or_knee_rejects_current_range(self):
        for index in (27, 25):
            points = synthetic_body()
            points[index]['visibility'] = .1
            self.assertFalse(self.evidence(points=points)['range_valid'])

    def test_wrong_extrinsics_do_not_match_legs(self):
        scan = synthetic_scan_frame()
        scan['camera_mount'][0] = .6
        self.assertFalse(self.evidence(scan)['range_valid'])

    def test_scan_time_odometry_compensation(self):
        scan = synthetic_scan_frame()
        # Same endpoints represented in a translated scan-time coordinate system.
        scan['scan']['pose'][0] = .3
        scan['scan']['mount'][0] = -.3
        e = self.evidence(scan)
        self.assertTrue(e['range_valid'], e)
        self.assertAlmostEqual(e['range_m'], 3.)

    def test_invalid_input_rejected_not_crashed(self):
        for key, value in [('pose', [0]), ('camera_mount', [0, float('nan'), 0]), ('width', 0), ('hfov_deg', float('inf'))]:
            scan = synthetic_scan_frame(); scan[key] = value
            self.assertFalse(self.evidence(scan)['identity_valid'])
        p = synthetic_body(); p[11]['x'] = float('nan')
        self.assertFalse(self.evidence(points=p)['identity_valid'])

    def test_stationary_raised_has_same_range_observation(self):
        self.assertTrue(self.evidence(points=synthetic_body(raised=True))['range_valid'])


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.tracker = PersonTracker()

    def step(self, i, t, items=None, **kw):
        items = items if items is not None else [person()]
        result = self.tracker.update(frame(i, t, **kw), items)
        return items, result

    def confirmed(self):
        for i, t in enumerate((1., 1.4, 1.8), 1):
            items, _ = self.step(i, t)
        self.assertTrue(items[0]['follow_evidence']['motion_eligible'])
        return items[0]['follow_evidence']['track_id']

    def test_single_frame_is_not_a_track_ready_to_follow(self):
        p, _ = self.step(1, 1.)
        self.assertEqual(p[0]['follow_evidence']['track_state'], 'tentative')
        self.assertFalse(p[0]['follow_evidence']['range_valid'])
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_three_fast_frames_do_not_bypass_time_confirmation(self):
        for i in range(1, 5):
            p, _ = self.step(i, 1+i*.01)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_persistent_identity_confirmed_after_fresh_interval(self):
        self.assertEqual(self.confirmed(), 1)

    def test_occlusion_stops_and_requires_reconfirmation_same_id(self):
        identity = self.confirmed()
        p, summary = self.step(4, 2.2, [])
        self.assertEqual(p, [])
        self.assertEqual(summary['tracks'][0]['state'], 'occluded')
        for i, t in enumerate((2.6, 3., 3.4), 5):
            p, _ = self.step(i, t)
            e = p[0]['follow_evidence']
            self.assertEqual(e['track_id'], identity)
            self.assertEqual(e['motion_eligible'], i == 7)

    def test_long_gap_expires_id_does_not_rebind(self):
        identity = self.confirmed()
        p, _ = self.step(4, 4.)
        self.assertNotEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_ambiguous_distractors_cannot_steal_identity(self):
        identity = self.confirmed()
        p, summary = self.step(4, 2.2, [person(-.025), person(.025)])
        self.assertTrue(all(x['follow_evidence']['track_state'] == 'ambiguous' for x in p))
        self.assertTrue(all(not x['follow_evidence']['motion_eligible'] for x in p))
        self.assertEqual(summary['tracks'][0]['track_id'], identity)
        p, _ = self.step(5, 2.6)
        self.assertEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_conflicting_appearance_does_not_receive_identity(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(bin_id=7)])
        self.assertIsNone(p[0]['follow_evidence']['track_id'])
        self.assertEqual(self.tracker.tracks[0].id, identity)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_matching_and_conflicting_poses_stop_in_both_orders(self):
        for reverse in (False, True):
            with self.subTest(reverse=reverse):
                self.tracker = PersonTracker()
                identity = self.confirmed()
                items = [person(), person(bin_id=7)]
                if reverse:
                    items.reverse()
                p, summary = self.step(4, 2.2, items)
                self.assertEqual({x['follow_evidence']['track_state'] for x in p},
                                 {'ambiguous', 'identity_conflict'})
                self.assertTrue(all(not x['follow_evidence']['motion_eligible'] for x in p))
                self.assertTrue(all(not x['follow_evidence']['gesture_release_eligible'] for x in p))
                self.assertEqual(len(self.tracker.tracks), 1)
                self.assertEqual(summary['tracks'][0]['state'], 'ambiguous')
                self.assertEqual(self.tracker.tracks[0].last_seen, 1.8)
                for i, t in enumerate((2.6, 3., 3.4), 5):
                    p, _ = self.step(i, t)
                    self.assertEqual(p[0]['follow_evidence']['track_id'], identity)
                    self.assertEqual(p[0]['follow_evidence']['motion_eligible'], i == 7)

    def test_conflict_does_not_refresh_identity_memory(self):
        identity = self.confirmed()
        for i, t in enumerate((2.2, 2.6, 3., 3.4), 4):
            p, _ = self.step(i, t, [person(bin_id=7)])
            self.assertEqual(p[0]['follow_evidence']['track_state'], 'identity_conflict')
            self.assertEqual(self.tracker.tracks[0].last_seen, 1.8)
        p, _ = self.step(8, 3.8, [person(bin_id=7)])
        self.assertNotEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertEqual(p[0]['follow_evidence']['track_state'], 'tentative')
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_appearance_outlier_cannot_create_competing_track(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(bin_id=7)])
        self.assertEqual(p[0]['follow_evidence']['track_state'], 'identity_conflict')
        self.assertEqual(len(self.tracker.tracks), 1)
        for i, t in enumerate((2.6, 3., 3.4), 5):
            p, _ = self.step(i, t)
            self.assertEqual(p[0]['follow_evidence']['track_id'], identity)
            self.assertEqual(p[0]['follow_evidence']['motion_eligible'], i == 7)

    def test_spatially_separate_person_can_have_distinct_track(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(bearing=.5, bin_id=7)])
        self.assertNotEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertIsNotNone(p[0]['follow_evidence']['track_id'])
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_out_of_order_duplicate_time_and_old_epoch_do_not_mutate(self):
        self.confirmed()
        before = copy.deepcopy(self.tracker.__dict__)
        for f in (frame(2, 1.4), frame(4, 1.8), frame(4, 2.2, epoch=-1)):
            p = [person()]
            self.tracker.update(f, p)
            self.assertFalse(p[0]['follow_evidence']['motion_eligible'])
            self.assertEqual(self.tracker.__dict__, before)

    def test_capture_clock_reversal_rejected(self):
        self.confirmed(); before = copy.deepcopy(self.tracker.__dict__)
        f = frame(4, 2.2); f['capture_monotonic_ms'] = 1000
        p = [person()]; self.tracker.update(f, p)
        self.assertEqual(p[0]['follow_evidence']['track_state'], 'out_of_order')
        self.assertEqual(self.tracker.__dict__, before)

    def test_new_epoch_resets_track_confirmation(self):
        self.confirmed()
        p, _ = self.step(1, 1., epoch=1)
        self.assertEqual(p[0]['follow_evidence']['track_epoch'], 1)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_pose_and_range_missing_never_replay_last_range(self):
        self.confirmed()
        p, _ = self.step(4, 2.2, [person(range_valid=False)])
        self.assertFalse(p[0]['follow_evidence']['range_valid'])
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_wall_range_jump_is_new_identity_and_stopped(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(depth=5.6)])
        # Invalid >5.5m observations never gain usable range.
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])
        p, _ = self.step(5, 2.6, [person(depth=5.3)])
        self.assertNotEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])

    def test_raised_blocks_measurement_and_release_needs_fresh_lowered(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(gesture='raised')])
        self.assertEqual(p[0]['gesture'], 'raised')
        self.assertEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])
        for i, t in enumerate((2.6, 3., 3.4), 5):
            p, _ = self.step(i, t)
            # The independent controller latch must continue holding until
            # release is eligible, even though current range is already valid.
            self.assertTrue(p[0]['follow_evidence']['motion_eligible'])
            self.assertEqual(p[0]['follow_evidence']['gesture_release_eligible'], i == 7)

    def test_unknown_hands_can_have_fresh_range_but_cannot_release_halt(self):
        for i, t in enumerate((1., 1.4, 1.8), 1):
            p, _ = self.step(i, t, [person(gesture='unknown')])
        self.assertTrue(p[0]['follow_evidence']['motion_eligible'])
        self.assertFalse(p[0]['follow_evidence']['gesture_release_eligible'])
        self.step(4, 2.2, [person(gesture='raised')])
        p, _ = self.step(5, 2.6, [person(gesture='unknown')])
        self.assertTrue(p[0]['follow_evidence']['motion_eligible'])
        self.assertFalse(p[0]['follow_evidence']['gesture_release_eligible'])
        self.assertEqual(p[0]['follow_evidence']['lowered_fresh_count'], 0)

    def test_unknown_pose_breaks_lowered_release_confirmation(self):
        self.confirmed()
        self.step(4, 2.2, [person(gesture='raised')])
        self.step(5, 2.6)
        self.step(6, 3., [person(gesture='unknown')])
        p, _ = self.step(7, 3.4)
        self.assertFalse(p[0]['follow_evidence']['gesture_release_eligible'])

    def test_scheduler_skip_breaks_fresh_lowered_confirmation(self):
        self.confirmed()
        self.tracker.update(frame(4, 2.2), [], pose_ran=False)
        p, _ = self.step(5, 2.6)
        self.assertFalse(p[0]['follow_evidence']['motion_eligible'])
        self.assertFalse(p[0]['follow_evidence']['gesture_release_eligible'])

    def test_ego_yaw_uses_odometry_not_image_center_only(self):
        identity = self.confirmed()
        p, _ = self.step(4, 2.2, [person(bearing=-.2)], pose=[0., 0., .2])
        self.assertEqual(p[0]['follow_evidence']['track_id'], identity)
        self.assertTrue(p[0]['follow_evidence']['motion_eligible'])

    def test_ego_translation_and_camera_mount_compensated(self):
        identity = self.confirmed()
        bearing = math.atan2(.15, 3.)
        p, _ = self.step(4, 2.2, [person(bearing=bearing)], pose=[.15, 0., 0.])
        self.assertEqual(p[0]['follow_evidence']['track_id'], identity)

    def test_untrusted_truth_fields_do_not_affect_output(self):
        a, b = PersonTracker(), PersonTracker()
        f = frame(); p = [person()]
        r1 = a.update(copy.deepcopy(f), copy.deepcopy(p))
        f.update(actor_position=[999, 0, 999], actor_id='somebody', truth_pose=[33, 4, 1])
        r2 = b.update(f, copy.deepcopy(p))
        self.assertEqual(r1, r2)
        self.assertEqual(a.__dict__, b.__dict__)


if __name__ == '__main__':
    unittest.main()
