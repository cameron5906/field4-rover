"""Bounded person-box continuation of an already pose-confirmed anonymous ID.

This does not invent keypoints or infer a gesture. Every continuing range uses
current measured lidar returns; prior range only gates association. A detector
box alone cannot create an ID, acquire a target, or release a hands-up halt.
"""
import math
import statistics

from landmarks import camera_origin, intrinsics, project_scan, rotate, sub
from person_evidence import _geometry_ok


MAX_POSE_ANCHOR_S = 5.0
MAX_CONTINUITY_GAP_S = .9
MIN_DETECTOR_SCORE = .5


def _finite(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in values)


def _box(detection, frame):
    if not isinstance(detection, dict):
        return None
    box = detection.get('bbox', [])
    score = detection.get('score')
    if detection.get('label') != 'person' or not _finite([score]) or score < MIN_DETECTOR_SCORE:
        return None
    if not isinstance(box, (list, tuple)) or len(box) != 4 or not _finite(box):
        return None
    x, y, w, h = box
    if min(w, h) < 12 or h < 45 or not 0 <= x < x+w <= frame['width'] or not 0 <= y < y+h <= frame['height']:
        return None
    return list(box)


def _iou(a, b):
    overlap = max(0, min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0])) * max(0, min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))
    return overlap / max(1., a[2]*a[3]+b[2]*b[3]-overlap)


class PersonDetectorBridge:
    def __init__(self):
        self.anchors = {}

    def remember(self, frame, poses, detections, tracker):
        """Bind only unambiguous co-observed boxes to strong confirmed poses."""
        if not _geometry_ok(frame):
            return
        existing = {track.id for track in tracker.tracks}
        self.anchors = {key: value for key, value in self.anchors.items()
                        if key in existing and value['epoch'] == frame.get('epoch', 0)
                        and 0 <= frame['timestamp_s']-value['time'] <= MAX_POSE_ANCHOR_S}
        boxes = [(index, _box(detection, frame)) for index, detection in enumerate(detections)]
        boxes = [(index, box) for index, box in boxes if box is not None]
        choices = []
        for pose in poses:
            evidence = pose.get('follow_evidence', {})
            points = pose.get('points', [])
            if evidence.get('track_state') != 'confirmed' or not evidence.get('range_valid') or len(points) < 29:
                continue
            torso = evidence.get('torso_box', [])
            if len(torso) != 4 or not _finite(torso):
                continue
            visible = [points[i] for i in (0, 11, 12, 23, 24, 25, 26, 27, 28)]
            if not all(_finite([p.get('x'), p.get('y')]) for p in visible):
                continue
            lo_x, hi_x = min(p['x'] for p in visible)*frame['width'], max(p['x'] for p in visible)*frame['width']
            lo_y, hi_y = min(p['y'] for p in visible)*frame['height'], max(p['y'] for p in visible)*frame['height']
            body = [lo_x, lo_y, hi_x-lo_x, hi_y-lo_y]
            candidates = [(index, box) for index, box in boxes if _iou(body, box) >= .3]
            if len(candidates) == 1:
                choices.append((pose, candidates[0]))
        for pose, (index, box) in choices:
            if sum(other[1][0] == index for other in choices) != 1:
                continue
            evidence = pose['follow_evidence']
            x, y, w, h = box
            tx, ty, tw, th = evidence['torso_box']
            relative = [(tx*frame['width']-x)/w, (ty*frame['height']-y)/h,
                        tw*frame['width']/w, th*frame['height']/h]
            if min(relative[:2]) < 0 or min(relative[2:]) <= 0 or relative[0]+relative[2] > 1 or relative[1]+relative[3] > 1:
                continue
            self.anchors[evidence['track_id']] = {
                'epoch': frame.get('epoch', 0), 'time': frame['timestamp_s'],
                'frame_id': frame['frame_id'], 'relative_torso': relative,
            }
            evidence.update(detector_anchor_valid=True, detector_anchor_frame_id=frame['frame_id'])

    def _evidence(self, frame, detection, box, anchor, track, rgb, histogram):
        x, y, w, h = box
        tx, ty, tw, th = anchor['relative_torso']
        torso = [x+tx*w, y+ty*h, tw*w, th*h]
        appearance = histogram(rgb, [int(torso[0]), int(torso[1]), max(1, int(torso[2])), max(1, int(torso[3]))])
        f, cx, _ = intrinsics(frame)
        center_x = torso[0]+torso[2]*.5
        evidence = {
            'valid': True, 'identity_valid': True, 'range_valid': False,
            'reason': 'detector_current_range_uncertain',
            'appearance': appearance,
            'torso_box': [torso[0]/frame['width'], torso[1]/frame['height'], torso[2]/frame['width'], torso[3]/frame['height']],
            'bearing_rad': -math.atan2(center_x-cx, f),
            'continuation_track_id': track.id,
            'observation_source': 'detector_lidar_continuation',
            'detection_bbox': box, 'detection_score': detection['score'],
            'pose_anchor_frame_id': anchor['frame_id'],
            'pose_anchor_age_s': frame['timestamp_s']-anchor['time'],
        }
        elapsed = frame['timestamp_s']-track.range_time
        if track.range_point is None or not 0 < elapsed <= MAX_CONTINUITY_GAP_S:
            evidence['reason'] = 'detector_prior_range_stale'
            return evidence
        prior_local = rotate(sub(track.range_point, camera_origin(frame)), -frame['pose'][2])
        prior_depth = -prior_local[1]
        if prior_depth <= 0:
            return evidence
        rays = project_scan(frame, include_free_limit=True)
        # The full detector box includes swinging arms and background. Use the
        # torso corridor learned from the genuine co-observed pose, transformed
        # by this frame's measured box, for both support and side isolation.
        # These are association gates, not synthesized leg landmarks.
        low, high = torso[0]-3., torso[0]+torso[2]+3.
        eligible = [ray for ray in rays if not ray['free_limit'] and low <= ray['u'] <= high
                    and y+.55*h <= ray['v'] <= y+h+5]
        bands = []
        for ray in sorted(eligible, key=lambda ray: ray['depth']):
            if not bands or ray['depth']-bands[-1][0]['depth'] > .38:
                bands.append([])
            bands[-1].append(ray)
        margin = max(20., (high-low)*.7)
        sides = [[ray['depth'] for ray in rays if low-margin <= ray['u'] < low],
                 [ray['depth'] for ray in rays if high < ray['u'] <= high+margin]]
        dmin, dmax = 1.20*f/h, 1.95*f/h
        allowed_change = .20+1.2*elapsed
        accepted = []
        for band in bands:
            if len({ray['i'] for ray in band}) < 3:
                continue
            depth = statistics.median(ray['depth'] for ray in band)
            if not max(.8, dmin) <= depth <= min(5.5, dmax) or abs(depth-prior_depth) > allowed_change:
                continue
            if not all(side and statistics.median(side) >= depth+.35 for side in sides):
                continue
            accepted.append((depth, band))
        if len(accepted) != 1:
            if len(accepted) > 1:
                evidence['reason'] = 'ambiguous_detector_range_clusters'
            return evidence
        depth, cluster = accepted[0]
        uncertainty = .35+.08*depth
        evidence.update(range_valid=True, range_m=depth,
                        range_lower_m=max(0., min(depth-uncertainty, dmin)),
                        range_uncertainty_m=uncertainty, support_rays=len(cluster),
                        range_source='current_lidar_foreground_with_pose_anchored_person_detector',
                        reason='detector_and_current_foreground_cluster_agree')
        return evidence

    def continuations(self, frame, detections, tracker, rgb, histogram):
        """Return explicit non-pose observations; never new tracks or gestures."""
        if not _geometry_ok(frame) or frame.get('epoch', 0) != tracker.epoch:
            return []
        candidates = []
        conflicted = set()
        now = frame['timestamp_s']
        for detection in detections:
            box = _box(detection, frame)
            if box is None:
                continue
            matches = []
            nearby_ids = set()
            for track in tracker.tracks:
                anchor = self.anchors.get(track.id)
                if not anchor or anchor['epoch'] != tracker.epoch or not 0 < now-anchor['time'] <= MAX_POSE_ANCHOR_S or not 0 < now-track.last_seen <= MAX_CONTINUITY_GAP_S:
                    continue
                evidence = self._evidence(frame, detection, box, anchor, track, rgb, histogram)
                tx, ty, tw, th = evidence['torso_box']
                observation = {'gesture': 'unknown', 'confidence': 0.,
                               'center': [tx+tw*.5, ty+th*.5], 'scale': th,
                               'observation_source': 'detector_lidar_continuation',
                               'follow_evidence': evidence}
                candidate = tracker._observation(frame, observation)
                if candidate is not None:
                    if tracker._cost(track, candidate, now, check_appearance=False) is not None:
                        nearby_ids.add(track.id)
                    if tracker._cost(track, candidate, now) is not None:
                        matches.append(observation)
            # A box that could belong to two remembered people supplies no ID.
            # Multiple boxes for one ID are later rejected by the tracker.
            if len(matches) == 1:
                candidates.append(matches[0])
            elif nearby_ids:
                # A contradictory crop or a box that could belong to multiple
                # existing IDs cannot disappear before the ambiguity check.
                # Resolve the entire frame before accepting any other box.
                conflicted.update(nearby_ids)
        return [candidate for candidate in candidates
                if candidate['follow_evidence']['continuation_track_id'] not in conflicted]
