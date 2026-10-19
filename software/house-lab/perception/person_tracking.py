"""Short-lived anonymous person association using observations and odometry only.

The IDs describe visual continuity, not a person's real identity. Prediction is
used only for association, never exported as a current range or motion command.
No simulator labels/poses, appearance classifier, network or third-party runtime.
"""
from dataclasses import dataclass, field
import math

from landmarks import camera_origin, rotate, add, sub, wrap


MAX_GAP_S = 1.8
MAX_FRAME_GAP_S = .9
CONFIRM_S = .6
LOWERED_S = .7


def _finite(values):
    return all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def _appearance_distance(a, b):
    return sum(abs(x-y) for x, y in zip(a, b)) * .5


@dataclass
class PersonTrack:
    id: int
    epoch: int
    first_seen: float
    last_seen: float
    first_match: float
    bearing_world: float
    camera: tuple
    scale: float
    appearance: list
    anchor_appearance: list
    hits: int = 1
    observations: int = 1
    state: str = 'tentative'
    range_point: tuple | None = None
    range_time: float = -1.
    range_observations: int = 0
    lowered_since: float = -1.
    lowered_count: int = 0
    last_frame: int = -1


class PersonTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.tracks = []
        self.epoch = None
        self.last_frame = -1
        self.last_time = -1.
        self.last_capture_ms = -1.
        self.next_id = 1

    def _observation(self, frame, pose):
        evidence = pose.get('follow_evidence', {})
        appearance = evidence.get('appearance', [])
        box = evidence.get('torso_box', [])
        bearing = evidence.get('bearing_rad')
        if not evidence.get('identity_valid') or len(appearance) != 24 or not _finite(appearance) or min(appearance) < 0 or not .95 <= sum(appearance) <= 1.05:
            return None
        if len(box) != 4 or not _finite(box) or min(box[2:]) <= 0 or not _finite([bearing]):
            return None
        observation = {'evidence': evidence, 'pose': pose, 'appearance': appearance,
                       'scale': box[3], 'bearing': bearing,
                       'angle': wrap(frame['pose'][2]+bearing), 'camera': camera_origin(frame)}
        if evidence.get('range_valid') and _finite([evidence.get('range_m'), evidence.get('range_lower_m')]) and 0 <= evidence['range_lower_m'] <= evidence['range_m'] <= 5.5:
            depth = evidence['range_m']
            # depth is forward camera z, not radial distance. Respect the same
            # pinhole convention and mount extrinsics as project_scan().
            local = (-depth*math.tan(bearing), -depth)
            observation['point'] = add(observation['camera'], rotate(local, frame['pose'][2]))
        return observation

    def _cost(self, track, observation, now, check_appearance=True):
        dt = now-track.last_seen
        if not 0 < dt <= MAX_GAP_S:
            return None
        ratio = observation['scale']/track.scale
        if not .68 <= ratio <= 1.48:
            return None
        app = _appearance_distance(observation['appearance'], track.appearance)
        anchor = _appearance_distance(observation['appearance'], track.anchor_appearance)
        if check_appearance and (app > .25 or anchor > .30):
            return None
        if track.range_point is not None and now-track.range_time <= MAX_GAP_S:
            delta = sub(track.range_point, observation['camera'])
            distance = math.hypot(*delta)
            expected = math.atan2(-delta[0], -delta[1])
            gate = .04 + min(.16, 1.0*dt/max(.8, distance))
            if 'point' in observation and math.dist(track.range_point, observation['point']) > .35 + 1.2*(now-track.range_time):
                return None
        else:
            expected = track.bearing_world
            # Bearing-only state can maintain continuity under small ego motion,
            # but it cannot authorize movement or manufacture a target range.
            moved = math.dist(observation['camera'], track.camera)
            if moved > .22:
                return None
            gate = .05 + min(.12, dt*.15+moved*.7)
        angle = abs(wrap(observation['angle']-expected))
        if angle > gate:
            return None
        return angle/max(gate, .001) + app

    def _withhold(self, poses, reason):
        for pose in poses:
            evidence = pose.setdefault('follow_evidence', {})
            evidence.update(range_valid=False, motion_eligible=False, track_id=None,
                            track_state=reason, track_confidence=0.,
                            gesture_release_eligible=False)
        return {'status': reason, 'tracks': []}

    def update(self, frame, poses, pose_ran=True):
        """Annotate only current poses; empty/missing frames never replay a pose.

        Wrong-epoch, duplicate and inverted frames do not mutate stored identity.
        Capture age at delivery remains the controller's job (different clocks).
        """
        if not isinstance(frame, dict) or not isinstance(frame.get('frame_id'), int) or isinstance(frame['frame_id'], bool) or not isinstance(frame.get('epoch', 0), int) or not _finite([frame.get('timestamp_s'), frame.get('capture_monotonic_ms')]) or frame['capture_monotonic_ms'] < 0:
            return self._withhold(poses, 'invalid_frame')
        pose = frame.get('pose', [])
        mount = frame.get('camera_mount', [0, .133, -.1078])
        if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not _finite(pose) or not isinstance(mount, (list, tuple)) or len(mount) != 3 or not _finite(mount):
            return self._withhold(poses, 'invalid_geometry')
        epoch, now = frame.get('epoch', 0), frame['timestamp_s']
        if self.epoch is not None and epoch < self.epoch:
            return self._withhold(poses, 'wrong_epoch')
        if self.epoch is None or epoch > self.epoch:
            self.reset()
            self.epoch = epoch
        if frame['frame_id'] <= self.last_frame or now <= self.last_time or frame['capture_monotonic_ms'] <= self.last_capture_ms:
            return self._withhold(poses, 'out_of_order')
        self.last_frame, self.last_time = frame['frame_id'], now
        self.last_capture_ms = frame['capture_monotonic_ms']
        self.tracks = [t for t in self.tracks if now-t.last_seen <= MAX_GAP_S]
        if not pose_ran:
            # A scheduler skip supplies no clear-pose evidence. Preserve only
            # bounded identity memory, never a release/confirmation streak.
            for track in self.tracks:
                track.state = 'occluded'
                track.hits, track.lowered_count, track.lowered_since = 0, 0, -1.
            return self._withhold(poses, 'pose_not_run')
        observations = [self._observation(frame, p) for p in poses]
        edges = {}
        geometry_edges = {}
        for index, observation in enumerate(observations):
            if observation:
                edges[index] = [(t, self._cost(t, observation, now)) for t in self.tracks]
                edges[index] = [(t, cost) for t, cost in edges[index] if cost is not None]
                continuation_id = observation['evidence'].get('continuation_track_id')
                if continuation_id is not None:
                    edges[index] = [(t, cost) for t, cost in edges[index] if t.id == continuation_id]
                geometry_edges[index] = [t for t in self.tracks
                                         if self._cost(t, observation, now, check_appearance=False) is not None]
        # Resolve all association conflicts against the pre-frame state. A
        # matching pose and an appearance outlier at its plausible location
        # must stop together, regardless of the detector's output order.
        conflicted = {t.id for index, choices in geometry_edges.items()
                      if not edges[index] for t in choices}
        # A many-to-one or one-to-many plausible match is unsafe regardless of
        # greedy ordering. Stop all involved candidates; never silently swap IDs.
        track_counts = {}
        for choices in edges.values():
            for track, _ in choices:
                track_counts[track.id] = track_counts.get(track.id, 0)+1
        matched = set()
        for index, observation in enumerate(observations):
            if observation is None:
                self._withhold([poses[index]], 'unusable_observation')
                continue
            evidence, item = observation['evidence'], observation['pose']
            raw_range_valid = bool(evidence.get('range_valid') and 'point' in observation)
            choices = edges.get(index, [])
            ambiguous = len(choices) > 1 or any(track_counts[t.id] > 1 or t.id in conflicted
                                              for t, _ in choices)
            if ambiguous:
                self._withhold([item], 'ambiguous')
                continue
            if not choices:
                if evidence.get('continuation_track_id') is not None:
                    self._withhold([item], 'detector_continuation_unmatched')
                    continue
                # A contradictory appearance at an existing person's plausible
                # location is an unresolved identity conflict, not proof that a
                # second person was born. Do not create a duplicate track from
                # one bad pose crop and then confuse future association with it.
                nearby = geometry_edges.get(index, [])
                if nearby:
                    self._withhold([item], 'identity_conflict')
                    continue
            if choices:
                track = choices[0][0]
                dt = now-track.last_seen
                if track.state in ('occluded', 'ambiguous') or dt > MAX_FRAME_GAP_S:
                    track.hits, track.first_match = 0, now
                    track.lowered_since, track.lowered_count = -1., 0
                track.hits += 1
                track.observations += 1
            else:
                track = PersonTrack(self.next_id, epoch, now, now, now, observation['angle'], observation['camera'], observation['scale'], observation['appearance'][:], observation['appearance'][:])
                self.next_id += 1
                self.tracks.append(track)
            matched.add(track.id)
            track.last_seen, track.last_frame = now, frame['frame_id']
            track.bearing_world, track.camera, track.scale = observation['angle'], observation['camera'], observation['scale']
            track.appearance = observation['appearance'][:]
            if raw_range_valid:
                track.range_point, track.range_time = observation['point'], now
                track.range_observations += 1
            lowered = item.get('gesture') == 'lowered' and _finite([item.get('confidence')]) and item['confidence'] >= .55
            if lowered:
                if track.lowered_since < 0:
                    track.lowered_since = now
                track.lowered_count += 1
            else:
                track.lowered_since, track.lowered_count = -1., 0
            confirmed = track.hits >= 3 and now-track.first_match >= CONFIRM_S
            track.state = 'confirmed' if confirmed else ('reacquiring' if track.observations > track.hits else 'tentative')
            release = lowered and track.lowered_count >= 3 and now-track.lowered_since >= LOWERED_S
            range_valid = raw_range_valid and confirmed and track.range_observations >= 2
            confidence = min(.95, .25+.12*track.hits) if confirmed else min(.59, .25+.12*track.hits)
            evidence.update(track_id=track.id, track_epoch=epoch, track_state=track.state,
                            track_confidence=round(confidence, 3), track_observations=track.observations,
                            observation_frame_id=frame['frame_id'], observation_timestamp_s=now,
                            range_observed=raw_range_valid, range_valid=range_valid,
                            # This is current person-measurement readiness, not
                            # a drive command or permission to clear a stop.
                            # Ordinary following does not need lowered-hand
                            # proof until a positive raised gesture has latched
                            # the controller's persistent stop. Its release
                            # consumes the separate fresh evidence below.
                            motion_eligible=bool(range_valid and evidence.get('valid')
                                                 and item.get('gesture') != 'raised'),
                            gesture_release_eligible=bool(confirmed and release),
                            lowered_fresh_count=track.lowered_count,
                            lowered_fresh_span_s=max(0., now-track.lowered_since) if lowered else 0.)
        for track in self.tracks:
            if track.id not in matched:
                track.state = 'ambiguous' if track_counts.get(track.id, 0) or track.id in conflicted else 'occluded'
                track.hits, track.lowered_count, track.lowered_since = 0, 0, -1.
        return {'status': 'updated', 'tracks': [
            {'track_id': t.id, 'epoch': t.epoch, 'state': t.state,
             'last_frame_id': t.last_frame, 'age_s': now-t.last_seen,
             'observations': t.observations} for t in self.tracks]}
