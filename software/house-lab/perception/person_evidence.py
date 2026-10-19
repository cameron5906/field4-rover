"""RGB person observations corroborated by synchronized, projected lidar.

No actor state, simulator identity or hidden depth. These are uncertain surface
observations, not obstacle clearance and not yet persistent tracks. The broad
adult-size band is a simulation assumption, not a hardware calibration.
"""
import math
import statistics
from landmarks import project_scan


def _finite(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def _point_ok(point, threshold=.55):
    return isinstance(point, dict) and _finite([point.get(k) for k in ('x', 'y', 'visibility', 'presence')]) and min(point['visibility'], point['presence']) >= threshold and .025 < point['x'] < .975 and .025 < point['y'] < .975


def _geometry_ok(frame):
    if not isinstance(frame, dict) or not _finite([frame.get(k) for k in ('width', 'height', 'timestamp_s')]):
        return False
    if min(frame['width'], frame['height']) < 16 or not _finite([frame.get('hfov_deg', 102)]) or not 20 < frame.get('hfov_deg', 102) < 150:
        return False
    for key, default in [('pose', []), ('camera_mount', [0, .133, -.1078])]:
        value = frame.get(key, default)
        if not isinstance(value, (list, tuple)) or len(value) != 3 or not _finite(value):
            return False
    scan = frame.get('scan', {})
    if not isinstance(scan, dict):
        return False
    scan_pose = scan.get('pose', [0., 0., 0.])
    if not isinstance(scan_pose, (list, tuple)) or len(scan_pose) != 3 or not _finite(scan_pose):
        return False
    if not isinstance(scan.get('ranges_m', []), (list, tuple)):
        return False
    mount = scan.get('mount', [0, .164, .062])
    return isinstance(mount, (list, tuple)) and len(mount) == 3 and _finite(mount) and _finite([frame.get('roll_rad', 0), frame.get('pitch_rad', 0), scan.get('roll_rad', 0), scan.get('pitch_rad', 0), scan.get('timestamp_s', -100)])


def _leg_x(points, hip, knee, ankle, y):
    """Image-space leg centre at the projected scan row; never infer depth."""
    chain = [points[hip], points[knee], points[ankle]]
    if not all(_point_ok(p) for p in chain):
        return None
    # Reject inverted/ambiguous leg geometry instead of extrapolating a pose.
    if not chain[0]['y'] < chain[1]['y'] < chain[2]['y']:
        return None
    for a, b in zip(chain, chain[1:]):
        if a['y'] <= y <= b['y'] + .012:
            fraction = min(1., max(0., (y - a['y']) / (b['y'] - a['y'])))
            return a['x'] + fraction * (b['x'] - a['x'])
    return None


def _range_candidates(rays, points, width, height, low, high, feet_y, torso_y):
    # Background returns between legs are retained for isolation checks but never
    # pooled into a person's range. Only visible leg-centre corridors nominate
    # hit clusters. Both legs must contribute independently.
    corridor = max(3., (high - low - 6.) * .15)
    eligible = []
    for ray in rays:
        if ray.get('free_limit') or not low <= ray['u'] <= high or not torso_y * height < ray['v'] < feet_y * height + 5:
            continue
        distances = []
        for leg, indexes in enumerate(((23, 25, 27), (24, 26, 28))):
            x = _leg_x(points, *indexes, ray['v'] / height)
            if x is not None and abs(ray['u'] - x * width) <= corridor:
                distances.append((abs(ray['u'] - x * width), leg))
        distances.sort()
        # Overlapping/inferred legs cannot share the same ray as independent
        # evidence. An equidistant ray is ambiguous and supports neither leg.
        if distances and (len(distances) == 1 or distances[1][0]-distances[0][0] >= 1.):
            eligible.append((ray, [distances[0][1]]))
    if len(eligible) < 2:
        return []
    # Complete-link depth bands avoid chaining a wall onto a foreground leg.
    # They deliberately allow a gap in bearing between two separate legs.
    bands = []
    for ray, legs in sorted(eligible, key=lambda item: item[0]['depth']):
        if not bands or ray['depth'] - bands[-1][0][0]['depth'] > .38:
            bands.append([])
        bands[-1].append((ray, legs))
    candidates = []
    for band in bands:
        if len(band) < 2:
            continue
        # Two independent rays are required, even when the leg corridors
        # overlap. One return cannot stand in for both legs.
        left = {r['i'] for r, legs in band if 0 in legs}
        right = {r['i'] for r, legs in band if 1 in legs}
        if not left or not right or len(left | right) < 2:
            continue
        depth = statistics.median(r['depth'] for r, _ in band)
        candidates.append((depth, [r for r, _ in band]))
    return candidates


def make_person_evidence(points, frame, rgb, histogram):
    invalid = {'valid': False, 'identity_valid': False, 'range_valid': False}
    if not _geometry_ok(frame):
        return dict(invalid, reason='invalid_sensor_geometry')
    if not isinstance(points, (list, tuple)) or len(points) < 29:
        return dict(invalid, reason='full_body_not_visible')
    torso = [points[i] for i in [11, 12, 23, 24]]
    if not all(_point_ok(p, .65) for p in torso):
        return dict(invalid, reason='person_torso_not_visible')
    w, h = frame['width'], frame['height']
    f = .5 * w / math.tan(math.radians(frame.get('hfov_deg', 102)) * .5)
    x0, x1 = min(p['x'] for p in torso), max(p['x'] for p in torso)
    y0, y1 = min(p['y'] for p in torso), max(p['y'] for p in torso)
    app = histogram(rgb, [int(x0*w), int(y0*h), max(1, int((x1-x0)*w)), max(1, int((y1-y0)*h))])
    center = sum(p['x'] for p in torso) / 4
    bearing = -math.atan2((center-.5)*w, f)
    result = {'valid': False, 'identity_valid': True, 'bearing_rad': bearing,
              'appearance': app, 'torso_box': [x0, y0, x1-x0, y1-y0],
              'range_valid': False, 'reason': 'person_cropped_or_occluded'}
    if not _point_ok(points[0], .65) or not all(_point_ok(points[i]) for i in (27, 28)):
        return result
    if max(min(points[i]['visibility'], points[i]['presence']) for i in (27, 28)) < .75:
        return result
    feet_y = (points[27]['y'] + points[28]['y']) * .5
    extent = (feet_y - points[0]['y']) * h
    if extent < 35 or points[23]['y'] >= feet_y or points[24]['y'] >= feet_y:
        result['reason'] = 'standing_body_geometry_uncertain'
        return result
    result.update(valid=True, reason='person_range_uncertain',
                  adult_scale_depth_interval_m=[1.20*f/extent, 1.95*f/extent])
    rays = project_scan(frame, include_free_limit=True)
    low = min(points[i]['x'] for i in (11, 12, 23, 24, 27, 28)) * w - 3
    high = max(points[i]['x'] for i in (11, 12, 23, 24, 27, 28)) * w + 3
    candidates = _range_candidates(rays, points, w, h, low, high, feet_y, y1)
    margin = max(20, (high-low)*.7)
    sides = [[r['depth'] for r in rays if low-margin <= r['u'] < low],
             [r['depth'] for r in rays if high < r['u'] <= high+margin]]
    dmin, dmax = result['adult_scale_depth_interval_m']
    accepted = [(depth, cluster) for depth, cluster in candidates
                if max(.8, dmin) <= depth <= min(5.5, dmax)
                and all(side and statistics.median(side) >= depth+.35 for side in sides)]
    if len(accepted) != 1:
        if len(accepted) > 1:
            result['reason'] = 'ambiguous_person_range_clusters'
        return result
    depth, cluster = accepted[0]
    uncertainty = .20 + .06*depth
    result.update(range_valid=True, range_m=depth,
                  range_lower_m=max(0, min(depth-uncertainty, dmin)),
                  range_uncertainty_m=uncertainty, support_rays=len(cluster),
                  range_cluster_bearing_rad=-math.atan2(statistics.median(r['u'] for r in cluster)-.5*w, f),
                  reason='rgb_legs_and_isolated_foreground_agree',
                  range_source='uncertain_lidar_leg_clusters_corroborated_by_adult_rgb_scale')
    return result
