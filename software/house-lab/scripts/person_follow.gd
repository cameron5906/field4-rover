extends RefCounted
const GridMotion = preload("res://scripts/grid_motion.gd")

# A follow mission owns one perception identity. Missing observations stop the
# drive; they are never replaced by a predicted person pose or a nearby person.
const MAX_AGE_MS := 900
const IDENTITY_GAP_MS := 1800
const STANDOFF_M := 2.0
const MAX_SPEED := 0.16
const FOOTPRINT_RADIUS_M := 0.28
const LOOKAHEAD_M := 0.35
const PERSON_APPROACH_BOUND_M_S := 0.6
const CARDINALS := [Vector2i.LEFT, Vector2i.RIGHT, Vector2i.UP, Vector2i.DOWN]

var enabled := false
var status := "off"
var reason := "Follow not started"
var generation := 0
var mission_version := -1
var epoch := -1
var started_ms := 0
var last_frame := -1
var last_capture_ms := -1
var last_identity_ms := -1
var acquired := false
var locked := false
var valid := false
var target_track_id := -1
var target_track_epoch := -1
var pose_lock_frame := -1
var observation_source := "none"
var stable_count := 0
var stable_since_ms := -1
var tracked_center := Vector2.ZERO
var tracked_scale := 0.0
var appearance: Array = []
var bearing_rad := 0.0
var range_lower_m := 0.0
var range_m := 0.0
var capture_pose := Vector3.ZERO
var capture_origin := Vector2.ZERO
var context: Dictionary = {}
var route_points: Array[Vector2] = []
var _resume_after_capture := -1
var _gesture_paused := false
var gesture_latched := false
var _clear_count := 0
var _clear_since_ms := -1
var _route_frame := -1
var _route_scan := -1
var _route_mapper_id := 0
var _safe_cache: Dictionary = {}

static func supports_mission(value: String) -> bool:
    var expression := RegEx.new()
    expression.compile("^(?:please\\s+)?(?:follow|track)\\s+(?:(?:the|a|this)\\s+)?(?:person|human|participant|walking person|me)(?:\\s+safely)?[.!]?$")
    return expression.search(value.strip_edges().to_lower()) != null

func start(version: int, map_epoch: int, now_ms: int) -> void:
    abort("New explicit follow start")
    enabled = true
    mission_version = version
    epoch = map_epoch
    started_ms = now_ms
    status = "acquiring"
    reason = "Waiting for one clearly visible person"
    last_frame = -1
    last_capture_ms = -1
    last_identity_ms = -1
    target_track_id = -1
    target_track_epoch = -1
    pose_lock_frame = -1
    observation_source = "none"
    locked = false
    stable_count = 0
    stable_since_ms = -1
    appearance.clear()
    tracked_scale = 0.0
    range_m = 0.0
    range_lower_m = 0.0
    _resume_after_capture = -1
    _gesture_paused = false

func abort(why: String = "Stopped") -> void:
    generation += 1
    enabled = false
    valid = false
    acquired = false
    locked = false
    gesture_latched = false
    _clear_count = 0
    _clear_since_ms = -1
    status = "cancelled"
    reason = why
    target_track_id = -1
    target_track_epoch = -1
    pose_lock_frame = -1
    observation_source = "none"
    _clear_route()
    context.clear()

func _clear_route() -> void:
    route_points.clear()
    _route_frame = -1
    _route_scan = -1
    _route_mapper_id = 0
    _safe_cache.clear()

func _withhold(why: String, next_status: String = "lost", preserve_continuity: bool = false) -> void:
    valid = false
    if not preserve_continuity:
        stable_count = 0
        stable_since_ms = -1
        _clear_count = 0
        _clear_since_ms = -1
    status = next_status if acquired else "acquiring"
    reason = why
    _clear_route()

func _number(value) -> bool:
    return (value is int or value is float) and is_finite(float(value))

func _integer(value) -> bool:
    return _number(value) and float(value) == floorf(float(value))

func _track_id(pose: Dictionary) -> int:
    var evidence = pose.get("follow_evidence", {})
    if not evidence is Dictionary or not _integer(evidence.get("track_id")):
        return -1
    if not _integer(evidence.get("track_epoch")) or int(evidence.track_epoch) != epoch:
        return -1
    return int(evidence.track_id)

func observe(result: Dictionary, metadata: Dictionary, now_ms: int) -> void:
    if not enabled:
        return
    if not result.get("ok", false):
        _withhold("Vision unavailable · stopped")
        return
    for key in ["epoch", "frame_id", "capture_monotonic_ms"]:
        if not _integer(result.get(key)) or not _integer(metadata.get(key)):
            _withhold("Invalid frame provenance · stopped")
            return
    # An old in-flight response must not renew or replace this mission's input.
    if int(result.epoch) != epoch or int(metadata.epoch) != epoch:
        return
    var tracking = result.get("person_tracks", {})
    if not tracking is Dictionary or tracking.get("status", "") != "updated":
        _withhold("Person tracker input rejected · stopped")
        return
    var captured := int(result.capture_monotonic_ms)
    var frame := int(result.frame_id)
    if int(metadata.frame_id) != frame or int(metadata.capture_monotonic_ms) != captured:
        _withhold("Mismatched capture metadata · stopped")
        return
    if captured < started_ms or frame <= last_frame or captured <= last_capture_ms:
        return
    if now_ms < captured or now_ms - captured > MAX_AGE_MS:
        _withhold("Person view stale · stopped")
        return
    if last_capture_ms >= 0 and captured - last_capture_ms > MAX_AGE_MS:
        stable_count = 0
        stable_since_ms = -1
        _clear_count = 0
        _clear_since_ms = -1
    last_frame = frame
    last_capture_ms = captured
    # Real pose observations and strictly bounded detector+current-lidar
    # continuations have distinct provenance. GestureGuard consumes only poses.
    var poses = result.get("person_observations", result.get("poses", []))
    if not poses is Array:
        _withhold("Invalid person observations · stopped")
        return
    var candidates: Array[Dictionary] = []
    for value in poses:
        if not value is Dictionary:
            continue
        var identity := _track_id(value)
        if identity > 0 and (target_track_id < 0 or identity == target_track_id):
            candidates.append(value)
    if candidates.size() != 1:
        locked = acquired
        _withhold("Selected person not visible · waiting for same identity" if acquired else "Waiting for one unambiguous tracked person")
        return
    var pose: Dictionary = candidates[0]
    var evidence: Dictionary = pose.follow_evidence
    var source := str(pose.get("observation_source", "pose_landmarks"))
    if source not in ["pose", "pose_landmarks", "detector_lidar_continuation"]:
        _withhold("Unknown person observation source · stopped")
        return
    if source == "detector_lidar_continuation":
        # Python may retain an identity across missions. That never grants this
        # mission a new target: it must have acquired a real full-pose lock first.
        if target_track_id < 0 or pose_lock_frame < 0 or not acquired:
            _withhold("Full pose required to acquire follow target")
            return
        if str(evidence.get("track_state", "")) != "confirmed" or not _integer(evidence.get("pose_anchor_frame_id")) or int(evidence.pose_anchor_frame_id) < pose_lock_frame or int(evidence.pose_anchor_frame_id) >= frame:
            _withhold("Detector continuation lacks this mission's pose anchor")
            return
        if not _number(evidence.get("pose_anchor_age_s")) or float(evidence.pose_anchor_age_s) < 0 or float(evidence.pose_anchor_age_s) > 5.0:
            _withhold("Detector continuation pose anchor expired")
            return
        if pose.get("gesture", "unknown") != "unknown" or evidence.get("gesture_release_eligible", false):
            _withhold("Detector observation cannot prove hand clearance")
            return
    if not _integer(evidence.get("observation_frame_id")) or int(evidence.observation_frame_id) != frame:
        _withhold("Person observation does not match frame · stopped")
        return
    if not _number(metadata.get("timestamp_s")) or not _number(evidence.get("observation_timestamp_s")) or absf(float(metadata.timestamp_s) - float(evidence.observation_timestamp_s)) > 0.000001:
        _withhold("Person observation timestamp does not match capture · stopped")
        return
    var track_state := str(evidence.get("track_state", ""))
    if track_state not in ["tentative", "confirmed", "reacquiring"] or not evidence.get("identity_valid", false):
        locked = acquired
        _withhold("Person association uncertain · waiting for same identity")
        return
    var center = pose.get("center", [])
    var scale = pose.get("scale", 0.0)
    if not center is Array or center.size() != 2 or not _number(center[0]) or not _number(center[1]) or not _number(scale) or float(scale) < 0.045:
        _withhold("Invalid person geometry · stopped")
        return
    if target_track_id < 0:
        target_track_id = int(evidence.track_id)
        target_track_epoch = epoch
    acquired = true
    locked = false
    tracked_center = Vector2(float(center[0]), float(center[1]))
    tracked_scale = float(scale)
    last_identity_ms = captured
    observation_source = source
    # The tracker performs motion-aware identity association. Do not apply a
    # second camera-fixed center/scale gate that loses a turning target.
    var estimated = metadata.get("pose", [])
    var mount = metadata.get("camera_mount", [0.0, 0.133, -0.1078])
    if not estimated is Array or estimated.size() != 3 or not mount is Array or mount.size() != 3:
        _withhold("Capture pose missing · stopped")
        return
    for component in estimated + mount:
        if not _number(component):
            _withhold("Invalid capture pose · stopped")
            return
    capture_pose = Vector3(float(estimated[0]), float(estimated[1]), float(estimated[2]))
    capture_origin = Vector2(capture_pose.x, capture_pose.y) + Vector2(
        cos(capture_pose.z) * float(mount[0]) + sin(capture_pose.z) * float(mount[2]),
        -sin(capture_pose.z) * float(mount[0]) + cos(capture_pose.z) * float(mount[2]))
    if stable_since_ms < 0:
        stable_since_ms = captured
    stable_count += 1
    var gesture := str(pose.get("gesture", "unknown"))
    var confident_hands := _number(pose.get("confidence")) and float(pose.confidence) >= 0.55
    if gesture == "raised" and confident_hands:
        gesture_latched = true
        _resume_after_capture = captured
        _clear_count = 0
        _clear_since_ms = -1
    if gesture_latched:
        if gesture == "lowered" and confident_hands:
            if _clear_since_ms < 0:
                _clear_since_ms = captured
            _clear_count += 1
        else:
            _clear_count = 0
            _clear_since_ms = -1
        if _clear_count >= 3 and captured - _clear_since_ms >= 700 and evidence.get("gesture_release_eligible", false):
            gesture_latched = false
        else:
            _withhold("Hands-up hold · fresh positively lowered hands required", "gesture_hold", true)
            return
    # Unknown hands are not a raised-arm detection. Ordinary acquisition uses
    # identity + fresh measured range. They can never release a latched hold;
    # the independent global GestureGuard also continues to veto step().
    if not evidence.get("valid", false) or not evidence.get("range_observed", false) or not evidence.get("range_valid", false):
        _withhold("Current person range uncertain · stopped", "acquiring" if track_state == "tentative" else "reacquiring", true)
        return
    for key in ["bearing_rad", "range_m", "range_lower_m"]:
        if not _number(evidence.get(key)):
            _withhold("Invalid person range · stopped")
            return
    bearing_rad = float(evidence.bearing_rad)
    range_m = float(evidence.range_m)
    range_lower_m = float(evidence.range_lower_m)
    if absf(bearing_rad) > deg_to_rad(75) or range_m <= 0 or range_m > 12 or range_lower_m <= 0 or range_lower_m > range_m:
        _withhold("Invalid person range bounds · stopped")
        return
    valid = track_state == "confirmed" and evidence.get("motion_eligible", false) and stable_count >= 3 and captured - stable_since_ms >= 600
    if valid and source in ["pose", "pose_landmarks"] and pose_lock_frame < 0:
        pose_lock_frame = frame
    status = "following" if valid else ("reacquiring" if track_state == "reacquiring" else "acquiring")
    reason = "Fresh selected person + measured range" if valid else "Confirming selected person continuity"

func _context() -> Dictionary:
    return {"navigation_valid": false, "linear_m_s": 0.0, "angular_rad_s": 0.0,
        "task_status": status, "reason": reason, "range_m": range_m, "range_lower_m": range_lower_m,
        "generation": generation, "mission_version": mission_version, "track_id": target_track_id,
        "track_epoch": target_track_epoch, "observation_frame_id": last_frame,
        "observation_source": observation_source, "pose_lock_frame_id": pose_lock_frame,
        "gesture_hold_latched": gesture_latched,
        "clearance_source": "independent_fresh_lidar_swept_footprint", "route_points": []}

func step(estimated_pose: Vector3, now_ms: int, gesture_active: bool, mapper = null) -> Dictionary:
    context = _context()
    if not enabled:
        return context
    if not estimated_pose.is_finite() or now_ms < started_ms or (last_capture_ms >= 0 and now_ms < last_capture_ms):
        _withhold("Invalid current estimated pose or clock · stopped")
        context = _context()
        return context
    if gesture_active:
        _gesture_paused = true
        _resume_after_capture = maxi(_resume_after_capture, now_ms)
        _clear_route()
        context.task_status = "gesture_hold"
        context.reason = "Hands up · same follow mission paused"
        return context
    if _gesture_paused:
        _gesture_paused = false
        _clear_route()
    if last_capture_ms < 0 or now_ms - last_capture_ms > MAX_AGE_MS:
        # A slow pipeline can leave a brief stale-output gap even when the
        # captures themselves form a continuous track. Stop now, but do not
        # destroy acquisition history on each physics tick. Observe() resets
        # it on a real capture gap, missing/ambiguous identity or gesture.
        _withhold("Person view stale · waiting for same identity", "lost", true)
        if acquired and now_ms - last_identity_ms > IDENTITY_GAP_MS:
            locked = true
            reason = "Selected person lost · fresh same-identity evidence required"
        context = _context()
        return context
    if not valid or locked or gesture_latched or last_capture_ms <= _resume_after_capture:
        return context
    var yaw_change := wrapf(estimated_pose.z - capture_pose.z, -PI, PI)
    var here := Vector2(estimated_pose.x, estimated_pose.y)
    var moved := here.distance_to(Vector2(capture_pose.x, capture_pose.y))
    var age := float(now_ms - last_capture_ms) / 1000.0
    var lower := range_lower_m - moved - age * PERSON_APPROACH_BOUND_M_S
    if absf(yaw_change) > 0.25 or moved > 0.22:
        context.reason = "Robot moved since capture · waiting for fresh view"
        return context
    var error := wrapf(bearing_rad - yaw_change, -PI, PI)
    var gap := maxf(0.0, lower - STANDOFF_M)
    if mapper != null and gap > 0.05:
        if not _valid_mapper(mapper):
            context.reason = "Measured map unavailable · stopped"
            return context
        # Camera depth is axial, not radial. This target is derived only from
        # sensor evidence and capture odometry, never the actor's scene pose.
        var radial := range_m / cos(bearing_rad)
        var heading := capture_pose.z + bearing_rad
        var target := capture_origin + Vector2(-sin(heading), -cos(heading)) * radial
        var exclusion := STANDOFF_M + maxf(0.0, radial - range_lower_m) + age * PERSON_APPROACH_BOUND_M_S
        var waypoint := _waypoint(mapper, here, target, exclusion)
        if waypoint.is_empty():
            context.reason = "No observed-free person approach · stopped"
            return context
        var offset: Vector2 = waypoint.point - here
        error = wrapf(atan2(-offset.x, -offset.y) - estimated_pose.z, -PI, PI)
        for point in route_points:
            context.route_points.append({"x": point.x, "z": point.y})
        context.route_source = "observed_free_padded_grid"
    var angular := clampf(error * 1.5, -0.4, 0.4) if absf(error) > 0.035 else 0.0
    var linear := minf(MAX_SPEED, gap * 0.22) if absf(error) < 0.32 else 0.0
    var map_turn_only := false
    if mapper != null:
        if not _valid_mapper(mapper):
            context.reason = "Measured map unavailable · stopped"
            return context
        if _route_scan != mapper.scan_count:
            _safe_cache.clear()
        # The steering arc, not just the waypoint chord, must remain in padded
        # observed-free cells. Raw current lidar/IMU supervision stays separate.
        if not GridMotion.command_safe(mapper, estimated_pose, linear, angular, 0.35, func(cell: Vector2i): return _safe_cell(mapper, cell)):
            linear = 0.0
            map_turn_only = true
            if not GridMotion.command_safe(mapper, estimated_pose, linear, angular, 0.35, func(cell: Vector2i): return _safe_cell(mapper, cell)):
                context.reason = "Current follow command leaves observed-free map · stopped"
                return context
    context.navigation_valid = true
    context.linear_m_s = linear
    context.angular_rad_s = angular
    status = "following" if linear > 0 else "holding"
    context.task_status = status
    # Holding distance is nonterminal. The next fresh receding-person input
    # resumes this same identity and mission without another AI press.
    reason = "Following selected visible person" if linear > 0 else "Holding stand-off · follow mission remains active"
    if map_turn_only:
        reason = "Turning before observed-free route corner"
    context.reason = reason
    context.bearing_error_rad = error
    context.range_lower_m = lower
    return context

func _valid_mapper(mapper) -> bool:
    return mapper != null and mapper.width > 0 and mapper.height > 0 and is_finite(mapper.resolution) and mapper.resolution > 0 and mapper.estimated_pose.is_finite() and mapper.observed.size() == mapper.width * mapper.height and mapper.log_odds.size() == mapper.width * mapper.height

func _safe_cell(mapper, cell: Vector2i) -> bool:
    if _safe_cache.has(cell):
        return bool(_safe_cache[cell])
    var padding: float = FOOTPRINT_RADIUS_M + mapper.resolution * sqrt(2.0) * 0.5
    var radius := ceili(padding / mapper.resolution)
    for row in range(-radius, radius + 1):
        for column in range(-radius, radius + 1):
            if Vector2(column, row).length() * mapper.resolution > padding:
                continue
            var check := cell + Vector2i(column, row)
            if not mapper.is_cell_in_bounds(check):
                _safe_cache[cell] = false
                return false
            var index: int = check.y * mapper.width + check.x
            if mapper.observed[index] == 0 or not is_finite(mapper.log_odds[index]) or mapper.log_odds[index] > mapper.FREE_THRESHOLD:
                _safe_cache[cell] = false
                return false
    _safe_cache[cell] = true
    return true

func _segment_safe(mapper, from: Vector2, to: Vector2, target: Vector2, exclusion: float) -> bool:
    if not from.is_finite() or not to.is_finite() or not target.is_finite() or not is_finite(exclusion) or exclusion < 0:
        return false
    # Closest point on the continuous chord catches even a very short incursion
    # into the person's stand-off disc, which point samples can miss.
    var direction := to - from
    var progress := clampf((target - from).dot(direction) / direction.length_squared(), 0.0, 1.0) if direction.length_squared() > 1e-12 else 0.0
    if (from + direction * progress).distance_to(target) < exclusion:
        return false
    return GridMotion.segment_safe(mapper, from, to, func(cell: Vector2i): return _safe_cell(mapper, cell))

func _plan_route(mapper, here: Vector2, target: Vector2, exclusion: float) -> Array[Vector2]:
    var empty: Array[Vector2] = []
    var start: Vector2i = mapper.world_to_cell(here)
    if not _safe_cell(mapper, start):
        return empty
    var approach := target + (here - target).normalized() * (exclusion + 0.12)
    if here.distance_to(target) <= exclusion + 0.12:
        return empty
    if _segment_safe(mapper, here, approach, target, exclusion):
        var direct: Array[Vector2] = [here, approach]
        return direct
    # Bounded breadth-first search over measured free cells. Unknown/occupied
    # cells and the person's stand-off region are never traversable.
    var queue: Array[Vector2i] = [start]
    var parent: Dictionary = {start: start}
    var cursor := 0
    while cursor < queue.size() and cursor < 4096:
        var cell := queue[cursor]
        cursor += 1
        var at: Vector2 = mapper.cell_to_world(cell)
        var distance := at.distance_to(target)
        if distance >= exclusion + 0.08 and distance <= exclusion + 0.30:
            var path: Array[Vector2] = [at]
            while cell != start:
                cell = parent[cell]
                path.push_front(mapper.cell_to_world(cell))
            path[0] = here
            return path
        for direction in CARDINALS:
            var next: Vector2i = cell + direction
            if parent.has(next) or not _safe_cell(mapper, next):
                continue
            var point: Vector2 = mapper.cell_to_world(next)
            if point.distance_to(target) < exclusion + 0.08:
                continue
            parent[next] = cell
            queue.append(next)
    return empty

func _waypoint(mapper, here: Vector2, target: Vector2, exclusion: float) -> Dictionary:
    if _route_frame != last_frame or _route_scan != mapper.scan_count or _route_mapper_id != mapper.get_instance_id():
        _safe_cache.clear()
        route_points = _plan_route(mapper, here, target, exclusion)
        _route_frame = last_frame
        _route_scan = mapper.scan_count
        _route_mapper_id = mapper.get_instance_id()
    if route_points.is_empty():
        return {}
    var nearest := 0
    for i in range(1, route_points.size()):
        if here.distance_to(route_points[i]) < here.distance_to(route_points[nearest]):
            nearest = i
    var next := mini(nearest + 1, route_points.size() - 1)
    while next < route_points.size() - 1 and here.distance_to(route_points[next]) < LOOKAHEAD_M:
        next += 1
    var point := route_points[next]
    # A direct open-space route may have a distant endpoint. Only steer toward
    # a short validated lookahead, so increasing age uncertainty at the distant
    # stand-off endpoint does not turn us back toward the old route origin.
    if here.distance_to(point) > LOOKAHEAD_M:
        point = here + (point - here).normalized() * LOOKAHEAD_M
    while next > nearest and not _segment_safe(mapper, here, point, target, exclusion):
        next -= 1
        point = route_points[next]
    if next == nearest and nearest < route_points.size() - 1:
        return {}
    if here.distance_to(point) < 0.025 or not _segment_safe(mapper, here, point, target, exclusion):
        return {}
    return {"point": point}
