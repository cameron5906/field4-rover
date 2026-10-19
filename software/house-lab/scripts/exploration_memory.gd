extends RefCounted
## Map-epoch search memory. Inputs are measured occupancy counts, estimated pose,
## and candidates which the caller has already checked against observed free space.
## This module never determines that a semantic mission has succeeded.
## Keep this object across planner/mission refreshes; reset only with the map epoch.

const REGION_RADIUS_M: float = 0.7
const MIN_INFORMATION_GAIN: int = 24
const LOCAL_INFORMATION_GAIN: int = 8
const LOCAL_HINT_IMPROVEMENT: int = 2
const PROGRESS_EPSILON_M: float = 0.09
const INSPECTION_YAW_BUDGET_RAD: float = TAU
const INSPECTION_TIME_BUDGET_MS: int = 24000

var _epoch: int = -1
var _has_epoch: bool = false
var _has_observation: bool = false
var _known_count: int = 0
var _known_high_water: int = 0
var _information_anchor: int = 0
var _information_generation: int = 0
var _now_ms: int = -1
var _pose: Vector2 = Vector2.ZERO
var _has_pose: bool = false

# One compact record per attempted spatial region, never one per planner refresh.
# Anchors do not move, so nearby goals cannot chain a blacklist across a corridor.
var _regions: Array[Dictionary] = []
var _region_buckets: Dictionary = {}
var _attempted_regions: int = 0
var _total_attempts: int = 0
var _no_information_attempts: int = 0
var _reopened_attempts: int = 0
var _last_goal_outcome: String = ""
var _last_goal_information_gain: int = 0
var _active_region: int = -1
var _active_goal: Vector2 = Vector2.ZERO
var _active_known: int = 0
var _active_hint: int = 1
var _active_started_ms: int = -1
var _active_start_distance_m: float = 0.0
var _active_best_distance_m: float = 0.0
var _active_progress_reference_m: float = 0.0
var _last_goal_progress_ms: int = -1

var _has_candidates: bool = false
var _inspection_paused: bool = false
var _inspection_started_ms: int = -1
var _last_inspection_sample_ms: int = -1
var _inspection_yaw_rad: float = 0.0


func reset(epoch: int) -> void:
    _epoch = epoch
    _has_epoch = true
    _has_observation = false
    _known_count = 0
    _known_high_water = 0
    _information_anchor = 0
    _information_generation = 0
    _now_ms = -1
    _pose = Vector2.ZERO
    _has_pose = false
    _regions.clear()
    _region_buckets.clear()
    _attempted_regions = 0
    _total_attempts = 0
    _no_information_attempts = 0
    _reopened_attempts = 0
    _last_goal_outcome = ""
    _last_goal_information_gain = 0
    _active_region = -1
    _active_goal = Vector2.ZERO
    _active_known = 0
    _active_hint = 1
    _active_started_ms = -1
    _active_start_distance_m = 0.0
    _active_best_distance_m = 0.0
    _active_progress_reference_m = 0.0
    _last_goal_progress_ms = -1
    _has_candidates = false
    _inspection_paused = false
    _inspection_started_ms = -1
    _last_inspection_sample_ms = -1
    _inspection_yaw_rad = 0.0


func observe(epoch: int, known_count: int, now_ms: int, pose: Vector2) -> void:
    if known_count < 0 or now_ms < 0:
        return
    if not _has_epoch or epoch != _epoch or (_now_ms >= 0 and now_ms < _now_ms):
        reset(epoch)
    var first_observation: bool = not _has_observation
    _has_observation = true
    _now_ms = now_ms
    _known_count = known_count
    # Temporary drops/recounting cannot manufacture information gain on recovery.
    _known_high_water = maxi(_known_high_water, known_count)
    _has_pose = pose.is_finite()
    if _has_pose:
        _pose = pose
    if first_observation:
        _information_anchor = _known_high_water
        _restart_inspection()
    elif _known_high_water - _information_anchor >= MIN_INFORMATION_GAIN:
        _information_anchor = _known_high_water
        _information_generation += 1
        _restart_inspection()
    if _active_region >= 0 and pose.is_finite():
        var distance: float = _pose.distance_to(_active_goal)
        _active_best_distance_m = minf(_active_best_distance_m, distance)
        # Moving away, rotating, or returning to an old best is not progress.
        if distance <= _active_progress_reference_m - PROGRESS_EPSILON_M:
            _active_progress_reference_m = distance
            _last_goal_progress_ms = now_ms


func can_attempt(point: Vector2, information_hint: int = 1) -> bool:
    if not _has_observation or not _has_pose or _known_count <= 0:
        return false
    if not point.is_finite() or information_hint < 0:
        return false
    var region: int = _find_region(point)
    if region < 0:
        return true
    if region == _active_region:
        return true
    var record: Dictionary = _regions[region]
    if int(record.attempts) == 0:
        return true
    var gained: int = _known_high_water - int(record.last_known)
    if gained >= MIN_INFORMATION_GAIN:
        return true
    # A stronger observed frontier hint still needs genuine measured map growth.
    # Clamp the hint to the caller's six-neighbor frontier contract.
    return gained >= LOCAL_INFORMATION_GAIN and clampi(information_hint, 0, 6) >= int(record.last_hint) + LOCAL_HINT_IMPROVEMENT


func begin_goal(point: Vector2, information_hint: int = 1) -> bool:
    if not _has_observation or not _has_pose or _known_count <= 0:
        return false
    if not point.is_finite() or information_hint < 0:
        return false
    if _active_region >= 0:
        # A delayed semantic refresh must not erase a committed frontier or reset
        # its progress/information baseline. The caller retains its actual route.
        return _find_region(point) == _active_region
    if not can_attempt(point, information_hint):
        return false
    var region: int = _find_region(point)
    if region < 0:
        region = _regions.size()
        _regions.append({
            "point": point,
            "attempts": 0,
            "no_information_attempts": 0,
            "last_known": _known_high_water,
            "last_hint": clampi(information_hint, 0, 6),
            "last_outcome": "",
            "last_finished_ms": -1,
        })
        var bucket: Vector2i = _bucket(point)
        if not _region_buckets.has(bucket):
            _region_buckets[bucket] = []
        _region_buckets[bucket].append(region)
    elif int(_regions[region].attempts) > 0:
        _reopened_attempts += 1
    _active_region = region
    _active_goal = point
    _active_known = _known_high_water
    _active_hint = clampi(information_hint, 0, 6)
    _active_started_ms = _now_ms
    _active_start_distance_m = _pose.distance_to(point)
    _active_best_distance_m = _active_start_distance_m
    _active_progress_reference_m = _active_start_distance_m
    _last_goal_progress_ms = _now_ms
    _has_candidates = true
    _restart_inspection()
    return true


func finish_goal(outcome: String) -> void:
    if _active_region < 0:
        return
    var clean_outcome: String = outcome.strip_edges().left(64)
    _last_goal_outcome = clean_outcome
    _last_goal_information_gain = _known_high_water - _active_known
    # Supersession is not an observation of the destination. Retain any earlier
    # real attempt history but do not consume gain or suppress an unvisited goal.
    var canceled: bool = clean_outcome in [
        "mission_changed", "new_semantic_goal", "visual_verification_handoff",
        "observation_epoch_changed", "visual_reinspection_timed_out", "canceled", "superseded",
    ] or clean_outcome.begins_with("semantic_") or clean_outcome.begins_with("object_")
    if not canceled:
        var record: Dictionary = _regions[_active_region]
        if int(record.attempts) == 0:
            _attempted_regions += 1
        record.attempts = int(record.attempts) + 1
        record.last_known = _known_high_water
        record.last_hint = _active_hint
        record.last_outcome = clean_outcome
        record.last_finished_ms = _now_ms
        _total_attempts += 1
        if _last_goal_information_gain < MIN_INFORMATION_GAIN:
            record.no_information_attempts = int(record.no_information_attempts) + 1
            _no_information_attempts += 1
    # Gain earned on a reached/failed attempt is consumed above. It cannot
    # immediately reopen that same region on the next planner refresh.
    _active_region = -1
    _active_started_ms = -1
    _has_candidates = false
    _inspection_paused = false
    _restart_inspection()


func note_inspection(delta_yaw_rad: float, has_candidates: bool, external_goal_active: bool = false) -> void:
    _has_candidates = has_candidates or external_goal_active
    # The parent owns remembered semantic routes, which intentionally do not
    # consume geometric frontier attempts. Their travel time is not inspection.
    if _active_region >= 0 or external_goal_active:
        _inspection_paused = true
        _restart_inspection()
        return
    if _inspection_paused:
        _inspection_paused = false
        _restart_inspection()
        return
    if not _has_observation or _now_ms == _last_inspection_sample_ms:
        return
    _last_inspection_sample_ms = _now_ms
    if _active_region >= 0 or has_candidates or not is_finite(delta_yaw_rad):
        return
    # Count measured travel, including back-and-forth turns: this is a finite
    # safety/search budget, not a claim that the whole room has been viewed.
    var measured_yaw: float = absf(wrapf(delta_yaw_rad, -PI, PI))
    _inspection_yaw_rad = minf(INSPECTION_YAW_BUDGET_RAD, _inspection_yaw_rad + measured_yaw)


func search_exhausted() -> bool:
    if not _has_observation or _active_region >= 0 or _has_candidates:
        return false
    return _inspection_yaw_rad >= INSPECTION_YAW_BUDGET_RAD or _inspection_elapsed_ms() >= INSPECTION_TIME_BUDGET_MS


func status_context() -> Dictionary:
    var exhausted: bool = search_exhausted()
    var reason: String = ""
    if exhausted:
        reason = "insufficient_observed_map" if _known_count <= 0 else "no_new_reachable_viewpoint"
    var active: bool = _active_region >= 0
    return {
        "map_epoch": _epoch,
        "known_cells": _known_count,
        "known_high_water": _known_high_water,
        "observed_map_available": _has_observation and _known_count > 0,
        "estimated_pose_available": _has_pose,
        "information_generation": _information_generation,
        "reopen_gain_cells": MIN_INFORMATION_GAIN,
        "attempted_regions": _attempted_regions,
        "total_attempts": _total_attempts,
        "no_information_attempts": _no_information_attempts,
        "reopened_attempts": _reopened_attempts,
        "last_goal_outcome": _last_goal_outcome,
        "last_goal_information_gain_cells": _last_goal_information_gain,
        "goal_active": active,
        "goal": {"x": _active_goal.x, "z": _active_goal.y} if active else {},
        "goal_age_ms": maxi(0, _now_ms - _active_started_ms) if active else 0,
        "last_goal_progress_age_ms": maxi(0, _now_ms - _last_goal_progress_ms) if active else 0,
        "goal_best_distance_m": _active_best_distance_m if active else 0.0,
        "goal_progress_m": maxf(0.0, _active_start_distance_m - _active_best_distance_m) if active else 0.0,
        "inspection_yaw_rad": _inspection_yaw_rad,
        "inspection_budget_rad": INSPECTION_YAW_BUDGET_RAD,
        "inspection_elapsed_ms": _inspection_elapsed_ms(),
        "inspection_budget_ms": INSPECTION_TIME_BUDGET_MS,
        "eligible_candidates_available": _has_candidates,
        "search_exhausted": exhausted,
        "exhaustion_reason": reason,
        "mission_complete": false,
        "evidence_source": "measured_occupancy_and_estimated_pose_only",
    }


func _restart_inspection() -> void:
    _inspection_started_ms = _now_ms
    _inspection_yaw_rad = 0.0
    # Do not charge the last travel/turn sample to a newly entered inspection.
    _last_inspection_sample_ms = _now_ms


func _inspection_elapsed_ms() -> int:
    if _inspection_paused or _inspection_started_ms < 0 or _now_ms < 0:
        return 0
    return maxi(0, _now_ms - _inspection_started_ms)


func _bucket(point: Vector2) -> Vector2i:
    return Vector2i(floori(point.x / REGION_RADIUS_M), floori(point.y / REGION_RADIUS_M))


func _find_region(point: Vector2) -> int:
    var bucket: Vector2i = _bucket(point)
    var nearest: int = -1
    var best_distance_squared: float = REGION_RADIUS_M * REGION_RADIUS_M
    for y in range(-1, 2):
        for x in range(-1, 2):
            var nearby: Vector2i = bucket + Vector2i(x, y)
            for index in _region_buckets.get(nearby, []):
                var anchor: Vector2 = _regions[int(index)].point
                var distance_squared: float = point.distance_squared_to(anchor)
                if distance_squared < best_distance_squared or (is_equal_approx(distance_squared, best_distance_squared) and (nearest < 0 or int(index) < nearest)):
                    nearest = int(index)
                    best_distance_squared = distance_squared
    return nearest
