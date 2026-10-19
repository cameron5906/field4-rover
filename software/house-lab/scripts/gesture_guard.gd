extends RefCounted

var active := false
var reason := "Hands clear"
var last_frame := -1
var last_capture_ms := -1
var tracked_center := Vector2.ZERO
var tracked_scale := 0.0
var clear_since_ms := -1
var clear_count := 0
var held_tracks: Dictionary = {}
var unidentified_hold := false
var legacy_hold := false
const MAX_AGE_MS := 900
const CLEAR_HOLD_MS := 700
const LEGACY_POLICY := "legacy_pixel_v07"

func reset_after_explicit_simulation_reset() -> void:
    # Only main.reset_simulation calls this, after Stop and before resetting the
    # scene/sensor epoch. AI restart, mission change and ordinary Stop do not.
    active = false
    reason = "Hands clear · simulation reset"
    last_frame = -1
    last_capture_ms = -1
    clear_since_ms = -1
    clear_count = 0
    tracked_center = Vector2.ZERO
    tracked_scale = 0.0
    held_tracks.clear()
    unidentified_hold = false
    legacy_hold = false

func _track_key(pose: Dictionary, result: Dictionary, release: bool = false) -> String:
    var evidence = pose.get("follow_evidence", {})
    if not evidence is Dictionary: return ""
    for value in [evidence.get("track_id"), evidence.get("track_epoch"), evidence.get("observation_frame_id")]:
        if typeof(value) not in [TYPE_INT, TYPE_FLOAT] or not is_finite(float(value)) or float(value) != floorf(float(value)): return ""
    var identity := int(evidence.track_id)
    var epoch := int(evidence.track_epoch)
    if identity <= 0 or epoch < 0 or epoch != int(result.get("epoch", -1)) or int(evidence.observation_frame_id) != int(result.get("frame_id", -1)): return ""
    if release:
        if evidence.get("track_state", "") not in ["confirmed", "reacquiring", "tentative"] or evidence.get("identity_valid", false) != true: return ""
        var tracks = result.get("person_tracks", {})
        if not tracks is Dictionary or tracks.get("status", "") != "updated": return ""
    return "%d:%d" % [epoch, identity]

func _pixel_pose(pose: Dictionary) -> Dictionary:
    var center = pose.get("center", [])
    var scale = pose.get("scale", 0.0)
    if not center is Array or center.size() != 2: return {}
    for value in [center[0],center[1],scale]:
        if typeof(value) not in [TYPE_INT,TYPE_FLOAT] or not is_finite(float(value)): return {}
    if float(scale) <= 0.0: return {}
    return {"center":Vector2(float(center[0]),float(center[1])),"scale":float(scale)}

func observe(result: Dictionary, now_ms: int) -> bool:
    var frame_id := int(result.get("frame_id", -1))
    var captured := int(result.get("capture_monotonic_ms", -1))
    if not result.get("ok", false) or frame_id <= last_frame or captured < 0 or captured <= last_capture_ms or now_ms-captured < 0 or now_ms-captured > MAX_AGE_MS:
        if active: reason = "Hands-up hold · fresh clear view required"
        clear_since_ms = -1; clear_count = 0
        return active
    last_frame = frame_id
    if last_capture_ms >= 0 and captured-last_capture_ms > MAX_AGE_MS:
        clear_since_ms = -1; clear_count = 0
    last_capture_ms = captured
    var poses = result.get("poses", [])
    if not poses is Array: poses = []
    var explicit_legacy: bool = result.get("gesture_identity_policy", "") == LEGACY_POLICY and not result.has("person_tracks")
    var raised := false
    for pose in poses:
        if not pose is Dictionary: continue
        var confidence := float(pose.get("confidence", 0.0))
        if pose.get("gesture", "") != "raised" or not is_finite(confidence) or confidence < 0.55: continue
        raised = true; active = true
        var key := _track_key(pose, result)
        if not key.is_empty():
            held_tracks[key] = true
        elif explicit_legacy and not pose.has("follow_evidence") and held_tracks.is_empty() and not unidentified_hold:
            var pixel := _pixel_pose(pose)
            if pixel.is_empty(): unidentified_hold = true
            else:
                legacy_hold = true; tracked_center = pixel.center; tracked_scale = pixel.scale
        else:
            unidentified_hold = true
    if raised:
        reason = "Hands up · paused" if not unidentified_hold else "Hands-up hold · identity unavailable; Reset simulation to recover"
        clear_since_ms = -1; clear_count = 0
        return true
    if not active: return false
    if unidentified_hold:
        reason = "Hands-up hold · identity unavailable; Reset simulation to recover"
        clear_since_ms = -1; clear_count = 0
        return true
    var matched: Dictionary = {}
    var release_ready: Dictionary = {}
    var legacy_match := false
    var all_clear: bool = not poses.is_empty()
    for pose in poses:
        if not pose is Dictionary:
            all_clear = false; continue
        var confidence := float(pose.get("confidence", 0.0))
        if pose.get("gesture", "") != "lowered" or not is_finite(confidence) or confidence < 0.55:
            all_clear = false; continue
        if not held_tracks.is_empty():
            var key := _track_key(pose, result, true)
            if key.is_empty(): all_clear = false
            else:
                matched[key] = int(matched.get(key, 0)) + 1
                var evidence: Dictionary = pose.follow_evidence
                release_ready[key] = evidence.get("track_state", "") == "confirmed" and evidence.get("gesture_release_eligible", false) == true
        if legacy_hold:
            if not explicit_legacy or pose.has("follow_evidence"):
                all_clear = false; continue
            var pixel := _pixel_pose(pose)
            if not pixel.is_empty() and pixel.center.distance_to(tracked_center) < 0.18 and pixel.scale > tracked_scale*0.6 and pixel.scale < tracked_scale*1.7:
                legacy_match = true; tracked_center = pixel.center; tracked_scale = pixel.scale
    var clear_match: bool = not held_tracks.is_empty() or (legacy_hold and legacy_match)
    var all_identities_confirmed := true
    for key in held_tracks:
        if int(matched.get(key,0)) != 1: clear_match = false
        if not release_ready.get(key,false): all_identities_confirmed = false
    if legacy_hold and not legacy_match: clear_match = false
    if clear_match and all_clear:
        if clear_since_ms < 0: clear_since_ms = captured
        clear_count += 1
        reason = "Hands lowered · confirming same person (%d/3)" % mini(clear_count,3)
        if clear_count >= 3 and captured-clear_since_ms >= CLEAR_HOLD_MS and all_identities_confirmed:
            active = false; reason = "Hands clear"; clear_since_ms = -1; clear_count = 0
            held_tracks.clear(); legacy_hold = false
    else:
        clear_since_ms = -1; clear_count = 0
        reason = "Hands-up hold · same identified person must lower hands" if not poses.is_empty() else "Hands-up hold · lowered person not visible"
    return active

func tick(now_ms: int) -> void:
    if active and (last_capture_ms < 0 or now_ms-last_capture_ms > MAX_AGE_MS):
        reason = "Hands-up hold · vision stale" if not unidentified_hold else "Hands-up hold · identity unavailable; Reset simulation to recover"
