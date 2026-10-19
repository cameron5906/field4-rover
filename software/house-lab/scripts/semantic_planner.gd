extends RefCounted
const GridMotion = preload("res://scripts/grid_motion.gd")

const ExplorationMemory = preload("res://scripts/exploration_memory.gd")
var exploration_memory = ExplorationMemory.new()


const ROOM_TYPES : = [ "bedroom" , "living_room" , "kitchen" , "bathroom" , "hallway" , "office" , "dining_room" , "utility_room" , "unknown" ]
const EVIDENCE : = [ "bed" , "desk" , "sofa" , "stove" , "sink" , "toilet" , "door" , "chair" , "table" , "cabinet" , "bookshelf" , "refrigerator" , "television" , "shower" , "bathtub" , "wardrobe" , "lamp" , "washing_machine" , "dryer" , "microwave" , "oven" , "mirror" , "window" , "plant" , "computer" , "monitor" , "no_identifying_feature" ]
const IDENTIFYING : = { "bedroom" : "bed" , "living_room" : "sofa" , "kitchen" : "stove" , "bathroom" : "toilet" , "office" : "desk" , "dining_room" : "table" , "utility_room" : "washing_machine" }
const OBJECT_ALIASES : = { "couch" : "sofa" , "fridge" : "refrigerator" , "tv" : "television" , "bookcase" : "bookshelf" , "shelf" : "bookshelf" , "shelves" : "bookshelf" , "work desk" : "desk" , "writing desk" : "desk" , "washing machine" : "washing_machine" }
const MIN_INTERVAL_MS : = 6000
const REASSESS_MS : = 25000
const MAX_RESPONSE_MS : = 25000
const GRAPH_INTERVAL_MS : = 1000
const STUCK_MS : = 9000
const BLACKLIST_MS : = 45000
const LOOKAHEAD_M : = 0.4
const FOOTPRINT_RADIUS_M : = 0.28
const CARDINALS : = [ Vector2i . LEFT , Vector2i . RIGHT , Vector2i . UP , Vector2i . DOWN ]
const DIGITS : = [ "111101101101111" , "010110010010111" , "111001111100111" , "111001111001111" , "101101111001001" , "111100111001111" , "111100111101111" , "111001001001001" , "111101111101111" , "111101111001111" ]
var mission : = ""
var mission_version : = 0
var status : = "idle"
var subgoal : Dictionary = { }
var room_memory : Array = [ ]
var visits : Array = [ ]
var candidates : Array = [ ]
var pending : = false
var plan_revision : = 0
var summary : = "Set a mission, then start AI"
var _request : Dictionary = { }


var _last_plan_ms : = -100000
var _last_observe_ms : = -100000
var _observation_ms : = 0
var _known_at_plan : = 0
var _known : = 0
var _pose : = Vector3 . ZERO
var _reason : = "mission_changed"
var _parents : Dictionary = { }
var _depth : Dictionary = { }
var _root : = Vector2i . ZERO
var _goal_cell : = Vector2i . ZERO
var _has_goal : = false
var _goal_source : = ""
var _goal_inspection_yaw := NAN
var _failed_attempts : = 0
var _route : Array [ Vector2i ] = [ ]
var _route_points : Array = [ ]
var _route_cursor : = 0
var _route_revision : = 0
var _graph_revision : = 0
var _graph_mapper_id : = 0
var _blacklist : Dictionary = { }
var _last_progress_ms : = 0
var _progress_anchor : = Vector2 . ZERO
var _best_route_distance_m : = INF
var _last_measured_pose : = Vector3 . ZERO
var _has_measured_pose : = false
var _measured_translation_m : = 0.0
var _measured_yaw_rad : = 0.0
var _stuck_count : = 0
var _inspect_until_ms : = -1
var _required_objects : Array [ String ] = [ ]
var _evidence_vocabulary : Array [ String ] = [ ]
var _unsupported_conditions : Array [ String ] = [ ]
var _reinspect : = false
var _inspection_pose : = Vector3 . ZERO
var _inspection_hold_started_ms : = -1
var landmark_catalog : Array = [ ]
var _landmark_generation : = 0
var _landmark_epoch : = -1
var _landmark_selection : Dictionary = { }
var _landmark_cooldown: Dictionary = {}
var _semantic_revisits: Dictionary = {}
var _next_view_id := 1

func set_landmark_catalog ( items : Array , task_generation : int , epoch : int ) -> void :

    landmark_catalog . clear ( )
    _landmark_generation = task_generation
    if _landmark_epoch != epoch: _landmark_cooldown.clear()
    _landmark_epoch = epoch
    for value in items :
        if not value is Dictionary or not value . get ( "id" ) is String : continue
        if value . id . length ( ) > 64 or value . get ( "generation" ) != task_generation or value . get ( "epoch" ) != epoch : continue
        if int(_landmark_cooldown.get(str(value.get("landmark_id", "")), -1)) > _observation_ms: continue
        landmark_catalog . append ( value . duplicate ( true ) )
        if landmark_catalog . size ( ) >= 12 : break

func local_motion_blocked(mapper, now_ms: int) -> void:
    if _has_goal: _blacklist[_goal_cell] = now_ms + BLACKLIST_MS
    _clear_goal("fresh_measured_path_blocked")
    invalidate_pending()
    _reinspect = false
    _inspection_hold_started_ms = -1
    _inspect_until_ms = -1
    _last_observe_ms = -100000
    _pose = mapper.estimated_pose
    _reason = "replan_after_measured_clearance_recovery"
    summary = "Local swept-path guard stopped the route; bounded measured-clear recovery"

func can_resume_room_mission() -> bool:
    return not _target_room().is_empty() and _unsupported_conditions.is_empty() and not status in ["found", "blocked"]

func resume_after_landmark(outcome: String, object_id, target: Dictionary, mapper, now_ms: int) -> bool:
    if not outcome in ["arrived", "blocked"] or not can_resume_room_mission(): return false
    # A remembered center only directs the CAMERA from our current safe pose.
    # It never grants route, motor authority or room-completion evidence.
    invalidate_pending()
    _clear_goal("object_" + outcome + "_inspect_current_room")
    _pose = mapper.estimated_pose
    _observation_ms = now_ms
    _landmark_cooldown[str(object_id)] = now_ms + BLACKLIST_MS
    landmark_catalog.clear()
    _reinspect = true
    _inspection_pose = _pose
    if outcome == "arrived" and target.has("x") and target.has("z"):
        var delta := Vector2(float(target.x) - _pose.x, float(target.z) - _pose.y)
        if delta.is_finite() and delta.length() > 0.1:
            _inspection_pose.z = atan2(-delta.x, -delta.y)
    _inspection_hold_started_ms = now_ms
    _inspect_until_ms = now_ms + 20000
    _last_plan_ms = -100000
    status = "inspect"
    summary = "Fresh visual verification at the object viewpoint" if outcome == "arrived" else "Approach stopped; inspect fresh evidence before choosing another route"
    return true

func take_landmark_selection ( ) -> Dictionary :
    var result : = _landmark_selection . duplicate ( true )
    _landmark_selection . clear ( )
    return result

func _init ( ) -> void :
    _parse_requirements ( )

func reset ( new_mission : String , version : int ) -> void :
    exploration_memory = ExplorationMemory.new()
    _semantic_revisits.clear()
    mission = new_mission
    mission_version = version
    status = "search"
    subgoal . clear ( )
    room_memory . clear ( )
    visits . clear ( )
    candidates . clear ( )
    pending = false
    _request . clear ( )
    _last_plan_ms = -100000
    _last_observe_ms = -100000
    _observation_ms = 0
    _known_at_plan = 0
    _known = 0
    _pose = Vector3 . ZERO
    _parents . clear ( )
    _depth . clear ( )
    _root = Vector2i . ZERO
    _goal_cell = Vector2i . ZERO
    _has_goal = false
    _goal_source = ""
    _goal_inspection_yaw = NAN
    _route . clear ( )
    _route_points . clear ( )
    _route_cursor = 0
    _route_revision += 1
    _graph_revision += 1
    _graph_mapper_id = 0
    _blacklist . clear ( )
    _last_progress_ms = 0
    _progress_anchor = Vector2 . ZERO
    _best_route_distance_m = INF
    _last_measured_pose = Vector3 . ZERO
    _has_measured_pose = false
    _measured_translation_m = 0.0
    _measured_yaw_rad = 0.0
    _stuck_count = 0
    _inspect_until_ms = -1
    _reinspect = false
    _inspection_pose = Vector3 . ZERO
    _inspection_hold_started_ms = -1
    landmark_catalog . clear ( )
    _landmark_selection . clear ( )
    _landmark_cooldown.clear()
    _reason = "mission_changed"
    _failed_attempts = 0
    plan_revision += 1
    _parse_requirements ( )
    summary = "Search reachable observed space while checking the current camera view"

func change_mission ( new_mission : String , version : int ) -> void :
    var remembered : = room_memory . duplicate ( true )
    var visited : = visits . duplicate ( true )
    var search_memory = exploration_memory
    search_memory.finish_goal("mission_changed")
    reset ( new_mission , version )
    room_memory = remembered
    visits = visited
    exploration_memory = search_memory

func resume ( ) -> void :
    _failed_attempts = 0
    _reason = "resume_mission"
    _last_plan_ms = -100000
    _last_progress_ms = _observation_ms
    _progress_anchor = Vector2 ( _pose . x , _pose . y )
    if status == "blocked" : status = "search"

func invalidate_pending ( ) -> void :
    pending = false
    _request . clear ( )
    _landmark_selection . clear ( )
    plan_revision += 1

func _free ( mapper , cell : Vector2i ) -> bool :
    if not mapper . is_cell_in_bounds ( cell ) : return false
    var index : int = cell . y * mapper . width + cell . x
    return mapper . observed [ index ] != 0 and mapper . log_odds [ index ] <= mapper . FREE_THRESHOLD

func _safe ( mapper , cell : Vector2i ) -> bool :

    var padded: float = FOOTPRINT_RADIUS_M + mapper.resolution * sqrt(2.0) * 0.5
    var radius: int = ceili(padded / mapper.resolution)
    for y in range ( - radius , radius + 1 ) :
        for x in range ( - radius , radius + 1 ) :
            if Vector2 ( x , y ) . length ( ) * mapper . resolution <= padded + 0.001 and not _free ( mapper , cell + Vector2i ( x , y ) ) : return false
    return true

func observe ( mapper , now_ms : int , _recent_actions : Array = [ ] ) -> void :
    _pose = mapper . estimated_pose
    _known = mapper . known_count

    if now_ms < _observation_ms or ( _graph_mapper_id != 0 and _graph_mapper_id != mapper . get_instance_id ( ) ) :
        _clear_goal ( "observation_epoch_changed" )
        _parents . clear ( )
        _depth . clear ( )
        _blacklist . clear ( )
        _last_observe_ms = -100000
        _has_measured_pose = false
        room_memory.clear()
        visits.clear()
        _landmark_cooldown.clear()
        _reinspect = false
        _inspection_hold_started_ms = -1
        _inspect_until_ms = -1
    _observation_ms = now_ms
    exploration_memory.observe(mapper.get_instance_id(), _known, now_ms, Vector2(_pose.x, _pose.y))
    if _reinspect and _inspection_hold_started_ms >= 0 and now_ms - _inspection_hold_started_ms >= 20000 :
        _reinspect = false
        _inspection_hold_started_ms = -1
        _inspect_until_ms = -1
        _clear_goal ( "visual_reinspection_timed_out" )
    if _reinspect and _at_inspection_position ( ) and _inspection_hold_started_ms < 0 :
        _inspection_hold_started_ms = now_ms
        _inspect_until_ms = now_ms + 20000
    if now_ms - _last_observe_ms < GRAPH_INTERVAL_MS : return
    _last_observe_ms = now_ms
    var point : = Vector2 ( _pose . x , _pose . y )
    _measured_translation_m = 0.0
    _measured_yaw_rad = 0.0
    if _has_measured_pose :
        _measured_translation_m = point . distance_to ( Vector2 ( _last_measured_pose . x , _last_measured_pose . y ) )
        _measured_yaw_rad = wrapf ( _pose . z - _last_measured_pose . z , - PI , PI )
    _last_measured_pose = _pose
    _has_measured_pose = true
    for cell in _blacklist . keys ( ) :
        if now_ms >= int ( _blacklist [ cell ] ) : _blacklist . erase ( cell )
    if visits . is_empty ( ) or point . distance_to ( Vector2 ( visits [ -1 ] . x , visits [ -1 ] . z ) ) > 0.45 :
        visits . append ( { "x" : snappedf ( point . x , 0.01 ) , "z" : snappedf ( point . y , 0.01 ) } )
        if visits . size ( ) > 60 : visits . pop_front ( )
    _build_candidates ( mapper )
    exploration_memory.note_inspection(_measured_yaw_rad, not candidates.is_empty() or _reinspect, _has_goal)
    if _has_goal :
        if not _parents . has ( _goal_cell ) :
            _blacklist [ _goal_cell ] = now_ms + BLACKLIST_MS
            _clear_goal ( "goal_no_longer_reachable" )
        elif point . distance_to ( mapper . cell_to_world ( _goal_cell ) ) < ( 0.18 if _reinspect else 0.3 ) :

            _blacklist [ _goal_cell ] = now_ms + 20000
            if is_finite(_goal_inspection_yaw):
                _reinspect = true
                _inspection_pose = Vector3(point.x, point.y, _goal_inspection_yaw)
                _inspection_hold_started_ms = now_ms
                _inspect_until_ms = now_ms + 20000
            _clear_goal ( "goal_reached_inspect_view" )
        elif float ( _depth [ _goal_cell ] ) * mapper . resolution < _best_route_distance_m - 0.09 :


            _best_route_distance_m = float ( _depth [ _goal_cell ] ) * mapper . resolution
            _progress_anchor = point
            _last_progress_ms = now_ms
        elif now_ms - _last_progress_ms >= STUCK_MS :
            _blacklist [ _goal_cell ] = now_ms + BLACKLIST_MS
            _stuck_count += 1
            _clear_goal ( "stuck_reconsider_route" )
    if _has_goal : _cache_route ( mapper )
    if _known - _known_at_plan > 180 and _reason . is_empty ( ) : _reason = "new_observed_space"

func _clear_goal ( reason : String ) -> void :
    exploration_memory.finish_goal(reason)
    _has_goal = false
    _goal_source = ""
    _goal_inspection_yaw = NAN
    _route . clear ( )
    _route_points . clear ( )
    _route_cursor = 0
    _route_revision += 1
    subgoal . clear ( )
    _reason = reason
    plan_revision += 1

func _is_blacklisted ( mapper , cell : Vector2i ) -> bool :
    for blocked in _blacklist :
        if _observation_ms < int ( _blacklist [ blocked ] ) and Vector2 ( cell - blocked ) . length ( ) * mapper . resolution < 0.7 : return true
    return false

func _build_candidates ( mapper ) -> void :
    candidates . clear ( )
    _parents . clear ( )
    _depth . clear ( )
    _graph_revision += 1
    _graph_mapper_id = mapper . get_instance_id ( )
    _root = mapper . world_to_cell ( Vector2 ( _pose . x , _pose . y ) )
    if not _safe ( mapper , _root ) : return
    var queue : Array [ Vector2i ] = [ _root ]
    _parents [ _root ] = _root
    _depth [ _root ] = 0
    var cursor : = 0
    var safe_cache : = { _root : true }
    while cursor < queue . size ( ) :
        var cell : = queue [ cursor ]
        cursor += 1
        for offset in CARDINALS :
            var next : Vector2i = cell + offset
            if _parents . has ( next ) : continue
            if not safe_cache . has ( next ) : safe_cache [ next ] = _safe ( mapper , next )
            if not safe_cache [ next ] : continue
            _parents [ next ] = cell
            _depth [ next ] = int ( _depth [ cell ] ) + 1
            queue . append ( next )
    var ranked : Array = [ ]
    for cell in queue :
        if posmod ( cell . x , 4 ) != 0 or posmod ( cell . y , 4 ) != 0 or _is_blacklisted ( mapper , cell ) : continue
        var point : Vector2 = mapper . cell_to_world ( cell )
        var distance : = point . distance_to ( Vector2 ( _pose . x , _pose . y ) )
        if distance < 0.5 : continue
        var unknown : = 0
        for offset in [ Vector2i ( 5 , 0 ) , Vector2i ( -5 , 0 ) , Vector2i ( 0 , 5 ) , Vector2i ( 0 , -5 ) , Vector2i ( 4 , 4 ) , Vector2i ( -4 , -4 ) ] :
            var check : Vector2i = cell + offset
            if mapper . is_cell_in_bounds ( check ) and mapper . observed [ check . y * mapper . width + check . x ] == 0 : unknown += 1
        if not exploration_memory.can_attempt(point, unknown): continue
        var visit_penalty : = 0.0
        for visit in visits :
            if point . distance_to ( Vector2 ( visit . x , visit . z ) ) < 0.65 : visit_penalty += 2.0
        var route_distance : float = int ( _depth [ cell ] ) * mapper . resolution
        ranked . append ( { "cell" : cell , "score" : unknown * 3.0 - route_distance * 0.25 - visit_penalty , "frontier" : unknown > 0 , "route_distance" : route_distance, "unknown_count": unknown } )
    var target_room := _target_room()
    if not target_room.is_empty():
        for remembered in room_memory:
            if remembered.room_type != target_room or float(remembered.confidence) < 0.6: continue
            if int(remembered.get("map_id", mapper.get_instance_id())) != mapper.get_instance_id(): continue
            var remembered_point := Vector2(float(remembered.x), float(remembered.z))
            var remembered_cell: Vector2i = mapper.world_to_cell(remembered_point)
            if remembered_point.distance_to(Vector2(_pose.x, _pose.y)) < 0.5: continue
            var memory_key := _room_view_key(remembered)
            if _semantic_revisits.has(memory_key): continue
            if not _parents.has(remembered_cell) or not _safe(mapper, remembered_cell) or _is_blacklisted(mapper, remembered_cell): continue
            ranked.append({"cell": remembered_cell, "score": 24.0 + float(remembered.confidence),
                "frontier": false, "route_distance": float(_depth[remembered_cell]) * mapper.resolution,
                "remembered_view": true, "memory_key": memory_key, "unknown_count": 0, "inspection_yaw_rad": float(remembered.get("yaw_rad", _pose.z))})
    ranked . sort_custom ( func ( a , b ) : return a . score > b . score )
    for item in ranked :
        var point : Vector2 = mapper . cell_to_world ( item . cell )
        var near : = false
        for candidate in candidates :
            if point . distance_to ( Vector2 ( candidate . x , candidate . z ) ) < 0.75 : near = true
        if near : continue
        var delta : = point - Vector2 ( _pose . x , _pose . y )
        candidates . append ( { "id" : "cell_%d_%d" % [ item . cell . x , item . cell . y ] , "annotation_id" : candidates . size ( ) + 1 , "x" : snappedf ( point . x , 0.01 ) , "z" : snappedf ( point . y , 0.01 ) , "kind" : "remembered_camera_viewpoint" if item.get("remembered_view", false) else ("frontier_viewpoint" if item.frontier else "observed_free_viewpoint") , "cell_x" : item . cell . x , "cell_y" : item . cell . y , "route_distance_m" : snappedf ( item . route_distance , 0.01 ) , "memory_key": str(item.get("memory_key", "")), "unknown_count": int(item.get("unknown_count", 0)), "inspection_yaw_rad": float(item.get("inspection_yaw_rad", _pose.z)), "heading_error_rad" : snappedf ( wrapf ( atan2 ( - delta . x , - delta . y ) - _pose . z , - PI , PI ) , 0.01 ) } )
        if candidates . size ( ) >= 12 : break

func ensure_local_goal ( mapper ) -> bool :
    _pose = mapper . estimated_pose
    if status in [ "found" , "blocked" ] : return false
    if _reinspect and _at_inspection_position ( ) : return false
    if _has_goal and not _route . is_empty ( ) : return true
    if _observation_ms < _inspect_until_ms : return false


    if _graph_mapper_id != mapper . get_instance_id ( ) or not current_footprint_safe(mapper): return false
    for candidate in candidates :
        var cell : = Vector2i ( candidate . cell_x , candidate . cell_y )
        if _is_blacklisted ( mapper , cell ) or not _parents . has ( cell ) or not _safe ( mapper , cell ) : continue
        if not _select_goal(candidate, "local_frontier", mapper): continue
        status = "search"
        summary = "Following a reachable observed-free viewpoint" + ( " while semantic analysis is pending" if pending else "" )
        return true
    return false

func _select_goal ( candidate : Dictionary , source : String , mapper ) -> bool :
    var point := Vector2(float(candidate.x), float(candidate.z))
    if _has_goal and not _route.is_empty() and source != "visual_reinspection":
        var committed: Vector2 = mapper.cell_to_world(_goal_cell)
        if committed.distance_to(point) < 0.7 and _parents.has(_goal_cell):
            # A semantic refresh cannot repeatedly erase measured progress/stuck age.
            return true
    if source == "visual_reinspection":
        exploration_memory.finish_goal("visual_verification_handoff")
    elif candidate.get("kind", "") == "remembered_camera_viewpoint":
        var key := str(candidate.get("memory_key", ""))
        if key.is_empty() or _semantic_revisits.has(key): return false
        exploration_memory.finish_goal("semantic_memory_reinspection")
        _semantic_revisits[key] = true
    else:
        if source == "semantic" and _has_goal and Vector2(_goal_cell - Vector2i(candidate.cell_x, candidate.cell_y)).length() * mapper.resolution >= 0.7:
            exploration_memory.finish_goal("new_semantic_goal")
        if not exploration_memory.begin_goal(point, int(candidate.get("unknown_count", 1))): return false
    _goal_cell = Vector2i ( candidate . cell_x , candidate . cell_y )
    _has_goal = true
    _goal_source = source
    _goal_inspection_yaw = float(candidate.get("inspection_yaw_rad", NAN)) if candidate.get("kind", "") == "remembered_camera_viewpoint" else NAN
    _progress_anchor = Vector2 ( _pose . x , _pose . y )
    _last_progress_ms = _observation_ms
    _best_route_distance_m = float ( _depth . get ( _goal_cell , 0 ) ) * mapper . resolution
    subgoal = { "goal_id" : candidate . id , "status" : "search" , "source" : source , "x" : candidate . x , "z" : candidate . z , "kind" : candidate . kind }
    plan_revision += 1
    _cache_route ( mapper )
    return true

func _cache_route ( mapper ) -> void :
    _route . clear ( )
    _route_points . clear ( )
    _route_cursor = 0
    _route_revision += 1
    if not _has_goal or not _parents . has ( _goal_cell ) : return
    var reverse_route : Array [ Vector2i ] = [ _goal_cell ]
    var cell : = _goal_cell
    while cell != _root and reverse_route . size ( ) <= mapper . width * mapper . height :
        if not _parents . has ( cell ) : return
        cell = _parents [ cell ]
        reverse_route . append ( cell )
    if cell != _root : return
    reverse_route . reverse ( )
    _route = reverse_route
    for route_cell in _route :
        var point : Vector2 = mapper . cell_to_world ( route_cell )
        _route_points . append ( { "x" : point . x , "z" : point . y } )

func needs_plan ( now_ms : int ) -> bool :
    if pending or status == "found" or _failed_attempts >= 3 : return false
    if exploration_memory.search_exhausted() and not _reinspect and not _has_goal: return false
    if _reinspect and ( not _at_inspection_position ( ) or absf ( wrapf ( _inspection_pose . z - _pose . z , - PI , PI ) ) >= 0.12 ) : return false
    if now_ms - _last_plan_ms < MIN_INTERVAL_MS : return false
    return not _reason . is_empty ( ) or now_ms - _last_plan_ms >= REASSESS_MS

func begin_request ( session : String , sequence : int , captured : int ) -> Dictionary :
    pending = true
    _last_plan_ms = captured
    _request = { "session_id" : session , "observation_seq" : sequence , "captured_at_ms" : captured , "mission_version" : mission_version , "candidates" : candidates . duplicate ( true ) , "pose" : _pose , "landmarks" : landmark_catalog . duplicate ( true ) , "task_generation" : _landmark_generation , "landmark_epoch" : _landmark_epoch }
    var context : = { "mission" : mission , "mission_version" : mission_version , "required_objects" : _required_objects . duplicate ( ) , "unsupported_completion_conditions" : _unsupported_conditions . duplicate ( ) , "completion_supported" : _unsupported_conditions . is_empty ( ) , "target_room" : _target_room ( ) , "same_current_view_required" : true , "trigger" : _reason , "current_plan" : subgoal . duplicate ( true ) , "room_hypotheses" : _room_context() , "visited_odometry_positions" : visits . duplicate ( true ) , "candidates" : candidates . duplicate ( true ) , "observation_seq" : sequence , "estimated_pose" : { "x" : _pose . x , "z" : _pose . y , "yaw_rad" : _pose . z } , "map_annotation_legend" : "Map is observed lidar occupancy only: red robot arrow is estimated pose/heading, blue line is cached observed-free route, orange numbered badges match candidate annotation_id. Columns +X, rows +Z; yaw 0 faces up, positive yaw turns left. No room labels or house truth." , "route" : _route_description ( ) , "policy" : "Choose only a supplied goal_id or look_here. Candidate coordinates are reachable observed-free positions, not room labels. Classify current robot POV only; previous hypotheses are uncertain. Search unvisited frontier viewpoints before repeating visited positions. Local observed-free motion continues while this request is pending. Use inspect/look_here when rotating could resolve evidence. Found requires confidence >= 0.8, the requested room type, and EVERY required_objects item visibly present together in this one current camera frame. A bedroom with a desk requires both bed and desk now; a bed alone never completes it. If completion_supported is false, do not return found; explain unsupported conditions and inspect or continue searching. Never combine evidence across frames, infer completion from coverage, or invent unseen rooms/objects." }
    _reason = ""
    _known_at_plan = _known
    context["exploration"] = exploration_memory.status_context()
    context [ "object_landmarks" ] = landmark_catalog . duplicate ( true )
    context [ "landmark_task_generation" ] = _landmark_generation
    context [ "landmark_map_epoch" ] = _landmark_epoch
    context [ "policy" ] += " You may also choose the exact id of an object_landmarks entry as goal_id with search or inspect. These are uncertain measured object hypotheses with checked stand-off viewpoints, not proven static objects or scene truth. Choosing one requests a locally revalidated, abortable observed-free A* approach; never invent coordinates or IDs, target the object center, or treat remembered landmarks as fresh visual proof of mission completion."
    return context

func response_schema ( ) -> Dictionary :
    return { "type" : "object" , "additionalProperties" : false , "required" : [ "status" , "goal_id" , "room_type" , "confidence" , "evidence" , "summary" ] , "properties" : { "status" : { "type" : "string" , "enum" : [ "search" , "inspect" , "found" , "blocked" ] } , "goal_id" : { "type" : "string" , "maxLength" : 64 } , "room_type" : { "type" : "string" , "enum" : ROOM_TYPES } , "confidence" : { "type" : "number" , "minimum" : 0 , "maximum" : 1 } , "evidence" : { "type" : "array" , "maxItems" : _evidence_vocabulary . size ( ) , "items" : { "type" : "string" , "enum" : _evidence_vocabulary } } , "summary" : { "type" : "string" , "maxLength" : 180 } } }

func fail_request ( reason : String = "Planner response unavailable or rejected" ) -> void :
    invalidate_pending ( )
    _failed_attempts += 1
    _reason = "retry_after_failure"
    summary = reason . left ( 140 ) + ( " · semantic retries paused; local search continues" if _failed_attempts >= 3 else " · retry pending" )

func accept_plan ( envelope : Dictionary , current_session : String , now_ms : int , mapper ) -> bool :
    if not pending : return false
    for key in [ "session_id" , "mission_version" , "observation_seq" , "captured_at_ms" ] :
        if envelope . get ( key ) != _request . get ( key ) : return false
    if current_session != _request . session_id or mission_version != _request . mission_version : return false
    if now_ms < int ( _request . captured_at_ms ) or now_ms - int ( _request . captured_at_ms ) > MAX_RESPONSE_MS :
        fail_request ( )
        return false
    var plan = envelope . get ( "plan" )
    if not plan is Dictionary or plan . size ( ) != 6 :
        fail_request ( )
        return false
    if not plan . get ( "status" ) in [ "search" , "inspect" , "found" , "blocked" ] or not plan . get ( "room_type" ) in ROOM_TYPES or not plan . get ( "summary" ) is String or not plan . get ( "goal_id" ) is String :
        fail_request ( )
        return false
    var confidence = plan . get ( "confidence" )
    if not ( confidence is int or confidence is float ) or not is_finite ( float ( confidence ) ) or confidence < 0 or confidence > 1 or not plan . get ( "evidence" ) is Array :
        fail_request ( )
        return false
    if plan . evidence . size ( ) > _evidence_vocabulary . size ( ) or plan . summary . length ( ) > 180 or plan . goal_id . length ( ) > 64 :
        fail_request ( "Planner output exceeded bounded contract" )
        return false
    var unique_evidence : = { }
    for evidence in plan . evidence :
        if not evidence is String or unique_evidence . has ( evidence ) or not evidence in _evidence_vocabulary :
            fail_request ( "Planner evidence outside the current mission contract" )
            return false
        unique_evidence [ evidence ] = true

    _pose = mapper . estimated_pose
    _build_candidates ( mapper )
    var chosen : Dictionary = { }
    var chosen_landmark : Dictionary = { }
    if plan . goal_id != "look_here" :
        for candidate in _request . candidates :
            if candidate . id == plan . goal_id : chosen = candidate
        if chosen . is_empty ( ) :
            for reference in _request . get ( "landmarks" , [ ] ) :
                if reference . id == plan . goal_id : chosen_landmark = reference
            if chosen_landmark . is_empty ( ) :
                fail_request ( "Unknown viewpoint or landmark reference" )
                return false
            if not plan . status in [ "search" , "inspect" ] or _request . get ( "task_generation" ) != _landmark_generation or _request . get ( "landmark_epoch" ) != _landmark_epoch :
                fail_request ( "Landmark selection belongs to an aborted or superseded task" )
                return false
            var current_reference : = false
            for reference in landmark_catalog :
                if reference . id == chosen_landmark . id and reference . get ( "landmark_id" ) == chosen_landmark . get ( "landmark_id" ) : current_reference = true
            if not current_reference :
                fail_request ( "Landmark is no longer a current reachable reference" )
                return false
        else :
            var cell : = Vector2i ( chosen . cell_x , chosen . cell_y )
            if not _safe ( mapper , cell ) or not _parents . has ( cell ) or _is_blacklisted ( mapper , cell ) or not _candidate_attempt_allowed(chosen):
                fail_request ( "Requested viewpoint is no longer reachable or was recently stuck" )
                return false
    var snapshot_pose : Vector3 = _request . pose
    var current_pose : Vector3 = mapper . estimated_pose
    var view_still_current : = Vector2 ( current_pose . x , current_pose . y ) . distance_to ( Vector2 ( snapshot_pose . x , snapshot_pose . y ) ) < 0.25 and absf ( wrapf ( current_pose . z - snapshot_pose . z , - PI , PI ) ) < 0.35
    var target : = _target_room ( )
    var all_objects : = not _required_objects . is_empty ( ) and _unsupported_conditions . is_empty ( )
    for required_object in _required_objects :
        if not plan . evidence . has ( required_object ) : all_objects = false
    var verified_found : bool = plan . status == "found" and ( target . is_empty ( ) or plan . room_type == target ) and confidence >= 0.8 and all_objects and view_still_current
    if plan . status == "found" and all_objects and confidence >= 0.8 and ( target . is_empty ( ) or plan . room_type == target ) and not view_still_current :
        _queue_reinspection ( snapshot_pose , mapper )
        return false
    if plan . status == "found" and not verified_found :
        fail_request ( "Completion needs all requested objects together in the current view" )
        return false
    if plan . room_type != "unknown" and not plan . evidence . is_empty ( ) :
        _remember_room_view(plan, snapshot_pose, _request, mapper.get_instance_id())
    pending = false
    _failed_attempts = 0
    _reinspect = false
    _inspection_hold_started_ms = -1
    _inspect_until_ms = -1
    plan_revision += 1
    status = plan . status
    summary = plan . summary . left ( 180 )
    if status in [ "found" , "blocked" ] or chosen . is_empty ( ) :
        exploration_memory.finish_goal("semantic_" + status)
        _has_goal = false
        _goal_source = ""
        _route . clear ( )
        _route_points . clear ( )
        _route_cursor = 0
    elif not chosen . is_empty ( ) :
        if not _select_goal(chosen, "semantic", mapper):
            fail_request("Viewpoint already examined without new measured information")
            return false
    subgoal = { "goal_id" : plan . goal_id , "status" : status , "summary" : summary , "room_type" : plan . room_type , "visual_evidence" : plan . evidence . duplicate ( ) , "confidence" : confidence , "source" : "semantic" }
    if not chosen_landmark . is_empty ( ) :
        _landmark_selection = { "landmark_id" : chosen_landmark . landmark_id , "generation" : _request . task_generation , "epoch" : _request . landmark_epoch }
        subgoal [ "source" ] = "estimated_landmark_approach"
        subgoal [ "landmark_id" ] = chosen_landmark . landmark_id
    if _has_goal :
        subgoal [ "x" ] = chosen . x
        subgoal [ "z" ] = chosen . z
    if status == "inspect" and chosen . is_empty ( ) and chosen_landmark . is_empty ( ) :
        _inspect_until_ms = _observation_ms + 2000
        _reason = "inspect_current_view"
    _request . clear ( )
    return true

func _room_view_key(view: Dictionary) -> String:
    return str(view.get("view_id", str(view.room_type) + "_%d_%d" % [roundi(float(view.x) / 0.8), roundi(float(view.z) / 0.8)]))

func _candidate_attempt_allowed(candidate: Dictionary) -> bool:
    if candidate.get("kind", "") == "remembered_camera_viewpoint":
        var key := str(candidate.get("memory_key", ""))
        return not key.is_empty() and (not _semantic_revisits.has(key) or (_has_goal and _goal_cell == Vector2i(candidate.cell_x, candidate.cell_y)))
    return exploration_memory.can_attempt(Vector2(float(candidate.x), float(candidate.z)), int(candidate.get("unknown_count", 1)))

func _room_context() -> Array:
    var result: Array = []
    for index in range(room_memory.size() - 1, -1, -1):
        var view: Dictionary = room_memory[index]
        result.append({"room_type":view.room_type, "confidence":view.confidence,
            "evidence":view.evidence.duplicate(), "x":view.x, "z":view.z,
            "yaw_rad":view.get("yaw_rad", 0.0), "observed_seq":view.observed_seq,
            "kind":"remembered_viewpoint_requires_current_verification"})
        if result.size() >= 8: break
    return result

func _remember_room_view(plan: Dictionary, snapshot_pose: Vector3, observation: Dictionary, map_id: int) -> void:
    # Store complete observations, never accumulate objects into false same-frame proof.
    var remembered := {"room_type": plan.room_type, "confidence": plan.confidence,
        "evidence": plan.evidence.duplicate(), "observed_seq": observation.observation_seq,
        "captured_at_ms": observation.captured_at_ms, "x": snapshot_pose.x,
        "z": snapshot_pose.y, "yaw_rad": snapshot_pose.z, "map_id": map_id,
        "kind": "remembered_camera_viewpoint_not_room_boundary", "observations": 1, "view_id":"room_view_" + str(_next_view_id)}
    _next_view_id += 1
    for index in range(room_memory.size() - 1, -1, -1):
        var previous: Dictionary = room_memory[index]
        if previous.get("map_id", map_id) != map_id: continue
        if previous.room_type != plan.room_type: continue
        var distance := Vector2(float(previous.x), float(previous.z)).distance_to(Vector2(snapshot_pose.x, snapshot_pose.y))
        if distance <= 0.8 and absf(wrapf(float(previous.get("yaw_rad", snapshot_pose.z)) - snapshot_pose.z, -PI, PI)) <= 0.7:
            remembered.observations = int(previous.get("observations", 1)) + 1
            remembered.view_id = _room_view_key(previous)
            room_memory.remove_at(index)
            break
    room_memory.append(remembered)
    if room_memory.size() > 24: room_memory.pop_front()

func _at_inspection_position ( ) -> bool :
    return Vector2 ( _pose . x , _pose . y ) . distance_to ( Vector2 ( _inspection_pose . x , _inspection_pose . y ) ) < 0.24

func _queue_reinspection ( snapshot_pose : Vector3 , mapper ) -> void :


    var cell : Vector2i = mapper . world_to_cell ( Vector2 ( snapshot_pose . x , snapshot_pose . y ) )
    pending = false
    _request . clear ( )
    if not _parents . has ( cell ) or not _safe ( mapper , cell ) or _is_blacklisted ( mapper , cell ) :
        _reason = "visual_target_capture_position_unreachable"
        summary = "Possible target in an earlier view; current safe route cannot return to verify it"
        return
    _reinspect = true
    _inspection_pose = snapshot_pose
    _inspection_hold_started_ms = -1
    _inspect_until_ms = -1
    var point : Vector2 = mapper . cell_to_world ( cell )
    _select_goal ( { "id" : "reinspect_%d_%d" % [ cell . x , cell . y ] , "cell_x" : cell . x , "cell_y" : cell . y , "x" : point . x , "z" : point . y , "kind" : "observed_capture_viewpoint" } , "visual_reinspection" , mapper )
    status = "search"
    _reason = "reinspect_recent_visual_target"
    summary = "Possible target in an earlier view; returning to its measured viewpoint for fresh confirmation"

func _target_room ( ) -> String :
    var text : = mission . to_lower ( ) . replace ( "_" , " " )
    for target in [ "bedroom" , "living room" , "kitchen" , "bathroom" , "hallway" , "office" , "dining room" , "utility room" ] :
        if target in text : return target . replace ( " " , "_" )
    return ""

func _parse_requirements ( ) -> void :
    _required_objects . clear ( )
    _unsupported_conditions . clear ( )
    _evidence_vocabulary . assign ( EVIDENCE )
    var target : = _target_room ( )
    if IDENTIFYING . has ( target ) : _required_objects . append ( IDENTIFYING [ target ] )
    var text : = mission . to_lower ( ) . replace ( "_" , " " )
    var words : = RegEx . new ( )
    words . compile ( "[a-z0-9]+" )
    var tokens : Array [ String ] = [ ]
    for match in words . search_all ( text ) : tokens . append ( match . get_string ( ) )
    var normalized : = " " + " " . join ( tokens ) + " "
    for object_name in EVIDENCE :
        if object_name == "no_identifying_feature" : continue
        var phrase : String = object_name . replace ( "_" , " " )
        if " " + phrase + " " in normalized or " " + phrase + "s " in normalized : _add_requirement ( object_name )
    for alias in OBJECT_ALIASES :
        if " " + alias + " " in normalized or " " + alias + "s " in normalized : _add_requirement ( OBJECT_ALIASES [ alias ] )



    var list_start : = RegEx . new ( )
    list_start . compile ( "\\b(?:with|containing|contains|has|including|includes)\\s+(.+)$" )
    var listing : = list_start . search ( text )
    var list_text : = ""
    if listing != null :
        list_text = listing . get_string ( 1 )
    elif target . is_empty ( ) :
        var prefix : = RegEx . new ( )
        prefix . compile ( "^(?:please\\s+)?(?:find|locate|search for|look for)\\s+" )
        list_text = prefix . sub ( text , "" )
    if not list_text . is_empty ( ) :
        var separators : = RegEx . new ( )
        separators . compile ( "\\s+(?:and|or)\\s+|[,;+&]" )
        for part in separators . sub ( list_text , "|" , true ) . split ( "|" ) :
            var noun_words : Array [ String ] = [ ]
            for match in words . search_all ( part ) :
                var word : = match . get_string ( )
                if not word in [ "a" , "an" , "the" , "some" , "one" , "visible" , "nearby" , "please" ] : noun_words . append ( word )
            var noun : = " " . join ( noun_words )
            if noun . is_empty ( ) : continue
            if OBJECT_ALIASES . has ( noun ) : noun = OBJECT_ALIASES [ noun ]
            else : noun = noun . replace ( " " , "_" )
            if noun . ends_with ( "s" ) and EVIDENCE . has ( noun . left ( -1 ) ) : noun = noun . left ( -1 )
            if not noun in ROOM_TYPES :
                if noun . length ( ) <= 48 : _add_requirement ( noun )
                else : _unsupported_conditions . append ( "Object description exceeds literal evidence-token limit" )
    var negative : = RegEx . new ( )
    negative . compile ( "\\b(?:without|no|not|never|except|neither|none|only|exactly|at least|at most|more than|less than)\\b" )
    if negative . search ( text ) != null : _unsupported_conditions . append ( "Negation or quantified restriction needs a richer visual verification contract" )


    var prefix_text : = text
    if listing != null : prefix_text = text . left ( listing . get_start ( ) )
    elif target . is_empty ( ) : prefix_text = ""
    for room in ROOM_TYPES : prefix_text = prefix_text . replace ( room . replace ( "_" , " " ) , " " )
    var permitted : = [ "find" , "locate" , "search" , "look" , "for" , "a" , "an" , "the" , "please" , "can" , "you" , "me" , "help" , "to" , "go" , "room" ]
    for match in words . search_all ( prefix_text ) :
        if not match . get_string ( ) in permitted : _unsupported_conditions . append ( "Unparsed room qualifier: " + match . get_string ( ) )

func _add_requirement ( object_name : String ) -> void :
    if not _required_objects . has ( object_name ) : _required_objects . append ( object_name )
    if not _evidence_vocabulary . has ( object_name ) : _evidence_vocabulary . append ( object_name )

func _route_description ( ) -> Dictionary :
    return { "source" : _goal_source , "revision" : _route_revision , "graph_revision" : _graph_revision , "cell_count" : _route . size ( ) , "cursor" : _route_cursor , "observed_free_only" : true , "lookahead_m" : LOOKAHEAD_M , "blacklisted_goal_count" : _blacklist . size ( ) , "stuck_replans" : _stuck_count , "last_measured_translation_m" : _measured_translation_m , "last_measured_yaw_delta_rad" : _measured_yaw_rad }

func controller_context ( mapper ) -> Dictionary :


    _pose = mapper . estimated_pose
    var context : = { "mission" : mission , "planner_status" : status , "subgoal" : subgoal . duplicate ( true ) , "plan_revision" : plan_revision , "pending_replan" : pending , "room_hypotheses" : room_memory . slice ( -4 ) , "allow_mission_complete" : status == "found" , "navigation_valid" : false , "estimated_pose" : { "x" : _pose . x , "z" : _pose . y , "yaw_rad" : _pose . z } , "required_objects" : _required_objects . duplicate ( ) , "unsupported_completion_conditions" : _unsupported_conditions . duplicate ( ) , "completion_supported" : _unsupported_conditions . is_empty ( ) , "inspection_heading_rad" : _inspection_pose . z , "hold_for_visual_confirmation" : _reinspect and _at_inspection_position ( ) , "route" : _route_description ( ) , "route_points" : _route_points , "instruction" : "Follow the cached observed-free route with measured local lidar guards. Visual hypotheses are uncertain. Completion requires all mission objects in the current view." }
    context["search_exhausted"] = exploration_memory.search_exhausted() and not _reinspect and not _has_goal
    context["exploration"] = exploration_memory.status_context()
    if not _has_goal or _route . is_empty ( ) or _graph_mapper_id != mapper . get_instance_id ( ) :
        context [ "instruction" ] = "Stop if found or blocked. Otherwise rotate safely to inspect and discover space; translate only with a grounded route."
        return context
    var point : = Vector2 ( _pose . x , _pose . y )

    var nearest : = _route_cursor
    var nearest_distance : float = point . distance_to ( mapper . cell_to_world ( _route [ nearest ] ) )
    for index in range ( _route_cursor + 1 , mini ( _route . size ( ) , _route_cursor + 9 ) ) :
        var distance : float = point . distance_to ( mapper . cell_to_world ( _route [ index ] ) )
        if distance <= nearest_distance :
            nearest = index
            nearest_distance = distance
    _route_cursor = nearest
    var next : = mini ( _route . size ( ) - 1 , _route_cursor + maxi ( 1 , int ( LOOKAHEAD_M / mapper . resolution ) ) )


    while next > _route_cursor and not _segment_safe ( mapper , point , mapper . cell_to_world ( _route [ next ] ) ) : next -= 1
    if not _segment_safe ( mapper , point , mapper . cell_to_world ( _route [ next ] ) ) or next == _route_cursor and nearest_distance < 0.025 and next < _route . size ( ) - 1 :
        _blacklist [ _goal_cell ] = _observation_ms + BLACKLIST_MS
        _clear_goal ( "route_changed" )
        context [ "instruction" ] = "Stop; measured route changed and requires a fresh reachable waypoint."
        return context
    var waypoint : Vector2 = mapper . cell_to_world ( _route [ next ] )
    var delta : = waypoint - point
    context [ "navigation_valid" ] = true
    context [ "waypoint" ] = { "x" : waypoint . x , "z" : waypoint . y , "distance_m" : delta . length ( ) , "heading_error_rad" : wrapf ( atan2 ( - delta . x , - delta . y ) - _pose . z , - PI , PI ) , "positive_heading" : "turn_left" , "remaining_route_m" : maxf ( 0.0 , ( _route . size ( ) - 1 - _route_cursor ) * mapper . resolution ) , "goal_distance_m" : point . distance_to ( mapper . cell_to_world ( _goal_cell ) ) }
    context [ "route" ] [ "cursor" ] = _route_cursor
    return context

func _segment_safe(mapper, start: Vector2, finish: Vector2) -> bool:
    return GridMotion.segment_safe(mapper,start,finish,func(cell: Vector2i): return _safe(mapper,cell))

func command_path_safe(mapper, speed: float, yaw_rate: float, duration: float = 0.35) -> bool:
    return GridMotion.command_safe(mapper,mapper.estimated_pose,speed,yaw_rate,duration,func(cell: Vector2i): return _safe(mapper,cell))

func current_footprint_safe(mapper) -> bool:
    var pose: Vector3 = mapper.estimated_pose
    return pose.is_finite() and _safe(mapper,mapper.world_to_cell(Vector2(pose.x,pose.y)))

func annotated_map ( mapper ) -> Image :


    var image : Image = mapper . get_image ( ) . duplicate ( )
    var scale : = 2
    image . resize ( mapper . width * scale , mapper . height * scale , Image . INTERPOLATE_NEAREST )
    var blue : = Color ( "2278e6" )
    for index in range ( 1 , _route . size ( ) ) :
        _draw_line ( image , _pixel_for_cell ( _route [ index - 1 ] , scale ) , _pixel_for_cell ( _route [ index ] , scale ) , blue , 1 )
    for candidate in candidates :
        var center : = _pixel_for_cell ( Vector2i ( candidate . cell_x , candidate . cell_y ) , scale )
        var label : = str ( candidate . annotation_id )
        var half_width : = maxi ( 5 , label . length ( ) * 4 )
        _draw_rect ( image , Rect2i ( center - Vector2i ( half_width , 5 ) , Vector2i ( half_width * 2 + 1 , 11 ) ) , Color ( "ee9b28" ) )
        _draw_number ( image , label , center - Vector2i ( label . length ( ) * 4 - 1 , 4 ) , Color ( "152638" ) )
    var pose : Vector3 = mapper . estimated_pose
    var robot : = Vector2i ( roundi ( ( pose . x - mapper . origin . x ) / mapper . resolution * scale ) , roundi ( ( pose . y - mapper . origin . y ) / mapper . resolution * scale ) )
    var forward : = Vector2 ( - sin ( pose . z ) , - cos ( pose . z ) )
    var tip : = robot + Vector2i ( forward * 17.0 )
    var red : = Color ( "e13535" )
    _draw_rect ( image , Rect2i ( robot - Vector2i ( 3 , 3 ) , Vector2i ( 7 , 7 ) ) , red )
    _draw_line ( image , robot , tip , red , 2 )
    _draw_line ( image , tip , tip + Vector2i ( forward . rotated ( 2.5 ) * 8.0 ) , red , 1 )
    _draw_line ( image , tip , tip + Vector2i ( forward . rotated ( -2.5 ) * 8.0 ) , red , 1 )
    return image

func _pixel_for_cell ( cell : Vector2i , scale : int ) -> Vector2i :
    return cell * scale + Vector2i ( scale / 2 , scale / 2 )

func _draw_rect ( image : Image , area : Rect2i , color : Color ) -> void :
    var clipped : = area . intersection ( Rect2i ( 0 , 0 , image . get_width ( ) , image . get_height ( ) ) )
    for y in range ( clipped . position . y , clipped . end . y ) :
        for x in range ( clipped . position . x , clipped . end . x ) : image . set_pixel ( x , y , color )

func _draw_line ( image : Image , start : Vector2i , finish : Vector2i , color : Color , radius : int ) -> void :
    var steps : = maxi ( abs ( finish . x - start . x ) , abs ( finish . y - start . y ) )
    for step in range ( steps + 1 ) :
        var point : = Vector2i ( Vector2 ( start ) . lerp ( Vector2 ( finish ) , float ( step ) / maxi ( 1 , steps ) ) )
        _draw_rect ( image , Rect2i ( point - Vector2i ( radius , radius ) , Vector2i ( radius * 2 + 1 , radius * 2 + 1 ) ) , color )

func _draw_number ( image : Image , number : String , start : Vector2i , color : Color ) -> void :
    for index in range ( number . length ( ) ) :
        var bitmap : String = DIGITS [ number . substr ( index , 1 ) . to_int ( ) ]
        for y in range ( 5 ) :
            for x in range ( 3 ) :
                if bitmap [ y * 3 + x ] == "1" : _draw_rect ( image , Rect2i ( start + Vector2i ( index * 8 + x * 2 , y * 2 ) , Vector2i ( 2 , 2 ) ) , color )
