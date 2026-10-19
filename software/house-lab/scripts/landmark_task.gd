extends RefCounted
const GridMotion = preload("res://scripts/grid_motion.gd")





const FOOTPRINT_RADIUS_M : = 0.28
const MAX_AGE_S : = 5.0
const MAX_UNCERTAINTY_M : = 0.6
const MAX_STUCK_S : = 10.0
const LOOKAHEAD_M : = 0.35
const CARDINALS : = [ Vector2i . LEFT , Vector2i . RIGHT , Vector2i . UP , Vector2i . DOWN ]
const STATIC_LABELS : = [ "bed" , "couch" , "chair" , "dining table" , "refrigerator" , "tv" , "toilet" , "oven" , "sink" , "microwave" , "potted plant" ]
var generation : = 0
var status : = "idle"
var reason : = ""
var landmark_id = null
var map_epoch : = -1
var target : Dictionary = { }
var route : Array [ Vector2i ] = [ ]
var route_points : Array = [ ]
var approach : = Vector2 . ZERO
var _mapper_id : = 0
var _cursor : = 0
var _gesture_paused : = false
var _resume_pending : = false
var _initial_target_point : = Vector2 . ZERO
var _initial_target_uncertainty : = 0.0
var _last_now : = -1.0
var _last_progress : = 0.0
var _best_remaining : = INF
var _last_scan_count : = -1
var _safe_cache : Dictionary = { }

func _number ( value ) -> bool :
    return ( value is int or value is float ) and is_finite ( float ( value ) )

func _valid_mapper ( mapper ) -> bool :
    return mapper != null and mapper . width > 0 and mapper . height > 0 and mapper . resolution > 0 and is_finite ( mapper . resolution ) and mapper . estimated_pose . is_finite ( ) and mapper . observed . size ( ) == mapper . width * mapper . height and mapper . log_odds . size ( ) == mapper . width * mapper . height

func _age ( item : Dictionary , now_s : float ) -> float :
    if not _number ( item . get ( "last_seen_s" ) ) : return INF
    return now_s - float ( item . last_seen_s )

func _eligible ( item : Dictionary , now_s : float ) -> bool :
    if not is_finite ( now_s ) or now_s < 0 : return false
    if not item . get ( "label" , "" ) in STATIC_LABELS : return false

    if item . get ( "status" , "" ) != "confirmed" : return false
    if not ( item . get ( "id" ) is int or item . get ( "id" ) is float or item . get ( "id" ) is String ) : return false
    if str ( item . id ) . is_empty ( ) or str ( item . id ) . length ( ) > 64 : return false
    for key in [ "x" , "z" , "uncertainty_m" , "object_extent_m" , "observations" , "baseline_m" , "parallax_deg" ] :
        if not _number ( item . get ( key ) ) : return false
    if absf ( float ( item . x ) ) > 10000 or absf ( float ( item . z ) ) > 10000 : return false
    if float ( item . uncertainty_m ) < 0.1 or float ( item . uncertainty_m ) > MAX_UNCERTAINTY_M : return false
    if float ( item . object_extent_m ) <= 0 or float ( item . object_extent_m ) > 5 : return false
    if float ( item . observations ) < 6 or float ( item . baseline_m ) < 0.35 or float ( item . parallax_deg ) < 5 : return false
    var age : = _age ( item , now_s )
    return age >= 0 and age <= MAX_AGE_S and str ( item . get ( "source" , "" ) ) . begins_with ( "multiview_bearings" )

func _find_landmark ( items : Array , requested_id , now_s : float , epoch : int = -1 ) -> Dictionary :
    var found : Dictionary = { }
    for value in items :
        if not value is Dictionary or str ( value . get ( "id" , "" ) ) != str ( requested_id ) : continue
        if epoch >= 0 and value . has ( "epoch" ) and value . epoch != epoch : return { }

        if not found . is_empty ( ) or not _eligible ( value , now_s ) : return { }
        found = value . duplicate ( true )
    return found

func _exclusion ( item : Dictionary ) -> float :
    return maxf ( 0.5 , float ( item . object_extent_m ) * 0.5 + float ( item . uncertainty_m ) + FOOTPRINT_RADIUS_M + 0.15 )

func _inner ( item : Dictionary ) -> float :
    return maxf ( 1.1 , _exclusion ( item ) + 0.2 )

func _free ( mapper , cell : Vector2i ) -> bool :
    if not mapper . is_cell_in_bounds ( cell ) : return false
    var index : int = cell . y * mapper . width + cell . x
    return mapper . observed [ index ] != 0 and is_finite ( mapper . log_odds [ index ] ) and mapper . log_odds [ index ] <= mapper . FREE_THRESHOLD

func _safe ( mapper , cell : Vector2i , item : Dictionary ) -> bool :
    if _safe_cache . has ( cell ) : return bool ( _safe_cache [ cell ] )
    var center : Vector2 = mapper . cell_to_world ( cell )
    var point : = Vector2 ( float ( item . x ) , float ( item . z ) )
    var half_cell : float = mapper . resolution * sqrt ( 2.0 ) * 0.5
    if center . distance_to ( point ) < _exclusion ( item ) + half_cell :
        _safe_cache [ cell ] = false
        return false

    var padded : float = FOOTPRINT_RADIUS_M + half_cell
    var radius : int = ceili ( padded / mapper . resolution )
    for row in range ( - radius , radius + 1 ) :
        for column in range ( - radius , radius + 1 ) :
            if Vector2 ( column , row ) . length ( ) * mapper . resolution <= padded and not _free ( mapper , cell + Vector2i ( column , row ) ) :
                _safe_cache [ cell ] = false
                return false
    _safe_cache [ cell ] = true
    return true

func _visible_approach ( mapper , point : Vector2 , item : Dictionary ) -> bool :


    var object_point : = Vector2 ( float ( item . x ) , float ( item . z ) )
    var delta : = object_point - point
    var stop_distance : float = float ( item . object_extent_m ) * 0.5 + float ( item . uncertainty_m )
    var length : = maxf ( 0.0 , delta . length ( ) - stop_distance )
    var count : = maxi ( 1 , ceili ( length / ( mapper . resolution * 0.4 ) ) )
    for index in range ( count + 1 ) :
        var check : = point + delta . normalized ( ) * length * float ( index ) / count
        if not _free ( mapper , mapper . world_to_cell ( check ) ) : return false
    return true

func _goal ( mapper , cell : Vector2i , item : Dictionary ) -> bool :
    var point : Vector2 = mapper . cell_to_world ( cell )
    var distance : = point . distance_to ( Vector2 ( float ( item . x ) , float ( item . z ) ) )
    return distance >= _inner ( item ) and distance <= _inner ( item ) + 0.45 and _visible_approach ( mapper , point , item )

func _heuristic ( mapper , cell : Vector2i , item : Dictionary ) -> float :
    var distance : float = mapper . cell_to_world ( cell ) . distance_to ( Vector2 ( float ( item . x ) , float ( item . z ) ) )


    return maxf ( 0.0 , maxf ( _inner ( item ) - distance , distance - _inner ( item ) - 0.45 ) ) / mapper . resolution

func _heap_push ( heap : Array , entry : Dictionary ) -> void :
    heap . append ( entry )
    var index : = heap . size ( ) - 1
    while index > 0 :
        var parent : = ( index - 1 ) / 2
        if float ( heap [ parent ] . f ) <= float ( heap [ index ] . f ) : break
        var swap = heap [ parent ]
        heap [ parent ] = heap [ index ]
        heap [ index ] = swap
        index = parent

func _heap_pop ( heap : Array ) -> Dictionary :
    var result : Dictionary = heap [ 0 ]
    var last = heap . pop_back ( )
    if heap . is_empty ( ) : return result
    heap [ 0 ] = last
    var index : = 0
    while true :
        var left : = index * 2 + 1
        if left >= heap . size ( ) : break
        var right : = left + 1
        var child : = right if right < heap . size ( ) and float ( heap [ right ] . f ) < float ( heap [ left ] . f ) else left
        if float ( heap [ index ] . f ) <= float ( heap [ child ] . f ) : break
        var swap = heap [ index ]
        heap [ index ] = heap [ child ]
        heap [ child ] = swap
        index = child
    return result

func _find_path ( mapper , item : Dictionary ) -> Array [ Vector2i ] :
    _safe_cache . clear ( )
    var empty : Array [ Vector2i ] = [ ]
    if not _valid_mapper ( mapper ) : return empty
    var pose : Vector3 = mapper . estimated_pose
    var root : Vector2i = mapper . world_to_cell ( Vector2 ( pose . x , pose . y ) )
    if not _safe ( mapper , root , item ) : return empty
    var heap : Array = [ ]
    var cost : = { root : 0 }
    var parents : = { root : root }
    _heap_push ( heap , { "cell" : root , "g" : 0 , "f" : _heuristic ( mapper , root , item ) } )
    var expanded : = 0
    while not heap . is_empty ( ) and expanded < mapper . width * mapper . height :
        var entry : = _heap_pop ( heap )
        var cell : Vector2i = entry . cell
        if int ( entry . g ) != int ( cost . get ( cell , -1 ) ) : continue
        expanded += 1
        if _goal ( mapper , cell , item ) :
            var path : Array [ Vector2i ] = [ cell ]
            while cell != root :
                cell = parents [ cell ]
                path . append ( cell )
            path . reverse ( )
            return path
        for offset in CARDINALS :
            var next : Vector2i = cell + offset
            var tentative : = int ( entry . g ) + 1
            if tentative >= int ( cost . get ( next , 2147483647 ) ) or not _safe ( mapper , next , item ) : continue
            cost [ next ] = tentative
            parents [ next ] = cell
            _heap_push ( heap , { "cell" : next , "g" : tentative , "f" : tentative + _heuristic ( mapper , next , item ) } )
    return empty

func catalog ( mapper , landmarks : Array , now_s : float , epoch : int ) -> Array :
    var result : Array = [ ]
    if not _valid_mapper ( mapper ) : return result
    var seen : = { }
    for value in landmarks :
        if not value is Dictionary or not _eligible ( value , now_s ) : continue
        var item : Dictionary = _find_landmark ( landmarks , value . id , now_s , epoch )
        if item . is_empty ( ) or seen . has ( str ( item . id ) ) : continue
        seen [ str ( item . id ) ] = true
        var path : = _find_path ( mapper , item )
        if path . is_empty ( ) : continue
        var point : Vector2 = mapper . cell_to_world ( path [ -1 ] )
        result . append ( { "id" : "landmark_%d_%s" % [ epoch , str ( item . id ) ] , "landmark_id" : item . id ,
            "epoch" : epoch , "generation" : generation , "label" : item . label , "x" : item . x , "z" : item . z ,
            "uncertainty_m" : item . uncertainty_m , "object_extent_m" : item . object_extent_m , "age_s" : _age ( item , now_s ) ,
            "kind" : "estimated_object_observation_viewpoint" , "approach" : { "x" : point . x , "z" : point . y } ,
            "standoff_m" : point . distance_to ( Vector2 ( float ( item . x ) , float ( item . z ) ) ) , "route_distance_m" : ( path . size ( ) - 1 ) * mapper . resolution } )
        if result . size ( ) >= 12 : break
    return result

func _install_route ( mapper , path : Array [ Vector2i ] ) -> void :
    route = path
    route_points . clear ( )
    for cell in route :
        var point : Vector2 = mapper . cell_to_world ( cell )
        route_points . append ( { "x" : point . x , "z" : point . y } )
    approach = mapper . cell_to_world ( route [ -1 ] )
    _cursor = 0

func start ( mapper , requested_id , landmarks : Array , now_s : float , epoch : int ) -> bool :

    abort ( "new_task" )
    map_epoch = epoch
    if not _valid_mapper ( mapper ) : status = "blocked" ; reason = "invalid_map" ; return false
    var item : = _find_landmark ( landmarks , requested_id , now_s , epoch )
    if item . is_empty ( ) : status = "blocked" ; reason = "landmark_unavailable_or_uncertain" ; return false
    var path : = _find_path ( mapper , item )
    if path . is_empty ( ) : status = "blocked" ; reason = "no_observed_free_approach" ; return false
    landmark_id = item . id
    target = item
    _initial_target_point = Vector2 ( float ( item . x ) , float ( item . z ) )
    _initial_target_uncertainty = float ( item . uncertainty_m )
    _mapper_id = mapper . get_instance_id ( )
    _last_scan_count = mapper . scan_count
    _install_route ( mapper , path )
    status = "active"
    reason = "following_observed_free_astar"
    _last_now = now_s
    _last_progress = now_s
    _best_remaining = ( route . size ( ) - 1 ) * mapper . resolution
    return true

func accept_selection ( expected_generation : int , mapper , requested_id , landmarks : Array , now_s : float , epoch : int ) -> bool :
    if expected_generation != generation : return false
    return start ( mapper , requested_id , landmarks , now_s , epoch )

func abort ( why : String = "manual_abort" ) -> void :
    generation += 1
    status = "aborted"
    reason = why
    landmark_id = null
    target . clear ( )
    route . clear ( )
    route_points . clear ( )
    _cursor = 0
    _gesture_paused = false
    _resume_pending = false
    _mapper_id = 0
    _last_now = -1.0
    _best_remaining = INF

func pause_gesture ( paused : bool ) -> void :

    if status not in [ "active" , "paused" ] : return
    if _gesture_paused and not paused : _resume_pending = true
    _gesture_paused = paused
    status = "paused" if paused else "active"
    reason = "visual_gesture_hold" if paused else "fresh_gesture_release_revalidate"

func _context ( ) -> Dictionary :
    return { "task_status" : status , "task_reason" : reason , "generation" : generation , "landmark_id" : landmark_id ,
        "epoch" : map_epoch , "navigation_valid" : false , "observed_free_only" : true ,
        "route_points" : route_points . duplicate ( true ) , "approach" : { "x" : approach . x , "z" : approach . y } }

func _blocked ( why : String ) -> Dictionary :
    status = "blocked"
    reason = why
    return _context ( )

func _segment_safe(mapper, from: Vector2, to: Vector2, item: Dictionary) -> bool:
    return GridMotion.segment_safe(mapper,from,to,func(cell: Vector2i): return _safe(mapper,cell,item))

func command_path_safe(mapper, speed: float, yaw_rate: float, duration: float = 0.35) -> bool:
    _safe_cache.clear()
    return GridMotion.command_safe(mapper,mapper.estimated_pose,speed,yaw_rate,duration,func(cell: Vector2i): return _safe(mapper,cell,target))

func step ( mapper , landmarks : Array , now_s : float , epoch : int ) -> Dictionary :
    if status not in [ "active" , "paused" ] : return _context ( )
    if not _valid_mapper ( mapper ) or mapper . get_instance_id ( ) != _mapper_id : return _blocked ( "map_replaced" )
    if epoch != map_epoch or not is_finite ( now_s ) or now_s < _last_now : return _blocked ( "map_or_time_epoch_changed" )
    _last_now = now_s
    var latest : = _find_landmark ( landmarks , landmark_id , now_s , epoch )
    if latest . is_empty ( ) : return _blocked ( "landmark_expired_or_withdrawn" )
    var old_point : = Vector2 ( float ( target . x ) , float ( target . z ) )
    var new_point : = Vector2 ( float ( latest . x ) , float ( latest . z ) )
    if old_point . distance_to ( new_point ) > maxf ( 0.35 , float ( target . uncertainty_m ) ) or _initial_target_point . distance_to ( new_point ) > maxf ( 0.35 , _initial_target_uncertainty ) : return _blocked ( "landmark_moved_or_inconsistent" )
    target = latest
    if _gesture_paused :
        _last_progress = now_s
        return _context ( )
    if _resume_pending :
        _resume_pending = false
        _last_progress = now_s
    if route . is_empty ( ) : return _blocked ( "missing_route" )
    _safe_cache . clear ( )
    var pose : Vector3 = mapper . estimated_pose
    var point : = Vector2 ( pose . x , pose . y )
    if mapper . scan_count != _last_scan_count :
        _last_scan_count = mapper . scan_count
        var route_valid : = true
        for index in range ( _cursor , route . size ( ) ) :
            if not _safe ( mapper , route [ index ] , target ) :
                route_valid = false
                break
        if not route_valid :
            var replacement : = _find_path ( mapper , target )
            if replacement . is_empty ( ) : return _blocked ( "measured_route_blocked" )
            _install_route ( mapper , replacement )
            _best_remaining = INF
            return _context ( )

    if not _safe ( mapper , route [ -1 ] , target ) or not _goal ( mapper , route [ -1 ] , target ) :
        var replacement : = _find_path ( mapper , target )
        if replacement . is_empty ( ) : return _blocked ( "approach_no_longer_safe" )
        _install_route ( mapper , replacement )
    if point . distance_to ( approach ) < 0.16 and _segment_safe ( mapper , point , approach , target ) :
        status = "arrived"
        reason = "at_estimated_object_observation_viewpoint"
        return _context ( )
    var nearest : = _cursor
    var distance : float = point . distance_to ( mapper . cell_to_world ( route [ nearest ] ) )
    for i in range ( _cursor + 1 , mini ( route . size ( ) , _cursor + 8 ) ) :
        var candidate_distance : float = point . distance_to ( mapper . cell_to_world ( route [ i ] ) )
        if candidate_distance < distance :
            nearest = i
            distance = candidate_distance
    _cursor = nearest
    var remaining : float = ( route . size ( ) - 1 - _cursor ) * mapper . resolution + distance
    if remaining < _best_remaining - 0.05 :
        _best_remaining = remaining
        _last_progress = now_s
    elif now_s - _last_progress > MAX_STUCK_S : return _blocked ( "no_measured_route_progress" )
    var next : = mini ( route . size ( ) - 1 , _cursor + maxi ( 1 , int ( LOOKAHEAD_M / mapper . resolution ) ) )
    while next > _cursor and not _segment_safe ( mapper , point , mapper . cell_to_world ( route [ next ] ) , target ) : next -= 1
    if not _segment_safe ( mapper , point , mapper . cell_to_world ( route [ next ] ) , target ) or next == _cursor and distance < 0.03 :
        var replacement : = _find_path ( mapper , target )
        if replacement . is_empty ( ) : return _blocked ( "measured_route_blocked" )
        _install_route ( mapper , replacement )
        _best_remaining = INF
        return _context ( )
    var waypoint : Vector2 = mapper . cell_to_world ( route [ next ] )
    var delta : = waypoint - point
    var result : = _context ( )
    result . navigation_valid = true
    result [ "waypoint" ] = { "x" : waypoint . x , "z" : waypoint . y , "distance_m" : delta . length ( ) ,
        "heading_error_rad" : wrapf ( atan2 ( - delta . x , - delta . y ) - pose . z , - PI , PI ) ,
        "remaining_route_m" : remaining , "goal_distance_m" : point . distance_to ( approach ) }
    return result
