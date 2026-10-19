extends RefCounted

const ACTIONS : = { "stop" : "stop" , "forward" : "forward" , "reverse" : "reverse" , "reverse_left" : "reverse_left" , "reverse_right" : "reverse_right" , "turn_left" : "rotate_left" , "turn_right" : "rotate_right" }
const REASONS : = [ "progress" , "reposition" , "obstacle" , "insufficient_evidence" , "mission_complete" ]
const MOTION_INSTRUCTIONS : = "Make measurable progress toward the current task using one bounded 500 ms action. Choose forward or a turn when supported by observed clearance; reverse can reposition from a dead end only with fresh rear lidar clearance. Stop when the task is complete, all useful motions are blocked, or evidence is insufficient. Do not stop merely because the map is incomplete: unknown cells are not free, but observed free space can support a bounded step. turn_left/right rotate in place. reverse_left/right move backward while the ROBOT NOSE yaws left/right (positive/negative yaw); these are not rear-path steering labels. Use controller_context reverse flags, measured directional clearances, recent action outcomes and map_progress to avoid repeating a stopped or ineffective choice. Observed grid fraction is not mission completion. If a useful observed-safe bounded motion exists, prefer progress or repositioning; never fabricate clearances or completion."
const REASON_INSTRUCTIONS : = "Classify the reason for the selected motion: progress toward task; reposition for a better route/view; obstacle blocks useful motion; insufficient_evidence for any safe useful motion; mission_complete only when observed evidence demonstrates completion."
const DEADLINE_MS : = 1000
const ACTION_MS : = 500
const HAZARD_MAX : = 0.25
var session_id : = ""
var observation_seq : = -1
var captured_at_ms : = -1
var pending : = false
var mission_version : = 0

func begin_observation ( session : String , sequence : int , captured : int , mission : int = 0 ) -> void :
    session_id = session
    observation_seq = sequence
    captured_at_ms = captured
    mission_version = mission
    pending = true

func cancel ( ) -> void :
    pending = false
    observation_seq = -1

func stop_result ( reason : String ) -> Dictionary :
    return { "action" : "stop" , "duration_s" : ACTION_MS / 1000.0 , "reason" : reason }

func probability ( value : Variant ) -> bool :
    return ( value is float or value is int ) and is_finite ( float ( value ) ) and value >= 0 and value <= 1

func parse_decision ( response : Dictionary ) -> Dictionary :
    var answers = response . get ( "answers" )
    if not answers is Array or answers . size ( ) not in [ 2 , 3 ] :
        return stop_result ( "Malformed answers" )
    var motion : Dictionary = { }
    var hazard : Dictionary = { }
    var rationale : Dictionary = { }
    for answer in answers :
        if not answer is Dictionary or answer . get ( "type" ) == "refusal" :
            return stop_result ( "Refused or malformed answer" )
        if answer . get ( "name" ) == "motion" and motion . is_empty ( ) :
            motion = answer
        elif answer . get ( "name" ) == "hazard" and hazard . is_empty ( ) :
            hazard = answer
        elif answer . get ( "name" ) == "reason" and rationale . is_empty ( ) :
            rationale = answer
        else :
            return stop_result ( "Unknown or duplicate answer" )
    if motion . get ( "type" ) != "choice" or hazard . get ( "type" ) != "predicate" :
        return stop_result ( "Missing typed answers" )
    var choice = motion . get ( "choice" )
    if not choice is String or not ACTIONS . has ( choice ) :
        return stop_result ( "Unknown action" )
    var confidence = motion . get ( "confidence" )
    var risk = hazard . get ( "probability" )
    if not probability ( confidence ) or not probability ( risk ) :
        return stop_result ( "Invalid probability" )
    if not rationale . is_empty ( ) and ( rationale . get ( "type" ) != "choice" or rationale . get ( "choice" ) not in REASONS or not probability ( rationale . get ( "confidence" ) ) ) :
        return stop_result ( "Malformed reason answer" )
    var reason : String = rationale . get ( "choice" , "not_provided" )

    if choice == "stop" :
        return { "action" : "stop" , "duration_s" : ACTION_MS / 1000.0 , "reason" : "Model stop: " + reason . replace ( "_" , " " ) , "model_reason" : reason }


    if risk >= HAZARD_MAX :
        return stop_result ( "Safety veto: hazard score >= 0.25" )
    return { "action" : ACTIONS [ choice ] , "duration_s" : ACTION_MS / 1000.0 , "reason" : "Accepted: " + reason . replace ( "_" , " " ) , "model_reason" : reason }

func matches_observation ( envelope : Dictionary , current_session : String , current_mission : int = 0 ) -> bool :

    return pending and current_session == session_id and current_mission == mission_version \
and envelope . get ( "session_id" ) == session_id and envelope . get ( "observation_seq" ) == observation_seq \
and envelope . get ( "captured_at_ms" ) == captured_at_ms and envelope . get ( "mission_version" , 0 ) == mission_version

func accept_response ( envelope : Dictionary , current_session : String , current_sequence : int , now_ms : int , current_mission : int = 0 ) -> Dictionary :
    var matched : = matches_observation ( envelope , current_session , current_mission )
    var valid : = matched and current_sequence >= observation_seq
    valid = valid and now_ms >= captured_at_ms and now_ms - captured_at_ms < DEADLINE_MS
    valid = valid and envelope . get ( "mode" ) in [ "dry_run" , "live" ]
    if matched : pending = false
    var command : = stop_result ( "Stale, canceled or mismatched response" )
    if valid and envelope . get ( "decision" ) is Dictionary :
        command = parse_decision ( envelope [ "decision" ] )
    command [ "validated_observation" ] = valid

    command [ "observation_seq" ] = current_sequence
    command [ "mission_version" ] = current_mission
    var expires : = mini ( now_ms + ACTION_MS , captured_at_ms + DEADLINE_MS ) if valid else now_ms + ACTION_MS
    command [ "duration_s" ] = maxf ( 0.001 , ( expires - now_ms ) / 1000.0 )
    command [ "expires_at_ms" ] = expires
    return command

func make_request ( session : String , sequence : int , captured : int , camera_image : Image , map_image : Image , sensor_state : Dictionary ) -> Dictionary :
    if camera_image == null or map_image == null or camera_image . is_empty ( ) or map_image . is_empty ( ) :
        return { }
    var camera : = camera_image . duplicate ( ) as Image
    if maxi ( camera . get_width ( ) , camera . get_height ( ) ) > 480 :
        var scale_factor : = 480.0 / maxi ( camera . get_width ( ) , camera . get_height ( ) )
        camera . resize ( maxi ( 1 , int ( camera . get_width ( ) * scale_factor ) ) , maxi ( 1 , int ( camera . get_height ( ) * scale_factor ) ) )
    var local_map : = map_image . duplicate ( ) as Image
    if maxi ( local_map . get_width ( ) , local_map . get_height ( ) ) > 480 :
        var scale_factor : = 480.0 / maxi ( local_map . get_width ( ) , local_map . get_height ( ) )
        local_map . resize ( maxi ( 1 , int ( local_map . get_width ( ) * scale_factor ) ) , maxi ( 1 , int ( local_map . get_height ( ) * scale_factor ) ) )
    var mission : = int ( sensor_state . get ( "mission_version" , 0 ) )
    begin_observation ( session , sequence , captured , mission )
    return { "session_id" : session , "observation_seq" : sequence , "captured_at_ms" : captured , "mission_version" : mission ,
        "camera_image" : "data:image/jpeg;base64," + Marshalls . raw_to_base64 ( camera . save_jpg_to_buffer ( 0.75 ) ) ,
        "map_image" : "data:image/png;base64," + Marshalls . raw_to_base64 ( local_map . save_png_to_buffer ( ) ) ,
        "state" : sensor_state }
