extends Node

signal completed ( result : Dictionary )
var process_id : = -1
var ipc : = ""
var worker_ready : = false
var capture_enabled : = true
var status : = "Vision starting"
var pending : = false
var frame_id : = 0
var epoch : = 0
var landmarks : Array = [ ]
var latest : Dictionary = { }
var latest_image : Image
var pending_image : Image
var pending_metadata : Dictionary = { }
var last_capture_ms : = 0
var sent_ms : = 0
var poll_elapsed : = 0.0
var discarded : = 0
var capture_interval_ms : = 333
var runtime_root : = ""
var start_ms : = 0
var profile_name : = "pi4_budget"
var input_bounds : = Vector2i ( 640 , 360 )
var limits : Dictionary = { }
var profile : Dictionary = { }
var skipped_captures : = 0
var next_opportunity_ms : = 0
var diagnostic_next_ms : = 0

func start ( ) -> void :
    if OS . has_feature ( "web" ) :
        status = "Vision requires native CPU runtime" ; return
    runtime_root = ProjectSettings . globalize_path ( "res://perception" ) if OS . has_feature ( "editor" ) else OS . get_executable_path ( ) . get_base_dir ( ) . path_join ( "perception" )
    var python : = runtime_root . path_join ( "runtime_windows/pythonw.exe" ) if OS . get_name ( ) == "Windows" else runtime_root . path_join ( ".venv/bin/python" )
    var script : = runtime_root . path_join ( "worker.py" )
    if not FileAccess . file_exists ( python ) or not FileAccess . file_exists ( script ) :
        status = "Vision runtime missing · see setup guide" ; return
    ipc = ProjectSettings . globalize_path ( "user://perception_" + str ( OS . get_process_id ( ) ) + "_" + str ( Time . get_ticks_usec ( ) ) )
    DirAccess . make_dir_recursive_absolute ( ipc )
    if "--native-vision" in OS . get_cmdline_user_args ( ) : profile_name = "native"
    if "--pi4-stress" in OS . get_cmdline_user_args ( ) : profile_name = "pi4_stress"
    process_id = OS . create_process ( python , PackedStringArray ( [ script , "--ipc" , ipc , "--profile" , profile_name ] ) )
    start_ms = Time . get_ticks_msec ( )
    if process_id < 0 : status = "Vision worker could not start"

func can_capture ( now_ms : int ) -> bool :
    return capture_enabled and worker_ready and not pending and now_ms - last_capture_ms >= capture_interval_ms

func submit ( image : Image , metadata : Dictionary ) -> bool :
    var submit_begin_ms := Time.get_ticks_msec()
    if not can_capture ( submit_begin_ms ) or image == null or image . is_empty ( ) : return false
    # Cadence is measured between acquisitions, not after the previous render,
    # image readback and PNG publication. Those costs already age the evidence.
    # Retain one-in-flight backpressure and reject invalid/reordered clocks.
    var captured = metadata.get("capture_monotonic_ms")
    if not (captured is int or captured is float) or not is_finite(float(captured)) or float(captured) != floorf(float(captured)):
        status = "Vision capture timestamp invalid"
        return false
    var captured_ms := int(captured)
    if captured_ms < 0 or captured_ms > submit_begin_ms or captured_ms < last_capture_ms or captured_ms - last_capture_ms < capture_interval_ms:
        status = "Vision capture timestamp outside cadence"
        return false
    frame_id += 1
    metadata = metadata . duplicate ( true )
    metadata [ "frame_id" ] = frame_id ; metadata [ "epoch" ] = epoch
    metadata["submission_begin_monotonic_ms"] = submit_begin_ms
    pending_image = image . duplicate ( )
    var ratio : = minf ( 1.0 , minf ( float ( input_bounds . x ) / image . get_width ( ) , float ( input_bounds . y ) / image . get_height ( ) ) )
    if ratio < 1 : pending_image . resize ( int ( image . get_width ( ) * ratio ) , int ( image . get_height ( ) * ratio ) , Image . INTERPOLATE_BILINEAR )
    var error : = pending_image . save_png ( ipc . path_join ( "frame.png" ) )
    if error != OK : status = "Vision frame write failed" ; return false
    metadata [ "width" ] = pending_image . get_width ( ) ; metadata [ "height" ] = pending_image . get_height ( )
    var file : = FileAccess . open ( ipc . path_join ( "request.tmp" ) , FileAccess . WRITE )
    if not file : status = "Vision request write failed" ; return false
    file . store_string ( JSON . stringify ( metadata ) ) ; file . close ( )
    error = DirAccess . rename_absolute ( ipc . path_join ( "request.tmp" ) , ipc . path_join ( "request.json" ) )
    if error != OK :
        status = "Vision request publish failed: " + error_string ( error ) ; return false
    pending = true ; pending_metadata = metadata ; sent_ms = Time . get_ticks_msec ( )
    last_capture_ms = captured_ms
    # This is producer-only timing, not a rewritten sensor timestamp. Publication
    # time is attached after the atomic rename, so it stays outside worker input.
    pending_metadata["request_published_monotonic_ms"] = sent_ms
    return true

func _process ( delta : float ) -> void :
    if process_id < 0 : return
    poll_elapsed += delta
    if poll_elapsed < 0.03 : return
    poll_elapsed = 0
    if "--vision-diagnostics" in OS . get_cmdline_user_args ( ) and Time . get_ticks_msec ( ) >= diagnostic_next_ms :
        diagnostic_next_ms = Time . get_ticks_msec ( ) + 2000
        print ( "VISION_DIAGNOSTIC " + JSON . stringify ( { "ready" : worker_ready , "pending" : pending , "frame_id" : frame_id , "latest_id" : latest . get ( "frame_id" , -1 ) , "status" : status , "now_ms" : Time . get_ticks_msec ( ) , "captured_ms" : latest . get ( "capture_monotonic_ms" , 0 ) , "ipc" : ipc , "process_running" : OS . is_process_running ( process_id ) } ) )
    if not worker_ready :
        var startup : = _read_json ( "ready.json" )
        if not startup . is_empty ( ) :
            worker_ready = bool ( startup . get ( "ok" , false ) ) ; status = "Vision ready" if worker_ready else "Vision error: " + str ( startup . get ( "error" , "startup" ) )
            if worker_ready :
                profile = startup . get ( "profile" , { } ) ; limits = startup . get ( "limits" , { } )
                capture_interval_ms = int ( 1000.0 / float ( profile . get ( "inference_rate_hz" , 3 ) ) )
                input_bounds = Vector2i ( int ( profile . get ( "max_input_width" , 640 ) ) , int ( profile . get ( "max_input_height" , 360 ) ) )
            else : _stop_worker ( )
        elif not OS . is_process_running ( process_id ) :
            status = "Vision worker exited during startup · see local log" ; _stop_worker ( )
        elif Time . get_ticks_msec ( ) - start_ms > 20000 :
            status = "Vision startup timeout" ; _stop_worker ( )
        return
    if not OS . is_process_running ( process_id ) :
        var failure : = _read_json ( "ready.json" )
        status = "Vision error: " + str ( failure . get ( "error" , "worker exited · see local log" ) )
        pending = false ; _stop_worker ( ) ; return
    if Time . get_ticks_msec ( ) >= next_opportunity_ms :
        next_opportunity_ms = Time . get_ticks_msec ( ) + capture_interval_ms
        if pending : skipped_captures += 1
    if not pending : return
    var result : = _read_json ( "result.json" )
    if not result . is_empty ( ) and int ( result . get ( "frame_id" , -1 ) ) == int ( pending_metadata . frame_id ) and int ( result . get ( "epoch" , -1 ) ) == int ( pending_metadata . epoch ) :
        pending = false
        if int ( result . epoch ) != epoch :
            discarded += 1 ; return
        var consumed_ms := Time.get_ticks_msec()
        result["producer_timing"] = {
            "capture_monotonic_ms": int(pending_metadata.capture_monotonic_ms),
            "submission_begin_monotonic_ms": int(pending_metadata.get("submission_begin_monotonic_ms", sent_ms)),
            "request_published_monotonic_ms": sent_ms,
            "consumed_monotonic_ms": consumed_ms,
            "cadence_anchor": "original_capture_timestamp", "max_in_flight": 1}
        if result . get ( "ok" , false ) :
            result [ "age_ms" ] = consumed_ms - int ( pending_metadata . capture_monotonic_ms )
            latest = result ; latest_image = pending_image
            landmarks = result . get ( "landmarks" , [ ] )
            status = "Pi budget · %d ms · %d boxes" % [ result . age_ms , result . get ( "detections" , [ ] ) . size ( ) ]
        else : status = "Vision error: " + str ( result . get ( "error" , "inference" ) )
        completed . emit ( result )
    elif Time . get_ticks_msec ( ) - sent_ms > 15000 :
        status = "Vision worker timeout" ; pending = false ; _stop_worker ( )

func reset_map ( ) -> void :
    epoch += 1 ; landmarks . clear ( ) ; latest . clear ( ) ; latest_image = null


func _read_json ( name : String ) -> Dictionary :
    var path : = ipc . path_join ( name )
    if not FileAccess . file_exists ( path ) : return { }
    var file : = FileAccess . open ( path , FileAccess . READ )
    if not file : return { }
    var text : = file . get_as_text ( )
    file . close ( )
    var data = JSON . parse_string ( text )
    return data if data is Dictionary else { }

func _stop_worker ( ) -> void :
    worker_ready = false
    if not ipc . is_empty ( ) :
        var file : = FileAccess . open ( ipc . path_join ( "stop" ) , FileAccess . WRITE )
        if file : file . store_string ( "stop" ) ; file . close ( )
    if process_id > 0 and OS . is_process_running ( process_id ) : OS . kill ( process_id )
    process_id = -1

func _exit_tree ( ) -> void :
    _stop_worker ( )

    if not ipc . is_empty ( ) :
        for name in [ "ready.json" , "ready.tmp" , "request.json" , "request.tmp" , "result.json" , "result.tmp" , "frame.png" , "stop" ] :
            DirAccess . remove_absolute ( ipc . path_join ( name ) )
        DirAccess . remove_absolute ( ipc )
