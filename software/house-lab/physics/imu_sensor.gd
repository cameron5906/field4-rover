extends RefCounted



const MODEL : = "WT901TTL"
const STANDARD_GRAVITY_M_S2 : = 9.80665
const NOMINAL_OUTPUT_HZ : = 100.0
const OUTPUT_PERIOD_S : = 1.0 / NOMINAL_OUTPUT_HZ
const ACCEL_FULL_SCALE_M_S2 : = 16.0 * STANDARD_GRAVITY_M_S2
const GYRO_FULL_SCALE_RAD_S : = 2000.0 * PI / 180.0
const SIGNED_SCALE : = 32768.0
const STALE_AFTER_S : = 0.05



var accel_noise_std_m_s2 : = 0.01
var gyro_noise_std_deg_s : = 0.07
var gyro_bias_deg_s : = Vector3 ( 0.05 , -0.05 , 0.05 )
var magnetic_noise_std_counts : = 1.5
var gravity_world_m_s2 : = Vector3 ( 0.0 , - STANDARD_GRAVITY_M_S2 , 0.0 )
var last_reading : Dictionary = { }

var _rng : = RandomNumberGenerator . new ( )
var _previous_velocity : = Vector3 . ZERO
var _have_previous_velocity : = false
var _previous_call_s : = -1.0
var _last_output_s : = -1.0
var _next_output_s : = -1.0
var _sequence : = 0


func _init ( ) -> void :
    reset ( )


func reset ( body : RigidBody3D = null , seed : int = 901 ) -> void :
    _rng . seed = seed
    _previous_velocity = body . linear_velocity if is_instance_valid ( body ) else Vector3 . ZERO
    _have_previous_velocity = is_instance_valid ( body ) and _previous_velocity . is_finite ( )
    _previous_call_s = -1.0
    _last_output_s = -1.0
    _next_output_s = -1.0
    _sequence = 0
    last_reading = _empty_reading ( )


func sample ( body : RigidBody3D , dt : float , sim_time : float , motors_running : bool = false ) -> Dictionary :
    if not is_instance_valid ( body ) :
        return _invalid_reading ( sim_time , "missing_body" )
    if not is_finite ( dt ) or dt <= 0.0 or not is_finite ( sim_time ) or sim_time < 0.0 :
        return _invalid_reading ( sim_time , "invalid_time" )
    if _previous_call_s >= 0.0 and sim_time <= _previous_call_s :
        return _invalid_reading ( sim_time , "non_monotonic_time; reset after time rewind" )
    var basis : = body . global_basis
    if not body . linear_velocity . is_finite ( ) or not body . angular_velocity . is_finite ( ) or not basis . is_finite ( ) or absf ( basis . determinant ( ) ) < 1e-06 :
        return _invalid_reading ( sim_time , "non_finite_body_state" )
    var elapsed : = sim_time - _previous_call_s if _previous_call_s >= 0.0 else dt
    var acceleration_world : = Vector3 . ZERO
    if _have_previous_velocity :
        acceleration_world = ( body . linear_velocity - _previous_velocity ) / elapsed
    _previous_velocity = body . linear_velocity
    _have_previous_velocity = true
    _previous_call_s = sim_time
    if _next_output_s >= 0.0 and sim_time + 1e-09 < _next_output_s :
        last_reading = get_reading ( sim_time )
        return last_reading . duplicate ( true )



    var scheduled_s : = sim_time if _next_output_s < 0.0 else _next_output_s
    var missed_slots : = maxi ( 0 , int ( floor ( ( sim_time - scheduled_s + 1e-09 ) / OUTPUT_PERIOD_S ) ) )
    _next_output_s = scheduled_s + ( missed_slots + 1 ) * OUTPUT_PERIOD_S
    var interval : = sim_time - _last_output_s if _last_output_s >= 0.0 else dt
    _last_output_s = sim_time
    _sequence += 1
    var inverse_basis : = basis . orthonormalized ( ) . transposed ( )
    var accel_sensor : = _body_to_sensor ( inverse_basis * ( acceleration_world - gravity_world_m_s2 ) )
    var gyro_sensor : = _body_to_sensor ( inverse_basis * body . angular_velocity )
    accel_sensor += _noise3 ( accel_noise_std_m_s2 )
    gyro_sensor += gyro_bias_deg_s * ( PI / 180.0 ) + _noise3 ( gyro_noise_std_deg_s * PI / 180.0 )
    var accel_counts : = _quantize ( accel_sensor , ACCEL_FULL_SCALE_M_S2 )
    var gyro_counts : = _quantize ( gyro_sensor , GYRO_FULL_SCALE_RAD_S )



    var magnetic : = _body_to_sensor ( inverse_basis * Vector3 ( 180.0 , -360.0 , -120.0 ) )
    magnetic = magnetic * Vector3 ( 1.08 , 0.93 , 1.04 ) + Vector3 ( 45.0 , -35.0 , 60.0 )
    if motors_running :
        magnetic += Vector3 ( 110.0 + 35.0 * sin ( sim_time * 19.0 ) , -80.0 + 25.0 * cos ( sim_time * 23.0 ) , 55.0 )
    magnetic += _noise3 ( magnetic_noise_std_counts )
    var moving : = body . linear_velocity . length ( ) > 0.005 or body . angular_velocity . length ( ) > 0.005
    last_reading = {
        "model" : MODEL , "provenance" : "simulated" , "calibrated" : false ,
        "valid" : true , "sequence" : _sequence , "fresh" : true , "stale" : false ,
        "timestamp_s" : sim_time , "sample_interval_s" : interval , "age_s" : 0.0 ,
        "nominal_output_hz" : NOMINAL_OUTPUT_HZ ,
        "timing_model" : "nominal deadlines sampled on actual physics ticks" ,
        "scheduled_timestamp_s" : scheduled_s , "scheduling_delay_s" : sim_time - scheduled_s ,
        "missed_nominal_slots" : missed_slots ,
        "axes" : [ "x_right" , "y_forward" , "z_up" ] ,
        "accel_m_s2" : _decode ( accel_counts , ACCEL_FULL_SCALE_M_S2 ) ,
        "gyro_rad_s" : _decode ( gyro_counts , GYRO_FULL_SCALE_RAD_S ) ,
        "accel_raw_counts" : accel_counts , "gyro_raw_counts" : gyro_counts ,
        "accel_saturated" : _saturated ( accel_sensor , ACCEL_FULL_SCALE_M_S2 ) ,
        "gyro_saturated" : _saturated ( gyro_sensor , GYRO_FULL_SCALE_RAD_S ) ,
        "magnetometer_raw_counts" : [ clampi ( roundi ( magnetic . x ) , -32768 , 32767 ) , clampi ( roundi ( magnetic . y ) , -32768 , 32767 ) , clampi ( roundi ( magnetic . z ) , -32768 , 32767 ) ] ,
        "magnetometer_unit" : "raw_counts; physical scale unverified" ,
        "magnetic_heading_untrusted" : true ,
        "magnetic_disturbance_active" : motors_running ,
        "magnetic_heading_warning" : "motion_or_motors_and_uncalibrated" if moving or motors_running else "uncalibrated_hard_and_soft_iron" ,
    }
    return last_reading . duplicate ( true )



func get_reading ( sim_time : float = -1.0 ) -> Dictionary :
    var reading : = last_reading . duplicate ( true )
    reading [ "fresh" ] = false
    if is_finite ( sim_time ) and sim_time >= 0.0 and _last_output_s >= 0.0 :
        reading [ "age_s" ] = maxf ( 0.0 , sim_time - _last_output_s )
    reading [ "stale" ] = not bool ( reading . get ( "valid" , false ) ) or float ( reading . get ( "age_s" , 0.0 ) ) > STALE_AFTER_S
    return reading


func _empty_reading ( ) -> Dictionary :
    return { "model" : MODEL , "provenance" : "simulated" , "calibrated" : false ,
        "valid" : false , "sequence" : 0 , "fresh" : false , "stale" : true ,
        "timestamp_s" : -1.0 , "sample_interval_s" : 0.0 , "age_s" : 0.0 ,
        "nominal_output_hz" : NOMINAL_OUTPUT_HZ , "accel_m_s2" : [ 0.0 , 0.0 , 0.0 ] ,
        "gyro_rad_s" : [ 0.0 , 0.0 , 0.0 ] , "magnetometer_raw_counts" : [ 0 , 0 , 0 ] ,
        "magnetic_heading_untrusted" : true , "reason" : "no_sample_since_reset" }


func _invalid_reading ( sim_time : float , reason : String ) -> Dictionary :
    var reading : = get_reading ( sim_time )
    reading [ "valid" ] = false
    reading [ "stale" ] = true
    reading [ "reason" ] = reason

    _have_previous_velocity = false
    last_reading = reading
    return reading . duplicate ( true )


func _body_to_sensor ( value : Vector3 ) -> Vector3 :
    return Vector3 ( value . x , - value . z , value . y )


func _noise3 ( standard_deviation : float ) -> Vector3 :
    return Vector3 ( _rng . randfn ( 0.0 , standard_deviation ) , _rng . randfn ( 0.0 , standard_deviation ) , _rng . randfn ( 0.0 , standard_deviation ) )


func _quantize ( value : Vector3 , full_scale : float ) -> Array [ int ] :
    return [ clampi ( roundi ( value . x / full_scale * SIGNED_SCALE ) , -32768 , 32767 ) , clampi ( roundi ( value . y / full_scale * SIGNED_SCALE ) , -32768 , 32767 ) , clampi ( roundi ( value . z / full_scale * SIGNED_SCALE ) , -32768 , 32767 ) ]


func _decode ( counts : Array [ int ] , full_scale : float ) -> Array [ float ] :
    return [ float ( counts [ 0 ] ) / SIGNED_SCALE * full_scale , float ( counts [ 1 ] ) / SIGNED_SCALE * full_scale , float ( counts [ 2 ] ) / SIGNED_SCALE * full_scale ]


func _saturated ( value : Vector3 , full_scale : float ) -> bool :
    var positive_limit : = full_scale * 32767.0 / SIGNED_SCALE
    return value . x < - full_scale or value . y < - full_scale or value . z < - full_scale or value . x > positive_limit or value . y > positive_limit or value . z > positive_limit
