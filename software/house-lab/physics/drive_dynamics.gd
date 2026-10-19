extends RefCounted


var config : Dictionary
var mass : = 2.49388
var yaw_inertia : = 0.0279945
var radius : = 0.048
var track : = 0.229
var wheelbase : = 0.18
var v : = 0.0
var w : = 0.0
var wheel_rates : Array [ float ] = [ 0.0 , 0.0 , 0.0 , 0.0 ]
var motor_currents : Array [ float ] = [ 0.0 , 0.0 , 0.0 , 0.0 ]
var configuration : = "bench_tethered_no_battery"
var elapsed : = 0.0

func _init ( ) -> void :
    var file : = FileAccess . open ( "res://physics/physics_estimates.json" , FileAccess . READ )
    if file :
        config = JSON . parse_string ( file . get_as_text ( ) )
        set_configuration ( configuration )

func set_configuration ( name : String ) -> bool :
    if not config . has ( "configurations" ) or not config . configurations . has ( name ) :
        return false
    configuration = name
    var c : Dictionary = config . configurations [ name ]
    mass = float ( c . mass_kg )
    yaw_inertia = float ( c . inertia_diagonal_godot_kg_m2 [ 1 ] )
    return true

func reset ( ) -> void :
    v = 0.0
    w = 0.0
    elapsed = 0.0
    for i in range ( 4 ) :
        wheel_rates [ i ] = 0.0
        motor_currents [ i ] = 0.0

func sync_body_velocity ( actual_v : float , actual_w : float ) -> void :

    v = actual_v
    w = actual_w

func step ( target_v : float , target_w : float , delta : float , surface : String = "wood" ) -> Dictionary :
    if config . is_empty ( ) :
        return { "v" : 0.0 , "w" : 0.0 , "error" : "Missing physics configuration" }
    var surf : Dictionary = config . surfaces . get ( surface , config . surfaces . wood )
    var motor : Dictionary = config . motor
    var mu : float = surf . longitudinal_friction_coefficient
    var mu_lat : float = surf . lateral_friction_coefficient
    var rr : float = surf . rolling_resistance_coefficient
    var ilim : float = motor . planned_current_limit_a
    var kt : float = motor . output_torque_per_amp_nm_a
    var ke : float = motor . estimated_back_emf_at_output_v_per_rad_s
    var resistance : float = motor . estimated_winding_resistance_ohm
    var friction : float = motor . estimated_output_friction_torque_nm
    var iw : float = motor . estimated_effective_rotating_inertia_kg_m2
    var omega0 : float = motor . no_load_speed_rad_s
    var dt : = clampf ( delta , 0.0 , 0.1 )
    var count : = maxi ( 1 , ceili ( dt / 0.001 ) )
    var h : = dt / count
    var peak_slip : = 0.0
    var current_limited : = false
    var traction_limited : = false
    var lateral_scrub : = 0.0
    var body_conf : Dictionary = config . configurations [ configuration ]
    var com_forward : = float ( body_conf . center_of_mass_cad_m [ 0 ] )
    var com_left : = float ( body_conf . center_of_mass_cad_m [ 1 ] )
    for _substep in range ( count ) :
        var force_forward : = 0.0
        var moment_yaw : = 0.0
        for i in range ( 4 ) :
            var a : = wheelbase * ( 0.5 if i < 2 else -0.5 )
            var b : = track * ( 0.5 if i % 2 == 0 else -0.5 )
            var normal : = mass * 9.80665 * ( 0.5 + signf ( a ) * com_forward / wheelbase ) * ( 0.5 + signf ( b ) * com_left / track )
            var contact_v : = v - w * b
            var desired_omega : = clampf ( ( target_v - target_w * b ) / radius , - omega0 , omega0 )
            var duty : = clampf ( desired_omega / omega0 + 0.5 * ( desired_omega - wheel_rates [ i ] ) / omega0 , -1.0 , 1.0 )
            if absf ( target_v ) + absf ( target_w ) < 1e-05 :
                duty = 0.0
            var requested_current : = ( 12.0 * duty - ke * wheel_rates [ i ] ) / resistance
            var current : = clampf ( requested_current , - ilim , ilim )
            motor_currents [ i ] = current
            current_limited = current_limited or absf ( requested_current ) > ilim
            var slip : = radius * wheel_rates [ i ] - contact_v
            peak_slip = maxf ( peak_slip , absf ( slip ) )
            var fx : = 100.0 * slip
            var fy : = - mu_lat * normal * tanh ( w * a / 0.025 )
            var ellipse : = sqrt ( pow ( fx / maxf ( mu * normal , 0.001 ) , 2.0 ) + pow ( fy / maxf ( mu_lat * normal , 0.001 ) , 2.0 ) )
            if ellipse > 1.0 :
                fx /= ellipse
                fy /= ellipse
                traction_limited = true
            lateral_scrub = maxf ( lateral_scrub , absf ( fy ) )
            var rolling_torque : = rr * normal * radius * tanh ( wheel_rates [ i ] / 0.3 )
            var applied_torque : = kt * current - fx * radius - rolling_torque


            var loss : = friction * signf ( wheel_rates [ i ] )
            if absf ( wheel_rates [ i ] ) < 0.02 :
                loss = clampf ( applied_torque , - friction , friction )
            var next_rate : = wheel_rates [ i ] + ( applied_torque - loss ) / iw * h
            if signf ( next_rate ) != signf ( wheel_rates [ i ] ) and absf ( applied_torque ) <= friction :
                next_rate = 0.0
            wheel_rates [ i ] = next_rate
            force_forward += fx
            moment_yaw += - b * fx + a * fy
        v += force_forward / mass * h
        w += moment_yaw / yaw_inertia * h
    elapsed += dt
    return { "v" : v , "w" : w , "wheel_rates" : wheel_rates . duplicate ( ) , "motor_currents" : motor_currents . duplicate ( ) , "traction_limited" : traction_limited , "current_limited" : current_limited , "slipping" : peak_slip > 0.03 , "peak_longitudinal_slip_m_s" : peak_slip , "lateral_scrub_force_n" : lateral_scrub , "mass_kg" : mass , "surface" : surface , "configuration" : configuration , "model" : "estimated planar four-contact dynamics; fixed ground, no vertical physics" }
