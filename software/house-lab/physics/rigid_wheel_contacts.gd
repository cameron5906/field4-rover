extends RefCounted


var config : Dictionary
var configuration : = "bench_tethered_no_battery"
var wheel_rates : Array [ float ] = [ 0.0 , 0.0 , 0.0 , 0.0 ]
var motor_currents : Array [ float ] = [ 0.0 , 0.0 , 0.0 , 0.0 ]
var centers : Array [ Vector3 ] = [ Vector3 ( -0.1145 , 0.048 , -0.09 ) , Vector3 ( 0.1145 , 0.048 , -0.09 ) , Vector3 ( -0.1145 , 0.048 , 0.09 ) , Vector3 ( 0.1145 , 0.048 , 0.09 ) ]
var radius : = 0.048
var contact_stiffness : = 2500.0
var contact_damping : = 15.0

func _init ( ) -> void :
    var file : = FileAccess . open ( "res://physics/physics_estimates.json" , FileAccess . READ )
    if file : config = JSON . parse_string ( file . get_as_text ( ) )

func configure ( body : RigidBody3D , name : String = "bench_tethered_no_battery" ) -> void :
    configuration = name
    var c : Dictionary = config . configurations [ name ]
    body . mass = float ( c . mass_kg )
    body . center_of_mass_mode = RigidBody3D . CENTER_OF_MASS_MODE_CUSTOM
    var cm : Array = c . center_of_mass_godot_m
    body . center_of_mass = Vector3 ( cm [ 0 ] , cm [ 1 ] , cm [ 2 ] )
    var inertia : Array = c . inertia_diagonal_godot_kg_m2
    body . inertia = Vector3 ( inertia [ 0 ] , inertia [ 1 ] , inertia [ 2 ] )
    body . gravity_scale = 1.0
    body . linear_damp = 0.0
    body . angular_damp = 0.0
    body . can_sleep = false
    body . continuous_cd = true

func reset ( ) -> void :
    for i in range ( 4 ) :
        wheel_rates [ i ] = 0.0
        motor_currents [ i ] = 0.0

func step ( body : RigidBody3D , target_v : float , target_w : float , delta : float , surface : String = "wood" ) -> Dictionary :
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
    var up : = body . global_basis . y . normalized ( )
    var forward : = - body . global_basis . z . normalized ( )
    var center_of_mass_world : = body . global_transform * body . center_of_mass
    var space : = body . get_world_3d ( ) . direct_space_state
    var normal_loads : Array [ float ] = [ 0.0 , 0.0 , 0.0 , 0.0 ]
    var contacts : = 0
    var peak_slip : = 0.0
    var traction_limited : = false
    var current_limited : = false
    var substeps : = maxi ( 1 , ceili ( delta / 0.001 ) )
    var h : = delta / substeps
    for i in range ( 4 ) :
        var axle : = body . global_transform * centers [ i ]
        var start : = axle + up * 0.02
        var query : = PhysicsRayQueryParameters3D . create ( start , axle - up * ( radius + 0.015 ) , 1 )
        query . exclude = [ body . get_rid ( ) ]
        var hit : = space . intersect_ray ( query )
        var point : = axle - up * radius
        var normal : = up
        var tangent_forward : = forward
        var tangent_right : = body . global_basis . x . normalized ( )
        var contact_v : = 0.0
        var lateral_v : = 0.0
        var normal_load : = 0.0
        if not hit . is_empty ( ) :
            point = hit . position
            normal = hit . normal
            var compression : = radius - ( axle - point ) . dot ( up )
            var point_velocity : = body . linear_velocity + body . angular_velocity . cross ( point - center_of_mass_world )
            if compression >= 0.0 and normal . dot ( up ) > 0.3 :
                normal_load = maxf ( 0.0 , contact_stiffness * minf ( compression , 0.015 ) - contact_damping * point_velocity . dot ( normal ) )
                if normal_load > 0.0 : contacts += 1
                tangent_forward = ( forward - normal * forward . dot ( normal ) ) . normalized ( )
                tangent_right = tangent_forward . cross ( normal ) . normalized ( )
                contact_v = point_velocity . dot ( tangent_forward )
                lateral_v = point_velocity . dot ( tangent_right )
            normal_loads [ i ] = normal_load
        var fx_sum : = 0.0
        var fy_sum : = 0.0
        for _substep in range ( substeps ) :

            var desired_omega : = clampf ( ( target_v + target_w * centers [ i ] . x ) / radius , - omega0 , omega0 )
            var duty : = clampf ( desired_omega / omega0 + 0.5 * ( desired_omega - wheel_rates [ i ] ) / omega0 , -1.0 , 1.0 )
            if absf ( target_v ) + absf ( target_w ) < 1e-05 : duty = 0.0
            var requested_current : = ( 12.0 * duty - ke * wheel_rates [ i ] ) / resistance
            var current : = clampf ( requested_current , - ilim , ilim )
            motor_currents [ i ] = current
            current_limited = current_limited or absf ( requested_current ) > ilim
            var slip : = radius * wheel_rates [ i ] - contact_v
            if normal_load > 0.0 : peak_slip = maxf ( peak_slip , absf ( slip ) )
            var fx : = 0.0
            var fy : = 0.0
            if normal_load > 0.0 :
                fx = 40.0 * slip
                fy = - mu_lat * normal_load * tanh ( lateral_v / 0.08 )


                var arm : = point - center_of_mass_world
                var minimum_inertia : = minf ( body . inertia . x , minf ( body . inertia . y , body . inertia . z ) )
                var inverse_effective_mass : = 4.0 / body . mass + 4.0 * arm . cross ( tangent_right ) . length_squared ( ) / minimum_inertia
                var lateral_limit : = absf ( lateral_v ) / maxf ( delta * inverse_effective_mass , 1e-05 )
                fy = clampf ( fy , - lateral_limit , lateral_limit )
                var ellipse : = sqrt ( pow ( fx / maxf ( mu * normal_load , 0.001 ) , 2.0 ) + pow ( fy / maxf ( mu_lat * normal_load , 0.001 ) , 2.0 ) )
                if ellipse > 1.0 :
                    fx /= ellipse
                    fy /= ellipse
                    traction_limited = true
            var torque : = kt * current - fx * radius - rr * normal_load * radius * tanh ( wheel_rates [ i ] / 0.3 )
            var loss : = friction * signf ( wheel_rates [ i ] )
            if absf ( wheel_rates [ i ] ) < 0.02 : loss = clampf ( torque , - friction , friction )
            var next_rate : = wheel_rates [ i ] + ( torque - loss ) / iw * h
            if signf ( next_rate ) != signf ( wheel_rates [ i ] ) and absf ( torque ) <= friction : next_rate = 0.0
            wheel_rates [ i ] = next_rate
            fx_sum += fx
            fy_sum += fy
        if normal_load > 0.0 :
            body . apply_force ( normal * normal_load + tangent_forward * fx_sum / substeps + tangent_right * fy_sum / substeps , point - body . global_position )
    return { "v" : body . linear_velocity . dot ( forward ) , "w" : body . angular_velocity . dot ( up ) , "wheel_rates" : wheel_rates . duplicate ( ) , "motor_currents" : motor_currents . duplicate ( ) , "normal_loads_n" : normal_loads , "contact_count" : contacts , "traction_limited" : traction_limited , "current_limited" : current_limited , "slipping" : peak_slip > 0.03 , "peak_longitudinal_slip_m_s" : peak_slip , "mass_kg" : body . mass , "surface" : surface , "configuration" : configuration , "model" : "estimated rigid chassis with four compliant ray contacts, uncalibrated" }
