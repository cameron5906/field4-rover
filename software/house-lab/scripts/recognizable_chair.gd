extends Node3D


var collidable : = true
func _material ( hex : String , roughness : float = 0.8 ) -> StandardMaterial3D :
    var mat : = StandardMaterial3D . new ( ) ; mat . albedo_color = Color ( hex ) ; mat . roughness = roughness ; return mat
func _round_part ( label : String , size : Vector3 , at : Vector3 , mat : Material ) -> MeshInstance3D :
    var mesh : = MeshInstance3D . new ( ) ; mesh . name = label
    var sphere : = SphereMesh . new ( ) ; sphere . radius = 0.5 ; sphere . height = 1 ; sphere . radial_segments = 24 ; sphere . rings = 12
    mesh . mesh = sphere ; mesh . position = at ; mesh . scale = size ; mesh . material_override = mat ; add_child ( mesh ) ; return mesh
func _box ( label : String , size : Vector3 , at : Vector3 , mat : Material ) -> MeshInstance3D :
    var mesh : = MeshInstance3D . new ( ) ; mesh . name = label ; var shape : = BoxMesh . new ( ) ; shape . size = size ; mesh . mesh = shape ; mesh . position = at ; mesh . material_override = mat ; add_child ( mesh ) ; return mesh
func _rod ( label : String , a : Vector3 , b : Vector3 , radius : float , mat : Material ) -> MeshInstance3D :
    var mesh : = MeshInstance3D . new ( ) ; mesh . name = label ; var shape : = CylinderMesh . new ( ) ; shape . top_radius = radius ; shape . bottom_radius = radius * 0.8 ; shape . height = a . distance_to ( b ) ; shape . radial_segments = 16 ; mesh . mesh = shape ; mesh . position = ( a + b ) * 0.5 ; mesh . basis = Basis ( Quaternion ( Vector3 . UP , ( b - a ) . normalized ( ) ) ) ; mesh . material_override = mat ; add_child ( mesh ) ; return mesh
func _ready ( ) -> void :
    name = "Upholstered_dining_chair"
    var oak : = _material ( "775335" )
    var textile : = _material ( "417a79" )
    var seam : = _material ( "325d5d" )


    for x in [ -1.0 , 1.0 ] :
        for z in [ -1.0 , 1.0 ] :
            _rod ( "Oak_leg" , Vector3 ( x * 0.245 , 0.035 , z * 0.232 ) , Vector3 ( x * 0.205 , 0.446 , z * 0.18 ) , 0.023 , oak )
        _box ( "Seat_side_rail" , Vector3 ( 0.036 , 0.062 , 0.45 ) , Vector3 ( x * 0.218 , 0.42 , 0 ) , oak )
        _rod ( "Back_upright" , Vector3 ( x * 0.215 , 0.424 , -0.192 ) , Vector3 ( x * 0.219 , 0.999 , -0.303 ) , 0.023 , oak )

        _rod ( "Arm_front_support" , Vector3 ( x * 0.246 , 0.452 , 0.118 ) , Vector3 ( x * 0.272 , 0.66 , 0.095 ) , 0.014 , oak )
        var arm : = _round_part ( "Armrest" , Vector3 ( 0.055 , 0.047 , 0.42 ) , Vector3 ( x * 0.271 , 0.672 , -0.035 ) , oak )
        arm . rotation_degrees . x = 5
    _box ( "Seat_front_rail" , Vector3 ( 0.45 , 0.06 , 0.032 ) , Vector3 ( 0 , 0.42 , 0.206 ) , oak )
    _box ( "Seat_core" , Vector3 ( 0.455 , 0.063 , 0.43 ) , Vector3 ( 0 , 0.464 , 0.008 ) , textile )
    _round_part ( "Seat_front_roll" , Vector3 ( 0.492 , 0.08 , 0.17 ) , Vector3 ( 0 , 0.475 , 0.181 ) , textile )
    for x in [ -0.202 , 0.202 ] : _round_part ( "Seat_soft_edge" , Vector3 ( 0.094 , 0.08 , 0.44 ) , Vector3 ( x , 0.475 , 0.005 ) , textile )
    var back : = _box ( "Upholstered_back" , Vector3 ( 0.442 , 0.356 , 0.072 ) , Vector3 ( 0 , 0.812 , -0.261 ) , textile )
    back . rotation_degrees . x = -11
    for x in [ -0.202 , 0.202 ] :
        var edge : = _round_part ( "Back_soft_edge" , Vector3 ( 0.086 , 0.369 , 0.078 ) , Vector3 ( x , 0.812 , -0.261 ) , textile ) ; edge . rotation_degrees . x = -11
    _round_part ( "Back_top_roll" , Vector3 ( 0.485 , 0.092 , 0.079 ) , Vector3 ( 0 , 0.987 , -0.295 ) , textile )
    _round_part ( "Back_bottom_roll" , Vector3 ( 0.465 , 0.079 , 0.08 ) , Vector3 ( 0 , 0.64 , -0.228 ) , textile )
    for x in [ -0.13 , 0.13 ] :
        var stitch : = _rod ( "Back_stitch" , Vector3 ( x , 0.684 , -0.19 ) , Vector3 ( x , 0.947 , -0.243 ) , 0.0017 , seam )
    if collidable :
        var body : = StaticBody3D . new ( ) ; body . collision_layer = 1 ; body . collision_mask = 0
        var col : = CollisionShape3D . new ( ) ; var box : = BoxShape3D . new ( ) ; box . size = Vector3 ( 0.58 , 1.04 , 0.58 ) ; col . shape = box ; col . position = Vector3 ( 0 , 0.52 , -0.015 ) ; body . add_child ( col ) ; add_child ( body )
