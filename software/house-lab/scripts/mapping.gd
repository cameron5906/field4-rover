class_name Field4Mapping
extends RefCounted
















const DEFAULT_WIDTH : int = 120
const DEFAULT_HEIGHT : int = 100
const DEFAULT_RESOLUTION : float = 0.1
const DEFAULT_ORIGIN : Vector2 = Vector2 ( -6.0 , -5.0 )
const FREE_EVIDENCE : float = -0.4
const OCCUPIED_EVIDENCE : float = 0.85
const MIN_LOG_ODDS : float = -4.0
const MAX_LOG_ODDS : float = 4.0
const OCCUPIED_THRESHOLD : float = 0.6
const FREE_THRESHOLD : float = -0.2
const UNKNOWN_COLOR : Color = Color ( 0.43 , 0.47 , 0.51 , 1.0 )
const FREE_COLOR : Color = Color ( 0.9 , 0.93 , 0.94 , 1.0 )
const OCCUPIED_COLOR : Color = Color ( 0.08 , 0.13 , 0.18 , 1.0 )

var width : int = DEFAULT_WIDTH
var height : int = DEFAULT_HEIGHT
var resolution : float = DEFAULT_RESOLUTION
var origin : Vector2 = DEFAULT_ORIGIN
var estimated_pose : Vector3 = Vector3 . ZERO


var log_odds : PackedFloat32Array = PackedFloat32Array ( )
var observed : PackedByteArray = PackedByteArray ( )
var known_count : int = 0
var occupied_count : int = 0
var free_count : int = 0
var scan_count : int = 0
var last_valid_ray_count : int = 0
var last_hit_count : int = 0
var odometry_distance : float = 0.0
var odometry_time : float = 0.0



var enable_wheel_bias : bool = false
var left_wheel_scale : float = 1.003
var right_wheel_scale : float = 0.997
var wheel_track : float = 0.2

var _image : Image
var _image_dirty : bool = true


func _init (
    map_width : int = DEFAULT_WIDTH ,
    map_height : int = DEFAULT_HEIGHT ,
    cell_resolution : float = DEFAULT_RESOLUTION ,
    map_origin : Vector2 = DEFAULT_ORIGIN
) -> void :
    assert ( map_width > 0 and map_height > 0 , "Map dimensions must be positive" )
    assert ( is_finite ( cell_resolution ) and cell_resolution > 0.0 , "Resolution must be positive" )
    assert ( map_origin . is_finite ( ) , "Map origin must be finite" )
    width = map_width
    height = map_height
    resolution = cell_resolution
    origin = map_origin
    reset ( )




func reset ( pose : Vector3 = Vector3 . ZERO ) -> void :
    log_odds . resize ( width * height )
    log_odds . fill ( 0.0 )
    observed . resize ( width * height )
    observed . fill ( 0 )
    known_count = 0
    occupied_count = 0
    free_count = 0
    scan_count = 0
    last_valid_ray_count = 0
    last_hit_count = 0
    odometry_distance = 0.0
    odometry_time = 0.0
    reset_pose ( pose )
    _image = Image . create_empty ( width , height , false , Image . FORMAT_RGB8 )
    _image . fill ( UNKNOWN_COLOR )
    _image_dirty = true




func reset_pose ( pose : Vector3 = Vector3 . ZERO ) -> void :
    if not pose . is_finite ( ) :
        return
    estimated_pose = Vector3 ( pose . x , pose . y , wrapf ( pose . z , - PI , PI ) )





func advance_odometry ( v : float , w : float , dt : float ) -> Vector3 :
    if not is_finite ( v ) or not is_finite ( w ) or not is_finite ( dt ) or dt <= 0.0 :
        return estimated_pose
    var linear : float = v
    var angular : float = w
    if enable_wheel_bias and wheel_track > 0.0 :
        var left : float = ( v - w * wheel_track * 0.5 ) * left_wheel_scale
        var right : float = ( v + w * wheel_track * 0.5 ) * right_wheel_scale
        linear = ( left + right ) * 0.5
        angular = ( right - left ) / wheel_track
    var yaw : float = estimated_pose . z
    var next_yaw : float = yaw + angular * dt
    if absf ( angular ) < 1e-08 :
        estimated_pose . x -= linear * dt * sin ( yaw )
        estimated_pose . y -= linear * dt * cos ( yaw )
    else :
        estimated_pose . x += linear / angular * ( cos ( next_yaw ) - cos ( yaw ) )
        estimated_pose . y -= linear / angular * ( sin ( next_yaw ) - sin ( yaw ) )
    estimated_pose . z = wrapf ( next_yaw , - PI , PI )
    odometry_distance += absf ( linear * dt )
    odometry_time += dt
    return estimated_pose








func update_scan (
    ranges : PackedFloat32Array ,
    pose : Vector3 ,
    lidar_offset : Vector2 = Vector2 ( 0.0 , 0.062 ) ,
    max_range : float = 6.0
) -> void :
    last_valid_ray_count = 0
    last_hit_count = 0
    if ranges . is_empty ( ) or not pose . is_finite ( ) or not lidar_offset . is_finite ( ) :
        return
    if not is_finite ( max_range ) or max_range <= 0.0 :
        return
    reset_pose ( pose )
    var c : float = cos ( pose . z )
    var s : float = sin ( pose . z )
    var sensor : Vector2 = Vector2 ( pose . x , pose . y ) + Vector2 (
        c * lidar_offset . x + s * lidar_offset . y ,
        - s * lidar_offset . x + c * lidar_offset . y
    )
    var free_mask : = PackedByteArray ( )
    var hit_mask : = PackedByteArray ( )
    free_mask . resize ( width * height )
    hit_mask . resize ( width * height )
    free_mask . fill ( 0 )
    hit_mask . fill ( 0 )
    for i in range ( ranges . size ( ) ) :
        var distance : float = ranges [ i ]
        if is_nan ( distance ) or distance <= 0.0 :
            continue
        var measured_hit : bool = is_finite ( distance ) and distance < max_range
        distance = minf ( distance , max_range )
        var angle : float = pose . z + float ( i ) * TAU / float ( ranges . size ( ) )
        var direction : Vector2 = Vector2 ( - sin ( angle ) , - cos ( angle ) )

        if absf ( direction . x ) < 1e-07 :
            direction . x = 0.0
        if absf ( direction . y ) < 1e-07 :
            direction . y = 0.0
        var endpoint : Vector2 = sensor + direction * distance
        var endpoint_cell : Vector2i = world_to_cell ( endpoint )
        var map_hit : bool = measured_hit and is_cell_in_bounds ( endpoint_cell )
        if map_hit :
            hit_mask [ _index ( endpoint_cell ) ] = 1
        _mark_free_ray ( sensor , endpoint , endpoint_cell , map_hit , free_mask )
        last_valid_ray_count += 1
        if measured_hit :
            last_hit_count += 1
    if last_valid_ray_count == 0 :
        return
    for index in range ( width * height ) :
        if hit_mask [ index ] != 0 :
            _apply_evidence ( index , OCCUPIED_EVIDENCE )
        elif free_mask [ index ] != 0 :
            _apply_evidence ( index , FREE_EVIDENCE )
    scan_count += 1
    _image_dirty = true


func world_to_cell ( world_xz : Vector2 ) -> Vector2i :
    var local : Vector2 = ( world_xz - origin ) / resolution
    return Vector2i ( floori ( local . x ) , floori ( local . y ) )


func cell_to_world ( cell : Vector2i ) -> Vector2 :
    return origin + ( Vector2 ( cell ) + Vector2 ( 0.5 , 0.5 ) ) * resolution


func is_cell_in_bounds ( cell : Vector2i ) -> bool :
    return cell . x >= 0 and cell . y >= 0 and cell . x < width and cell . y < height


func get_probability ( cell : Vector2i ) -> float :
    if not is_cell_in_bounds ( cell ) :
        return 0.5
    return 1.0 / ( 1.0 + exp ( - log_odds [ _index ( cell ) ] ) )




func get_image ( ) -> Image :
    if _image_dirty :
        for z in range ( height ) :
            for x in range ( width ) :
                var index : int = z * width + x
                var color : Color = UNKNOWN_COLOR
                if observed [ index ] != 0 :
                    if log_odds [ index ] >= OCCUPIED_THRESHOLD :
                        color = OCCUPIED_COLOR
                    elif log_odds [ index ] <= FREE_THRESHOLD :
                        color = FREE_COLOR
                    else :
                        color = UNKNOWN_COLOR . lerp ( FREE_COLOR , 0.25 )
                _image . set_pixel ( x , z , color )
        _image_dirty = false
    return _image


func get_metrics ( ) -> Dictionary :
    var total : int = width * height
    return {
        "mode" : "Occupancy mapping from supplied estimated pose (not full SLAM)" ,
        "width" : width ,
        "height" : height ,
        "resolution_m" : resolution ,
        "origin_xz" : origin ,
        "total_cells" : total ,
        "known_count" : known_count ,
        "occupied_count" : occupied_count ,
        "free_count" : free_count ,
        "uncertain_count" : known_count - occupied_count - free_count ,
        "unknown_count" : total - known_count ,
        "known_fraction" : float ( known_count ) / float ( total ) ,
        "known_area_m2" : known_count * resolution * resolution ,
        "scan_count" : scan_count ,
        "last_valid_ray_count" : last_valid_ray_count ,
        "last_hit_count" : last_hit_count ,
        "odometry_distance_m" : odometry_distance ,
        "odometry_time_s" : odometry_time ,
        "estimated_pose" : estimated_pose ,
        "wheel_bias_enabled" : enable_wheel_bias ,
    }


func _index ( cell : Vector2i ) -> int :
    return cell . y * width + cell . x


func _apply_evidence ( index : int , evidence : float ) -> void :
    var previous : float = log_odds [ index ]
    if observed [ index ] == 0 :
        observed [ index ] = 1
        known_count += 1
    else :
        if previous >= OCCUPIED_THRESHOLD :
            occupied_count -= 1
        elif previous <= FREE_THRESHOLD :
            free_count -= 1
    var updated : float = clampf ( previous + evidence , MIN_LOG_ODDS , MAX_LOG_ODDS )
    log_odds [ index ] = updated
    updated = log_odds [ index ]
    if updated >= OCCUPIED_THRESHOLD :
        occupied_count += 1
    elif updated <= FREE_THRESHOLD :
        free_count += 1




func _clip_segment ( start : Vector2 , end : Vector2 ) -> PackedVector2Array :
    var delta : Vector2 = end - start
    var t_enter : float = 0.0
    var t_exit : float = 1.0
    var inset : float = resolution * 0.0001
    var low : Vector2 = origin + Vector2 . ONE * inset
    var high : Vector2 = origin + Vector2 ( width , height ) * resolution - Vector2 . ONE * inset
    for axis in range ( 2 ) :
        if absf ( delta [ axis ] ) < 1e-09 :
            if start [ axis ] < low [ axis ] or start [ axis ] > high [ axis ] :
                return PackedVector2Array ( )
            continue
        var near_t : float = ( low [ axis ] - start [ axis ] ) / delta [ axis ]
        var far_t : float = ( high [ axis ] - start [ axis ] ) / delta [ axis ]
        if near_t > far_t :
            var swap : float = near_t
            near_t = far_t
            far_t = swap
        t_enter = maxf ( t_enter , near_t )
        t_exit = minf ( t_exit , far_t )
        if t_enter > t_exit :
            return PackedVector2Array ( )
    return PackedVector2Array ( [ start + delta * t_enter , start + delta * t_exit ] )



func _mark_free_ray (
    start : Vector2 ,
    end : Vector2 ,
    hit_cell : Vector2i ,
    has_hit : bool ,
    free_mask : PackedByteArray
) -> void :
    var clipped : PackedVector2Array = _clip_segment ( start , end )
    if clipped . is_empty ( ) :
        return
    var a : Vector2 = ( clipped [ 0 ] - origin ) / resolution
    var b : Vector2 = ( clipped [ 1 ] - origin ) / resolution
    var delta : Vector2 = b - a
    var cell : = Vector2i ( floori ( a . x ) , floori ( a . y ) )
    var final_cell : = Vector2i ( floori ( b . x ) , floori ( b . y ) )
    var step : = Vector2i ( int ( signf ( delta . x ) ) , int ( signf ( delta . y ) ) )
    var t_delta : = Vector2 ( INF , INF )
    var t_max : = Vector2 ( INF , INF )
    if step . x != 0 :
        t_delta . x = 1.0 / absf ( delta . x )
        var boundary_x : float = float ( cell . x + 1 ) if step . x > 0 else float ( cell . x )
        t_max . x = ( boundary_x - a . x ) / delta . x
    if step . y != 0 :
        t_delta . y = 1.0 / absf ( delta . y )
        var boundary_y : float = float ( cell . y + 1 ) if step . y > 0 else float ( cell . y )
        t_max . y = ( boundary_y - a . y ) / delta . y

    for _iteration in range ( width + height + 2 ) :
        if is_cell_in_bounds ( cell ) and not ( has_hit and cell == hit_cell ) :
            free_mask [ _index ( cell ) ] = 1
        if cell == final_cell :
            break
        if t_max . x < t_max . y :
            cell . x += step . x
            t_max . x += t_delta . x
        elif t_max . y < t_max . x :
            cell . y += step . y
            t_max . y += t_delta . y
        else :


            cell += step
            t_max += t_delta
