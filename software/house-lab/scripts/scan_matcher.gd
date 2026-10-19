class_name Field4ScanMatcher
extends RefCounted






const MAX_TRANSLATION : float = 0.05
const MAX_ROTATION : float = PI / 90.0
const MIN_MAP_SCANS : int = 8
const MIN_KNOWN_CELLS : int = 150
const MIN_STABLE_CELLS : int = 35
const STABLE_LOG_ODDS : float = 1.7
const MIN_HITS : int = 18
const MIN_SCORE : float = 0.55
const MIN_IMPROVEMENT : float = 0.004
const MIN_OVERLAP : float = 0.6
const MIN_EIGENVALUE : float = 0.0015
const MIN_EIGEN_RATIO : float = 0.035
const MIN_DISTANT_MARGIN : float = 0.002
const MIN_MOTION_METERS : float = 0.004
const MIN_MOTION_RADIANS : float = PI / 900.0
const FIELD_SIGMA : float = 0.12
const FIELD_RADIUS : int = 4

var last_result : Dictionary = { }
var _have_previous_pose : bool = false
var _previous_output : Vector3 = Vector3 . ZERO
var _width : int = 0
var _height : int = 0
var _resolution : float = 0.1
var _origin : Vector2 = Vector2 . ZERO
var _field : PackedFloat32Array = PackedFloat32Array ( )
var _stable : PackedByteArray = PackedByteArray ( )
var _points : PackedVector2Array = PackedVector2Array ( )
var _evaluations : int = 0




func reset ( initial_estimate : Vector3 = Vector3 . ZERO ) -> void :
    _have_previous_pose = initial_estimate . is_finite ( )
    _previous_output = initial_estimate
    last_result = { }
    _field . clear ( )
    _stable . clear ( )
    _points . clear ( )





func correct_pose (
    mapper ,
    ranges : PackedFloat32Array ,
    predicted_pose : Vector3 ,
    lidar_offset : Vector2 = Vector2 ( 0.0 , 0.062 ) ,
    max_range : float = 6.0 ,
    min_range : float = 0.15 ,
    motion_allowed : bool = true
) -> Dictionary :
    _evaluations = 0
    var result : Dictionary = {
        "pose" : predicted_pose , "accepted" : false , "score" : 0.0 ,
        "baseline_score" : 0.0 , "reason" : "invalid_input" ,
        "translation_m" : 0.0 , "rotation_rad" : 0.0 ,
        "diagnostics" : { "mode" : "Bounded local scan-to-map matching (not global SLAM)" } ,
    }
    if not predicted_pose . is_finite ( ) or not lidar_offset . is_finite ( ) :
        return _finish ( result , false )
    if mapper == null or not is_finite ( max_range ) or not is_finite ( min_range ) :
        return _finish ( result )
    if min_range <= 0.0 or max_range <= min_range or ranges . is_empty ( ) :
        return _finish ( result )
    if not _have_previous_pose :
        result . reason = "first_scan"
        return _finish ( result )
    var travel : float = Vector2 ( predicted_pose . x - _previous_output . x ,
        predicted_pose . y - _previous_output . y ) . length ( )
    var turn : float = absf ( wrapf ( predicted_pose . z - _previous_output . z , - PI , PI ) )
    if not motion_allowed or ( travel < MIN_MOTION_METERS and turn < MIN_MOTION_RADIANS ) :
        result . reason = "stationary"
        return _finish ( result )
    if mapper . scan_count < MIN_MAP_SCANS or mapper . known_count < MIN_KNOWN_CELLS :
        result . reason = "map_not_established"
        return _finish ( result )
    _width = mapper . width
    _height = mapper . height
    _resolution = mapper . resolution
    _origin = mapper . origin
    if _width <= 1 or _height <= 1 or not is_finite ( _resolution ) or _resolution <= 0.0 :
        return _finish ( result )
    if not _origin . is_finite ( ) or mapper . log_odds . size ( ) != _width * _height :
        return _finish ( result )
    if mapper . observed . size ( ) != _width * _height :
        return _finish ( result )
    var stable_cells : int = _build_field ( mapper )
    result . diagnostics [ "stable_cells" ] = stable_cells
    if stable_cells < MIN_STABLE_CELLS :
        result . reason = "insufficient_stable_evidence"
        return _finish ( result )
    _points . clear ( )

    var stride : int = maxi ( 1 , ceili ( float ( ranges . size ( ) ) / 180.0 ) )
    for i in range ( 0 , ranges . size ( ) , stride ) :
        var distance : float = ranges [ i ]
        if not is_finite ( distance ) or distance < min_range or distance >= max_range :
            continue
        var angle : float = float ( i ) * TAU / float ( ranges . size ( ) )
        _points . append ( lidar_offset + Vector2 ( - sin ( angle ) , - cos ( angle ) ) * distance )
    result . diagnostics [ "hit_count" ] = _points . size ( )
    if _points . size ( ) < MIN_HITS :
        result . reason = "insufficient_returns"
        return _finish ( result )
    _evaluations = 0
    var baseline : float = _score ( predicted_pose )
    result . baseline_score = baseline
    result . score = baseline
    var best_pose : Vector3 = predicted_pose
    var best_score : float = baseline
    var candidates : Array [ Dictionary ] = [ ]


    for ix in range ( -2 , 3 ) :
        for iz in range ( -2 , 3 ) :
            for ia in range ( -2 , 3 ) :
                var delta : = Vector3 ( float ( ix ) * 0.025 , float ( iz ) * 0.025 ,
                    float ( ia ) * MAX_ROTATION * 0.5 )
                if not _within_bounds ( delta ) :
                    continue
                var pose : Vector3 = predicted_pose + delta
                var score : float = _score ( pose )
                candidates . append ( { "pose" : pose , "score" : score } )
                if score > best_score + 1e-08 :
                    best_score = score
                    best_pose = pose
    var coarse_best : Vector3 = best_pose
    for ix in range ( -1 , 2 ) :
        for iz in range ( -1 , 2 ) :
            for ia in range ( -1 , 2 ) :
                var pose : Vector3 = coarse_best + Vector3 ( float ( ix ) * 0.0125 ,
                    float ( iz ) * 0.0125 , float ( ia ) * MAX_ROTATION * 0.25 )
                if not _within_bounds ( pose - predicted_pose ) :
                    continue
                var score : float = _score ( pose )
                candidates . append ( { "pose" : pose , "score" : score } )
                if score > best_score + 1e-08 :
                    best_score = score
                    best_pose = pose
    var improvement : float = best_score - baseline
    result . diagnostics [ "candidate_score" ] = best_score
    result . diagnostics [ "score_improvement" ] = improvement
    if best_score < MIN_SCORE or improvement < MIN_IMPROVEMENT :
        result . reason = "no_confident_improvement"
        return _finish ( result )
    var overlap : float = _overlap ( best_pose )
    result . diagnostics [ "overlap_fraction" ] = overlap
    if overlap < MIN_OVERLAP :
        result . reason = "insufficient_overlap"
        return _finish ( result )


    var normal_geometry : Dictionary = _wall_normal_geometry ( best_pose )
    result . diagnostics [ "wall_normal_ratio" ] = normal_geometry . ratio
    result . diagnostics [ "wall_normal_samples" ] = normal_geometry . samples
    if normal_geometry . samples < 10 or normal_geometry . ratio < 0.12 :
        result . reason = "unobservable_geometry"
        return _finish ( result )


    var eigenvalues : Vector3 = _curvature_eigenvalues ( best_pose )
    var eigen_ratio : float = eigenvalues . x / maxf ( eigenvalues . z , 1e-09 )
    result . diagnostics [ "curvature_eigenvalues" ] = eigenvalues
    result . diagnostics [ "observability_ratio" ] = eigen_ratio
    if eigenvalues . x < MIN_EIGENVALUE or eigen_ratio < MIN_EIGEN_RATIO :
        result . reason = "unobservable_geometry"
        return _finish ( result )
    var distant_score : float = -1.0
    for candidate in candidates :
        var difference : Vector3 = candidate . pose - best_pose
        if Vector2 ( difference . x , difference . y ) . length ( ) >= 0.039 or absf ( difference . z ) >= MAX_ROTATION * 0.74 :
            distant_score = maxf ( distant_score , candidate . score )
    var margin : float = best_score - distant_score
    result . diagnostics [ "distant_peak_margin" ] = margin
    if distant_score >= 0.0 and margin < MIN_DISTANT_MARGIN :
        result . reason = "ambiguous_peak"
        return _finish ( result )
    var correction : Vector3 = best_pose - predicted_pose


    if not _within_bounds ( correction ) or best_score <= baseline :
        result . reason = "rejected_bounds_or_score"
        return _finish ( result )
    best_pose . z = wrapf ( best_pose . z , - PI , PI )
    result . pose = best_pose
    result . accepted = true
    result . score = best_score
    result . reason = "accepted"
    result . translation_m = Vector2 ( correction . x , correction . y ) . length ( )
    result . rotation_rad = correction . z
    return _finish ( result )


func _finish ( result : Dictionary , update_previous : bool = true ) -> Dictionary :
    result . diagnostics [ "score_evaluations" ] = _evaluations
    if update_previous and result . pose . is_finite ( ) :
        _previous_output = result . pose
        _have_previous_pose = true
    last_result = result
    return result


func _within_bounds ( delta : Vector3 ) -> bool :
    return delta . is_finite ( ) and Vector2 ( delta . x , delta . y ) . length ( ) <= MAX_TRANSLATION + 1e-08 and absf ( delta . z ) <= MAX_ROTATION + 1e-08




func _build_field ( mapper ) -> int :
    _field . resize ( _width * _height )
    _field . fill ( 0.0 )
    _stable . resize ( _width * _height )
    _stable . fill ( 0 )
    var stable_cells : int = 0
    for z in range ( _height ) :
        for x in range ( _width ) :
            var index : int = z * _width + x
            if mapper . observed [ index ] == 0 or mapper . log_odds [ index ] < STABLE_LOG_ODDS :
                continue
            stable_cells += 1
            _stable [ index ] = 1
            for dz in range ( - FIELD_RADIUS , FIELD_RADIUS + 1 ) :
                if z + dz < 0 or z + dz >= _height :
                    continue
                for dx in range ( - FIELD_RADIUS , FIELD_RADIUS + 1 ) :
                    if x + dx < 0 or x + dx >= _width :
                        continue
                    var distance_squared : float = float ( dx * dx + dz * dz ) * _resolution * _resolution
                    var likelihood : float = exp ( -0.5 * distance_squared / ( FIELD_SIGMA * FIELD_SIGMA ) )
                    var target : int = ( z + dz ) * _width + x + dx
                    _field [ target ] = maxf ( _field [ target ] , likelihood )
    return stable_cells


func _sample ( point : Vector2 ) -> float :
    var grid : Vector2 = ( point - _origin ) / _resolution - Vector2 ( 0.5 , 0.5 )
    var x : int = floori ( grid . x )
    var z : int = floori ( grid . y )
    if x < 0 or z < 0 or x + 1 >= _width or z + 1 >= _height :
        return 0.0
    var fx : float = grid . x - float ( x )
    var fz : float = grid . y - float ( z )
    var index : int = z * _width + x
    return lerpf ( lerpf ( _field [ index ] , _field [ index + 1 ] , fx ) ,
        lerpf ( _field [ index + _width ] , _field [ index + _width + 1 ] , fx ) , fz )


func _score ( pose : Vector3 ) -> float :
    _evaluations += 1
    var c : float = cos ( pose . z )
    var s : float = sin ( pose . z )
    var score : float = 0.0
    for point in _points :
        var world : = Vector2 ( pose . x + c * point . x + s * point . y ,
            pose . y - s * point . x + c * point . y )
        score += _sample ( world )
    return score / float ( _points . size ( ) )


func _overlap ( pose : Vector3 ) -> float :
    var c : float = cos ( pose . z )
    var s : float = sin ( pose . z )
    var supported : int = 0
    for point in _points :
        var world : = Vector2 ( pose . x + c * point . x + s * point . y ,
            pose . y - s * point . x + c * point . y )
        if _sample ( world ) >= 0.5 :
            supported += 1
    return float ( supported ) / float ( _points . size ( ) )





func _curvature_eigenvalues ( pose : Vector3 ) -> Vector3 :
    var steps : Array [ Vector3 ] = [ Vector3 ( 0.1 , 0 , 0 ) , Vector3 ( 0 , 0.1 , 0 ) ,
        Vector3 ( 0 , 0 , MAX_ROTATION ) ]
    var center : float = _score ( pose )
    var matrix : Array [ float ] = [ ]
    matrix . resize ( 9 )
    matrix . fill ( 0.0 )
    for i in range ( 3 ) :
        matrix [ i * 3 + i ] = 2.0 * center - _score ( pose + steps [ i ] ) - _score ( pose - steps [ i ] )
        for j in range ( i ) :
            var cross_value : float = - ( _score ( pose + steps [ i ] + steps [ j ] )
                - _score ( pose + steps [ i ] - steps [ j ] )
                - _score ( pose - steps [ i ] + steps [ j ] )
                + _score ( pose - steps [ i ] - steps [ j ] ) ) * 0.25
            matrix [ i * 3 + j ] = cross_value
            matrix [ j * 3 + i ] = cross_value
    return _symmetric_eigenvalues ( matrix )


func _symmetric_eigenvalues ( m : Array [ float ] ) -> Vector3 :
    var q : float = ( m [ 0 ] + m [ 4 ] + m [ 8 ] ) / 3.0
    var p : float = sqrt ( ( ( m [ 0 ] - q ) * ( m [ 0 ] - q ) + ( m [ 4 ] - q ) * ( m [ 4 ] - q )
        + ( m [ 8 ] - q ) * ( m [ 8 ] - q ) + 2.0 * ( m [ 1 ] * m [ 1 ] + m [ 2 ] * m [ 2 ] + m [ 5 ] * m [ 5 ] ) ) / 6.0 )
    if p < 1e-09 :
        return Vector3 ( q , q , q )
    var a : float = ( m [ 0 ] - q ) / p
    var b : float = m [ 1 ] / p
    var c : float = m [ 2 ] / p
    var d : float = ( m [ 4 ] - q ) / p
    var e : float = m [ 5 ] / p
    var f : float = ( m [ 8 ] - q ) / p
    var determinant : float = a * ( d * f - e * e ) - b * ( b * f - e * c ) + c * ( b * e - d * c )
    var phi : float = acos ( clampf ( determinant * 0.5 , -1.0 , 1.0 ) ) / 3.0
    var largest : float = q + 2.0 * p * cos ( phi )
    var smallest : float = q + 2.0 * p * cos ( phi + TAU / 3.0 )
    return Vector3 ( smallest , 3.0 * q - largest - smallest , largest )





func _wall_normal_geometry ( pose : Vector3 ) -> Dictionary :
    var c : float = cos ( pose . z )
    var s : float = sin ( pose . z )
    var normal_xx : float = 0.0
    var normal_xz : float = 0.0
    var normal_zz : float = 0.0
    var samples : int = 0
    var radius : int = maxi ( 2 , ceili ( 0.3 / _resolution ) )
    for point_index in range ( 0 , _points . size ( ) , 2 ) :
        var point : Vector2 = _points [ point_index ]
        var world : = Vector2 ( pose . x + c * point . x + s * point . y ,
            pose . y - s * point . x + c * point . y )
        var grid : Vector2 = ( world - _origin ) / _resolution
        var cx : int = floori ( grid . x )
        var cz : int = floori ( grid . y )
        var count : int = 0
        var mean : = Vector2 . ZERO
        var xx : float = 0.0
        var xz : float = 0.0
        var zz : float = 0.0
        for z in range ( maxi ( 0 , cz - radius ) , mini ( _height , cz + radius + 1 ) ) :
            for x in range ( maxi ( 0 , cx - radius ) , mini ( _width , cx + radius + 1 ) ) :
                if _stable [ z * _width + x ] == 0 :
                    continue

                var dx : float = float ( x - cx )
                var dz : float = float ( z - cz )
                mean += Vector2 ( dx , dz )
                xx += dx * dx
                xz += dx * dz
                zz += dz * dz
                count += 1
        if count < 4 :
            continue
        mean /= float ( count )
        xx = xx / float ( count ) - mean . x * mean . x
        xz = xz / float ( count ) - mean . x * mean . y
        zz = zz / float ( count ) - mean . y * mean . y
        var separation : float = sqrt ( ( xx - zz ) * ( xx - zz ) + 4.0 * xz * xz )
        if separation / maxf ( xx + zz , 1e-06 ) < 0.75 :
            continue
        var tangent_angle : float = 0.5 * atan2 ( 2.0 * xz , xx - zz )
        var normal : = Vector2 ( - sin ( tangent_angle ) , cos ( tangent_angle ) )
        normal_xx += normal . x * normal . x
        normal_xz += normal . x * normal . y
        normal_zz += normal . y * normal . y
        samples += 1
    var separation : float = sqrt ( ( normal_xx - normal_zz ) * ( normal_xx - normal_zz ) + 4.0 * normal_xz * normal_xz )
    var largest : float = ( normal_xx + normal_zz + separation ) * 0.5
    var smallest : float = ( normal_xx + normal_zz - separation ) * 0.5
    return { "samples" : samples , "ratio" : smallest / maxf ( largest , 1e-06 ) }
