extends Control

var ranges : = PackedFloat32Array ( )
var sampled_at : = -100.0
var now : = 0.0
var max_range : = 6.0
var stale_after : = 0.45
var sensor_offset_xy : = Vector2 ( 0.0 , -0.062 )

static func scan_to_points ( values : PackedFloat32Array , maximum : float = 6.0 , offset : Vector2 = Vector2 ( 0 , -0.062 ) ) -> PackedVector2Array :
    var points : = PackedVector2Array ( )
    if values . is_empty ( ) or not is_finite ( maximum ) or maximum <= 0.15 : return points
    for i in range ( values . size ( ) ) :
        var distance : float = values [ i ]
        if not is_finite ( distance ) or distance < 0.15 or distance >= maximum : continue
        var angle : = float ( i ) * TAU / values . size ( )
        points . append ( Vector2 ( - sin ( angle ) * distance , cos ( angle ) * distance ) + offset )
    return points

func set_scan ( values : PackedFloat32Array , timestamp : float ) -> void :
    ranges = values . duplicate ( )
    sampled_at = timestamp
    queue_redraw ( )

func clear_scan ( ) -> void :
    ranges . clear ( )
    sampled_at = -100
    queue_redraw ( )

func fresh ( ) -> bool :
    return sampled_at >= 0 and now >= sampled_at and now - sampled_at <= stale_after

func visible_points ( ) -> PackedVector2Array :
    return scan_to_points ( ranges , max_range , sensor_offset_xy ) if fresh ( ) else PackedVector2Array ( )

func _draw ( ) -> void :
    var style : = StyleBoxFlat . new ( )
    style . bg_color = Color ( 0.94 , 0.96 , 0.93 , 0.9 )
    style . set_corner_radius_all ( 8 )
    draw_style_box ( style , Rect2 ( Vector2 . ZERO , size ) )
    var center : = size * 0.5
    var scale_m : = minf ( size . x , size . y ) * 0.4 / max_range
    var font : = get_theme_default_font ( )
    var grid : = Color ( 0.45 , 0.58 , 0.53 , 0.25 )
    for meters in [ 3.0 , 6.0 ] :
        draw_arc ( center , meters * scale_m , 0 , TAU , 64 , grid , 1 )
    draw_line ( center + Vector2 ( - max_range * scale_m , 0 ) , center + Vector2 ( max_range * scale_m , 0 ) , grid , 1 )
    draw_line ( center + Vector2 ( 0 , - max_range * scale_m ) , center + Vector2 ( 0 , max_range * scale_m ) , grid , 1 )
    for point in visible_points ( ) :
        draw_circle ( center + Vector2 ( point . x , - point . y ) * scale_m , 1.65 , Color ( "458479" ) )

    draw_circle ( center , 3 , Color ( "c89551" ) )
    draw_line ( center , center + Vector2 ( 0 , -12 ) , Color ( "c89551" ) , 1.5 )
    draw_string ( font , Vector2 ( 12 , 18 ) , "+Y forward" , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , Color ( "526662" ) )
    draw_string ( font , Vector2 ( size . x - 52 , center . y - 7 ) , "+X" , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , Color ( "526662" ) )
    draw_string ( font , Vector2 ( 12 , size . y - 11 ) , "3 / 6 m" , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , Color ( "526662" ) )
    var status : = "%d ms" % roundi ( ( now - sampled_at ) * 1000 ) if fresh ( ) else "STALE · no points"
    draw_string ( font , Vector2 ( 88 , size . y - 11 ) , status , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , Color ( "526662" ) )
