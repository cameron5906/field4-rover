extends Control

var result : Dictionary = { }
var image_size : = Vector2 ( 640 , 360 )
var age_ms : = 0
func _draw ( ) -> void :
    if result . is_empty ( ) : return
    var scale_value : = minf ( size . x / image_size . x , size . y / image_size . y )
    var offset : = ( size - image_size * scale_value ) * 0.5
    for item in result . get ( "detections" , [ ] ) :
        var b : Array = item . get ( "bbox" , [ ] )
        if b . size ( ) != 4 : continue
        var rect : = Rect2 ( offset + Vector2 ( b [ 0 ] , b [ 1 ] ) * scale_value , Vector2 ( b [ 2 ] , b [ 3 ] ) * scale_value )
        var color : = Color ( "eabb6d" ) if item . get ( "status" , "" ) in [ "bearing_only" , "bearing_unusable" ] else Color ( "83d0b7" )
        if item . get ( "status" , "" ) == "dynamic" : color = Color ( "82b7e2" )
        draw_rect ( rect , color , false , 1.4 )
        var label : = "%s %.0f%%" % [ item . get ( "label" , "?" ) , float ( item . get ( "score" , 0 ) ) * 100 ]
        draw_string ( ThemeDB . fallback_font , Vector2 ( clampf ( rect . position . x , 1 , size . x - 110 ) , maxf ( 12 , rect . position . y - 3 ) ) , label , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , color )
    for pose in result . get ( "poses" , [ ] ) :
        var points : Array = pose . get ( "points" , [ ] )
        if points . size ( ) < 25 : continue
        var color : = Color ( "ff8269" ) if pose . get ( "gesture" , "" ) == "raised" else Color ( "d9eacd" )
        for pair in [ [ 11 , 12 ] , [ 11 , 13 ] , [ 13 , 15 ] , [ 12 , 14 ] , [ 14 , 16 ] , [ 11 , 23 ] , [ 12 , 24 ] , [ 23 , 24 ] ] :
            var a : Dictionary = points [ pair [ 0 ] ] ; var b : Dictionary = points [ pair [ 1 ] ]
            if minf ( float ( a . get ( "visibility" , 0 ) ) , float ( b . get ( "visibility" , 0 ) ) ) < 0.5 : continue
            var p : = offset + Vector2 ( a . x * image_size . x , a . y * image_size . y ) * scale_value
            var q : = offset + Vector2 ( b . x * image_size . x , b . y * image_size . y ) * scale_value
            draw_line ( p , q , color , 1.3 )
