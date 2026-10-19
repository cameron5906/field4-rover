extends Control
var sim : Node
func _draw ( ) -> void :
    if sim == null : return
    var p : Vector3 = sim . mapper . estimated_pose
    var origin : Vector2 = sim . mapper . origin
    var at : = Vector2 ( ( p . x - origin . x ) / ( sim . mapper . width * sim . mapper . resolution ) * size . x , ( p . y - origin . y ) / ( sim . mapper . height * sim . mapper . resolution ) * size . y )

    var nav : Dictionary = sim . local_navigation_context
    var points : Array = nav . get ( "route_points" , [ ] )
    var previous : = at
    for point in points :
        var next : = Vector2 ( ( float ( point . x ) - origin . x ) / ( sim . mapper . width * sim . mapper . resolution ) * size . x , ( float ( point . z ) - origin . y ) / ( sim . mapper . height * sim . mapper . resolution ) * size . y )
        draw_line ( previous , next , Color ( "438bbc" ) , 2 )
        previous = next
    for candidate in sim . semantic_planner . candidates :
        var marker : = Vector2 ( ( float ( candidate . x ) - origin . x ) / ( sim . mapper . width * sim . mapper . resolution ) * size . x , ( float ( candidate . z ) - origin . y ) / ( sim . mapper . height * sim . mapper . resolution ) * size . y )
        draw_circle ( marker , 3 , Color ( "77a476" ) )
        if candidate . has ( "annotation_id" ) :
            draw_string ( ThemeDB . fallback_font , marker + Vector2 ( 4 , -3 ) , str ( candidate . annotation_id ) , HORIZONTAL_ALIGNMENT_LEFT , -1 , 11 , Color ( "224e40" ) )
    if sim . vision :
        for landmark in sim . vision . landmarks :
            if landmark . get ( "status" , "" ) != "confirmed" or sim . sim_time - float ( landmark . get ( "last_seen_s" , -100 ) ) > 5 : continue
            var spot : = Vector2 ( ( float ( landmark . x ) - origin . x ) / ( sim . mapper . width * sim . mapper . resolution ) * size . x , ( float ( landmark . z ) - origin . y ) / ( sim . mapper . height * sim . mapper . resolution ) * size . y )
            var radius : = maxf ( 3 , float ( landmark . uncertainty_m ) * size . x / ( sim . mapper . width * sim . mapper . resolution ) )
            draw_arc ( spot , radius , 0 , TAU , 20 , Color ( 0.33 , 0.63 , 0.67 , 0.55 ) , 1 )
            draw_circle ( spot , 2 , Color ( "396b72" ) )
            draw_string ( ThemeDB . fallback_font , spot + Vector2 ( 4 , -4 ) , str ( landmark . label ) + " " + str ( landmark . id ) , HORIZONTAL_ALIGNMENT_LEFT , -1 , 9 , Color ( "294d52" ) )
    draw_circle ( at , 5 , Color ( "f5b554" ) )
    var direction : = Vector2 ( - sin ( p . z ) , - cos ( p . z ) )
    draw_line ( at , at + direction * 17 , Color ( "f5b554" ) , 3 )

    draw_arc ( at , 10 , 0 , TAU , 24 , Color ( "f5b554" ) , 1 )
