extends RefCounted
## Shared geometry only. The caller supplies its observed-free footprint and
## optional object stand-off predicate; this module never changes map evidence.
const NUMERIC_MARGIN_M := 0.000001

static func _line_intersects_rect(a: Vector2, b: Vector2, low: Vector2, high: Vector2) -> bool:
    var enter := 0.0
    var leave := 1.0
    var direction := b-a
    for axis in range(2):
        if absf(direction[axis]) < 1e-12:
            if a[axis] < low[axis] or a[axis] > high[axis]: return false
        else:
            var first: float = (low[axis]-a[axis])/direction[axis]
            var second: float = (high[axis]-a[axis])/direction[axis]
            enter = maxf(enter,minf(first,second)); leave = minf(leave,maxf(first,second))
            if enter > leave: return false
    return true

static func _check_touched(mapper, cell: Vector2i, start: Vector2, finish: Vector2, padding: float, safe_cell: Callable, checked: Dictionary) -> bool:
    # Include every incident endpoint/edge/corner cell. Neighbors also enclose
    # the small conservative arc-to-chord deviation supplied by command_safe.
    var margin := Vector2.ONE*(padding+NUMERIC_MARGIN_M)
    for y in range(-1,2):
        for x in range(-1,2):
            var adjacent := cell+Vector2i(x,y)
            if checked.has(adjacent): continue
            var low: Vector2 = mapper.origin+Vector2(adjacent)*mapper.resolution
            if not _line_intersects_rect(start,finish,low-margin,low+Vector2.ONE*mapper.resolution+margin): continue
            if not safe_cell.call(adjacent): return false
            checked[adjacent] = true
    return true

static func segment_safe(mapper, start: Vector2, finish: Vector2, safe_cell: Callable, padding_m: float = 0.0) -> bool:
    if not start.is_finite() or not finish.is_finite() or not is_finite(padding_m) or padding_m < 0 or padding_m >= mapper.resolution*0.25: return false
    var cell: Vector2i = mapper.world_to_cell(start)
    var last: Vector2i = mapper.world_to_cell(finish)
    var from: Vector2 = (start-mapper.origin)/mapper.resolution
    var to: Vector2 = (finish-mapper.origin)/mapper.resolution
    var direction: Vector2 = to-from
    var step_x: int = int(signf(direction.x))
    var step_y: int = int(signf(direction.y))
    var checked: Dictionary = {}
    if not _check_touched(mapper,cell,start,finish,padding_m,safe_cell,checked): return false
    if not _check_touched(mapper,last,start,finish,padding_m,safe_cell,checked): return false
    if cell == last: return true
    var next_x: float = (float(cell.x+(1 if step_x > 0 else 0))-from.x)/direction.x if step_x else INF
    var next_y: float = (float(cell.y+(1 if step_y > 0 else 0))-from.y)/direction.y if step_y else INF
    var delta_x: float = absf(1.0/direction.x) if step_x else INF
    var delta_y: float = absf(1.0/direction.y) if step_y else INF
    var limit: int = absi(last.x-cell.x)+absi(last.y-cell.y)+3
    for unused in range(limit):
        # The endpoint's complete incident-cell set is already checked. At an
        # exact mixed-direction endpoint, stepping diagonally can overshoot its
        # floor cell; stop by segment parameter, not by that cell identity.
        if minf(next_x,next_y) >= 1.0: return true
        if absf(next_x-next_y) < 0.000001:
            cell += Vector2i(step_x,step_y)
            next_x += delta_x; next_y += delta_y
        elif next_x < next_y:
            cell.x += step_x; next_x += delta_x
        else:
            cell.y += step_y; next_y += delta_y
        if not _check_touched(mapper,cell,start,finish,padding_m,safe_cell,checked): return false
        if cell == last: return true
    return false

static func command_safe(mapper, pose: Vector3, speed: float, yaw_rate: float, duration: float, safe_cell: Callable) -> bool:
    if not pose.is_finite() or not is_finite(speed) or not is_finite(yaw_rate) or not is_finite(duration) or duration <= 0.0 or duration > 1.0 or absf(speed) > 0.22 or absf(yaw_rate) > 0.65: return false
    var point := Vector2(pose.x,pose.y)
    var previous := point
    var steps: int = maxi(1,ceili(maxf(duration/0.1,absf(speed)*duration/(mapper.resolution*0.5))))
    var interval: float = duration/steps
    var sagitta: float = absf(speed/yaw_rate)*(1.0-cos(absf(yaw_rate)*interval*0.5)) if absf(yaw_rate) >= 0.000001 else 0.0
    # The exact constant-command circular arc lies within this sagitta tube
    # around each chord. Real skid/slip/dynamics still require measured guards.
    var cache: Dictionary = {}
    var cached_safe := func(cell: Vector2i) -> bool:
        if not cache.has(cell): cache[cell] = bool(safe_cell.call(cell))
        return bool(cache[cell])
    for step in range(1,steps+1):
        var t: float = duration*float(step)/steps
        var next: Vector2
        if absf(yaw_rate) < 0.000001:
            next = point+Vector2(-sin(pose.z),-cos(pose.z))*speed*t
        else:
            var theta: float = pose.z+yaw_rate*t
            next = point+Vector2(speed/yaw_rate*(cos(theta)-cos(pose.z)), -speed/yaw_rate*(sin(theta)-sin(pose.z)))
        if not segment_safe(mapper,previous,next,cached_safe,sagitta): return false
        previous = next
    return true
