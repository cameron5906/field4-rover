extends SceneTree
const Mapper = preload("res://scripts/mapping.gd")
const Planner = preload("res://scripts/semantic_planner.gd")
var checks := 0
var failures := 0
func check(ok: bool, name: String):
    checks += 1
    if not ok:
        failures += 1
        push_error(name)
func fixture():
    var m = Mapper.new(24,24,0.2,Vector2(-2.4,-2.4))
    m.observed.fill(1); m.log_odds.fill(-2.0)
    m.known_count = 24*24; m.free_count = 24*24; m.scan_count = 1
    return m
func _initialize():
    var m = fixture()
    var p = Planner.new()
    p.reset("Find a bedroom",1)
    p.observe(m,1000)
    check(p.ensure_local_goal(m),"select initial observed-free viewpoint")
    var original: Vector2i = p._goal_cell
    var started: int = p._last_progress_ms
    var candidate: Dictionary = p.subgoal.duplicate(true)
    candidate.cell_x = original.x; candidate.cell_y = original.y
    p._observation_ms = 7000
    check(p._select_goal(candidate,"semantic",m),"same-region semantic refresh accepts commitment")
    check(p._goal_cell == original and p._last_progress_ms == started,"same-region refresh preserves goal and progress clock")
    p.observe(m,11000)
    check(not p._has_goal and p._stuck_count == 1,"no-motion route becomes stuck despite semantic refresh")
    check(not p.exploration_memory.can_attempt(m.cell_to_world(original),0),"stuck region remains ineligible without new observations")
    # A NEW semantic mission gets a bounded camera revisit even in mapped space.
    p = Planner.new(); p.reset("Explore the house",2); p.observe(m,12000)
    var view := Vector2(1.4,0.2)
    check(p.exploration_memory.begin_goal(view,0),"precondition: examined old-map viewpoint")
    p.exploration_memory.finish_goal("goal_reached_inspect_view")
    p._remember_room_view({"room_type":"bedroom","confidence":0.9,"evidence":["bed"]},Vector3(view.x,view.y,1.2),{"observation_seq":1,"captured_at_ms":12000},m.get_instance_id())
    p.change_mission("Find a bedroom",3); p.observe(m,13000)
    var remembered: Dictionary = {}
    for item in p.candidates:
        if item.kind == "remembered_camera_viewpoint": remembered = item
    check(not remembered.is_empty(),"new mission can revisit remembered room without map-count gain")
    if not remembered.is_empty():
        check(p._select_goal(remembered,"semantic",m),"select bounded semantic memory revisit")
        p.exploration_memory.observe(m.get_instance_id(),m.known_count,45000,Vector2.ZERO)
        p.exploration_memory.note_inspection(0.0,false,p._has_goal)
        check(not p.controller_context(m).search_exhausted,"external remembered route cannot be halted by expired blind-inspection budget")
        check(p.needs_plan(45000),"long active remembered route retains semantic reassessment")
        check(p.exploration_memory.status_context().inspection_elapsed_ms == 0,"remembered travel does not age no-goal inspection")
        var at: Vector2 = m.cell_to_world(p._goal_cell)
        m.estimated_pose = Vector3(at.x,at.y,0)
        p.observe(m,46000)
        var context = p.controller_context(m)
        check(context.hold_for_visual_confirmation,"revisit arrival holds for fresh visual proof")
        check(is_equal_approx(float(context.inspection_heading_rad),1.2),"revisit restores observed camera heading")
        check(not context.allow_mission_complete,"remembered bed alone never completes mission")
        p._reinspect = false; p._inspect_until_ms = -1
        m.estimated_pose = Vector3.ZERO
        p.observe(m,70000)
        var repeats := 0
        for item in p.candidates:
            if item.kind == "remembered_camera_viewpoint": repeats += 1
        check(repeats == 0,"same mission cannot repeatedly prioritize remembered view after cooldown")
    # A mission change before arrival cannot label its old destination examined.
    m = fixture(); p = Planner.new(); p.reset("Explore the house",4); p.observe(m,71000)
    check(p.ensure_local_goal(m),"select a goal before mission replacement")
    var unvisited: Vector2 = m.cell_to_world(p._goal_cell)
    p.change_mission("Find a bedroom",5); p.observe(m,72000)
    check(p.exploration_memory.can_attempt(unvisited,0),"mission replacement retains the unvisited region's eligibility")
    check(p.exploration_memory.status_context().attempted_regions == 0,"mission replacement does not invent an inspected destination")
    # Fully mapped, unchanged reachable space has a finite observation budget.
    m = fixture(); p = Planner.new(); p.reset("Find a bedroom",4)
    var now := 1000
    var goals := 0
    for tick in range(100):
        p.observe(m,now)
        if p.ensure_local_goal(m):
            var destination: Vector2 = m.cell_to_world(p._goal_cell)
            m.estimated_pose = Vector3(destination.x,destination.y,m.estimated_pose.z)
            goals += 1
        else:
            m.estimated_pose.z = wrapf(m.estimated_pose.z+0.4,-PI,PI)
        now += 1100
        if p.controller_context(m).search_exhausted: break
    check(p.controller_context(m).search_exhausted,"known unchanged room eventually pauses instead of cycling")
    check(p.status != "found","exhaustion is never mission success")
    check(goals < 30,"bounded number of distinct no-information viewpoints")
    print("ROOM_SEARCH_INTEGRATION " + JSON.stringify({"checks":checks,"failed":failures,"goals":goals,"elapsed_fixture_ms":now,"synthetic_measured_map":true,"paid_api_calls":0}))
    quit(1 if failures else 0)
