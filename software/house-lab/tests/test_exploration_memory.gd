extends SceneTree

const Memory = preload("res://scripts/exploration_memory.gd")
var _checks: int = 0
var _failures: Array[String] = []


func _initialize() -> void:
    _test_goal_region_merge()
    _test_negative_progress()
    _test_frontier_commitment()
    _test_information_reopening()
    _test_local_hint_reopening()
    _test_information_is_consumed_at_finish()
    _test_epoch_and_clock_reset()
    _test_finite_inspection_yaw()
    _test_finite_stationary_inspection()
    _test_candidate_flicker()
    _test_unknown_map_never_completes()
    _test_invalid_measurements()
    _test_many_regions_remain_finite()
    _test_cancellation_is_not_inspection()
    _test_external_route_pauses_inspection()
    if _failures.is_empty():
        print("EXPLORATION_MEMORY_PASS checks=%d" % _checks)
        quit(0)
    else:
        for failure in _failures:
            push_error(failure)
        print("EXPLORATION_MEMORY_FAIL checks=%d failures=%d" % [_checks, _failures.size()])
        quit(1)


func _check(condition: bool, description: String) -> void:
    _checks += 1
    if not condition:
        _failures.append(description)


func _fresh(known: int = 100):
    var memory = Memory.new()
    memory.observe(7, known, 0, Vector2.ZERO)
    return memory


func _test_goal_region_merge() -> void:
    var memory = _fresh()
    _check(memory.begin_goal(Vector2.ZERO, 0), "Unattempted observed-free low-information view is usable once")
    memory.finish_goal("reached")
    _check(not memory.can_attempt(Vector2(0.69, 0.0), 0), "Nearby goal IDs merge into one attempted spatial region")
    _check(not memory.begin_goal(Vector2(0.3, 0.0)), "A finished nearby goal cannot restart without information")
    _check(not memory.can_attempt(Vector2(0.7, 0.0)), "Region boundary is included")
    _check(memory.can_attempt(Vector2(0.71, 0.0)), "Different spatial regions remain available")
    # A query in the old region must not shift its anchor and suppress this point.
    memory.can_attempt(Vector2(0.65, 0.0))
    _check(memory.begin_goal(Vector2(1.3, 0.0), 0), "Region anchors do not drift through a chain of nearby queries")
    memory.finish_goal("stuck")
    memory.finish_goal("stuck")
    var context: Dictionary = memory.status_context()
    _check(context.attempted_regions == 2 and context.total_attempts == 2, "Repeated finish is idempotent and records stay compact")
    _check(context.no_information_attempts == 2, "Both reached-without-gain and stuck goals count negative information results")
    memory.observe(7, 100, 120000, Vector2.ZERO)
    _check(not memory.can_attempt(Vector2.ZERO), "Time alone does not expire stagnant goal memory")
    # Exercise the spatial hash around negative bucket coordinates.
    _check(memory.begin_goal(Vector2(-2.0, -2.0)), "Negative map coordinates are valid")
    memory.finish_goal("unreachable")
    _check(not memory.can_attempt(Vector2(-2.3, -1.7)), "Neighboring negative buckets merge consistently")


func _test_negative_progress() -> void:
    var memory = _fresh()
    memory.begin_goal(Vector2(3.0, 0.0))
    memory.observe(7, 100, 1000, Vector2(1.0, 0.0))
    _check(is_equal_approx(memory.status_context().goal_progress_m, 1.0), "Closer measured pose records positive progress")
    memory.observe(7, 100, 2000, Vector2(-2.0, 0.0))
    var context: Dictionary = memory.status_context()
    _check(is_equal_approx(context.goal_best_distance_m, 2.0), "Moving away does not improve best goal distance")
    _check(context.last_goal_progress_age_ms == 1000, "Moving away does not refresh the progress watchdog")
    memory.observe(7, 100, 3000, Vector2(1.0, 0.0))
    _check(memory.status_context().last_goal_progress_age_ms == 2000, "Returning to the old best is not new progress")
    memory.observe(7, 100, 4000, Vector2(1.04, 0.0))
    _check(memory.status_context().last_goal_progress_age_ms == 3000, "Sub-threshold pose jitter does not refresh progress")
    memory.observe(7, 100, 5000, Vector2(1.10, 0.0))
    _check(memory.status_context().last_goal_progress_age_ms == 0, "Cumulative meaningful approach does refresh progress")


func _test_frontier_commitment() -> void:
    var memory = _fresh()
    var first: Vector2 = Vector2(2.0, 0.0)
    var second: Vector2 = Vector2(-2.0, 0.0)
    _check(memory.begin_goal(first), "First frontier commits")
    memory.observe(7, 110, 6000, Vector2(0.1, 0.0))
    _check(memory.begin_goal(first), "Same-goal semantic refresh is allowed without restarting")
    _check(memory.begin_goal(first + Vector2(0.2, 0.0)), "Same anchored region refresh retains the original commitment")
    _check(not memory.begin_goal(second), "Another frontier cannot replace an unfinished commitment")
    var context: Dictionary = memory.status_context()
    _check(context.goal.x == first.x and context.goal.z == first.y, "Refresh leaves the original goal unchanged")
    _check(context.goal_age_ms == 6000, "Refresh does not reset the goal age")
    memory.observe(7, 124, 9000, Vector2(0.2, 0.0))
    memory.finish_goal("route_changed")
    _check(memory.status_context().no_information_attempts == 0, "Refresh preserves the original information baseline")
    _check(not memory.begin_goal(first), "A finished commitment cannot immediately reopen using its own gain")
    _check(memory.begin_goal(second), "Finishing a failed commitment releases another frontier")


func _test_information_reopening() -> void:
    var memory = _fresh()
    memory.begin_goal(Vector2(2.0, 0.0))
    memory.finish_goal("stuck")
    memory.observe(7, 123, 1000, Vector2.ZERO)
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Small map growth stays below reopening hysteresis")
    memory.observe(7, 124, 2000, Vector2.ZERO)
    _check(memory.can_attempt(Vector2(2.0, 0.0)), "Meaningful measured growth reopens an attempted region")
    _check(memory.begin_goal(Vector2(2.1, 0.0)), "A nearby changed candidate can use the reopened region")
    memory.finish_goal("reached")
    memory.observe(7, 100, 3000, Vector2.ZERO)
    memory.observe(7, 124, 4000, Vector2.ZERO)
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Count dips and recovery do not manufacture information")
    memory.observe(7, 147, 5000, Vector2.ZERO)
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Every repeated attempt requires fresh gain")
    memory.observe(7, 148, 6000, Vector2.ZERO)
    _check(memory.can_attempt(Vector2(2.0, 0.0)), "Reopening is renewable instead of a permanent blacklist")
    _check(memory.status_context().reopened_attempts == 1, "Only a begun retry counts as reopened, not repeated eligibility queries")


func _test_local_hint_reopening() -> void:
    var memory = _fresh()
    var point: Vector2 = Vector2(2.0, 0.0)
    memory.begin_goal(point, 0)
    memory.finish_goal("reached")
    _check(not memory.can_attempt(point, 6), "A changed hint alone cannot defeat no-information memory")
    memory.observe(7, 107, 1000, Vector2.ZERO)
    _check(not memory.can_attempt(point, 6), "Even a strong frontier hint needs genuine minimum growth")
    memory.observe(7, 108, 2000, Vector2.ZERO)
    _check(not memory.can_attempt(point, 1), "One additional unknown neighbor is below local reopening hysteresis")
    _check(memory.can_attempt(point, 2), "A stronger local frontier and measured growth reopen a region earlier")
    _check(memory.begin_goal(point, 2), "Local-evidence reopening is accepted by begin_goal too")
    memory.finish_goal("stuck")
    memory.observe(7, 116, 3000, Vector2.ZERO)
    _check(not memory.can_attempt(point, 2), "The same frontier hint does not fund another no-information repeat")
    _check(memory.can_attempt(point, 4), "Subsequently stronger local evidence remains usable")


func _test_information_is_consumed_at_finish() -> void:
    var memory = _fresh()
    memory.begin_goal(Vector2(2.0, 0.0))
    memory.observe(7, 150, 1000, Vector2(2.0, 0.0))
    memory.finish_goal("reached")
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Information gathered on arrival is consumed by that attempt")
    _check(memory.status_context().no_information_attempts == 0, "A productive goal is not mislabeled no-information")
    memory.observe(7, 174, 2000, Vector2(2.0, 0.0))
    _check(memory.can_attempt(Vector2(2.0, 0.0)), "Fresh information after arrival allows later reconsideration")


func _test_epoch_and_clock_reset() -> void:
    var memory = _fresh()
    memory.begin_goal(Vector2(2.0, 0.0))
    memory.finish_goal("stuck")
    memory.observe(7, 100, 1000, Vector2.ZERO)
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Same-map refresh retains attempted regions")
    memory.observe(8, 100, 2000, Vector2.ZERO)
    _check(memory.can_attempt(Vector2(2.0, 0.0)), "A new map epoch clears old spatial attempts")
    _check(memory.status_context().total_attempts == 0, "Map reset clears counters too")
    memory.begin_goal(Vector2(2.0, 0.0))
    memory.finish_goal("stuck")
    memory.observe(8, 100, 0, Vector2.ZERO)
    _check(memory.can_attempt(Vector2(2.0, 0.0)), "Clock rewind also starts a clean observation epoch")
    memory.begin_goal(Vector2(2.0, 0.0))
    memory.reset(9)
    var context: Dictionary = memory.status_context()
    _check(not context.goal_active and context.total_attempts == 0 and context.map_epoch == 9, "Explicit reset drops commitment and all prior-map memory")
    _check(not memory.can_attempt(Vector2(2.0, 0.0)), "Reset waits for a fresh measured observation")
    _check(not memory.search_exhausted(), "A reset is not instant search exhaustion")


func _test_finite_inspection_yaw() -> void:
    var memory = _fresh()
    memory.observe(7, 100, 1000, Vector2.ZERO)
    memory.note_inspection(0.5, false)
    memory.note_inspection(0.5, false)
    _check(is_equal_approx(memory.status_context().inspection_yaw_rad, 0.5), "A repeated controller read cannot double-count one measured yaw sample")
    for step in range(2, 14):
        memory.observe(7, 100, step * 1000, Vector2.ZERO)
        memory.note_inspection(-0.5 if step % 2 == 0 else 0.5, false)
    _check(memory.search_exhausted(), "Bounded back-and-forth inspection cannot spin forever")
    var context: Dictionary = memory.status_context()
    _check(context.inspection_yaw_rad <= TAU, "Inspection accumulation remains bounded")
    _check(context.exhaustion_reason == "no_new_reachable_viewpoint", "Exhaustion is an explicit search pause")
    _check(not context.mission_complete, "Search exhaustion never claims semantic success")
    memory.note_inspection(0.0, true)
    _check(not memory.search_exhausted(), "New eligible candidates can continue search after a completed inspection")
    _check(memory.begin_goal(Vector2(2.0, 0.0)), "Inspection exhaustion does not blacklist a new observed-free region")
    memory.observe(7, 100, 14000, Vector2.ZERO)
    memory.note_inspection(1.0, false)
    _check(not memory.search_exhausted() and memory.status_context().inspection_yaw_rad == 0.0, "Goal-following yaw is not charged as blind inspection")
    memory.finish_goal("reached")
    memory.note_inspection(1.0, false)
    _check(memory.status_context().inspection_yaw_rad == 0.0, "Travel yaw in the finishing sample is not charged to the next inspection")


func _test_finite_stationary_inspection() -> void:
    var memory = _fresh()
    memory.observe(7, 100, 23999, Vector2.ZERO)
    memory.note_inspection(0.0, false)
    _check(not memory.search_exhausted(), "Stationary inspection remains available inside its time budget")
    memory.observe(7, 100, 24000, Vector2.ZERO)
    memory.note_inspection(0.0, false)
    _check(memory.search_exhausted(), "A safety-blocked zero-motion inspection also has a finite timeout")
    memory.observe(7, 123, 25000, Vector2.ZERO)
    _check(memory.search_exhausted(), "Tiny count changes do not perpetually restart inspection")
    memory.observe(7, 124, 26000, Vector2.ZERO)
    _check(not memory.search_exhausted(), "Meaningful new observations reopen an exhausted search")
    _check(memory.status_context().inspection_elapsed_ms == 0, "Reopened inspection gets a fresh bounded interval")
    memory.observe(7, 124, 50000, Vector2.ZERO)
    _check(memory.search_exhausted(), "Reopening does not remove the finite inspection limit")


func _test_candidate_flicker() -> void:
    var memory = _fresh()
    memory.observe(7, 100, 23000, Vector2.ZERO)
    memory.note_inspection(0.0, true)
    _check(not memory.search_exhausted(), "Eligible candidates prevent idle exhaustion")
    memory.observe(7, 100, 24000, Vector2.ZERO)
    memory.note_inspection(0.0, false)
    _check(memory.search_exhausted(), "A transient unselected candidate does not endlessly reset the no-information budget")


func _test_unknown_map_never_completes() -> void:
    var memory = _fresh(0)
    _check(not memory.can_attempt(Vector2.ZERO), "An entirely unknown map cannot authorize a navigation goal")
    _check(not memory.begin_goal(Vector2(1.0, 0.0)), "No known cells means no measured-free goal can begin")
    memory.observe(7, 0, 24000, Vector2.ZERO)
    memory.note_inspection(0.0, false)
    var context: Dictionary = memory.status_context()
    _check(memory.search_exhausted(), "Unknown-map safe inspection is bounded too")
    _check(context.exhaustion_reason == "insufficient_observed_map", "Unknown-map exhaustion is distinguished from absence of fresh viewpoints")
    _check(not context.observed_map_available and not context.mission_complete, "Unknown space is never treated as completed coverage or mission success")
    _check(not context.has("coverage") and not context.has("found"), "Memory has no coverage or found shortcut")
    memory.observe(7, 24, 25000, Vector2.ZERO)
    _check(not memory.search_exhausted() and memory.can_attempt(Vector2(1.0, 0.0)), "Fresh real observations can recover an initially unknown map")


func _test_invalid_measurements() -> void:
    var memory = _fresh()
    _check(not memory.can_attempt(Vector2(NAN, 0.0)), "NaN goal cannot enter region lookup")
    _check(not memory.begin_goal(Vector2(INF, 0.0)), "Infinite goal cannot begin")
    _check(not memory.can_attempt(Vector2.ZERO, -1), "Invalid frontier hint cannot authorize a goal")
    memory.observe(7, -10, 1000, Vector2.ZERO)
    _check(memory.status_context().known_cells == 100, "Invalid negative known count is ignored")
    memory.observe(7, 100, -1, Vector2.ZERO)
    _check(memory.status_context().map_epoch == 7, "Invalid negative time does not reset memory")
    memory.observe(7, 100, 1000, Vector2.ZERO)
    memory.note_inspection(NAN, false)
    _check(memory.status_context().inspection_yaw_rad == 0.0, "Nonfinite yaw cannot poison or exhaust the inspection budget")
    var unlocalized = Memory.new()
    unlocalized.observe(7, 100, 0, Vector2(NAN, 0.0))
    _check(not unlocalized.begin_goal(Vector2.ZERO), "A goal requires a finite measured pose")
    memory.observe(7, 100, 2000, Vector2(NAN, 0.0))
    _check(not memory.can_attempt(Vector2.ZERO), "A stale previously valid pose cannot stand in for a current invalid pose")
    _check(not memory.status_context().estimated_pose_available, "Invalid current localization is explicit in status")
    memory.observe(7, 100, 3000, Vector2.ZERO)
    _check(memory.begin_goal(Vector2(2.0, 0.0)), "Finite measured localization permits recovery")
    memory.observe(7, 100, 4000, Vector2(NAN, 0.0))
    _check(not memory.begin_goal(Vector2(2.0, 0.0)), "A committed-goal refresh cannot bypass current pose validation")


func _test_many_regions_remain_finite() -> void:
    var memory = _fresh()
    var points: Array[Vector2] = []
    for y in range(-8, 9):
        for x in range(-8, 9):
            var point: Vector2 = Vector2(x, y) * 1.6
            points.append(point)
            _check(memory.begin_goal(point, 0), "Each distinct observed-free region is initially usable")
            memory.finish_goal("reached_without_new_information")
    _check(memory.status_context().attempted_regions == points.size(), "A large stagnant search records each region exactly once")
    _check(memory.status_context().total_attempts == points.size(), "Attempt counters remain exact for many regions")
    for point in points:
        _check(not memory.can_attempt(point + Vector2(0.2, -0.3), 0), "Spatial buckets suppress nearby repeats across a large positive/negative map")
    memory.observe(7, 100, 24000, Vector2.ZERO)
    memory.note_inspection(0.0, false)
    _check(memory.search_exhausted(), "Finite observed-free regions end in a bounded pause instead of a cycle")
    memory.observe(7, 124, 25000, Vector2.ZERO)
    _check(not memory.search_exhausted(), "New map evidence reopens the bounded many-region search")
    _check(memory.can_attempt(points[0]), "Even the oldest region survives until meaningful reopening")


func _test_cancellation_is_not_inspection() -> void:
    var cancellations := ["mission_changed", "new_semantic_goal", "visual_verification_handoff",
        "semantic_memory_reinspection", "semantic_inspect", "semantic_search", "semantic_found",
        "semantic_blocked", "object_arrived_inspect_current_room", "canceled", "superseded"]
    for outcome in cancellations:
        var memory = _fresh()
        var target := Vector2(2.0, 0.0)
        _check(memory.begin_goal(target), "Begin goal before cancellation: " + outcome)
        memory.observe(7, 100, 1000, Vector2.ZERO)
        memory.finish_goal(outcome)
        var context: Dictionary = memory.status_context()
        _check(not context.goal_active and context.total_attempts == 0 and context.attempted_regions == 0,
            "Cancellation clears commitment without an examined region: " + outcome)
        _check(context.no_information_attempts == 0 and memory.can_attempt(target),
            "Canceled unreached goal remains available: " + outcome)
        _check(memory.begin_goal(target), "Canceled goal can be deliberately reselected: " + outcome)
        memory.finish_goal("stuck")
        _check(memory.status_context().attempted_regions == 1 and not memory.can_attempt(target),
            "Real subsequent failure still consumes region: " + outcome)
    var memory = _fresh()
    var target := Vector2(2.0, 0.0)
    memory.begin_goal(target); memory.finish_goal("reached")
    memory.observe(7,124,1000,Vector2.ZERO)
    memory.begin_goal(target); memory.finish_goal("new_semantic_goal")
    _check(memory.status_context().total_attempts == 1 and memory.status_context().attempted_regions == 1,
        "Canceling reopened goal preserves earlier actual attempt history")
    _check(memory.can_attempt(target), "Canceling reopened goal does not consume the new information grant")


func _test_external_route_pauses_inspection() -> void:
    var memory = _fresh()
    for second in range(1,61):
        memory.observe(7,100,second*1000,Vector2(second*0.1,0.0))
        memory.note_inspection(0.3,false,true)
    _check(not memory.search_exhausted(), "Long external remembered route is not idle search")
    _check(memory.status_context().inspection_elapsed_ms == 0 and memory.status_context().inspection_yaw_rad == 0.0,
        "External route travel time and turns consume no inspection budget")
    memory.observe(7,100,61000,Vector2(6,0))
    memory.note_inspection(0.3,false,false)
    _check(memory.status_context().inspection_elapsed_ms == 0 and memory.status_context().inspection_yaw_rad == 0.0,
        "Finishing external route begins a fresh budget without charging the last travel sample")
    memory.observe(7,100,84999,Vector2(6,0)); memory.note_inspection(0.0,false)
    _check(not memory.search_exhausted(), "Post-route inspection has its own full finite budget")
    memory.observe(7,100,85000,Vector2(6,0)); memory.note_inspection(0.0,false)
    _check(memory.search_exhausted(), "Post-route inspection still stops at24 seconds")
