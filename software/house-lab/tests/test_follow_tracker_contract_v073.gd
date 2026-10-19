extends SceneTree
const Follow = preload("res://scripts/person_follow.gd")
const Guard = preload("res://scripts/gesture_guard.gd")

func _initialize() -> void:
    var document = JSON.parse_string(FileAccess.get_file_as_string("res://test_results/follow_tracker_contract_synthetic_v073.json"))
    if not document is Dictionary:
        push_error("Run build_follow_contract_fixture_v073.py first")
        quit(1)
        return
    var failures: Array = []
    var results: Array = []
    for scenario in document.scenarios:
        var follow = Follow.new()
        var guard = Guard.new()
        follow.start(12, 4, 1000)
        for step in scenario.steps:
            var now: int = int(step.metadata.capture_monotonic_ms) + 20
            if step.explicit_restart:
                follow.start(13, 4, 8200)
            if step.get("action", "") == "stop":
                follow.abort("Explicit test manual Stop")
            elif step.get("action", "") == "restart":
                follow.start(14, 4, int(step.metadata.capture_monotonic_ms) - 20)
            guard.observe(step.result, now)
            follow.observe(step.result, step.metadata, now)
            var command: Dictionary = follow.step(Vector3.ZERO, now, guard.active)
            var moving: bool = command.linear_m_s > 0
            var passed: bool = moving == step.expected_moving and follow.target_track_id == int(step.expected_identity)
            var other_identity := -1
            if step.get("expect_observed_other_identity", false):
                other_identity = int(step.result.poses[0].follow_evidence.get("track_id", -1))
                passed = passed and other_identity > 0 and other_identity != follow.target_track_id
            if step.get("expect_confirmed_other_identity", false):
                passed = passed and step.result.poses[0].follow_evidence.track_state == "confirmed" and step.result.poses[0].follow_evidence.gesture_release_eligible
            if step.get("expect_non_pose_continuation", false):
                var observation: Dictionary = step.result.person_observations[0]
                passed = passed and step.result.poses.is_empty() and not observation.has("points") and observation.observation_source == "detector_lidar_continuation" and not observation.follow_evidence.gesture_release_eligible
            results.append({"scenario": scenario.name, "frame": step.result.frame_id, "moving": moving,
                "selected_id": follow.target_track_id, "state": command.task_status,
                "same_target_gesture_latched": follow.gesture_latched, "observed_other_identity": other_identity,
                "global_guard_active": guard.active, "passed": passed})
            if not passed:
                failures.append(results[-1])
    print("FOLLOW_TRACKER_CONTRACT_RESULTS " + JSON.stringify(results))
    print("FOLLOW_TRACKER_CONTRACT_TOTAL %d/%d" % [results.size() - failures.size(), results.size()])
    quit(0 if failures.is_empty() else 1)
