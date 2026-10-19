extends SceneTree

const Client = preload("res://scripts/perception_client.gd")
var checks: Dictionary = {}
var received: Array = []

func _initialize() -> void:
    call_deferred("run")

func check(label: String, passed: bool) -> void:
    checks[label] = passed
    if not passed:
        push_error("FAIL: " + label)

func write_result(client, captured: int) -> void:
    var file := FileAccess.open(client.ipc.path_join("result.json"), FileAccess.WRITE)
    file.store_string(JSON.stringify({"ok": true, "epoch": client.pending_metadata.epoch,
        "frame_id": client.pending_metadata.frame_id, "capture_monotonic_ms": captured, "poses": []}))
    file.close()

func run() -> void:
    # Exercise real image/request publication, but no inference/model process.
    await create_timer(0.65).timeout
    var client = Client.new()
    client.ipc = ProjectSettings.globalize_path("res://test_results/cadence_fixture_" + str(Time.get_ticks_usec()))
    DirAccess.make_dir_recursive_absolute(client.ipc)
    client.worker_ready = true
    var pixels := Image.create(8, 8, false, Image.FORMAT_RGB8)
    pixels.fill(Color.WHITE)
    var now := Time.get_ticks_msec()
    for bad in [null, true, "1", NAN, INF, 1.5, -1, now + 10000]:
        check("reject_timestamp_" + str(bad), not client.submit(pixels, {"capture_monotonic_ms": bad}))
    check("invalid_input_cannot_increment_frame", client.frame_id == 0 and not client.pending)
    var captured := Time.get_ticks_msec() - 120
    var metadata := {"capture_monotonic_ms": captured, "timestamp_s": 1.0, "pose": [0, 0, 0]}
    check("real_request_published", client.submit(pixels, metadata))
    check("original_capture_is_cadence_anchor", client.last_capture_ms == captured)
    check("publication_does_not_retime_capture", client.sent_ms > captured and client.pending_metadata.capture_monotonic_ms == captured)
    check("metadata_input_unchanged", not metadata.has("frame_id") and not metadata.has("submission_begin_monotonic_ms"))
    check("producer_timing_present", client.pending_metadata.submission_begin_monotonic_ms >= captured and client.pending_metadata.request_published_monotonic_ms >= client.pending_metadata.submission_begin_monotonic_ms)
    check("one_in_flight_backpressure", not client.can_capture(captured + 9999) and not client.submit(pixels, {"capture_monotonic_ms": captured + 1}))
    client.process_id = OS.create_process("/bin/sleep", PackedStringArray(["20"])) # Idle local test child, no inference.
    client.completed.connect(func(result): received.append(result))
    write_result(client, captured)
    client._process(0.1)
    check("one_matching_result_consumed", received.size() == 1 and not client.pending)
    check("original_age_preserved", received[0].age_ms >= 120 and received[0].capture_monotonic_ms == captured)
    var timing: Dictionary = received[0].producer_timing
    check("timing_clocks_monotonic", timing.capture_monotonic_ms <= timing.submission_begin_monotonic_ms and timing.submission_begin_monotonic_ms <= timing.request_published_monotonic_ms and timing.request_published_monotonic_ms <= timing.consumed_monotonic_ms)
    check("cannot_exceed_profile_rate", not client.can_capture(captured + 332))
    check("render_and_publish_cost_not_double_delayed", client.can_capture(captured + 333) and captured + 333 < client.sent_ms + 333)
    check("reordered_capture_rejected", not client.submit(pixels, {"capture_monotonic_ms": captured - 1}))
    client.capture_enabled = false
    check("disabled_capture_stays_disabled", not client.can_capture(captured + 9999))
    client.capture_enabled = true
    client.worker_ready = false
    check("unready_worker_stays_blocked", not client.can_capture(captured + 9999))
    client.worker_ready = true
    # Reset invalidates the old epoch even if its worker response arrives later.
    client.pending = true
    client.reset_map()
    client._process(0.1)
    check("epoch_reset_discards_in_flight_reply", received.size() == 1 and client.discarded == 1 and client.latest.is_empty())
    client._stop_worker()
    client.free()
    var passed := checks.values().count(true)
    print("CAPTURE_CADENCE_V073 " + JSON.stringify({"passed": passed, "total": checks.size(), "checks": checks}))
    quit(0 if passed == checks.size() else 1)
