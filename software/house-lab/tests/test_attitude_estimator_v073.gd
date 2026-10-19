extends SceneTree
const Estimator = preload("res://physics/attitude_estimator.gd")
const Imu = preload("res://physics/imu_sensor.gd")
const G := 9.80665
var checks := 0
var failed := 0
var sequence := 0
var timestamp := 0.0
var bias := Vector3(0.0008, -0.0008, 0.0008)

func check(value: bool, label: String) -> void:
    checks += 1
    if not value:
        failed += 1
        push_error(label)

func sample(accel: Vector3, gyro: Vector3) -> Dictionary:
    sequence += 1
    timestamp += 0.01
    return {"valid":true, "stale":false, "sequence":sequence, "timestamp_s":timestamp,
        "accel_m_s2":[accel.x, accel.y, accel.z], "gyro_rad_s":[gyro.x, gyro.y, gyro.z],
        "accel_saturated":false, "gyro_saturated":false}

func tick(filter, accel: Vector3, gyro: Vector3, speed: float = 0.0, demand: bool = false) -> Dictionary:
    var reading := sample(accel, gyro)
    return filter.update(reading, speed, demand, timestamp)

func initialize(filter, gravity: Vector3 = Vector3(0, 0, 1)) -> Dictionary:
    var state := {}
    for index in range(100): state = tick(filter, gravity * G, bias)
    return state

func _initialize() -> void:
    _run.call_deferred()

func _run() -> void:
    var f = Estimator.new()
    check(not f.get_estimate(0.0).valid, "startup has no trusted attitude")
    var state := tick(f, Vector3(0, 0, G), bias)
    check(not state.valid, "one still sample does not initialize")
    state = initialize(f)
    check(state.valid and state.projection_safe, "sustained stationary initialization")
    check(absf(state.roll_rad) < 0.0001 and absf(state.pitch_rad) < 0.0001, "level axes")
    check(Vector3(state.gyro_bias_rad_s[0], state.gyro_bias_rad_s[1], state.gyro_bias_rad_s[2]).distance_to(bias) < 0.00001, "stationary measured bias")
    var last: Dictionary = sample(Vector3(0, 0, G), bias)
    state = f.update(last, 0.0, false, timestamp)
    var repeated: Dictionary = f.update(last, 0.0, false, timestamp)
    check(state == repeated, "duplicate sample cannot integrate twice")
    check(not f.get_estimate(timestamp + 0.051).valid, "stale orientation invalidates read")
    # A norm-only tilt estimator accepts this force and falsely infers 0.20rad.
    check(absf(Vector3(0, 2, G).length() - G) < 0.55 and atan2(2, G) > 0.19, "reproduce raw-acceleration tilt defect")
    for index in range(100): state = tick(f, Vector3(0, 2, G), bias, (index + 1) * 0.02, true)
    check(state.valid and state.projection_safe, "one-second translation preserves bounded gyro estimate")
    check(absf(state.pitch_rad) < 0.0001 and not state.correction_accepted, "translation never rotates gravity estimate")
    check(state.uncertainty_rad > Estimator.INITIAL_UNCERTAINTY_RAD, "uncorrected uncertainty grows")
    for index in range(100): state = tick(f, Vector3(0, 0.5, G), bias, 2.0 + (index + 1) * 0.005, true)
    check(absf(state.pitch_rad) < 0.0001 and not state.correction_accepted, "norm-near-gravity acceleration rejected by innovation and encoder derivative")
    for index in range(1500): state = tick(f, Vector3(0, 0, G), bias, 2.5, true)
    check(state.valid and state.projection_safe, "long steady translation does not inevitably expire")
    check(state.reason == "qualified_steady_motion_gravity_correction", "steady motion qualified separately")
    check(state.uncertainty_rad >= Estimator.STEADY_UNCERTAINTY_RAD, "steady correction retains larger uncertainty")
    check(Vector3(state.gyro_bias_rad_s[0], state.gyro_bias_rad_s[1], state.gyro_bias_rad_s[2]).distance_to(bias) < 0.00001, "moving samples never relearn gyro bias")
    state = tick(f, Vector3(0, 0, G), bias, 0, false)
    check(state.uncertainty_rad >= Estimator.STEADY_UNCERTAINTY_RAD, "one stopped sample cannot claim stationary precision")
    for index in range(100): state = tick(f, Vector3(0, 0, G), bias)
    check(state.valid and state.uncertainty_rad < 0.007, "sustained stop restores tighter bound")
    # True tilt is propagated using measured angular rates, even while accel is unusable.
    for index in range(50):
        var gravity := Vector3(0, 0, 1).rotated(Vector3.RIGHT, -(index + 1) * 0.002)
        state = tick(f, gravity * G, bias + Vector3(0.2, 0, 0), 0.2, true)
    check(absf(state.pitch_rad - 0.1) < 0.002, "positive true pitch from gyro")
    check(state.valid and not state.projection_safe, "true pitch preserves rejection")
    for index in range(50):
        var gravity := Vector3(0, 0, 1).rotated(Vector3.RIGHT, -0.1 + (index + 1) * 0.002)
        state = tick(f, gravity * G, bias + Vector3(-0.2, 0, 0), 0.2, true)
    check(absf(state.pitch_rad) < 0.002, "opposite gyro rotates back")
    f.reset()
    state = initialize(f, Vector3(0, 0, 1).rotated(Vector3(0, 1, 0), 0.12))
    check(state.valid and absf(state.roll_rad - 0.12) < 0.002 and not state.projection_safe, "static tilted start is measured, not reset to level")
    f.reset()
    for index in range(120): state = tick(f, Vector3(0, 0, G), bias, 0.1, true)
    check(not state.valid, "moving startup cannot initialize")
    state = initialize(f)
    for index in range(700): state = tick(f, Vector3(0, 2, G), bias + Vector3(0.001, 0, 0), index * 0.02, true)
    check(not state.valid and not state.projection_safe and state.reason == "gyro_drift_or_correction_disagreement_exceeded", "long uncorrected gyro drift fails closed")
    for index in range(100): state = tick(f, Vector3(0, 0, G), bias)
    check(state.valid, "qualified rest can recover bounded drift")
    last = sample(Vector3(0, 0, G), bias)
    last.gyro_saturated = true
    check(not f.update(last, 0.0, false, timestamp).valid, "saturation invalidates")
    check(not tick(f, Vector3(0, 0, G), bias).valid, "invalid signal requires fresh stationary initialization")
    state = initialize(f)
    last = sample(Vector3(0, 0, G), bias)
    last.timestamp_s -= 0.5
    check(not f.update(last, 0.0, false, timestamp).valid, "old IMU sample fails closed")
    state = initialize(f)
    last = sample(Vector3(0, 0, G), bias)
    last.sequence -= 2
    check(not f.update(last, 0.0, false, timestamp).valid, "reordered sequence fails closed")
    state = initialize(f)
    timestamp += 0.1
    check(not tick(f, Vector3(0, 0, G), bias).valid, "missing integration interval requires initialization")
    for bad in [NAN, INF, 1e100, 1e30, 1000.0, "0", true, null]:
        state = initialize(f)
        last = sample(Vector3(0, 0, G), bias)
        last.accel_m_s2[1] = bad
        check(not f.update(last, 0.0, false, timestamp).valid, "malformed force component rejected " + str(bad))
    state = initialize(f)
    last = sample(Vector3(0, 0, G), bias)
    check(not f.update(last, NAN, false, timestamp).valid, "non-finite encoder rejected")
    check(not f.update(last, 0.0, false, NAN).valid, "non-finite read time rejected")
    f.reset()
    check(not f.get_estimate(timestamp).valid, "explicit reset discards attitude and bias")
    # Integration with the unchanged production quantized/noisy IMU simulator.
    # Body state is fixture-only stimulus; the estimator receives measured fields.
    var body := RigidBody3D.new()
    root.add_child(body)
    var imu = Imu.new()
    imu.reset(body, 901)
    f.reset()
    var t := 0.0
    var dt := 1.0 / 120.0
    var raw := {}
    for index in range(240):
        t += dt
        raw = imu.sample(body, dt, t, false)
        state = f.update(raw, 0.0, false, t)
    check(state.valid and state.projection_safe, "production noisy quantized IMU initializes")
    for index in range(60):
        t += dt
        body.linear_velocity = Vector3(0, 0, -(index + 1) * dt * 2.0)
        raw = imu.sample(body, dt, t, true)
        state = f.update(raw, (index + 1) * dt * 2.0, true, t)
    check(atan2(raw.accel_m_s2[1], raw.accel_m_s2[2]) > 0.19, "production IMU reproduces specific-force apparent tilt")
    check(state.valid and state.projection_safe and absf(state.pitch_rad) < 0.004, "production acceleration does not invent pitch")
    for index in range(60):
        t += dt
        body.rotation.x = (index + 1) * dt * 0.2
        body.angular_velocity = Vector3(0.2, 0, 0)
        raw = imu.sample(body, dt, t, true)
        state = f.update(raw, 1.0, true, t)
    check(absf(state.pitch_rad - 0.1) < 0.005 and not state.projection_safe, "production true rotation remains rejected with correct axis sign")
    body.free()
    # Independent review repros: real slow gyro tilt plus compensating external
    # translation keeps specific force level. Tiny innovations are not proof of
    # gravity, and repeated moving corrections must not erase their disagreement.
    for rate in [0.002, 0.0075]:
        f.reset()
        state = initialize(f)
        var steps := 2000 if rate < 0.003 else 400
        var unsafe_while_true_tilt_large := false
        for index in range(steps):
            state = tick(f, Vector3(0, 0, G), bias + Vector3(rate, 0, 0), 0.1, true)
            if (index + 1) * 0.01 * rate > 0.025 and state.projection_safe:
                unsafe_while_true_tilt_large = true
        check(not unsafe_while_true_tilt_large and not state.projection_safe, "compensated true tilt cannot pass: " + str(rate))
        check(state.moving_correction_disagreement_rad > 0.02 and not state.valid, "persistent moving correction charged to uncertainty")
        # Continuing apparently steady measurements cannot silently erase it.
        for index in range(100): state = tick(f, Vector3(0, 0, G), bias, 0.1, true)
        check(not state.valid, "steady motion cannot reset disagreement budget")
        for index in range(100): state = tick(f, Vector3(0, 0, G), bias)
        check(state.valid and state.moving_correction_disagreement_rad == 0, "qualified measured rest can recover under rest assumption")
    f.reset()
    state = initialize(f)
    var truth_gravity := Vector3(0, 0, 1)
    var yaw_fixture_unsafe := false
    for segment in [Vector3(0.0075, 0, 0), Vector3(0, 0, PI / 2.0), Vector3(0, -0.0075, 0)]:
        var count := 100 if segment.z != 0 else 200
        for index in range(count):
            truth_gravity = truth_gravity.rotated(segment.normalized(), -segment.length() * 0.01)
            state = tick(f, Vector3(0, 0, G), bias + segment, 0.1, true)
            if truth_gravity.angle_to(Vector3(0, 0, 1)) > 0.025 and state.projection_safe:
                yaw_fixture_unsafe = true
    check(truth_gravity.angle_to(Vector3(0, 0, 1)) > 0.029 and not yaw_fixture_unsafe, "yaw transport cannot cancel physical correction disagreement")
    check(not state.valid and state.moving_correction_disagreement_rad > 0.02, "yaw-rotated uncertainty remains charged")
    print("ATTITUDE_ESTIMATOR_CHECKS ", checks - failed, "/", checks)
    quit(1 if failed else 0)
