extends RefCounted
## Sensor-only roll/pitch estimate. No rigid-body transform or simulator truth input.
## Acceleration is specific force, not gravity during translation. Propagate a
## gravity direction using measured gyros; correct it only after measured rest.
## Bounds below are explicit simulation assumptions, not WT901 calibration.

const GRAVITY := 9.80665
const STALE_AFTER_S := 0.05
const INITIALIZATION_S := 0.7
const CORRECTION_REST_S := 0.35
const MIN_STATIC_SAMPLES := 40
const INITIAL_UNCERTAINTY_RAD := 0.004
const STEADY_UNCERTAINTY_RAD := 0.014
const MAX_UNCERTAINTY_RAD := 0.020
const GYRO_DRIFT_BOUND_RAD_S := 0.003
const PROJECTION_TILT_BOUND_RAD := 0.025
const MAX_REST_ENCODER_SPEED := 0.006
const MAX_STATIC_GYRO_RAD_S := 0.008
const GRAVITY_NORM_TOLERANCE := 0.12
const STATIC_DIRECTION_TOLERANCE_RAD := 0.010
const MAX_CORRECTION_INNOVATION_RAD := 0.035
const MAX_STEADY_INNOVATION_RAD := 0.010
const MAX_STEADY_ENCODER_ACCEL_M_S2 := 0.05
const STEADY_CORRECTION_S := 0.7

var _initialized := false
var _gravity := Vector3.UP # Sensor axes are x-right, y-forward, z-up; set on reset.
var _bias := Vector3.ZERO
var _uncertainty := INITIAL_UNCERTAINTY_RAD
var _last_timestamp := -1.0
var _last_sequence := -1
var _last_encoder_speed := 0.0
var _rest_start := -1.0
var _rest_count := 0
var _rest_mode := 0
var _rest_accel_sum := Vector3.ZERO
var _rest_gyro_sum := Vector3.ZERO
var _last_correction := -1.0
var _reason := "stationary_initialization_required"
var _correction_accepted := false
var _last_innovation := 0.0
var _gyro_gravity_anchor := Vector3(0, 0, 1)

func _init() -> void:
    reset()

func reset() -> void:
    _initialized = false
    _gravity = Vector3(0, 0, 1)
    _bias = Vector3.ZERO
    _uncertainty = INITIAL_UNCERTAINTY_RAD
    _last_timestamp = -1.0
    _last_sequence = -1
    _last_encoder_speed = 0.0
    _last_correction = -1.0
    _reason = "stationary_initialization_required"
    _correction_accepted = false
    _last_innovation = 0.0
    _gyro_gravity_anchor = Vector3(0, 0, 1)
    _clear_rest()

func _clear_rest() -> void:
    _rest_start = -1.0
    _rest_count = 0
    _rest_mode = 0
    _rest_accel_sum = Vector3.ZERO
    _rest_gyro_sum = Vector3.ZERO

func _number(value) -> bool:
    return (value is float or value is int) and is_finite(float(value))

func _vector_ok(value) -> bool:
    return value is Array and value.size() == 3 and _number(value[0]) and _number(value[1]) and _number(value[2])

func _reject(reason: String, now_s: float) -> Dictionary:
    reset()
    _reason = reason
    return get_estimate(now_s)

func update(reading: Dictionary, encoder_speed_m_s: float, motors_commanded: bool, now_s: float) -> Dictionary:
    if not is_finite(now_s) or now_s < 0 or not is_finite(encoder_speed_m_s):
        return _reject("invalid_time_or_encoder", now_s)
    if reading.get("valid") != true or reading.get("stale") != false or reading.get("accel_saturated", false) or reading.get("gyro_saturated", false):
        return _reject("invalid_stale_or_saturated_imu", now_s)
    if not _vector_ok(reading.get("accel_m_s2")) or not _vector_ok(reading.get("gyro_rad_s")):
        return _reject("malformed_imu_vectors", now_s)
    if not _number(reading.get("timestamp_s")) or not _number(reading.get("sequence")):
        return _reject("malformed_imu_clock", now_s)
    var timestamp := float(reading.timestamp_s)
    var sequence := int(reading.sequence)
    if timestamp < 0 or timestamp > now_s + 0.000001 or now_s - timestamp > STALE_AFTER_S or sequence < 1 or float(sequence) != float(reading.sequence):
        return _reject("invalid_imu_clock", now_s)
    if sequence == _last_sequence and is_equal_approx(timestamp, _last_timestamp):
        return get_estimate(now_s) # Never integrate one sample twice.
    if _last_sequence >= 0 and (sequence <= _last_sequence or timestamp <= _last_timestamp):
        return _reject("non_monotonic_imu", now_s)
    var dt := timestamp - _last_timestamp if _last_timestamp >= 0 else 0.0
    if dt > STALE_AFTER_S:
        return _reject("imu_gap_requires_reinitialization", now_s)
    var a: Array = reading.accel_m_s2
    var w: Array = reading.gyro_rad_s
    var accel := Vector3(a[0], a[1], a[2])
    var gyro := Vector3(w[0], w[1], w[2])
    # JSON doubles can be finite while overflowing Godot's Vector3 components.
    if not accel.is_finite() or not gyro.is_finite() or maxf(absf(accel.x), maxf(absf(accel.y), absf(accel.z))) > 16.0 * GRAVITY or accel.length() < 0.1 or gyro.length() > deg_to_rad(2000):
        return _reject("invalid_imu_magnitude", now_s)
    var encoder_acceleration := absf(encoder_speed_m_s - _last_encoder_speed) / dt if dt > 0 else INF
    _last_encoder_speed = encoder_speed_m_s
    _last_timestamp = timestamp
    _last_sequence = sequence
    _correction_accepted = false
    if _initialized:
        var rate := gyro - _bias
        if rate.length() > 0.0000001:
            _gravity = _gravity.rotated(rate.normalized(), -rate.length() * dt).normalized()
            _gyro_gravity_anchor = _gyro_gravity_anchor.rotated(rate.normalized(), -rate.length() * dt).normalized()
        _uncertainty += GYRO_DRIFT_BOUND_RAD_S * dt
    _last_innovation = _gravity.angle_to(accel.normalized())
    var stable_force := absf(accel.length() - GRAVITY) <= GRAVITY_NORM_TOLERANCE
    var static_sample := not motors_commanded and absf(encoder_speed_m_s) <= MAX_REST_ENCODER_SPEED and stable_force and gyro.length() <= MAX_STATIC_GYRO_RAD_S
    var steady_sample := _initialized and stable_force and encoder_acceleration <= MAX_STEADY_ENCODER_ACCEL_M_S2 and (gyro - _bias).length() <= MAX_STATIC_GYRO_RAD_S and _last_innovation <= MAX_STEADY_INNOVATION_RAD
    var correction_sample := static_sample or steady_sample
    var correction_mode := 1 if static_sample else 2
    if correction_sample and _rest_count > 0 and correction_mode != _rest_mode:
        _clear_rest()
    if correction_sample and _rest_count > 0:
        correction_sample = accel.angle_to(_rest_accel_sum / _rest_count) <= STATIC_DIRECTION_TOLERANCE_RAD
    if not correction_sample:
        _clear_rest()
    else:
        if _rest_start < 0: _rest_start = timestamp
        _rest_mode = correction_mode
        _rest_count += 1
        _rest_accel_sum += accel
        _rest_gyro_sum += gyro
    var rest_s := timestamp - _rest_start if _rest_start >= 0 else 0.0
    if not _initialized:
        if rest_s >= INITIALIZATION_S and _rest_count >= MIN_STATIC_SAMPLES:
            _gravity = _rest_accel_sum.normalized()
            _gyro_gravity_anchor = _gravity
            _bias = _rest_gyro_sum / _rest_count
            _uncertainty = INITIAL_UNCERTAINTY_RAD
            _initialized = true
            _last_correction = timestamp
            _correction_accepted = true
            _reason = "stationary_initialized"
        else:
            _reason = "stationary_initialization_required"
    elif correction_sample and rest_s >= (CORRECTION_REST_S if static_sample else STEADY_CORRECTION_S) and _last_innovation <= (MAX_CORRECTION_INNOVATION_RAD if static_sample else MAX_STEADY_INNOVATION_RAD):
        # Do not snap orientation to an acceleration spike. Innovation, encoder,
        # demand, magnitude and sustained stability all qualify each correction.
        var weight := 1.0 - exp(-dt / 0.25)
        var corrected := _gravity.lerp(accel.normalized(), weight).normalized()
        if static_sample:
            # This reset requires a sustained measured stop, never steady drive.
            # External forces can still violate the explicitly stated rest assumption.
            _gyro_gravity_anchor = corrected
        # Moving gravity correction NEVER re-anchors the independent gyro-only
        # direction. Their exact angle in the current sensor frame measures
        # accumulated disagreement, including through yaw and combined rotations.
        _gravity = corrected
        var floor_bound := INITIAL_UNCERTAINTY_RAD if static_sample else STEADY_UNCERTAINTY_RAD
        _uncertainty = maxf(floor_bound, _uncertainty * (1.0 - weight))
        # Moving gyros must not be learned as stationary sensor bias.
        if static_sample:
            _bias = _bias.lerp(_rest_gyro_sum / _rest_count, 1.0 - exp(-dt / 5.0))
        _last_correction = timestamp
        _correction_accepted = true
        _reason = "qualified_stationary_gravity_correction" if static_sample else "qualified_steady_motion_gravity_correction"
    elif _initialized:
        _reason = "gyro_propagation_acceleration_not_assumed_gravity"
    return get_estimate(now_s)

func get_estimate(now_s: float) -> Dictionary:
    var age := now_s - _last_timestamp if is_finite(now_s) and _last_timestamp >= 0 else -1.0
    var fresh := age >= -0.000001 and age <= STALE_AFTER_S
    var disagreement := _gravity.angle_to(_gyro_gravity_anchor)
    var total_uncertainty := _uncertainty + disagreement
    var valid := _initialized and fresh and total_uncertainty <= MAX_UNCERTAINTY_RAD
    var roll := atan2(_gravity.x, _gravity.z)
    var pitch := atan2(_gravity.y, _gravity.z)
    var reason := _reason
    if _initialized and not fresh: reason = "stale_orientation"
    elif _initialized and total_uncertainty > MAX_UNCERTAINTY_RAD: reason = "gyro_drift_or_correction_disagreement_exceeded"
    return {"valid":valid, "initialized":_initialized, "timestamp_s":_last_timestamp,
        "sequence":_last_sequence, "age_s":maxf(0.0, age), "roll_rad":roll, "pitch_rad":pitch,
        "uncertainty_rad":total_uncertainty, "propagation_uncertainty_rad":_uncertainty,
        "moving_correction_disagreement_rad":disagreement,
        "gyro_gravity_anchor_unit_sensor":[_gyro_gravity_anchor.x, _gyro_gravity_anchor.y, _gyro_gravity_anchor.z],
        "projection_safe":valid and maxf(absf(roll), absf(pitch)) + total_uncertainty <= PROJECTION_TILT_BOUND_RAD,
        "gravity_unit_sensor":[_gravity.x, _gravity.y, _gravity.z], "gyro_bias_rad_s":[_bias.x, _bias.y, _bias.z],
        "correction_accepted":_correction_accepted, "last_correction_timestamp_s":_last_correction,
        "accel_innovation_rad":_last_innovation, "stationary_samples":_rest_count,
        "reason":reason, "provenance":"sensor_only_gyro_gravity_filter_uncalibrated_simulation",
        "calibrated":false, "drift_bound_rad_s":GYRO_DRIFT_BOUND_RAD_S}
