extends RefCounted
# Deliberately downsampled instantaneous simulation, not an A1M8 calibration or
# a rolling-scan hardware model. Raw measurements drive all local safety guards.
const SUPPORTED_COUNTS := [180, 360, 720]
const DEFAULT_RAYS := 720
const SCAN_HZ := 5.0
const CONTEXT_RAYS := 180
const PROFILE_PREFIX := "uniform_instantaneous_"

static func supported(ray_count: int) -> bool:
    return ray_count in SUPPORTED_COUNTS

static func from_arguments(arguments: PackedStringArray) -> int:
    var selected := DEFAULT_RAYS
    for argument in arguments:
        if argument.begins_with("--lidar-rays="):
            var value := argument.trim_prefix("--lidar-rays=")
            if value not in ["180", "360", "720"]: return -1
            selected = int(value)
    return selected

static func metadata(ray_count: int) -> Dictionary:
    if not supported(ray_count): return {}
    return {"profile": PROFILE_PREFIX + str(ray_count), "ray_count": ray_count,
        "angle_increment_rad": TAU / ray_count, "field_of_view_rad": TAU,
        "scan_hz": SCAN_HZ, "measurements_per_second": ray_count * SCAN_HZ,
        "calibrated": false, "scan_model": "instantaneous_uniform_360deg",
        "sampling_status": "approximate_downsampled_simulation_not_hardware_calibration"}

static func context_scan(values: PackedFloat32Array, sequence: int, timestamp_s: float) -> Dictionary:
    if not supported(values.size()) or sequence <= 0 or not is_finite(timestamp_s): return {}
    var stride := values.size() / CONTEXT_RAYS
    var reduced: Array = []
    for bin_index in range(CONTEXT_RAYS):
        var minimum := 6.0
        var valid := true
        for offset in range(stride):
            var distance := float(values[bin_index * stride + offset])
            if not is_finite(distance) or distance < 0.15 or distance > 6.0:
                valid = false
            else:
                minimum = minf(minimum, distance)
        # Downward millimetre quantization is conservative and avoids long
        # float32 JSON expansions. Full-resolution raw safety data is unchanged.
        reduced.append(floor(minimum * 1000.0) / 1000.0 if valid else -1.0)
    return {"ranges_m": reduced, "sampling": {
        "source_profile": PROFILE_PREFIX + str(values.size()),
        "source_ray_count": values.size(), "source_angle_increment_rad": TAU / values.size(),
        "source_sequence": sequence, "source_timestamp_s": timestamp_s,
        "scan_hz": SCAN_HZ, "ray_count": CONTEXT_RAYS,
        "angle_increment_rad": TAU / CONTEXT_RAYS, "field_of_view_rad": TAU,
        "aggregation": "sector_minimum_invalid_if_any", "range_quantization_m": 0.001, "calibrated": false}}

static func valid_context(sampling: Variant, sequence: int) -> bool:
    if not sampling is Dictionary: return false
    var count = sampling.get("source_ray_count")
    if not (count is int or count is float) or not is_finite(float(count)) or float(count) != floor(float(count)) or not supported(int(count)): return false
    if sampling.get("source_profile") != PROFILE_PREFIX + str(int(count)): return false
    if sampling.get("source_sequence") != sequence or sequence <= 0: return false
    if sampling.get("ray_count") != CONTEXT_RAYS or sampling.get("scan_hz") != SCAN_HZ: return false
    if sampling.get("range_quantization_m") != 0.001: return false
    if sampling.get("aggregation") != "sector_minimum_invalid_if_any" or sampling.get("calibrated") != false: return false
    for key in ["source_timestamp_s", "source_angle_increment_rad", "angle_increment_rad", "field_of_view_rad"]:
        var value = sampling.get(key)
        if not (value is int or value is float) or not is_finite(float(value)): return false
    return float(sampling.source_timestamp_s) >= 0.0 and is_equal_approx(float(sampling.source_angle_increment_rad), TAU / float(count)) and is_equal_approx(float(sampling.angle_increment_rad), TAU / CONTEXT_RAYS) and is_equal_approx(float(sampling.field_of_view_rad), TAU)
