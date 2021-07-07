# control/controller_base.gd
class_name ControllerBase
extends RefCounted

# Abstract base class for lateral and longitudinal vehicle controllers.
# Standardizes actuator outputs (steering angle, throttle ratio, brake pressure),
# enforces physical actuator slew rate limits, and computes tracking telemetry.

var controller_name: String = "base_controller"
var wheelbase_m: float = 2.8
var max_steering_angle_rad: float = deg_to_rad(35.0) # ~0.61 rad
var max_steering_rate_radps: float = deg_to_rad(30.0) # Slew rate limit

var current_steering_rad: float = 0.0
var current_throttle_ratio: float = 0.0 # [0.0, 1.0]
var current_brake_pressure_kpa: float = 0.0 # kPa

# Telemetry tracking metrics
var accumulated_cross_track_error_sq: float = 0.0
var tracking_step_count: int = 0
var max_cross_track_error: float = 0.0

func _init(p_name: String = "base_controller", p_wheelbase: float = 2.8) -> void:
	controller_name = p_name
	wheelbase_m = p_wheelbase

# Limits the rate of change of steering angle according to physical actuator dynamics
func apply_steering_slew_rate(target_steer_rad: float, dt: float) -> float:
	target_steer_rad = clampf(target_steer_rad, -max_steering_angle_rad, max_steering_angle_rad)
	var max_delta: float = max_steering_rate_radps * dt
	var delta: float = clampf(target_steer_rad - current_steering_rad, -max_delta, max_delta)
	current_steering_rad += delta
	return current_steering_rad

# Records tracking metrics for analysis
func record_tracking_error(cross_track_error: float) -> void:
	var abs_err: float = abs(cross_track_error)
	accumulated_cross_track_error_sq += abs_err * abs_err
	tracking_step_count += 1
	if abs_err > max_cross_track_error:
		max_cross_track_error = abs_err

func get_root_mean_square_error() -> float:
	if tracking_step_count == 0:
		return 0.0
	return sqrt(accumulated_cross_track_error_sq / float(tracking_step_count))

func reset_metrics() -> void:
	accumulated_cross_track_error_sq = 0.0
	tracking_step_count = 0
	max_cross_track_error = 0.0
	current_steering_rad = 0.0
	current_throttle_ratio = 0.0
	current_brake_pressure_kpa = 0.0
