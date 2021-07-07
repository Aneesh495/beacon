# control/stanley_controller.gd
class_name StanleyController
extends "res://control/controller_base.gd"

# Non-linear Stanley tracking controller for front-wheel steered vehicles.
# Computes steering angle command based on front-axle cross-track error and yaw alignment:
# delta(t) = psi_e(t) + atan(k * e_ct(t) / (v(t) + epsilon)) - k_d * d(e_ct)/dt

const GeometryUtils = preload("res://core/spatial/geometry_utils.gd")

var gain_k: float = 1.2 # Cross-track error gain
var gain_soft: float = 0.5 # Softening velocity epsilon in m/s
var gain_kd: float = 0.1 # Derivative damping gain
var has_prev_error: bool = false
var prev_cross_track_error: float = 0.0

func _init(p_wheelbase: float = 2.8, p_gain_k: float = 1.2) -> void:
	super("stanley_controller", p_wheelbase)
	gain_k = p_gain_k

# Computes steering angle delta given vehicle rear-axle pose (x, y, theta), speed v, and target path
# path: Array of Vector2 waypoints
func compute_steering(
	rear_x: float, rear_y: float, heading_rad: float, speed_mps: float,
	path: Array[Vector2], dt: float = 0.05
) -> float:
	if path.size() < 2:
		return 0.0
		
	# Front axle position: center of steered front wheels
	var front_x: float = rear_x + wheelbase_m * cos(heading_rad)
	var front_y: float = rear_y + wheelbase_m * sin(heading_rad)
	var front_pos: Vector2 = Vector2(front_x, front_y)
	
	# Project front axle position onto path polyline
	var proj: Dictionary = GeometryUtils.project_onto_polyline(front_pos, path)
	var pt_on_path: Vector2 = proj["point"]
	var seg_idx: int = proj["segment_index"]
	
	# Path tangent vector along closest segment
	var p1: Vector2 = path[seg_idx]
	var p2: Vector2 = path[seg_idx + 1]
	var path_heading: float = (p2 - p1).angle()
	
	# Yaw alignment error: psi_e = path_heading - vehicle_heading
	var yaw_error: float = fposmod(path_heading - heading_rad + PI, 2.0 * PI) - PI
	
	# Lateral cross-track error: positive when vehicle is to the left of the path
	var path_dir: Vector2 = (p2 - p1).normalized()
	var to_front: Vector2 = front_pos - pt_on_path
	var cross_prod: float = path_dir.x * to_front.y - path_dir.y * to_front.x
	var cross_track_error: float = -signf(cross_prod) * to_front.length()
	
	# Non-linear cross-track steering term
	var cross_track_term: float = atan2(gain_k * cross_track_error, maxf(0.1, speed_mps + gain_soft))
	
	# Derivative damping term (avoid derivative kick on first evaluation)
	var d_error: float = 0.0
	if has_prev_error and dt > 1e-6:
		d_error = (cross_track_error - prev_cross_track_error) / dt
	else:
		has_prev_error = true
	prev_cross_track_error = cross_track_error
	var damping_term: float = gain_kd * d_error
	
	var raw_steering: float = yaw_error + cross_track_term - damping_term
	
	# Apply slew rate limit
	var actual_steering: float = apply_steering_slew_rate(raw_steering, dt)
	record_tracking_error(cross_track_error)
	
	return actual_steering
