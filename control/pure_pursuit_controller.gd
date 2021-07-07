# control/pure_pursuit_controller.gd
class_name PurePursuitController
extends "res://control/controller_base.gd"

# Adaptive lookahead Pure Pursuit geometric path tracking controller.
# Calculates steering angle delta based on lookahead waypoint and circular arc geometry:
# delta = atan(2 * L * sin(alpha) / L_d)

var lookahead_gain_kv: float = 0.8 # Seconds (lookahead time)
var min_lookahead_dist_m: float = 2.0
var max_lookahead_dist_m: float = 25.0

func _init(p_wheelbase: float = 2.8, p_kv: float = 0.8) -> void:
	super("pure_pursuit", p_wheelbase)
	lookahead_gain_kv = p_kv

# Computes steering angle delta given rear-axle position (x, y), heading psi, speed v, and path
func compute_steering(
	rear_x: float, rear_y: float, heading_rad: float, speed_mps: float,
	path: Array[Vector2], dt: float = 0.05
) -> float:
	if path.size() < 2:
		return 0.0
		
	var rear_pos: Vector2 = Vector2(rear_x, rear_y)
	
	# Compute adaptive lookahead distance L_d = clamp(k_v * v, L_min, L_max)
	var lookahead_dist: float = clampf(lookahead_gain_kv * abs(speed_mps), min_lookahead_dist_m, max_lookahead_dist_m)
	
	# Search for lookahead target point along path at distance lookahead_dist
	var target_pt: Vector2 = path[path.size() - 1]
	for i in range(path.size() - 1):
		var p1: Vector2 = path[i]
		var p2: Vector2 = path[i + 1]
		var d1: float = rear_pos.distance_to(p1)
		var d2: float = rear_pos.distance_to(p2)
		
		if (d1 <= lookahead_dist and d2 >= lookahead_dist) or (d1 >= lookahead_dist and i == 0):
			var seg_len: float = p1.distance_to(p2)
			var frac: float = clampf((lookahead_dist - d1) / seg_len, 0.0, 1.0) if seg_len > 1e-6 else 0.0
			target_pt = p1.lerp(p2, frac)
			break
			
	# Vector from rear axle to lookahead target point
	var to_target: Vector2 = target_pt - rear_pos
	var angle_to_target: float = to_target.angle()
	
	# Angle alpha between vehicle heading vector and lookahead vector
	var alpha: float = fposmod(angle_to_target - heading_rad + PI, 2.0 * PI) - PI
	
	# Pure pursuit geometric steering formula
	var actual_dist: float = maxf(1.0, to_target.length())
	var raw_steering: float = atan2(2.0 * wheelbase_m * sin(alpha), actual_dist)
	
	var actual_steering: float = apply_steering_slew_rate(raw_steering, dt)
	var cross_track_dist: float = abs(actual_dist * sin(alpha))
	record_tracking_error(cross_track_dist)
	
	return actual_steering
