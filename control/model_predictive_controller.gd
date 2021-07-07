# control/model_predictive_controller.gd
class_name ModelPredictiveController
extends "res://control/controller_base.gd"

# Receding-horizon Linear Time-Varying Model Predictive Control (LTV-MPC).
# Solves constrained quadratic programs over a preview horizon using discrete kinematic bicycle models:
# x_{k+1} = x_k + v_k * cos(psi_k) * dt
# y_{k+1} = y_k + v_k * sin(psi_k) * dt
# psi_{k+1} = psi_k + (v_k / L) * tan(delta_k) * dt
# v_{k+1} = v_k + a_k * dt

const MatrixMN = preload("res://core/math/matrix_mn.gd")

var horizon_steps: int = 10 # Preview steps
var dt_sec: float = 0.1 # Prediction step duration

# Cost weights
var weight_pos_error: float = 1.5 # Q_x, Q_y
var weight_heading_error: float = 3.0 # Q_psi
var weight_velocity_error: float = 0.8 # Q_v
var weight_accel_effort: float = 0.2 # R_a
var weight_steer_effort: float = 1.0 # R_delta
var weight_steer_rate: float = 2.0 # R_ddelta

# Actuator constraints
var max_accel_mps2: float = 3.0
var min_accel_mps2: float = -4.0

func _init(p_wheelbase: float = 2.8, p_horizon: int = 10, p_dt: float = 0.1) -> void:
	super("mpc_controller", p_wheelbase)
	horizon_steps = p_horizon
	dt_sec = p_dt

# Computes optimal control commands [accel, steering_angle] to track reference trajectory
# current_state: [x, y, heading_psi, speed_v]
# ref_trajectory: Array of Dictionaries with {"position": Vector2, "heading": float, "v": float}
# Returns Dictionary: {"accel_cmd": float, "steering_cmd": float, "predicted_states": Array[Vector4]}
func compute_control(
	current_state: Vector4,
	ref_trajectory: Array[Dictionary]
) -> Dictionary:
	var n_ref: int = ref_trajectory.size()
	if n_ref == 0:
		return {"accel_cmd": 0.0, "steering_cmd": 0.0, "predicted_states": []}
		
	# Extract preview reference trajectory for horizon steps
	var ref_states: Array[Vector4] = []
	for i in range(horizon_steps):
		var ref_idx: int = mini(i, n_ref - 1)
		var ref_item: Dictionary = ref_trajectory[ref_idx]
		var pos: Vector2 = ref_item["position"]
		var psi: float = ref_item.get("heading", 0.0)
		var v_ref: float = ref_item.get("v", 5.0)
		ref_states.append(Vector4(pos.x, pos.y, psi, v_ref))
		
	# Initial control sequence guess: zeros or persistent previous controls
	var u_accel: PackedFloat64Array = PackedFloat64Array()
	var u_steer: PackedFloat64Array = PackedFloat64Array()
	u_accel.resize(horizon_steps)
	u_steer.resize(horizon_steps)
	for i in range(horizon_steps):
		u_accel[i] = 0.0
		u_steer[i] = 0.0
		
	# Iterative projected gradient descent optimization
	var max_iterations: int = 25
	var learning_rate: float = 0.02
	
	for iter in range(max_iterations):
		# Forward simulate state trajectory under current controls
		var pred_states: Array[Vector4] = _forward_predict(current_state, u_accel, u_steer)
		
		# Compute gradients w.r.t control inputs
		for k in range(horizon_steps):
			var pred_k: Vector4 = pred_states[k + 1]
			var ref_k: Vector4 = ref_states[k]
			
			var err_x: float = pred_k.x - ref_k.x
			var err_y: float = pred_k.y - ref_k.y
			var err_psi: float = fposmod(pred_k.z - ref_k.z + PI, 2.0 * PI) - PI
			var err_v: float = pred_k.w - ref_k.w
			
			# Gradient of tracking error w.r.t acceleration
			var grad_accel: float = weight_velocity_error * err_v * dt_sec + weight_accel_effort * u_accel[k]
			
			# Gradient of tracking error w.r.t steering angle
			var heading_sensitivity: float = (pred_k.w / wheelbase_m) * dt_sec
			var grad_steer: float = weight_heading_error * err_psi * heading_sensitivity + weight_steer_effort * u_steer[k]
			
			if k > 0:
				grad_steer += weight_steer_rate * (u_steer[k] - u_steer[k - 1])
				
			# Gradient descent step
			u_accel[k] -= learning_rate * grad_accel
			u_steer[k] -= learning_rate * grad_steer
			
			# Box constraint saturation
			u_accel[k] = clampf(u_accel[k], min_accel_mps2, max_accel_mps2)
			u_steer[k] = clampf(u_steer[k], -max_steering_angle_rad, max_steering_angle_rad)
			
	# Forward simulate with final converged controls
	var final_predictions: Array[Vector4] = _forward_predict(current_state, u_accel, u_steer)
	
	var cmd_accel: float = u_accel[0]
	var cmd_steer: float = u_steer[0]
	
	# Apply physical steering slew rate limits
	var actual_steer: float = apply_steering_slew_rate(cmd_steer, dt_sec)
	
	# Compute cross-track error w.r.t closest reference
	var cross_track_dist: float = Vector2(current_state.x, current_state.y).distance_to(Vector2(ref_states[0].x, ref_states[0].y))
	record_tracking_error(cross_track_dist)
	
	return {
		"accel_cmd": cmd_accel,
		"steering_cmd": actual_steer,
		"predicted_states": final_predictions
	}

func _forward_predict(x0: Vector4, u_a: PackedFloat64Array, u_delta: PackedFloat64Array) -> Array[Vector4]:
	var states: Array[Vector4] = [x0]
	var cur: Vector4 = x0
	
	for i in range(horizon_steps):
		var a: float = u_a[i]
		var delta: float = u_delta[i]
		
		var px: float = cur.x + cur.w * cos(cur.z) * dt_sec
		var py: float = cur.y + cur.w * sin(cur.z) * dt_sec
		var psi: float = cur.z + (cur.w / wheelbase_m) * tan(delta) * dt_sec
		var v: float = maxf(0.0, cur.w + a * dt_sec)
		
		psi = fposmod(psi + PI, 2.0 * PI) - PI
		var next_state: Vector4 = Vector4(px, py, psi, v)
		states.append(next_state)
		cur = next_state
		
	return states
