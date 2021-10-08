# core/math/kalman_smoother.gd
class_name KalmanSmoother
extends RefCounted

# Rauch-Tung-Striebel (RTS) fixed-interval optimal Kalman smoother.
# Performs backward smoothing pass over forward EKF state trajectories,
# producing optimal minimum-variance state estimates given the entire batch history:
# G_k = P_{k|k} * F_{k+1}^T * P_{k+1|k}^{-1}
# x_{k|N} = x_{k|k} + G_k * (x_{k+1|N} - x_{k+1|k})
# P_{k|N} = P_{k|k} + G_k * (P_{k+1|N} - P_{k+1|k}) * G_k^T

const MatrixMN = preload("res://core/math/matrix_mn.gd")

# Forward filter history: Array of Dictionary
# {"x": PackedFloat64Array, "P": MatrixMN, "F": MatrixMN, "x_pred": PackedFloat64Array, "P_pred": MatrixMN}
var filter_history: Array[Dictionary] = []

func _init() -> void:
	filter_history = []

# Records a forward filtering step
func record_step(
	x_filtered: PackedFloat64Array,
	p_filtered: MatrixMN,
	f_transition: MatrixMN,
	x_predicted: PackedFloat64Array,
	p_predicted: MatrixMN
) -> void:
	filter_history.append({
		"x": x_filtered.duplicate(),
		"P": p_filtered.clone(),
		"F": f_transition.clone(),
		"x_pred": x_predicted.duplicate(),
		"P_pred": p_predicted.clone()
	})

# Performs RTS backward smoothing pass over recorded history
# Returns Array of Dictionary: [{"x_smooth": PackedFloat64Array, "P_smooth": MatrixMN}, ...]
func compute_smoothed_trajectory() -> Array[Dictionary]:
	var n: int = filter_history.size()
	if n == 0:
		return []
		
	var smoothed: Array[Dictionary] = []
	smoothed.resize(n)
	
	# Terminal state is identical to last filtered state
	var last_step: Dictionary = filter_history[n - 1]
	smoothed[n - 1] = {
		"x_smooth": last_step["x"].duplicate(),
		"P_smooth": last_step["P"].clone()
	}
	
	var dim: int = last_step["x"].size()
	
	# Backward recursion from N-2 down to 0
	for k in range(n - 2, -1, -1):
		var step_k: Dictionary = filter_history[k]
		var P_k: MatrixMN = step_k["P"]
		var F_k: MatrixMN = step_k["F"]
		var x_k: PackedFloat64Array = step_k["x"]
		
		var P_pred_next: MatrixMN = filter_history[k + 1]["P_pred"]
		var x_pred_next: PackedFloat64Array = filter_history[k + 1]["x_pred"]
		
		var x_smooth_next: PackedFloat64Array = smoothed[k + 1]["x_smooth"]
		var P_smooth_next: MatrixMN = smoothed[k + 1]["P_smooth"]
		
		# Compute smoother gain: G_k = P_k * F_k^T * (P_pred_next)^{-1}
		var F_k_T: MatrixMN = F_k.transpose()
		var P_pred_inv: MatrixMN = P_pred_next.inverse()
		var G_k: MatrixMN = P_k.multiply(F_k_T).multiply(P_pred_inv)
		
		# State smoothing: x_k_smooth = x_k + G_k * (x_smooth_next - x_pred_next)
		var diff_x: PackedFloat64Array = PackedFloat64Array()
		diff_x.resize(dim)
		for d in range(dim):
			diff_x[d] = x_smooth_next[d] - x_pred_next[d]
			
		var delta_x: PackedFloat64Array = G_k.multiply_vector(diff_x)
		var x_k_smooth: PackedFloat64Array = PackedFloat64Array()
		x_k_smooth.resize(dim)
		for d in range(dim):
			x_k_smooth[d] = x_k[d] + delta_x[d]
			
		# Covariance smoothing: P_k_smooth = P_k + G_k * (P_smooth_next - P_pred_next) * G_k^T
		var diff_P: MatrixMN = P_smooth_next.subtract(P_pred_next)
		var G_k_T: MatrixMN = G_k.transpose()
		var P_k_smooth: MatrixMN = P_k.add(G_k.multiply(diff_P).multiply(G_k_T))
		P_k_smooth.regularize_covariance()
		
		smoothed[k] = {
			"x_smooth": x_k_smooth,
			"P_smooth": P_k_smooth
		}
		
	return smoothed

func clear() -> void:
	filter_history.clear()
