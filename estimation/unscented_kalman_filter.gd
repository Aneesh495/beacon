# estimation/unscented_kalman_filter.gd
class_name UnscentedKalmanFilter
extends RefCounted

# Unscented Kalman Filter (UKF) for non-linear vehicle dynamics.
# Utilizes the Van der Merwe scaled unscented transform to propagate
# sigma points through non-linear kinematic bicycle equations with side-slip angles.
# State vector: [x, y, psi (heading), v (speed), delta (steering angle)] in R^5.

const MatrixMN = preload("res://core/math/matrix_mn.gd")

const L_DIM: int = 5 # State dimension
const NUM_SIGMA: int = 2 * L_DIM + 1 # 11 sigma points

var wheelbase_m: float = 2.8

# State mean x in R^5
var state_x: PackedFloat64Array = PackedFloat64Array([0.0, 0.0, 0.0, 0.0, 0.0])
# State covariance P (5x5)
var P: MatrixMN = MatrixMN.identity(L_DIM)

# Unscented Transform scaling parameters
var alpha: float = 1.0
var beta: float = 2.0
var kappa: float = 0.0
var lambda_param: float = 0.0

var weights_mean: PackedFloat64Array = PackedFloat64Array()
var weights_cov: PackedFloat64Array = PackedFloat64Array()

func _init(p_wheelbase: float = 2.8) -> void:
	wheelbase_m = p_wheelbase
	P = MatrixMN.diag(PackedFloat64Array([0.1, 0.1, 0.001, 0.1, 0.001]))
	_setup_weights()
	
func _setup_weights() -> void:
	lambda_param = alpha * alpha * (float(L_DIM) + kappa) - float(L_DIM)
	var c: float = float(L_DIM) + lambda_param
	
	weights_mean.resize(NUM_SIGMA)
	weights_cov.resize(NUM_SIGMA)
	
	weights_mean[0] = lambda_param / c
	weights_cov[0] = lambda_param / c + (1.0 - alpha * alpha + beta)
	
	var w_i: float = 0.5 / c
	for i in range(1, NUM_SIGMA):
		weights_mean[i] = w_i
		weights_cov[i] = w_i

# Generates 2L+1 sigma points from current mean state_x and covariance P
# Returns Array of PackedFloat64Array (each length 5)
func generate_sigma_points() -> Array[PackedFloat64Array]:
	var c: float = float(L_DIM) + lambda_param
	var sqrt_P: MatrixMN = P.scale(c).decompose_cholesky()
	
	var sigma_pts: Array[PackedFloat64Array] = []
	sigma_pts.resize(NUM_SIGMA)
	
	# Point 0 is the mean
	sigma_pts[0] = state_x.duplicate()
	
	# Points 1..L: mean + col_i, Points L+1..2L: mean - col_i
	for i in range(L_DIM):
		var col_i: PackedFloat64Array = sqrt_P.to_packed_column(i)
		var pt_plus: PackedFloat64Array = PackedFloat64Array()
		var pt_minus: PackedFloat64Array = PackedFloat64Array()
		pt_plus.resize(L_DIM)
		pt_minus.resize(L_DIM)
		
		for j in range(L_DIM):
			pt_plus[j] = state_x[j] + col_i[j]
			pt_minus[j] = state_x[j] - col_i[j]
			
		sigma_pts[1 + i] = pt_plus
		sigma_pts[1 + L_DIM + i] = pt_minus
		
	return sigma_pts

# Propagates sigma points through non-linear kinematic bicycle model with side-slip:
# dot_x = v * cos(psi + beta_slip)
# dot_y = v * sin(psi + beta_slip)
# dot_psi = (v / L) * cos(beta_slip) * tan(delta)
func predict(accel_cmd: float, steer_rate_cmd: float, dt: float) -> void:
	var sigmas: Array[PackedFloat64Array] = generate_sigma_points()
	var transformed_sigmas: Array[PackedFloat64Array] = []
	transformed_sigmas.resize(NUM_SIGMA)
	
	for i in range(NUM_SIGMA):
		var pt: PackedFloat64Array = sigmas[i]
		var px: float = pt[0]
		var py: float = pt[1]
		var psi: float = pt[2]
		var v: float = pt[3]
		var delta: float = pt[4]
		
		# Side slip angle beta_slip = atan(0.5 * tan(delta))
		var beta_slip: float = atan(0.5 * tan(delta))
		
		var dx: float = v * cos(psi + beta_slip) * dt
		var dy: float = v * sin(psi + beta_slip) * dt
		var dpsi: float = (v / wheelbase_m) * cos(beta_slip) * tan(delta) * dt
		var dv: float = accel_cmd * dt
		var ddelta: float = steer_rate_cmd * dt
		
		transformed_sigmas[i] = PackedFloat64Array([
			px + dx,
			py + dy,
			psi + dpsi,
			v + dv,
			delta + ddelta
		])
		
	# Compute predicted mean
	for j in range(L_DIM):
		var sum_val: float = 0.0
		for i in range(NUM_SIGMA):
			sum_val += weights_mean[i] * transformed_sigmas[i][j]
		state_x[j] = sum_val
		
	# Compute predicted covariance P = sum w_c * (chi - x) * (chi - x)^T + Q
	var new_P: MatrixMN = MatrixMN.zeros(L_DIM, L_DIM)
	for i in range(NUM_SIGMA):
		var diff: PackedFloat64Array = PackedFloat64Array()
		diff.resize(L_DIM)
		for j in range(L_DIM):
			diff[j] = transformed_sigmas[i][j] - state_x[j]
		# Angle wrapping on heading difference
		diff[2] = fposmod(diff[2] + PI, 2.0 * PI) - PI
		
		var w_c: float = weights_cov[i]
		for r in range(L_DIM):
			for c in range(L_DIM):
				new_P.add_element(r, c, w_c * diff[r] * diff[c])
				
	# Add discrete process noise Q
	new_P.add_element(0, 0, 0.05 * dt)
	new_P.add_element(1, 1, 0.05 * dt)
	new_P.add_element(2, 2, 0.01 * dt)
	new_P.add_element(3, 3, 0.1 * dt)
	new_P.add_element(4, 4, 0.02 * dt)
	
	P = new_P.regularize_covariance(1e-9)

# Updates state with 2D position observation [x, y]
func update_position_2d(measured_pos: Vector2, meas_std_dev: float = 0.1) -> void:
	var sigmas: Array[PackedFloat64Array] = generate_sigma_points()
	var z_sigmas: Array[PackedFloat64Array] = []
	z_sigmas.resize(NUM_SIGMA)
	
	# Measurement function h(x) extracts [x, y]
	for i in range(NUM_SIGMA):
		z_sigmas[i] = PackedFloat64Array([sigmas[i][0], sigmas[i][1]])
		
	# Predicted measurement mean z_hat
	var z_hat: PackedFloat64Array = PackedFloat64Array([0.0, 0.0])
	for i in range(NUM_SIGMA):
		z_hat[0] += weights_mean[i] * z_sigmas[i][0]
		z_hat[1] += weights_mean[i] * z_sigmas[i][1]
		
	# Innovation covariance S and cross-covariance P_xz
	var S: MatrixMN = MatrixMN.zeros(2, 2)
	var P_xz: MatrixMN = MatrixMN.zeros(L_DIM, 2)
	
	var r_var: float = meas_std_dev * meas_std_dev
	S.set_element(0, 0, r_var)
	S.set_element(1, 1, r_var)
	
	for i in range(NUM_SIGMA):
		var z_diff: PackedFloat64Array = PackedFloat64Array([
			z_sigmas[i][0] - z_hat[0],
			z_sigmas[i][1] - z_hat[1]
		])
		var x_diff: PackedFloat64Array = PackedFloat64Array()
		x_diff.resize(L_DIM)
		for j in range(L_DIM):
			x_diff[j] = sigmas[i][j] - state_x[j]
		x_diff[2] = fposmod(x_diff[2] + PI, 2.0 * PI) - PI
		
		var w_c: float = weights_cov[i]
		for r in range(2):
			for c in range(2):
				S.add_element(r, c, w_c * z_diff[r] * z_diff[c])
				
		for r in range(L_DIM):
			for c in range(2):
				P_xz.add_element(r, c, w_c * x_diff[r] * z_diff[c])
				
	# Kalman gain: K = P_xz * S^{-1}
	var K: MatrixMN = P_xz.multiply(S.inverse())
	
	# Innovation residual y = z - z_hat
	var y: PackedFloat64Array = PackedFloat64Array([measured_pos.x - z_hat[0], measured_pos.y - z_hat[1]])
	var update_step: PackedFloat64Array = K.multiply_vector(y)
	
	# State update
	for j in range(L_DIM):
		state_x[j] += update_step[j]
	state_x[2] = fposmod(state_x[2] + PI, 2.0 * PI) - PI
	
	# Covariance update: P = P - K * S * K^T
	var KS: MatrixMN = K.multiply(S)
	var KSKt: MatrixMN = KS.multiply(K.transpose())
	P = P.subtract(KSKt).regularize_covariance(1e-9)
