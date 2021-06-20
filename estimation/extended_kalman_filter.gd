# estimation/extended_kalman_filter.gd
class_name ExtendedKalmanFilter
extends RefCounted

# 15-state Error-State Extended Kalman Filter (ES-EKF) for 6-DOF vehicle state estimation.
# Fuses high-rate IMU strapdown mechanization with asynchronous GNSS, wheel odometry, and LiDAR.
# Error state: [delta_p (3), delta_v (3), delta_theta (3), delta_bg (3), delta_ba (3)] in R^15.

const MatrixMN = preload("res://core/math/matrix_mn.gd")
const QuaternionMath = preload("res://core/math/quaternion_math.gd")
const Statistics = preload("res://core/math/statistics.gd")

const STATE_DIM: int = 15 # Error state dimension
const GRAVITY_MSS: float = 9.80665

# Nominal states
var position: Vector3 = Vector3.ZERO
var velocity: Vector3 = Vector3.ZERO
var orientation: Quaternion = Quaternion.IDENTITY
var gyro_bias: Vector3 = Vector3.ZERO
var accel_bias: Vector3 = Vector3.ZERO

# Covariance matrix P (15x15)
var P: MatrixMN = MatrixMN.identity(STATE_DIM)

# Process noise parameters
var process_noise_accel: float = 0.1 # m/s^2
var process_noise_gyro: float = 0.01 # rad/s
var process_noise_ba: float = 0.0005 # accel bias walk
var process_noise_bg: float = 0.0001 # gyro bias walk

# Outlier rejection gate
var enable_mahalanobis_gate: bool = true

func _init() -> void:
	reset_state(Vector3.ZERO, Vector3.ZERO, Quaternion.IDENTITY)

func reset_state(init_pos: Vector3, init_vel: Vector3, init_quat: Quaternion) -> void:
	position = init_pos
	velocity = init_vel
	orientation = init_quat.normalized()
	gyro_bias = Vector3.ZERO
	accel_bias = Vector3.ZERO
	
	P = MatrixMN.identity(STATE_DIM)
	# Initialize diagonal uncertainties
	for i in range(3):
		P.set_element(i, i, 1.0) # Pos: 1m var
		P.set_element(3 + i, 3 + i, 0.5) # Vel: 0.5 m/s var
		P.set_element(6 + i, 6 + i, 0.01) # Att: 0.1 rad var
		P.set_element(9 + i, 9 + i, 0.001) # Gyro bias: 0.001 var
		P.set_element(12 + i, 12 + i, 0.01) # Accel bias: 0.01 var

# Propagates nominal state and error covariance P using high-rate IMU measurement [a_m, w_m]
func predict_imu(accel_m: Vector3, gyro_m: Vector3, dt: float) -> void:
	# Correct IMU inputs with current estimated bias
	var w_hat: Vector3 = gyro_m - gyro_bias
	var a_hat: Vector3 = accel_m - accel_bias
	
	# Current rotation matrix R_b_to_w (vehicle body to world frame)
	var R: MatrixMN = QuaternionMath.quaternion_to_matrix3(orientation)
	
	# Rotate specific force into world frame
	var a_world: PackedFloat64Array = R.multiply_vector(PackedFloat64Array([a_hat.x, a_hat.y, a_hat.z]))
	var accel_net_world: Vector3 = Vector3(a_world[0], a_world[1], a_world[2] - GRAVITY_MSS)
	
	# Nominal state integration
	position += velocity * dt + 0.5 * accel_net_world * dt * dt
	velocity += accel_net_world * dt
	orientation = QuaternionMath.integrate_quaternion(orientation, w_hat, dt)
	
	# Construct 15x15 error state transition matrix F_x
	var F_x: MatrixMN = MatrixMN.identity(STATE_DIM)
	
	# delta_p += delta_v * dt
	for i in range(3):
		F_x.set_element(i, 3 + i, dt)
		
	# delta_v += - [R * a_hat]^ * delta_theta * dt - R * delta_ba * dt
	var a_rot: Vector3 = Vector3(a_world[0], a_world[1], a_world[2])
	var a_rot_skew: MatrixMN = QuaternionMath.skew_symmetric(a_rot)
	for i in range(3):
		for j in range(3):
			F_x.set_element(3 + i, 6 + j, -a_rot_skew.get_element(i, j) * dt)
			F_x.set_element(3 + i, 12 + j, -R.get_element(i, j) * dt)
			
	# delta_theta = exp(-w_hat * dt) * delta_theta - R * delta_bg * dt
	var rot_step: MatrixMN = QuaternionMath.exp_so3(-w_hat * dt)
	for i in range(3):
		for j in range(3):
			F_x.set_element(6 + i, 6 + j, rot_step.get_element(i, j))
			F_x.set_element(6 + i, 9 + j, -R.get_element(i, j) * dt)
			
	# Construct discrete process noise covariance Q_d
	var Q_d: MatrixMN = MatrixMN.zeros(STATE_DIM, STATE_DIM)
	var q_v: float = process_noise_accel * process_noise_accel * dt
	var q_theta: float = process_noise_gyro * process_noise_gyro * dt
	var q_bg: float = process_noise_bg * process_noise_bg * dt
	var q_ba: float = process_noise_ba * process_noise_ba * dt
	
	for i in range(3):
		Q_d.set_element(3 + i, 3 + i, q_v)
		Q_d.set_element(6 + i, 6 + i, q_theta)
		Q_d.set_element(9 + i, 9 + i, q_bg)
		Q_d.set_element(12 + i, 12 + i, q_ba)
		
	# Covariance propagation: P = F_x * P * F_x^T + Q_d
	var FP: MatrixMN = F_x.multiply(P)
	P = FP.multiply(F_x.transpose()).add(Q_d)
	P = P.regularize_covariance(1e-10)

# Updates state with 3D GNSS position measurement: z_pos in R^3
func update_gnss_position(gnss_pos: Vector3, meas_std_dev: float = 0.05) -> bool:
	var z: PackedFloat64Array = PackedFloat64Array([gnss_pos.x, gnss_pos.y, gnss_pos.z])
	var h: PackedFloat64Array = PackedFloat64Array([position.x, position.y, position.z])
	
	# Innovation residual y = z - h
	var y: PackedFloat64Array = PackedFloat64Array([z[0] - h[0], z[1] - h[1], z[2] - h[2]])
	
	# Measurement matrix H: 3 x 15 (observes position delta_p)
	var H: MatrixMN = MatrixMN.zeros(3, STATE_DIM)
	for i in range(3):
		H.set_element(i, i, 1.0)
		
	# Measurement covariance R: 3 x 3
	var var_val: float = meas_std_dev * meas_std_dev
	var R_meas: MatrixMN = MatrixMN.zeros(3, 3)
	for i in range(3):
		R_meas.set_element(i, i, var_val)
		
	return _apply_measurement_update(y, H, R_meas)

# Updates state with wheel speed measurement: z_spd in R^1
func update_wheel_speed(speed_mps: float, meas_std_dev: float = 0.1) -> bool:
	# Body forward axis in world frame is column 0 of rotation matrix R
	var R: MatrixMN = QuaternionMath.quaternion_to_matrix3(orientation)
	var forward_world: Vector3 = Vector3(R.get_element(0, 0), R.get_element(1, 0), R.get_element(2, 0))
	
	var pred_speed: float = velocity.dot(forward_world)
	var y: PackedFloat64Array = PackedFloat64Array([speed_mps - pred_speed])
	
	# Measurement matrix H: 1 x 15 (observes velocity projected on forward axis)
	var H: MatrixMN = MatrixMN.zeros(1, STATE_DIM)
	H.set_element(0, 3, forward_world.x)
	H.set_element(0, 4, forward_world.y)
	H.set_element(0, 5, forward_world.z)
	
	var R_meas: MatrixMN = MatrixMN.zeros(1, 1)
	R_meas.set_element(0, 0, meas_std_dev * meas_std_dev)
	
	return _apply_measurement_update(y, H, R_meas)

# Generic Kalman measurement update with Joseph stabilized covariance
func _apply_measurement_update(y: PackedFloat64Array, H: MatrixMN, R_meas: MatrixMN) -> bool:
	var m_dim: int = y.size()
	
	# Innovation covariance: S = H * P * H^T + R
	var HP: MatrixMN = H.multiply(P)
	var S: MatrixMN = HP.multiply(H.transpose()).add(R_meas)
	
	# Mahalanobis distance validation gating
	if enable_mahalanobis_gate:
		var valid: bool = Statistics.validate_mahalanobis_gate(y, S, "p01")
		if not valid:
			return false # Reject measurement outlier
			
	# Kalman Gain: K = P * H^T * S^{-1}
	var P_Ht: MatrixMN = P.multiply(H.transpose())
	var S_inv: MatrixMN = S.inverse()
	var K: MatrixMN = P_Ht.multiply(S_inv)
	
	# Error state correction: delta_x = K * y
	var delta_x: PackedFloat64Array = K.multiply_vector(y)
	
	# Inject error state into nominal state
	position.x += delta_x[0]
	position.y += delta_x[1]
	position.z += delta_x[2]
	
	velocity.x += delta_x[3]
	velocity.y += delta_x[4]
	velocity.z += delta_x[5]
	
	# Orientation correction: q = q * exp(0.5 * delta_theta)
	var delta_theta: Vector3 = Vector3(delta_x[6], delta_x[7], delta_x[8])
	var delta_q: Quaternion = QuaternionMath.exp_quaternion(delta_theta)
	orientation = (orientation * delta_q).normalized()
	
	gyro_bias.x += delta_x[9]
	gyro_bias.y += delta_x[10]
	gyro_bias.z += delta_x[11]
	
	accel_bias.x += delta_x[12]
	accel_bias.y += delta_x[13]
	accel_bias.z += delta_x[14]
	
	# Joseph stabilized covariance update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
	var I_KH: MatrixMN = MatrixMN.identity(STATE_DIM).subtract(K.multiply(H))
	var I_KH_P: MatrixMN = I_KH.multiply(P)
	var term1: MatrixMN = I_KH_P.multiply(I_KH.transpose())
	var KR: MatrixMN = K.multiply(R_meas)
	var term2: MatrixMN = KR.multiply(K.transpose())
	
	P = term1.add(term2).regularize_covariance(1e-10)
	return true
