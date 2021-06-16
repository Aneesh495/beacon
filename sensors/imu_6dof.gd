# sensors/imu_6dof.gd
class_name Imu6Dof
extends "res://sensors/sensor_base.gd"

# 6-DOF Inertial Measurement Unit (IMU) simulator.
# Generates realistic triaxial accelerometer and gyroscope measurements with
# Allan variance noise models (Angle Random Walk and Rate Random Walk bias drift).

const GRAVITY_MSS: float = 9.80665

# Allan variance noise parameters (typical tactical MEMS grade)
var gyro_noise_density_rad_s: float = 0.002 # ARW (rad/s / sqrt(Hz))
var gyro_bias_instability_rad_s: float = 0.0001 # RRW bias drift step
var accel_noise_density_mps2: float = 0.02 # VRW (m/s^2 / sqrt(Hz))
var accel_bias_instability_mps2: float = 0.001 # Accel bias drift step

# Dynamic bias states (random walk)
var gyro_bias: Vector3 = Vector3(0.0005, -0.0003, 0.0002)
var accel_bias: Vector3 = Vector3(0.005, -0.004, 0.003)

func _init(p_rate_hz: float = 100.0) -> void:
	super("imu_6dof", p_rate_hz)

# Generates IMU readings given true body linear acceleration, angular rate, and orientation quaternion
# Returns Dictionary: {"linear_acceleration": Vector3, "angular_velocity": Vector3, "timestamp": float}
func measure(
	true_linear_accel_body: Vector3,
	true_angular_vel_body: Vector3,
	orientation_quat: Quaternion,
	dt: float
) -> Dictionary:
	if state == SensorState.FAILED or state == SensorState.OFFLINE:
		return {"linear_acceleration": Vector3.ZERO, "angular_velocity": Vector3.ZERO, "timestamp": 0.0}
		
	# Update stochastic bias random walk: b_{k+1} = b_k + N(0, Q_bias * dt)
	var sqrt_dt: float = sqrt(dt)
	if noise_enabled:
		gyro_bias += _gaussian_vector3(gyro_bias_instability_rad_s * sqrt_dt)
		accel_bias += _gaussian_vector3(accel_bias_instability_mps2 * sqrt_dt)
		
	# Fault injection modes
	if simulated_fault_active:
		if fault_type == "bias_jump":
			gyro_bias += Vector3(0.05, -0.05, 0.02)
			accel_bias += Vector3(0.5, -0.4, 0.3)
		elif fault_type == "high_noise":
			gyro_noise_density_rad_s *= 5.0
			accel_noise_density_mps2 *= 5.0
			
	# Earth gravity vector in world frame: [0, 0, +g] pointing UP on standard accelerometer proof mass
	var gravity_world: Vector3 = Vector3(0.0, 0.0, GRAVITY_MSS)
	# Transform gravity into vehicle body frame: R^T * g_world = q^{-1} * g_world
	var gravity_body: Vector3 = orientation_quat.inverse() * gravity_world
	
	# Specific force measured by accelerometer: f_b = a_b + g_b + b_a + w_a
	var measured_accel: Vector3 = true_linear_accel_body + gravity_body + accel_bias
	if noise_enabled:
		measured_accel += _gaussian_vector3(accel_noise_density_mps2 / sqrt_dt)
		
	# Angular rate measured by gyroscope: omega_meas = omega_b + b_g + w_g
	var measured_gyro: Vector3 = true_angular_vel_body + gyro_bias
	if noise_enabled:
		measured_gyro += _gaussian_vector3(gyro_noise_density_rad_s / sqrt_dt)
		
	mark_updated()
	return {
		"linear_acceleration": measured_accel,
		"angular_velocity": measured_gyro,
		"timestamp": float(total_measurements_produced) * dt_nominal
	}

func _gaussian_vector3(std_dev: float) -> Vector3:
	return Vector3(
		_box_muller(std_dev),
		_box_muller(std_dev),
		_box_muller(std_dev)
	)

func _box_muller(std_dev: float) -> float:
	var u1: float = maxf(1e-9, randf())
	var u2: float = randf()
	return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2) * std_dev
