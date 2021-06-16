# sensors/wheel_odometry.gd
class_name WheelOdometry
extends "res://sensors/sensor_base.gd"

# Wheel encoder odometry simulator for Ackermann and differential drive vehicles.
# Integrates wheel rotations via Runge-Kutta 2nd and 4th order methods,
# modeling tire slip ratio, quantization, and encoder noise.

var wheel_radius_m: float = 0.35 # Standard tire radius
var track_width_m: float = 1.6 # Distance between left and right wheels
var wheelbase_m: float = 2.8 # Distance between front and rear axles
var pulses_per_revolution: int = 1024 # High resolution optical encoder

# Accumulated dead reckoning pose [x, y, theta]
var pose_x: float = 0.0
var pose_y: float = 0.0
var pose_theta: float = 0.0

var linear_velocity_mps: float = 0.0
var angular_velocity_radps: float = 0.0

func _init(p_wheel_radius: float = 0.35, p_track_width: float = 1.6, p_rate_hz: float = 50.0) -> void:
	super("wheel_odometry", p_rate_hz)
	wheel_radius_m = p_wheel_radius
	track_width_m = p_track_width

func reset_odometry(start_pos: Vector2 = Vector2.ZERO, start_heading: float = 0.0) -> void:
	pose_x = start_pos.x
	pose_y = start_pos.y
	pose_theta = start_heading
	linear_velocity_mps = 0.0
	angular_velocity_radps = 0.0

# Updates dead reckoning pose using Runge-Kutta 2nd order (Midpoint) integration
# v_linear: speed along body forward axis (m/s)
# steering_angle_rad: front wheel steering angle delta (radians)
func update_ackermann_rk2(true_v: float, true_steering_angle: float, dt: float) -> Dictionary:
	if state == SensorState.FAILED or state == SensorState.OFFLINE:
		return {"pose": Vector3(pose_x, pose_y, pose_theta), "v": 0.0, "omega": 0.0}
		
	# Add tire slip and measurement noise
	var measured_v: float = true_v
	var measured_delta: float = true_steering_angle
	if noise_enabled:
		var slip_factor: float = 1.0 - 0.02 * (abs(true_v) / 20.0) # 2% slip at 20 m/s
		measured_v *= slip_factor
		measured_v += (randf() - 0.5) * 0.05
		measured_delta += (randf() - 0.5) * 0.005
		
	linear_velocity_mps = measured_v
	angular_velocity_radps = (measured_v / wheelbase_m) * tan(measured_delta)
	
	# Midpoint integration (RK2)
	var half_dt: float = 0.5 * dt
	var mid_theta: float = pose_theta + 0.5 * angular_velocity_radps * half_dt
	var dx: float = linear_velocity_mps * cos(mid_theta) * dt
	var dy: float = linear_velocity_mps * sin(mid_theta) * dt
	var dtheta: float = angular_velocity_radps * dt
	
	pose_x += dx
	pose_y += dy
	pose_theta = fposmod(pose_theta + dtheta + PI, 2.0 * PI) - PI # Wrap to [-pi, pi]
	
	mark_updated()
	return {
		"pose": Vector3(pose_x, pose_y, pose_theta),
		"v": linear_velocity_mps,
		"omega": angular_velocity_radps
	}
