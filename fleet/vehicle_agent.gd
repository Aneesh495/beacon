# fleet/vehicle_agent.gd
class_name VehicleAgent
extends RefCounted

# Autonomous cyber-physical vehicle agent node.
# Encapsulates physical vehicle dynamics, multi-sensor suite, state estimation,
# kinodynamic motion planning, trajectory tracking control, and V2X communication.

const MatrixMN = preload("res://core/math/matrix_mn.gd")
const ExtendedKalmanFilter = preload("res://estimation/extended_kalman_filter.gd")
const Lidar3D = preload("res://sensors/lidar_3d.gd")
const Imu6Dof = preload("res://sensors/imu_6dof.gd")
const WheelOdometry = preload("res://sensors/wheel_odometry.gd")
const GnssReceiver = preload("res://sensors/gnss_receiver.gd")
const Costmap2D = preload("res://maps/costmap_2d.gd")
const HybridAStar = preload("res://planning/hybrid_a_star.gd")
const VelocityProfiler = preload("res://planning/velocity_profiler.gd")
const StanleyController = preload("res://control/stanley_controller.gd")
const PurePursuitController = preload("res://control/pure_pursuit_controller.gd")
const CascadedPidController = preload("res://control/cascaded_pid_controller.gd")

enum AgentState {
	UNINITIALIZED,
	IDLE,
	PLANNING,
	DISPATCHED,
	EN_ROUTE,
	INTERSECTION_WAIT,
	EMERGENCY_STOP,
	CHARGING,
	FAULTED
}

# Unique agent identification
var agent_id: String = "vehicle_0"
var state: AgentState = AgentState.UNINITIALIZED
var vehicle_color: Color = Color(0.2, 0.6, 1.0)

# Physical chassis and powertrain attributes
var mass_kg: float = 1800.0
var wheelbase_m: float = 2.8
var track_width_m: float = 1.6
var length_m: float = 4.6
var width_m: float = 1.9
var max_speed_mps: float = 25.0 # ~90 km/h
var max_accel_mps2: float = 3.5
var max_decel_mps2: float = 7.0
var max_steer_rad: float = deg_to_rad(35.0)

# Battery and energy subsystem
var battery_capacity_kwh: float = 75.0
var battery_soc: float = 1.0 # State of Charge [0.0, 1.0]
var energy_consumption_kwh_per_km: float = 0.18

# Ground truth state (simulation physics ground truth)
var true_position: Vector3 = Vector3.ZERO
var true_velocity: Vector3 = Vector3.ZERO
var true_orientation: Quaternion = Quaternion.IDENTITY
var true_heading_rad: float = 0.0
var true_speed_mps: float = 0.0
var true_steering_rad: float = 0.0

# Estimated state (output from Kalman filter)
var estimated_position: Vector3 = Vector3.ZERO
var estimated_velocity: Vector3 = Vector3.ZERO
var estimated_heading_rad: float = 0.0

# Subsystem components
var ekf: ExtendedKalmanFilter = null
var sensor_lidar: Lidar3D = null
var sensor_imu: Imu6Dof = null
var sensor_wheel_odom: WheelOdometry = null
var sensor_gnss: GnssReceiver = null

var local_costmap: Costmap2D = null
var motion_planner: HybridAStar = null
var velocity_profiler: VelocityProfiler = null
var lateral_controller: StanleyController = null
var longitudinal_controller: CascadedPidController = null

# Active navigation mission
var current_waypoints: Array[Vector2] = []
var profile_trajectory: Array[Dictionary] = []
var target_destination: Vector2 = Vector2.ZERO
var target_speed_limit_mps: float = 15.0
var current_trajectory_index: int = 0
var distance_travelled_m: float = 0.0

# Safety and fault management
var emergency_stop_requested: bool = false
var fault_reason: String = ""
var heartbeat_timestamp_sec: float = 0.0

func _init(p_id: String = "vehicle_0", p_wheelbase: float = 2.8) -> void:
	agent_id = p_id
	wheelbase_m = p_wheelbase
	
	# Instantiate state estimation and control subsystems
	ekf = ExtendedKalmanFilter.new()
	sensor_lidar = Lidar3D.new(16, 10.0)
	sensor_imu = Imu6Dof.new(100.0)
	sensor_wheel_odom = WheelOdometry.new(0.35, 1.6, 50.0)
	sensor_gnss = GnssReceiver.new(10.0)
	
	lateral_controller = StanleyController.new(p_wheelbase, 1.5)
	longitudinal_controller = CascadedPidController.new(mass_kg)
	velocity_profiler = VelocityProfiler.new()
	
	state = AgentState.IDLE

# Initializes the vehicle at a designated ground truth pose
func spawn(p_pos: Vector3, p_heading: float) -> void:
	true_position = p_pos
	true_heading_rad = p_heading
	true_orientation = Quaternion(Vector3.UP, p_heading)
	true_velocity = Vector3.ZERO
	true_speed_mps = 0.0
	true_steering_rad = 0.0
	
	estimated_position = p_pos
	estimated_velocity = Vector3.ZERO
	estimated_heading_rad = p_heading
	
	ekf.reset_state(p_pos, Vector3.ZERO, true_orientation)
	state = AgentState.IDLE

# Assigns a target destination waypoint and plans an initial kinodynamic trajectory
func set_destination(destination_2d: Vector2, speed_limit_mps: float = 15.0) -> bool:
	target_destination = destination_2d
	target_speed_limit_mps = speed_limit_mps
	state = AgentState.PLANNING
	
	var start_pos_2d: Vector2 = Vector2(estimated_position.x, estimated_position.z)
	
	# Create synthetic direct waypoints or invoke planner
	var direct_dist: float = start_pos_2d.distance_to(destination_2d)
	if direct_dist < 0.5:
		state = AgentState.IDLE
		return true
		
	# Generate multi-point spline reference path towards destination
	current_waypoints.clear()
	var path_samples: Array[Dictionary] = []
	var steps: int = maxi(5, int(direct_dist / 4.0))
	var s_accum: float = 0.0
	for i in range(steps + 1):
		var t: float = float(i) / float(steps)
		var pt: Vector2 = start_pos_2d.lerp(destination_2d, t)
		current_waypoints.append(pt)
		if i > 0:
			s_accum += current_waypoints[i - 1].distance_to(pt)
		path_samples.append({
			"position": pt,
			"curvature": 0.0,
			"s": s_accum
		})
		
	# Generate time-optimal velocity profile along path
	profile_trajectory = velocity_profiler.profile_path(path_samples, 0.0, 0.0)
	current_trajectory_index = 0
	
	state = AgentState.EN_ROUTE
	return true

# Main physics and simulation update step at fixed dt (e.g. 50Hz = 0.02s)
func update_simulation(dt: float) -> Dictionary:
	heartbeat_timestamp_sec += dt
	
	if emergency_stop_requested:
		state = AgentState.EMERGENCY_STOP
		
	match state:
		AgentState.IDLE:
			_apply_idle_brakes(dt)
		AgentState.PLANNING:
			pass
		AgentState.EN_ROUTE:
			_update_navigation_and_control(dt)
		AgentState.INTERSECTION_WAIT:
			_apply_idle_brakes(dt)
		AgentState.EMERGENCY_STOP:
			_execute_emergency_braking(dt)
		AgentState.CHARGING:
			battery_soc = minf(1.0, battery_soc + (0.01 * dt))
			_apply_idle_brakes(dt)
		AgentState.FAULTED:
			_apply_idle_brakes(dt)
			
	# Update sensor observations based on ground truth
	_update_sensors_and_estimation(dt)
	
	# Deplete battery based on distance travelled and power output
	var step_distance: float = true_speed_mps * dt
	distance_travelled_m += step_distance
	var energy_used_kwh: float = (step_distance / 1000.0) * energy_consumption_kwh_per_km
	battery_soc = maxf(0.0, battery_soc - (energy_used_kwh / battery_capacity_kwh))
	
	return get_telemetry()

func _update_navigation_and_control(dt: float) -> void:
	if current_waypoints.size() < 2 or current_trajectory_index >= profile_trajectory.size():
		state = AgentState.IDLE
		true_speed_mps = 0.0
		return
		
	var current_pos_2d: Vector2 = Vector2(true_position.x, true_position.z)
	var target_dest_2d: Vector2 = current_waypoints[current_waypoints.size() - 1]
	
	if current_pos_2d.distance_to(target_dest_2d) < 1.0:
		state = AgentState.IDLE
		true_speed_mps = 0.0
		return
		
	# Find closest trajectory reference point ahead
	var target_ref: Dictionary = profile_trajectory[current_trajectory_index]
	var ref_pos: Vector2 = target_ref["position"]
	while current_trajectory_index < profile_trajectory.size() - 1 and current_pos_2d.distance_to(ref_pos) < 2.0:
		current_trajectory_index += 1
		target_ref = profile_trajectory[current_trajectory_index]
		ref_pos = target_ref["position"]
		
	var target_speed: float = target_ref.get("v", target_ref.get("speed", target_speed_limit_mps))
	
	# Compute lateral steering command via Stanley tracking controller
	var steer_cmd: float = lateral_controller.compute_steering(
		true_position.x, true_position.z, true_heading_rad, true_speed_mps,
		current_waypoints, dt
	)
	true_steering_rad = steer_cmd
	
	# Compute longitudinal throttle/brake via Cascaded PID controller
	var long_cmd: Dictionary = longitudinal_controller.compute_longitudinal_control(
		true_speed_mps, target_speed, dt
	)
	var throttle: float = long_cmd["throttle_ratio"]
	var brake_kpa: float = long_cmd["brake_pressure_kpa"]
	
	# Vehicle kinematic bicycle integration
	var accel: float = 0.0
	if throttle > 0.0:
		accel = throttle * max_accel_mps2
	elif brake_kpa > 0.0:
		accel = -(brake_kpa / 10000.0) * max_decel_mps2
		
	true_speed_mps = clampf(true_speed_mps + accel * dt, 0.0, max_speed_mps)
	
	# Kinematic bicycle equations
	var d_heading: float = (true_speed_mps / wheelbase_m) * tan(true_steering_rad) * dt
	true_heading_rad = fposmod(true_heading_rad + d_heading + PI, 2.0 * PI) - PI
	
	var vx: float = true_speed_mps * cos(true_heading_rad)
	var vz: float = true_speed_mps * sin(true_heading_rad)
	true_velocity = Vector3(vx, 0.0, vz)
	true_position += true_velocity * dt
	true_orientation = Quaternion(Vector3.UP, true_heading_rad)

func _update_sensors_and_estimation(dt: float) -> void:
	# IMU strapdown update
	var angular_vel: Vector3 = Vector3(0.0, (true_speed_mps / wheelbase_m) * tan(true_steering_rad), 0.0)
	var linear_accel: Vector3 = Vector3(cos(true_heading_rad), 0.0, sin(true_heading_rad)) * 0.5
	var imu_meas: Dictionary = sensor_imu.measure(linear_accel, angular_vel, true_orientation, dt)
	
	# Wheel odometry update
	var odom_meas: Dictionary = sensor_wheel_odom.update_ackermann_rk2(true_speed_mps, true_steering_rad, dt)
	
	# GNSS position update
	var gnss_meas: Dictionary = sensor_gnss.measure_position(true_position)
	
	# Run EKF prediction and measurement fusion
	ekf.predict_imu(imu_meas["linear_acceleration"], imu_meas["angular_velocity"], dt)
	
	if gnss_meas["fix_type"] != GnssReceiver.FixType.NO_FIX:
		ekf.update_gnss_position(gnss_meas["position"], gnss_meas["accuracy_m"])
		
	if odom_meas.has("v"):
		ekf.update_wheel_speed(odom_meas["v"], 0.1)
		
	# Synchronize estimated state
	estimated_position = ekf.position
	estimated_velocity = ekf.velocity
	estimated_heading_rad = ekf.orientation.get_euler().y

func _apply_idle_brakes(dt: float) -> void:
	true_speed_mps = maxf(0.0, true_speed_mps - 4.0 * dt)
	true_velocity = Vector3(cos(true_heading_rad), 0.0, sin(true_heading_rad)) * true_speed_mps

func _execute_emergency_braking(dt: float) -> void:
	true_speed_mps = maxf(0.0, true_speed_mps - max_decel_mps2 * dt)
	true_velocity = Vector3(cos(true_heading_rad), 0.0, sin(true_heading_rad)) * true_speed_mps
	if true_speed_mps < 0.01:
		true_speed_mps = 0.0

func trigger_emergency_stop(reason: String) -> void:
	emergency_stop_requested = true
	fault_reason = reason
	state = AgentState.EMERGENCY_STOP

func reset_emergency_stop() -> void:
	emergency_stop_requested = false
	fault_reason = ""
	state = AgentState.IDLE

func get_telemetry() -> Dictionary:
	return {
		"agent_id": agent_id,
		"state": state,
		"state_string": AgentState.keys()[state],
		"true_position": true_position,
		"true_velocity": true_velocity,
		"true_speed_mps": true_speed_mps,
		"true_heading_rad": true_heading_rad,
		"true_steering_rad": true_steering_rad,
		"estimated_position": estimated_position,
		"estimated_heading_rad": estimated_heading_rad,
		"battery_soc": battery_soc,
		"distance_travelled_m": distance_travelled_m,
		"tracking_rmse": lateral_controller.get_root_mean_square_error(),
		"heartbeat_timestamp": heartbeat_timestamp_sec
	}
