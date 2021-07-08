# tests/unit/test_controllers.gd
class_name TestControllers
extends "res://tests/test_base.gd"

# Unit verification suite for vehicle controllers: MPC, Stanley, Pure Pursuit, and Cascaded PID.

const ControllerBase = preload("res://control/controller_base.gd")
const ModelPredictiveController = preload("res://control/model_predictive_controller.gd")
const StanleyController = preload("res://control/stanley_controller.gd")
const PurePursuitController = preload("res://control/pure_pursuit_controller.gd")
const CascadedPidController = preload("res://control/cascaded_pid_controller.gd")

func before_each() -> void:
	reset_test()

func test_controller_base_slew_rate() -> Dictionary:
	var base: ControllerBase = ControllerBase.new("test_base", 2.8)
	base.max_steering_rate_radps = deg_to_rad(20.0) # 20 deg/s limit
	
	# Attempt instantaneous 30 deg target steer with dt = 0.1s
	# Max allowed step is 20 * 0.1 = 2 deg = ~0.0349 rad
	var actual: float = base.apply_steering_slew_rate(deg_to_rad(30.0), 0.1)
	assert_almost_equal(actual, deg_to_rad(2.0), 1e-4, "Steering slew rate clamping check")
	
	return get_results()

func test_mpc_control_generation() -> Dictionary:
	var mpc: ModelPredictiveController = ModelPredictiveController.new(2.8, 10, 0.1)
	
	# Current state: [x=0, y=0, psi=0, v=10]
	var current_state: Vector4 = Vector4(0.0, 0.0, 0.0, 10.0)
	
	# Reference straight trajectory at 10 m/s
	var ref_trajectory: Array[Dictionary] = []
	for i in range(15):
		ref_trajectory.append({
			"position": Vector2(float(i) * 1.0, 0.0),
			"heading": 0.0,
			"v": 10.0
		})
		
	var ctrl: Dictionary = mpc.compute_control(current_state, ref_trajectory)
	assert_true(ctrl.has("accel_cmd") and ctrl.has("steering_cmd"))
	
	# On straight trajectory with matching speed, control effort should be near 0
	assert_almost_equal(ctrl["accel_cmd"], 0.0, 0.2, "MPC acceleration on straight path")
	assert_almost_equal(ctrl["steering_cmd"], 0.0, 0.1, "MPC steering on straight path")
	assert_equal(ctrl["predicted_states"].size(), 11, "Horizon states count check")
	
	return get_results()

func test_stanley_controller_tracking() -> Dictionary:
	var stanley: StanleyController = StanleyController.new(2.8, 1.2)
	
	# Path along X-axis
	var path: Array[Vector2] = [Vector2(0.0, 0.0), Vector2(100.0, 0.0)]
	
	# Vehicle offset 1 meter to the left of path (y = 1.0) heading parallel (psi = 0) at 10 m/s
	# Step controller over 4 cycles (200 ms) to allow slew-rate ramp
	var steer: float = 0.0
	for _step in range(4):
		steer = stanley.compute_steering(0.0, 1.0, 0.0, 10.0, path, 0.05)
	
	# Steering should turn right (negative steering angle) to steer back towards y=0 centerline
	assert_true(steer < 0.0, "Stanley should steer right to correct left offset")
	assert_true(abs(steer) > 0.05, "Steering magnitude should be significant")
	
	return get_results()

func test_pure_pursuit_controller_tracking() -> Dictionary:
	var pp: PurePursuitController = PurePursuitController.new(2.8, 1.0)
	var path: Array[Vector2] = [Vector2(0.0, 0.0), Vector2(100.0, 0.0)]
	
	# Vehicle offset 2 meters to the right of path (y = -2.0)
	var steer: float = pp.compute_steering(0.0, -2.0, 0.0, 10.0, path, 0.05)
	
	# Steering should turn left (positive steering angle) to steer back towards y=0
	assert_true(steer > 0.0, "Pure pursuit should steer left to correct right offset")
	
	return get_results()

func test_cascaded_pid_longitudinal() -> Dictionary:
	var pid: CascadedPidController = CascadedPidController.new(1800.0)
	
	# Speeding up from 0 to 10 m/s: should produce throttle
	var accel_res: Dictionary = pid.compute_longitudinal_control(0.0, 10.0, 0.05)
	assert_true(accel_res["throttle_ratio"] > 0.0, "Should apply throttle to accelerate")
	assert_almost_equal(accel_res["brake_pressure_kpa"], 0.0, 1e-4, "No brake when accelerating")
	
	# Slowing down from 20 to 5 m/s: should apply hydraulic brakes
	var brake_res: Dictionary = pid.compute_longitudinal_control(20.0, 5.0, 0.05)
	assert_almost_equal(brake_res["throttle_ratio"], 0.0, 1e-4, "No throttle when braking")
	assert_true(brake_res["brake_pressure_kpa"] > 0.0, "Should apply brake pressure to decelerate")
	
	return get_results()
