# tests/unit/test_velocity_profiler.gd
class_name TestVelocityProfiler
extends "res://tests/test_base.gd"

# Unit verification suite for time-optimal S-curve velocity profiling.

const VelocityProfiler = preload("res://planning/velocity_profiler.gd")

func before_each() -> void:
	reset_test()

func test_straight_line_profile_acceleration_and_braking() -> Dictionary:
	var profiler: VelocityProfiler = VelocityProfiler.new()
	profiler.max_velocity_mps = 10.0
	profiler.max_acceleration_mps2 = 2.0
	profiler.max_deceleration_mps2 = 2.0
	
	# 100-meter straight path with 50 samples
	var samples: Array[Dictionary] = []
	for i in range(51):
		var s_val: float = float(i) * 2.0
		samples.append({
			"position": Vector2(s_val, 0.0),
			"curvature": 0.0,
			"s": s_val
		})
		
	var profiled: Array[Dictionary] = profiler.profile_path(samples, 0.0, 0.0)
	assert_equal(profiled.size(), 51)
	
	# Boundary speeds must match 0.0
	assert_almost_equal(profiled[0]["v"], 0.0, 1e-4, "Start speed must be 0")
	assert_almost_equal(profiled[50]["v"], 0.0, 1e-4, "End speed must be 0")
	
	# Midpoint speed should have accelerated close to maximum speed
	assert_true(profiled[25]["v"] > 5.0, "Vehicle should accelerate to cruising speed")
	
	# Total time should be positive and realistic
	assert_true(profiled[50]["t"] > 5.0, "Total duration must be positive")
	
	return get_results()

func test_curvature_governor() -> Dictionary:
	var profiler: VelocityProfiler = VelocityProfiler.new()
	profiler.max_velocity_mps = 20.0
	profiler.max_lateral_accel_mps2 = 2.0 # v_max = sqrt(2.0 / kappa)
	
	# Path with a sharp 90-degree curve in the middle (radius = 5 meters => kappa = 0.2)
	# Permissible curve speed: sqrt(2.0 / 0.2) = sqrt(10) ~= 3.16 m/s
	var samples: Array[Dictionary] = []
	for i in range(30):
		var s_val: float = float(i) * 2.0
		var curv: float = 0.2 if (i >= 12 and i <= 18) else 0.0
		samples.append({
			"position": Vector2(s_val, 0.0),
			"curvature": curv,
			"s": s_val
		})
		
	var profiled: Array[Dictionary] = profiler.profile_path(samples, 10.0, 10.0)
	
	# Speed at sharp curve (index 15) must not exceed sqrt(10) ~= 3.162 m/s
	var curve_speed: float = profiled[15]["v"]
	assert_true(curve_speed <= 3.2, "Curvature speed governor must slow down vehicle")
	
	return get_results()
