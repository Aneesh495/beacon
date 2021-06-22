# tests/unit/test_ukf_estimation.gd
class_name TestUkfEstimation
extends "res://tests/test_base.gd"

# Unit verification suite for Unscented Kalman Filter (UKF) vehicle state estimation.

const UnscentedKalmanFilter = preload("res://estimation/unscented_kalman_filter.gd")

func before_each() -> void:
	reset_test()

func test_sigma_points_generation() -> Dictionary:
	var ukf: UnscentedKalmanFilter = UnscentedKalmanFilter.new(2.8)
	ukf.state_x = PackedFloat64Array([10.0, 5.0, 0.5, 8.0, 0.1])
	
	var sigmas: Array[PackedFloat64Array] = ukf.generate_sigma_points()
	assert_equal(sigmas.size(), 11, "Expected 2*L + 1 = 11 sigma points")
	
	# Point 0 must equal the mean state exactly
	assert_almost_equal(sigmas[0][0], 10.0, 1e-9)
	assert_almost_equal(sigmas[0][1], 5.0, 1e-9)
	assert_almost_equal(sigmas[0][2], 0.5, 1e-9)
	
	return get_results()

func test_ukf_prediction_and_measurement_update() -> Dictionary:
	var ukf: UnscentedKalmanFilter = UnscentedKalmanFilter.new(2.8)
	ukf.state_x = PackedFloat64Array([0.0, 0.0, 0.0, 10.0, 0.0]) # moving forward at 10 m/s
	
	# Predict 1 step: dt = 0.1s
	ukf.predict(0.0, 0.0, 0.1)
	
	# Expected forward displacement ~= 1.0 meter
	assert_almost_equal(ukf.state_x[0], 1.0, 0.05, "Predicted X position")
	assert_almost_equal(ukf.state_x[1], 0.0, 0.05, "Predicted Y position")
	
	# Update with observation at (1.05, 0.02)
	ukf.update_position_2d(Vector2(1.05, 0.02), 0.05)
	
	assert_almost_equal(ukf.state_x[0], 1.05, 0.05, "Updated X position")
	assert_almost_equal(ukf.state_x[1], 0.02, 0.05, "Updated Y position")
	
	return get_results()
