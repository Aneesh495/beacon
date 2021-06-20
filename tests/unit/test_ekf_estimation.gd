# tests/unit/test_ekf_estimation.gd
class_name TestEkfEstimation
extends "res://tests/test_base.gd"

# Unit verification suite for 15-state Error-State Extended Kalman Filter (ES-EKF).

const ExtendedKalmanFilter = preload("res://estimation/extended_kalman_filter.gd")

func before_each() -> void:
	reset_test()

func test_ekf_prediction_and_covariance_growth() -> Dictionary:
	var ekf: ExtendedKalmanFilter = ExtendedKalmanFilter.new()
	ekf.reset_state(Vector3(0.0, 0.0, 0.0), Vector3(5.0, 0.0, 0.0), Quaternion.IDENTITY)
	
	var initial_pos_var: float = ekf.P.get_element(0, 0)
	
	# Propagate 10 steps of 0.05s with level IMU (accel = [0, 0, 9.80665], gyro = [0, 0, 0])
	for i in range(10):
		ekf.predict_imu(Vector3(0.0, 0.0, 9.80665), Vector3.ZERO, 0.05)
		
	# Traveled distance at 5 m/s for 0.5s should be approx 2.5m
	assert_almost_equal(ekf.position.x, 2.5, 1e-3, "EKF forward dead-reckoning X")
	assert_almost_equal(ekf.position.y, 0.0, 1e-3, "EKF lateral Y should remain 0")
	
	# Covariance uncertainty should grow during blind dead reckoning
	var propagated_pos_var: float = ekf.P.get_element(0, 0)
	assert_true(propagated_pos_var > initial_pos_var, "Position variance must grow without updates")
	
	return get_results()

func test_ekf_gnss_update_and_covariance_shrinkage() -> Dictionary:
	var ekf: ExtendedKalmanFilter = ExtendedKalmanFilter.new()
	ekf.reset_state(Vector3(0.0, 0.0, 0.0), Vector3.ZERO, Quaternion.IDENTITY)
	
	# Inject initial position offset: true is at (1.0, 2.0, 0.0)
	var true_pos: Vector3 = Vector3(1.0, 2.0, 0.0)
	
	var var_before: float = ekf.P.get_element(0, 0)
	var updated: bool = ekf.update_gnss_position(true_pos, 0.05)
	assert_true(updated, "GNSS update should be accepted")
	
	var var_after: float = ekf.P.get_element(0, 0)
	# Covariance should shrink significantly after high-precision RTK GNSS update
	assert_true(var_after < var_before, "Variance must shrink after GNSS update")
	
	# Estimated position should converge towards true position
	assert_almost_equal(ekf.position.x, 1.0, 0.1, "Position X convergence")
	assert_almost_equal(ekf.position.y, 2.0, 0.1, "Position Y convergence")
	
	return get_results()

func test_ekf_mahalanobis_outlier_rejection() -> Dictionary:
	var ekf: ExtendedKalmanFilter = ExtendedKalmanFilter.new()
	ekf.reset_state(Vector3(0.0, 0.0, 0.0), Vector3.ZERO, Quaternion.IDENTITY)
	ekf.enable_mahalanobis_gate = true
	
	# Send absurd GNSS measurement: 500 meters away (glitch/multipath jump)
	var wild_measurement: Vector3 = Vector3(500.0, 500.0, 0.0)
	var accepted: bool = ekf.update_gnss_position(wild_measurement, 0.05)
	
	# Gate should reject the wild outlier
	assert_false(accepted, "EKF Mahalanobis gate must reject 500m outlier jump")
	# State position should remain near 0
	assert_almost_equal(ekf.position.x, 0.0, 1e-4)
	
	return get_results()
