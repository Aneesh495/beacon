# tests/unit/test_kalman_smoother.gd
class_name TestKalmanSmoother
extends "res://tests/test_base.gd"

# Unit verification suite for Rauch-Tung-Striebel (RTS) optimal Kalman smoother.

const KalmanSmoother = preload("res://core/math/kalman_smoother.gd")
const MatrixMN = preload("res://core/math/matrix_mn.gd")

func before_each() -> void:
	reset_test()

func test_rts_smoother_variance_reduction() -> Dictionary:
	var smoother: KalmanSmoother = KalmanSmoother.new()
	
	# Simulate a 1D constant velocity system [pos, vel]
	# F = [[1, 0.1], [0, 1]]
	var F: MatrixMN = MatrixMN.new(2, 2)
	F.set_element(0, 0, 1.0)
	F.set_element(0, 1, 0.1)
	F.set_element(1, 0, 0.0)
	F.set_element(1, 1, 1.0)
	
	# Step 0
	var x0: PackedFloat64Array = PackedFloat64Array([0.0, 10.0])
	var P0: MatrixMN = MatrixMN.new(2, 2)
	P0.set_element(0, 0, 1.0)
	P0.set_element(1, 1, 0.5)
	
	# Step 1
	var x1_pred: PackedFloat64Array = PackedFloat64Array([1.0, 10.0])
	var P1_pred: MatrixMN = MatrixMN.new(2, 2)
	P1_pred.set_element(0, 0, 1.5)
	P1_pred.set_element(1, 1, 0.8)
	
	var x1_filt: PackedFloat64Array = PackedFloat64Array([1.1, 9.9])
	var P1_filt: MatrixMN = MatrixMN.new(2, 2)
	P1_filt.set_element(0, 0, 0.4)
	P1_filt.set_element(1, 1, 0.3)
	
	smoother.record_step(x0, P0, F, x0, P0)
	smoother.record_step(x1_filt, P1_filt, F, x1_pred, P1_pred)
	
	var smoothed: Array[Dictionary] = smoother.compute_smoothed_trajectory()
	assert_equal(smoothed.size(), 2, "Smoothed trajectory length check")
	
	# Smoothed variance at step 0 should be strictly less than or equal to forward filtered variance
	var P0_smooth: MatrixMN = smoothed[0]["P_smooth"]
	assert_true(P0_smooth.get_element(0, 0) <= P0.get_element(0, 0) + 1e-6, "Smoothed position variance reduction")
	assert_true(P0_smooth.get_element(1, 1) <= P0.get_element(1, 1) + 1e-6, "Smoothed velocity variance reduction")
	
	return get_results()
