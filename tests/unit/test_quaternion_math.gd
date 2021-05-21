# tests/unit/test_quaternion_math.gd
class_name TestQuaternionMath
extends "res://tests/test_base.gd"

# Unit verification suite for SO(3) Lie group and unit quaternion algebra.

const MatrixMN = preload("res://core/math/matrix_mn.gd")
const QuaternionMath = preload("res://core/math/quaternion_math.gd")

func before_each() -> void:
	reset_test()

func test_skew_symmetric_and_unskew() -> Dictionary:
	var v: Vector3 = Vector3(1.5, -2.4, 3.1)
	var skew: MatrixMN = QuaternionMath.skew_symmetric(v)
	
	# Diagonal should be zero
	assert_almost_equal(skew.get_element(0, 0), 0.0, 1e-9)
	assert_almost_equal(skew.get_element(1, 1), 0.0, 1e-9)
	assert_almost_equal(skew.get_element(2, 2), 0.0, 1e-9)
	
	# Skew symmetry: skew^T = -skew
	var skew_t: MatrixMN = skew.transpose()
	for i in range(3):
		for j in range(3):
			assert_almost_equal(skew.get_element(i, j), -skew_t.get_element(i, j), 1e-9)
			
	# Roundtrip via unskew
	var v_rec: Vector3 = QuaternionMath.unskew(skew)
	assert_vector3_almost_equal(v_rec, v, 1e-9, "Unskew roundtrip error")
	
	return get_results()

func test_so3_exp_and_log_roundtrip() -> Dictionary:
	var w: Vector3 = Vector3(0.2, -0.4, 0.6)
	var R: MatrixMN = QuaternionMath.exp_so3(w)
	
	# Verify R is orthogonal: R * R^T = I
	var Rt: MatrixMN = R.transpose()
	var eye_check: MatrixMN = R.multiply(Rt)
	for i in range(3):
		for j in range(3):
			var expected: float = 1.0 if i == j else 0.0
			assert_almost_equal(eye_check.get_element(i, j), expected, 1e-6, "SO(3) orthogonality check")
			
	# Roundtrip via log_so3
	var w_rec: Vector3 = QuaternionMath.log_so3(R)
	assert_vector3_almost_equal(w_rec, w, 1e-6, "SO(3) exp-log roundtrip error")
	
	return get_results()

func test_quaternion_matrix_conversions() -> Dictionary:
	var q_orig: Quaternion = Quaternion(Vector3(0.3, 0.5, -0.2).normalized(), 0.785).normalized()
	var R: MatrixMN = QuaternionMath.quaternion_to_matrix3(q_orig)
	var q_rec: Quaternion = QuaternionMath.matrix3_to_quaternion(R)
	
	# Quaternions q and -q represent the same rotation
	var dot_prod: float = abs(q_orig.dot(q_rec))
	assert_almost_equal(dot_prod, 1.0, 1e-5, "Quaternion-Matrix3 roundtrip error")
	
	return get_results()

func test_quaternion_exp_log_roundtrip() -> Dictionary:
	var w: Vector3 = Vector3(0.15, 0.25, -0.35)
	var q: Quaternion = QuaternionMath.exp_quaternion(w)
	var w_rec: Vector3 = QuaternionMath.log_quaternion(q)
	
	assert_vector3_almost_equal(w_rec, w, 1e-6, "Quaternion exp-log roundtrip error")
	
	return get_results()

func test_euler_conversions() -> Dictionary:
	var rpy: Vector3 = Vector3(0.2, -0.15, 0.8)
	var q: Quaternion = QuaternionMath.from_euler_rpy(rpy)
	var rpy_rec: Vector3 = QuaternionMath.to_euler_rpy(q)
	
	assert_vector3_almost_equal(rpy_rec, rpy, 1e-4, "Euler RPY roundtrip error")
	
	return get_results()

func test_strapdown_integration() -> Dictionary:
	var q0: Quaternion = Quaternion.IDENTITY
	var omega: Vector3 = Vector3(0.0, 0.0, 1.0) # 1 rad/s yaw
	var dt: float = 0.1
	
	var q1: Quaternion = QuaternionMath.integrate_quaternion(q0, omega, dt)
	var euler1: Vector3 = QuaternionMath.to_euler_rpy(q1)
	
	# After 0.1 seconds at 1 rad/s yaw, yaw should be approximately 0.1 rad
	assert_almost_equal(euler1.z, 0.1, 1e-4, "Strapdown integration yaw error")
	assert_almost_equal(euler1.x, 0.0, 1e-4, "Strapdown roll should remain 0")
	assert_almost_equal(euler1.y, 0.0, 1e-4, "Strapdown pitch should remain 0")
	
	return get_results()
