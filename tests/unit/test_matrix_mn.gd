# tests/unit/test_matrix_mn.gd
class_name TestMatrixMN
extends "res://tests/test_base.gd"

# Unit verification suite for MatrixMN arbitrary dense linear algebra routines.

const MatrixMN = preload("res://core/math/matrix_mn.gd")

func before_each() -> void:
	reset_test()

func test_matrix_creation_and_indexing() -> Dictionary:
	var m: MatrixMN = MatrixMN.new(3, 4, 2.5)
	assert_equal(m.rows, 3, "Rows count mismatch")
	assert_equal(m.cols, 4, "Cols count mismatch")
	assert_almost_equal(m.get_element(0, 0), 2.5, 1e-9)
	assert_almost_equal(m.get_element(2, 3), 2.5, 1e-9)
	
	m.set_element(1, 2, 7.8)
	assert_almost_equal(m.get_element(1, 2), 7.8, 1e-9)
	m.add_element(1, 2, 1.2)
	assert_almost_equal(m.get_element(1, 2), 9.0, 1e-9)
	
	var eye: MatrixMN = MatrixMN.identity(3)
	assert_almost_equal(eye.get_element(0, 0), 1.0, 1e-9)
	assert_almost_equal(eye.get_element(1, 1), 1.0, 1e-9)
	assert_almost_equal(eye.get_element(0, 1), 0.0, 1e-9)
	assert_almost_equal(eye.trace(), 3.0, 1e-9)
	
	return get_results()

func test_matrix_arithmetic() -> Dictionary:
	var a: MatrixMN = MatrixMN.from_2d_array([
		[1.0, 2.0],
		[3.0, 4.0]
	])
	var b: MatrixMN = MatrixMN.from_2d_array([
		[5.0, 6.0],
		[7.0, 8.0]
	])
	
	var c_add: MatrixMN = a.add(b)
	assert_almost_equal(c_add.get_element(0, 0), 6.0, 1e-9)
	assert_almost_equal(c_add.get_element(1, 1), 12.0, 1e-9)
	
	var c_sub: MatrixMN = b.subtract(a)
	assert_almost_equal(c_sub.get_element(0, 0), 4.0, 1e-9)
	assert_almost_equal(c_sub.get_element(1, 1), 4.0, 1e-9)
	
	var c_scale: MatrixMN = a.scale(2.5)
	assert_almost_equal(c_scale.get_element(0, 1), 5.0, 1e-9)
	assert_almost_equal(c_scale.get_element(1, 0), 7.5, 1e-9)
	
	# Matrix multiplication: [1 2; 3 4] * [5 6; 7 8] = [19 22; 43 50]
	var c_mul: MatrixMN = a.multiply(b)
	assert_almost_equal(c_mul.get_element(0, 0), 19.0, 1e-9)
	assert_almost_equal(c_mul.get_element(0, 1), 22.0, 1e-9)
	assert_almost_equal(c_mul.get_element(1, 0), 43.0, 1e-9)
	assert_almost_equal(c_mul.get_element(1, 1), 50.0, 1e-9)
	
	# Vector multiplication: [1 2; 3 4] * [2; 3] = [8; 18]
	var vec: PackedFloat64Array = PackedFloat64Array([2.0, 3.0])
	var res_vec: PackedFloat64Array = a.multiply_vector(vec)
	assert_almost_equal(res_vec[0], 8.0, 1e-9)
	assert_almost_equal(res_vec[1], 18.0, 1e-9)
	
	return get_results()

func test_matrix_transpose_and_norm() -> Dictionary:
	var m: MatrixMN = MatrixMN.from_2d_array([
		[1.0, 2.0, 3.0],
		[4.0, 5.0, 6.0]
	])
	var mt: MatrixMN = m.transpose()
	assert_equal(mt.rows, 3)
	assert_equal(mt.cols, 2)
	assert_almost_equal(mt.get_element(0, 1), 4.0, 1e-9)
	assert_almost_equal(mt.get_element(2, 0), 3.0, 1e-9)
	
	# Frobenius norm: sqrt(1 + 4 + 9 + 16 + 25 + 36) = sqrt(91) ~= 9.539392
	assert_almost_equal(m.frobenius_norm(), sqrt(91.0), 1e-6)
	
	return get_results()

func test_lu_decomposition_and_determinant() -> Dictionary:
	var a: MatrixMN = MatrixMN.from_2d_array([
		[2.0, 1.0, 1.0],
		[4.0, -6.0, 0.0],
		[-2.0, 7.0, 2.0]
	])
	var lu: Dictionary = a.decompose_lu()
	var L: MatrixMN = lu["L"]
	var U: MatrixMN = lu["U"]
	var P: MatrixMN = lu["P"]
	var det: float = lu["det"]
	
	# Verify P * A = L * U
	var pa: MatrixMN = P.multiply(a)
	var lu_prod: MatrixMN = L.multiply(U)
	for i in range(3):
		for j in range(3):
			assert_almost_equal(pa.get_element(i, j), lu_prod.get_element(i, j), 1e-7, "LU reconstruction error")
			
	# Determinant of this matrix is -16.0
	assert_almost_equal(det, -16.0, 1e-7, "Determinant mismatch")
	
	return get_results()

func test_cholesky_decomposition() -> Dictionary:
	# Symmetric positive-definite matrix
	var a: MatrixMN = MatrixMN.from_2d_array([
		[4.0, 12.0, -16.0],
		[12.0, 37.0, -43.0],
		[-16.0, -43.0, 98.0]
	])
	var L: MatrixMN = a.decompose_cholesky()
	var Lt: MatrixMN = L.transpose()
	var recon: MatrixMN = L.multiply(Lt)
	
	for i in range(3):
		for j in range(3):
			assert_almost_equal(a.get_element(i, j), recon.get_element(i, j), 1e-6, "Cholesky reconstruction error")
			
	return get_results()

func test_qr_decomposition() -> Dictionary:
	var a: MatrixMN = MatrixMN.from_2d_array([
		[12.0, -51.0, 4.0],
		[6.0, 167.0, -68.0],
		[-4.0, 24.0, -41.0]
	])
	var qr: Dictionary = a.decompose_qr()
	var Q: MatrixMN = qr["Q"]
	var R: MatrixMN = qr["R"]
	
	# Verify Q * R = A
	var qr_prod: MatrixMN = Q.multiply(R)
	for i in range(3):
		for j in range(3):
			assert_almost_equal(a.get_element(i, j), qr_prod.get_element(i, j), 1e-6, "QR reconstruction error")
			
	# Verify Q^T * Q = I
	var Qt: MatrixMN = Q.transpose()
	var eye_test: MatrixMN = Qt.multiply(Q)
	for i in range(3):
		for j in range(3):
			var expected: float = 1.0 if i == j else 0.0
			assert_almost_equal(eye_test.get_element(i, j), expected, 1e-6, "Q orthogonality error")
			
	return get_results()

func test_matrix_inverse_and_linear_solve() -> Dictionary:
	var a: MatrixMN = MatrixMN.from_2d_array([
		[1.0, 2.0, 3.0],
		[0.0, 1.0, 4.0],
		[5.0, 6.0, 0.0]
	])
	var inv_a: MatrixMN = a.inverse()
	var prod: MatrixMN = a.multiply(inv_a)
	
	for i in range(3):
		for j in range(3):
			var expected: float = 1.0 if i == j else 0.0
			assert_almost_equal(prod.get_element(i, j), expected, 1e-6, "Inverse identity check failed")
			
	# Test solve_linear: A * x = b
	var b: PackedFloat64Array = PackedFloat64Array([14.0, 11.0, 17.0])
	var x: PackedFloat64Array = a.solve_linear(b)
	# Check that A * x matches b
	var ax: PackedFloat64Array = a.multiply_vector(x)
	for i in range(3):
		assert_almost_equal(ax[i], b[i], 1e-6, "Linear solve residual error")
		
	return get_results()

func test_covariance_regularization() -> Dictionary:
	var cov: MatrixMN = MatrixMN.from_2d_array([
		[1.0, 0.5, 0.2],
		[0.4, 0.8, -0.1],
		[0.2, -0.1, -0.05] # Negative diagonal to test clamping
	])
	var reg: MatrixMN = cov.regularize_covariance(1e-6)
	
	# Verify symmetry
	for i in range(3):
		for j in range(3):
			assert_almost_equal(reg.get_element(i, j), reg.get_element(j, i), 1e-9, "Symmetry check")
			
	# Verify diagonal floor
	assert_true(reg.get_element(2, 2) >= 1e-6, "Diagonal floor check")
	
	return get_results()
