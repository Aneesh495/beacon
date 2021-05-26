# tests/unit/test_polynomial_spline.gd
class_name TestPolynomialSpline
extends "res://tests/test_base.gd"

# Unit verification suite for polynomials, quintic splines, clothoids, and statistics.

const Polynomial = preload("res://core/math/polynomial.gd")
const HermiteSpline = preload("res://core/math/hermite_spline.gd")
const Clothoid = preload("res://core/math/clothoid.gd")
const Statistics = preload("res://core/math/statistics.gd")
const MatrixMN = preload("res://core/math/matrix_mn.gd")

func before_each() -> void:
	reset_test()

func test_polynomial_arithmetic_and_derivatives() -> Dictionary:
	# P(x) = 2 - 3*x + x^2 = (x - 1)*(x - 2)
	var p: Polynomial = Polynomial.new(PackedFloat64Array([2.0, -3.0, 1.0]))
	assert_equal(p.degree(), 2, "Degree check")
	assert_almost_equal(p.evaluate(0.0), 2.0, 1e-9)
	assert_almost_equal(p.evaluate(1.0), 0.0, 1e-9) # Root at 1
	assert_almost_equal(p.evaluate(2.0), 0.0, 1e-9) # Root at 2
	assert_almost_equal(p.evaluate(3.0), 2.0, 1e-9)
	
	# P'(x) = -3 + 2*x
	var dp: Polynomial = p.derivative()
	assert_equal(dp.degree(), 1)
	assert_almost_equal(dp.evaluate(1.5), 0.0, 1e-9, "Minimum at x = 1.5")
	
	# Integral of P(x) from 0 to 3: [2*x - 1.5*x^2 + (1/3)*x^3]_0^3 = 6 - 13.5 + 9 = 1.5
	var int_val: float = p.definite_integral(0.0, 3.0)
	assert_almost_equal(int_val, 1.5, 1e-6, "Definite integral mismatch")
	
	# Root finding
	var roots: PackedFloat64Array = p.find_real_roots()
	assert_equal(roots.size(), 2, "Expected 2 real roots")
	assert_almost_equal(roots[0], 1.0, 1e-4)
	assert_almost_equal(roots[1], 2.0, 1e-4)
	
	return get_results()

func test_least_squares_polynomial_fit() -> Dictionary:
	# True model: y = 1.5 + 2.0*x - 0.5*x^2
	var x_pts: PackedFloat64Array = PackedFloat64Array([-2.0, -1.0, 0.0, 1.0, 2.0, 3.0])
	var y_pts: PackedFloat64Array = PackedFloat64Array()
	y_pts.resize(x_pts.size())
	for i in range(x_pts.size()):
		var xi: float = x_pts[i]
		y_pts[i] = 1.5 + 2.0 * xi - 0.5 * xi * xi
		
	var fitted: Polynomial = Polynomial.fit_least_squares(x_pts, y_pts, 2)
	assert_almost_equal(fitted.coeffs[0], 1.5, 1e-6, "Fit c0")
	assert_almost_equal(fitted.coeffs[1], 2.0, 1e-6, "Fit c1")
	assert_almost_equal(fitted.coeffs[2], -0.5, 1e-6, "Fit c2")
	
	return get_results()

func test_quintic_hermite_boundary_conditions() -> Dictionary:
	var p0: Vector2 = Vector2(0.0, 0.0)
	var v0: Vector2 = Vector2(10.0, 0.0) # moving east at 10 m/s
	var a0: Vector2 = Vector2(0.0, 0.0)
	
	var p1: Vector2 = Vector2(50.0, 20.0) # lane change to (50, 20)
	var v1: Vector2 = Vector2(12.0, 0.0) # moving east at 12 m/s
	var a1: Vector2 = Vector2(0.0, 0.0)
	
	var spline: HermiteSpline = HermiteSpline.create_quintic(p0, v0, a0, p1, v1, a1)
	
	# Verify t = 0 boundary conditions
	assert_vector2_almost_equal(spline.evaluate_position(0.0), p0, 1e-6, "p(0) boundary")
	assert_vector2_almost_equal(spline.evaluate_velocity(0.0), v0, 1e-6, "v(0) boundary")
	assert_vector2_almost_equal(spline.evaluate_acceleration(0.0), a0, 1e-6, "a(0) boundary")
	
	# Verify t = 1 boundary conditions
	assert_vector2_almost_equal(spline.evaluate_position(1.0), p1, 1e-6, "p(1) boundary")
	assert_vector2_almost_equal(spline.evaluate_velocity(1.0), v1, 1e-6, "v(1) boundary")
	assert_vector2_almost_equal(spline.evaluate_acceleration(1.0), a1, 1e-6, "a(1) boundary")
	
	# Arc length should be greater than Euclidean distance
	var direct_dist: float = p0.distance_to(p1)
	assert_true(spline.total_arc_length >= direct_dist, "Arc length must be >= direct distance")
	
	# Curvature should be 0 at start and end since a0 and a1 are zero
	assert_almost_equal(spline.evaluate_curvature(0.0), 0.0, 1e-6, "Curvature at start")
	assert_almost_equal(spline.evaluate_curvature(1.0), 0.0, 1e-6, "Curvature at end")
	
	return get_results()

func test_clothoid_curvature_and_fresnel() -> Dictionary:
	# Fresnel values known constants: C(0) = 0, S(0) = 0
	assert_almost_equal(Clothoid.fresnel_c(0.0), 0.0, 1e-9)
	assert_almost_equal(Clothoid.fresnel_s(0.0), 0.0, 1e-9)
	
	# C(1.0) ~= 0.779893, S(1.0) ~= 0.438259
	assert_almost_equal(Clothoid.fresnel_c(1.0), 0.779893, 1e-3, "Fresnel C(1.0)")
	assert_almost_equal(Clothoid.fresnel_s(1.0), 0.438259, 1e-3, "Fresnel S(1.0)")
	
	# Clothoid transition curve test
	var start_pos: Vector2 = Vector2(0.0, 0.0)
	var clothoid: Clothoid = Clothoid.build_transition_curve(start_pos, 0.0, 0.0, 0.1, 20.0)
	assert_almost_equal(clothoid.sigma, 0.005, 1e-9, "Clothoid sigma check")
	
	var res_mid: Dictionary = clothoid.evaluate(10.0)
	assert_almost_equal(res_mid["curvature"], 0.05, 1e-6, "Midpoint curvature")
	
	var res_end: Dictionary = clothoid.evaluate(20.0)
	assert_almost_equal(res_end["curvature"], 0.10, 1e-6, "Endpoint curvature")
	
	return get_results()

func test_mahalanobis_distance_and_chi2_gate() -> Dictionary:
	var residual: PackedFloat64Array = PackedFloat64Array([1.0, -0.5])
	var cov: MatrixMN = MatrixMN.from_2d_array([
		[2.0, 0.2],
		[0.2, 1.5]
	])
	
	var d_sq: float = Statistics.mahalanobis_distance_sq(residual, cov)
	assert_true(d_sq > 0.0, "Mahalanobis distance squared should be positive")
	
	# Small residual should pass 99% gate (DOF=2 threshold is 9.210)
	var passes_gate: bool = Statistics.validate_mahalanobis_gate(residual, cov, "p01")
	assert_true(passes_gate, "Small innovation residual should pass Chi2 gate")
	
	# Huge residual (outlier) should fail gate
	var huge_residual: PackedFloat64Array = PackedFloat64Array([10.0, 12.0])
	var passes_outlier: bool = Statistics.validate_mahalanobis_gate(huge_residual, cov, "p01")
	assert_false(passes_outlier, "Huge innovation outlier should be rejected by gate")
	
	return get_results()
