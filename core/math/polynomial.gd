# core/math/polynomial.gd
class_name Polynomial
extends RefCounted

# Arbitrary-degree real polynomial representation: P(x) = a0 + a1*x + a2*x^2 + ... + an*x^n
# Implements Horner evaluation, differentiation, integration, least-squares fitting,
# and Durand-Kerner iterative root extraction.

const MatrixMN = preload("res://core/math/matrix_mn.gd")

var coeffs: PackedFloat64Array = PackedFloat64Array()

func _init(p_coeffs: PackedFloat64Array = PackedFloat64Array()) -> void:
	coeffs = p_coeffs.duplicate()
	_trim_leading_zeros()

func degree() -> int:
	return maxi(0, coeffs.size() - 1)

func _trim_leading_zeros() -> void:
	while coeffs.size() > 1 and abs(coeffs[coeffs.size() - 1]) < 1e-14:
		coeffs.remove_at(coeffs.size() - 1)

static func from_roots(roots: PackedFloat64Array) -> Polynomial:
	var p: Polynomial = Polynomial.new(PackedFloat64Array([1.0]))
	for r in roots:
		# Multiply by (x - r)
		var factor: Polynomial = Polynomial.new(PackedFloat64Array([-r, 1.0]))
		p = p.multiply(factor)
	return p

func evaluate(x: float) -> float:
	var n: int = coeffs.size()
	if n == 0:
		return 0.0
	# Horner's method
	var val: float = coeffs[n - 1]
	for i in range(n - 2, -1, -1):
		val = val * x + coeffs[i]
	return val

func derivative() -> Polynomial:
	var n: int = coeffs.size()
	if n <= 1:
		return Polynomial.new(PackedFloat64Array([0.0]))
	var d_coeffs: PackedFloat64Array = PackedFloat64Array()
	d_coeffs.resize(n - 1)
	for i in range(1, n):
		d_coeffs[i - 1] = coeffs[i] * float(i)
	return Polynomial.new(d_coeffs)

func integral(constant_c: float = 0.0) -> Polynomial:
	var n: int = coeffs.size()
	var int_coeffs: PackedFloat64Array = PackedFloat64Array()
	int_coeffs.resize(n + 1)
	int_coeffs[0] = constant_c
	for i in range(n):
		int_coeffs[i + 1] = coeffs[i] / float(i + 1)
	return Polynomial.new(int_coeffs)

func definite_integral(a: float, b: float) -> float:
	var anti: Polynomial = integral(0.0)
	return anti.evaluate(b) - anti.evaluate(a)

func add(other: Polynomial) -> Polynomial:
	var max_len: int = maxi(coeffs.size(), other.coeffs.size())
	var res: PackedFloat64Array = PackedFloat64Array()
	res.resize(max_len)
	for i in range(max_len):
		var a: float = coeffs[i] if i < coeffs.size() else 0.0
		var b: float = other.coeffs[i] if i < other.coeffs.size() else 0.0
		res[i] = a + b
	return Polynomial.new(res)

func subtract(other: Polynomial) -> Polynomial:
	var max_len: int = maxi(coeffs.size(), other.coeffs.size())
	var res: PackedFloat64Array = PackedFloat64Array()
	res.resize(max_len)
	for i in range(max_len):
		var a: float = coeffs[i] if i < coeffs.size() else 0.0
		var b: float = other.coeffs[i] if i < other.coeffs.size() else 0.0
		res[i] = a - b
	return Polynomial.new(res)

func scale(scalar: float) -> Polynomial:
	var res: PackedFloat64Array = PackedFloat64Array()
	res.resize(coeffs.size())
	for i in range(coeffs.size()):
		res[i] = coeffs[i] * scalar
	return Polynomial.new(res)

func multiply(other: Polynomial) -> Polynomial:
	var n: int = coeffs.size()
	var m: int = other.coeffs.size()
	if n == 0 or m == 0:
		return Polynomial.new(PackedFloat64Array([0.0]))
	var res_len: int = n + m - 1
	var res: PackedFloat64Array = PackedFloat64Array()
	res.resize(res_len)
	for i in range(res_len):
		res[i] = 0.0
		
	for i in range(n):
		for j in range(m):
			res[i + j] += coeffs[i] * other.coeffs[j]
	return Polynomial.new(res)

# Fits a polynomial of specified degree to (x, y) points via linear least squares.
# Uses QR decomposition on the Vandermonde matrix.
static func fit_least_squares(x_pts: PackedFloat64Array, y_pts: PackedFloat64Array, target_degree: int) -> Polynomial:
	var m_pts: int = x_pts.size()
	assert(m_pts == y_pts.size(), "Point arrays must have identical length")
	assert(m_pts > target_degree, "Point count must exceed target polynomial degree")
	
	var n_cols: int = target_degree + 1
	var V: MatrixMN = MatrixMN.new(m_pts, n_cols)
	for i in range(m_pts):
		var xi: float = x_pts[i]
		var cur_pow: float = 1.0
		for j in range(n_cols):
			V.set_element(i, j, cur_pow)
			cur_pow *= xi
			
	# Solve normal equations: V^T * V * a = V^T * y
	var Vt: MatrixMN = V.transpose()
	var VtV: MatrixMN = Vt.multiply(V)
	var Vty: PackedFloat64Array = Vt.multiply_vector(y_pts)
	var fitted_coeffs: PackedFloat64Array = VtV.solve_linear(Vty)
	
	return Polynomial.new(fitted_coeffs)

# Durand-Kerner (Weierstrass) simultaneous iterative polynomial root-finding algorithm.
# Extracts real roots with imaginary component below imag_tolerance.
func find_real_roots(max_iterations: int = 100, tolerance: float = 1e-8) -> PackedFloat64Array:
	var deg: int = degree()
	if deg <= 0:
		return PackedFloat64Array()
	if deg == 1:
		# a0 + a1 * x = 0 => x = -a0 / a1
		return PackedFloat64Array([-coeffs[0] / coeffs[1]])
	if deg == 2:
		# Quadratic formula
		var c: float = coeffs[0]
		var b: float = coeffs[1]
		var a: float = coeffs[2]
		var disc: float = b * b - 4.0 * a * c
		if disc >= 0.0:
			var sqrt_disc: float = sqrt(disc)
			return PackedFloat64Array([(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)])
		return PackedFloat64Array()
		
	# Normalize monic polynomial: P_monic(x) = x^n + b_{n-1} x^{n-1} + ... + b0
	var lead: float = coeffs[deg]
	var monic_coeffs: PackedFloat64Array = PackedFloat64Array()
	monic_coeffs.resize(deg + 1)
	for i in range(deg + 1):
		monic_coeffs[i] = coeffs[i] / lead
		
	# Initial complex roots placed on Aberth circle in complex plane
	var roots_re: PackedFloat64Array = PackedFloat64Array()
	var roots_im: PackedFloat64Array = PackedFloat64Array()
	roots_re.resize(deg)
	roots_im.resize(deg)
	
	var radius: float = 1.0 + monic_coeffs[0] # Initial guess radius
	for i in range(deg):
		var angle: float = (2.0 * PI * float(i) + 0.4) / float(deg)
		roots_re[i] = pow(radius, float(i) / float(deg)) * cos(angle)
		roots_im[i] = pow(radius, float(i) / float(deg)) * sin(angle)
		
	# Iterative Durand-Kerner updates
	for iter in range(max_iterations):
		var max_diff: float = 0.0
		for i in range(deg):
			# Evaluate monic polynomial at complex root (re, im)
			var val_re: float = monic_coeffs[deg]
			var val_im: float = 0.0
			for k in range(deg - 1, -1, -1):
				var next_re: float = val_re * roots_re[i] - val_im * roots_im[i] + monic_coeffs[k]
				var next_im: float = val_re * roots_im[i] + val_im * roots_re[i]
				val_re = next_re
				val_im = next_im
				
			# Compute product over all j != i: (z_i - z_j)
			var denom_re: float = 1.0
			var denom_im: float = 0.0
			for j in range(deg):
				if i != j:
					var diff_re: float = roots_re[i] - roots_re[j]
					var diff_im: float = roots_im[i] - roots_im[j]
					var next_d_re: float = denom_re * diff_re - denom_im * diff_im
					var next_d_im: float = denom_re * diff_im + denom_im * diff_re
					denom_re = next_d_re
					denom_im = next_d_im
					
			var denom_mag_sq: float = denom_re * denom_re + denom_im * denom_im
			if denom_mag_sq < 1e-20:
				continue
				
			# Complex division: delta = val / denom
			var delta_re: float = (val_re * denom_re + val_im * denom_im) / denom_mag_sq
			var delta_im: float = (val_im * denom_re - val_re * denom_im) / denom_mag_sq
			
			roots_re[i] -= delta_re
			roots_im[i] -= delta_im
			
			var diff_mag: float = sqrt(delta_re * delta_re + delta_im * delta_im)
			if diff_mag > max_diff:
				max_diff = diff_mag
				
		if max_diff < tolerance:
			break
			
	var real_roots: PackedFloat64Array = PackedFloat64Array()
	for i in range(deg):
		if abs(roots_im[i]) < 1e-4:
			real_roots.append(roots_re[i])
			
	real_roots.sort()
	return real_roots
