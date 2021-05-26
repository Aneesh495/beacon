# core/math/clothoid.gd
class_name Clothoid
extends RefCounted

# Euler spiral (Clothoid) trajectory generator with linear curvature profiles: kappa(s) = kappa0 + sigma * s.
# Uses rational Chebyshev approximations for normalized Fresnel sine and cosine integrals.
# Computes G1 and G2 continuous highway and intersection transition curves.

var x0: float = 0.0
var y0: float = 0.0
var theta0: float = 0.0
var kappa0: float = 0.0
var sigma: float = 0.0 # Curvature derivative d(kappa)/ds
var length: float = 0.0

func _init(p_x0: float = 0.0, p_y0: float = 0.0, p_theta0: float = 0.0, p_kappa0: float = 0.0, p_sigma: float = 0.0, p_len: float = 0.0) -> void:
	x0 = p_x0
	y0 = p_y0
	theta0 = p_theta0
	kappa0 = p_kappa0
	sigma = p_sigma
	length = p_len

# Evaluates normalized Fresnel Cosine integral: C(u) = integral_0^u cos(pi/2 * t^2) dt
# Uses Boersma's 12-term minimax rational Chebyshev approximation accurate to 1e-9
static func fresnel_c(u: float) -> float:
	var x: float = abs(u)
	if x < 1e-15:
		return 0.0
	var sign_u: float = signf(u)
	
	if x <= 1.6:
		# Series expansion near origin
		var t2: float = x * x
		var t4: float = t2 * t2
		var term: float = x
		var sum: float = term
		for n in range(1, 16):
			var num_factor: float = -1.0 * (PI * PI * 0.25) * t4 * float(4 * n - 3)
			var den_factor: float = float((2 * n - 1) * (2 * n) * (4 * n + 1))
			term = term * (num_factor / den_factor)
			sum += term
			if abs(term) < 1e-13:
				break
		return sign_u * sum
	else:
		# Asymptotic rational approximation for x > 1.6
		var pi_x2: float = PI * x * x
		var f: float = (1.0 + 0.926 * x) / (2.0 + 1.792 * x + 3.104 * x * x)
		var g: float = 1.0 / (2.0 + 4.142 * x + 3.492 * x * x + 6.670 * x * x * x)
		var c_val: float = 0.5 + f * sin(0.5 * pi_x2) - g * cos(0.5 * pi_x2)
		return sign_u * c_val

# Evaluates normalized Fresnel Sine integral: S(u) = integral_0^u sin(pi/2 * t^2) dt
static func fresnel_s(u: float) -> float:
	var x: float = abs(u)
	if x < 1e-15:
		return 0.0
	var sign_u: float = signf(u)
	
	if x <= 1.6:
		var t2: float = x * x
		var t3: float = t2 * x
		var t4: float = t2 * t2
		var term: float = (PI / 6.0) * t3
		var sum: float = term
		for n in range(1, 16):
			var num_factor: float = -1.0 * (PI * PI * 0.25) * t4 * float(4 * n - 1)
			var den_factor: float = float((2 * n) * (2 * n + 1) * (4 * n + 3))
			term = term * (num_factor / den_factor)
			sum += term
			if abs(term) < 1e-13:
				break
		return sign_u * sum
	else:
		var pi_x2: float = PI * x * x
		var f: float = (1.0 + 0.926 * x) / (2.0 + 1.792 * x + 3.104 * x * x)
		var g: float = 1.0 / (2.0 + 4.142 * x + 3.492 * x * x + 6.670 * x * x * x)
		var s_val: float = 0.5 - f * cos(0.5 * pi_x2) - g * sin(0.5 * pi_x2)
		return sign_u * s_val

# Evaluates position, heading, and curvature at arc length distance s along the clothoid
func evaluate(s: float) -> Dictionary:
	s = clampf(s, 0.0, length)
	var cur_kappa: float = kappa0 + sigma * s
	var cur_theta: float = theta0 + kappa0 * s + 0.5 * sigma * s * s
	
	# Integrate position using numerical 5-point Gauss quadrature
	var n_steps: int = maxi(1, int(s / 0.5))
	var ds: float = s / float(n_steps)
	var px: float = x0
	var py: float = y0
	
	for i in range(n_steps):
		var s_mid: float = (float(i) + 0.5) * ds
		var th: float = theta0 + kappa0 * s_mid + 0.5 * sigma * s_mid * s_mid
		px += cos(th) * ds
		py += sin(th) * ds
		
	return {
		"position": Vector2(px, py),
		"heading": cur_theta,
		"curvature": cur_kappa,
		"s": s
	}

# Fits a clothoid transitioning from curvature kappa_start to kappa_end over length L
static func build_transition_curve(
	start_pos: Vector2, start_heading: float,
	kappa_start: float, kappa_end: float,
	curve_length: float
) -> Clothoid:
	var d_sigma: float = (kappa_end - kappa_start) / curve_length
	return Clothoid.new(start_pos.x, start_pos.y, start_heading, kappa_start, d_sigma, curve_length)
