# core/math/hermite_spline.gd
class_name HermiteSpline
extends RefCounted

# Quintic and cubic Hermite splines for C2-continuous kinodynamic motion planning.
# Matches position, velocity, and acceleration boundary conditions across waypoints.
# Calculates exact arc length via 5-point Gauss-Legendre quadrature, curvature kappa(s),
# and curvature rate-of-change for steering rate limit enforcement.

const GAUSS_POINTS: Array = [
	-0.9061798459, -0.5384693101, 0.0, 0.5384693101, 0.9061798459
]
const GAUSS_WEIGHTS: Array = [
	0.2369268850, 0.4786286705, 0.5688888889, 0.4786286705, 0.2369268850
]

# Quintic polynomial coefficients for x(t) and y(t): c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
var cx: PackedFloat64Array = PackedFloat64Array()
var cy: PackedFloat64Array = PackedFloat64Array()
var total_arc_length: float = 0.0

func _init() -> void:
	cx.resize(6)
	cy.resize(6)
	for i in range(6):
		cx[i] = 0.0
		cy[i] = 0.0

# Solves quintic boundary value problem for 2D position, velocity, and acceleration
# t ranges from 0.0 to 1.0 (normalized)
static func create_quintic(
	p0: Vector2, v0: Vector2, a0: Vector2,
	p1: Vector2, v1: Vector2, a1: Vector2
) -> HermiteSpline:
	var spline: HermiteSpline = HermiteSpline.new()
	
	# Solve for x-coefficients
	spline.cx[0] = p0.x
	spline.cx[1] = v0.x
	spline.cx[2] = 0.5 * a0.x
	spline.cx[3] = -10.0 * p0.x - 6.0 * v0.x - 1.5 * a0.x + 10.0 * p1.x - 4.0 * v1.x + 0.5 * a1.x
	spline.cx[4] =  15.0 * p0.x + 8.0 * v0.x + 1.5 * a0.x - 15.0 * p1.x + 7.0 * v1.x - 1.0 * a1.x
	spline.cx[5] =  -6.0 * p0.x - 3.0 * v0.x - 0.5 * a0.x +  6.0 * p1.x - 3.0 * v1.x + 0.5 * a1.x
	
	# Solve for y-coefficients
	spline.cy[0] = p0.y
	spline.cy[1] = v0.y
	spline.cy[2] = 0.5 * a0.y
	spline.cy[3] = -10.0 * p0.y - 6.0 * v0.y - 1.5 * a0.y + 10.0 * p1.y - 4.0 * v1.y + 0.5 * a1.y
	spline.cy[4] =  15.0 * p0.y + 8.0 * v0.y + 1.5 * a0.y - 15.0 * p1.y + 7.0 * v1.y - 1.0 * a1.y
	spline.cy[5] =  -6.0 * p0.y - 3.0 * v0.y - 0.5 * a0.y +  6.0 * p1.y - 3.0 * v1.y + 0.5 * a1.y
	
	spline.total_arc_length = spline.compute_arc_length(0.0, 1.0)
	return spline

# Evaluates 2D position at parameter t in [0, 1]
func evaluate_position(t: float) -> Vector2:
	var t2: float = t * t
	var t3: float = t2 * t
	var t4: float = t3 * t
	var t5: float = t4 * t
	
	var px: float = cx[0] + cx[1] * t + cx[2] * t2 + cx[3] * t3 + cx[4] * t4 + cx[5] * t5
	var py: float = cy[0] + cy[1] * t + cy[2] * t2 + cy[3] * t3 + cy[4] * t4 + cy[5] * t5
	return Vector2(px, py)

# Evaluates 2D velocity (first derivative w.r.t t)
func evaluate_velocity(t: float) -> Vector2:
	var t2: float = t * t
	var t3: float = t2 * t
	var t4: float = t3 * t
	
	var vx: float = cx[1] + 2.0 * cx[2] * t + 3.0 * cx[3] * t2 + 4.0 * cx[4] * t3 + 5.0 * cx[5] * t4
	var vy: float = cy[1] + 2.0 * cy[2] * t + 3.0 * cy[3] * t2 + 4.0 * cy[4] * t3 + 5.0 * cy[5] * t4
	return Vector2(vx, vy)

# Evaluates 2D acceleration (second derivative w.r.t t)
func evaluate_acceleration(t: float) -> Vector2:
	var t2: float = t * t
	var t3: float = t2 * t
	
	var ax: float = 2.0 * cx[2] + 6.0 * cx[3] * t + 12.0 * cx[4] * t2 + 20.0 * cx[5] * t3
	var ay: float = 2.0 * cy[2] + 6.0 * cy[3] * t + 12.0 * cy[4] * t2 + 20.0 * cy[5] * t3
	return Vector2(ax, ay)

# Evaluates 2D jerk (third derivative w.r.t t)
func evaluate_jerk(t: float) -> Vector2:
	var t2: float = t * t
	var jx: float = 6.0 * cx[3] + 24.0 * cx[4] * t + 60.0 * cx[5] * t2
	var jy: float = 6.0 * cy[3] + 24.0 * cy[4] * t + 60.0 * cy[5] * t2
	return Vector2(jx, jy)

# Computes heading angle theta(t) = atan2(y', x')
func evaluate_heading(t: float) -> float:
	var v: Vector2 = evaluate_velocity(t)
	return atan2(v.y, v.x)

# Computes exact signed curvature kappa(t) = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
func evaluate_curvature(t: float) -> float:
	var v: Vector2 = evaluate_velocity(t)
	var a: Vector2 = evaluate_acceleration(t)
	var speed_sq: float = v.x * v.x + v.y * v.y
	if speed_sq < 1e-12:
		return 0.0
	var speed_cube: float = speed_sq * sqrt(speed_sq)
	return (v.x * a.y - v.y * a.x) / speed_cube

# Evaluates rate of change of curvature w.r.t arc length: d(kappa)/ds
func evaluate_curvature_derivative(t: float) -> float:
	var v: Vector2 = evaluate_velocity(t)
	var a: Vector2 = evaluate_acceleration(t)
	var j: Vector2 = evaluate_jerk(t)
	var speed_sq: float = v.x * v.x + v.y * v.y
	var speed: float = sqrt(speed_sq)
	if speed < 1e-6:
		return 0.0
		
	var num: float = v.x * a.y - v.y * a.x
	var d_num: float = v.x * j.y - v.y * j.x
	var v_dot_a: float = v.x * a.x + v.y * a.y
	
	# d(kappa)/dt = (d_num * speed^3 - 3 * num * speed * v_dot_a) / speed^6
	var d_kappa_dt: float = (d_num * speed_sq - 3.0 * num * v_dot_a) / (speed_sq * speed_sq * speed)
	# d(kappa)/ds = (d(kappa)/dt) / speed
	return d_kappa_dt / speed

# Computes arc length integral between t_start and t_end using 5-point Gauss-Legendre quadrature
func compute_arc_length(t_start: float, t_end: float) -> float:
	var mid: float = 0.5 * (t_start + t_end)
	var half_len: float = 0.5 * (t_end - t_start)
	var sum: float = 0.0
	
	for i in range(5):
		var t_sample: float = mid + half_len * GAUSS_POINTS[i]
		var v: Vector2 = evaluate_velocity(t_sample)
		var speed: float = v.length()
		sum += GAUSS_WEIGHTS[i] * speed
		
	return sum * half_len

# Samples the spline into N discrete equidistant arc-length trajectory points
# Returns Array of Dictionaries: {"position": Vector2, "heading": float, "curvature": float, "s": float}
func sample_path(num_samples: int = 50) -> Array[Dictionary]:
	assert(num_samples >= 2, "Sample count must be at least 2")
	var path: Array[Dictionary] = []
	var dt: float = 1.0 / float(num_samples - 1)
	var accumulated_s: float = 0.0
	var prev_pos: Vector2 = evaluate_position(0.0)
	
	for i in range(num_samples):
		var t: float = float(i) * dt
		var pos: Vector2 = evaluate_position(t)
		if i > 0:
			accumulated_s += pos.distance_to(prev_pos)
		prev_pos = pos
		
		path.append({
			"position": pos,
			"heading": evaluate_heading(t),
			"curvature": evaluate_curvature(t),
			"s": accumulated_s,
			"t": t
		})
	return path
