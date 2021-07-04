# planning/orca_collision_avoidance.gd
class_name OrcaCollisionAvoidance
extends RefCounted

# Optimal Reciprocal Collision Avoidance (ORCA / RVO2) in 2D continuous velocity space.
# Constructs velocity obstacle half-planes and computes optimal collision-free velocities
# via 2D Linear Programming (LP), guaranteeing multi-agent mutual avoidance.

class OrcaLine:
	var point: Vector2 # Point on the line
	var direction: Vector2 # Unit direction vector
	var normal: Vector2 # Normal pointing into feasible half-plane

# Computes preferred collision-free velocity for agent A avoiding agent B
# time_horizon: lookahead time tau (e.g. 2.0s)
static func compute_orca_halfplane(
	pos_a: Vector2, vel_a: Vector2, radius_a: float,
	pos_b: Vector2, vel_b: Vector2, radius_b: float,
	time_horizon: float = 2.0
) -> OrcaLine:
	var line: OrcaLine = OrcaLine.new()
	var rel_pos: Vector2 = pos_b - pos_a
	var rel_vel: Vector2 = vel_a - vel_b
	var combined_radius: float = radius_a + radius_b
	var dist_sq: float = rel_pos.length_squared()
	var dist: float = sqrt(dist_sq)
	
	var inv_tau: float = 1.0 / time_horizon
	var w: Vector2 = rel_vel - rel_pos * inv_tau # Vector from cutoff circle center to rel_vel
	var w_len_sq: float = w.length_squared()
	var dot_product: float = w.dot(rel_pos)
	
	var u: Vector2 = Vector2.ZERO
	var normal: Vector2 = Vector2.ZERO
	
	if dot_product < 0.0 and (dot_product * dot_product) > (dist_sq * combined_radius * combined_radius * inv_tau * inv_tau):
		# Project on cutoff circle
		var w_len: float = sqrt(w_len_sq)
		var unit_w: Vector2 = w / w_len if w_len > 1e-9 else Vector2.ZERO
		normal = unit_w
		u = unit_w * (combined_radius * inv_tau - w_len)
	else:
		# Project on legs of cone
		var leg_dist: float = sqrt(maxf(0.0, dist_sq - combined_radius * combined_radius))
		if (rel_pos.x * rel_vel.y - rel_pos.y * rel_vel.x) > 0.0:
			# Left leg
			var leg_dir: Vector2 = Vector2(
				rel_pos.x * leg_dist - rel_pos.y * combined_radius,
				rel_pos.x * combined_radius + rel_pos.y * leg_dist
			) / dist_sq
			normal = Vector2(-leg_dir.y, leg_dir.x)
			u = leg_dir * rel_vel.dot(leg_dir) - rel_vel
		else:
			# Right leg
			var leg_dir: Vector2 = Vector2(
				rel_pos.x * leg_dist + rel_pos.y * combined_radius,
				-rel_pos.x * combined_radius + rel_pos.y * leg_dist
			) / dist_sq
			normal = Vector2(leg_dir.y, -leg_dir.x)
			u = leg_dir * rel_vel.dot(leg_dir) - rel_vel
			
	# ORCA half-plane: point = vel_a + 0.5 * u, direction = perpendicular to normal
	line.point = vel_a + u * 0.5
	line.normal = u.normalized() if u.length_squared() > 1e-9 else normal
	line.direction = Vector2(-line.normal.y, line.normal.x)
	return line

# 2D Linear Programming solver: Finds velocity closest to preferred velocity satisfying all half-planes
# and maximum speed constraint: ||v|| <= max_speed
static func solve_2d_lp(
	lines: Array[OrcaLine],
	max_speed: float,
	pref_velocity: Vector2
) -> Vector2:
	var result: Vector2 = pref_velocity
	if result.length() > max_speed:
		result = result.normalized() * max_speed
		
	for i in range(lines.size()):
		var line: OrcaLine = lines[i]
		# Check if current result violates line i: (result - point) . normal < 0
		if (result - line.point).dot(line.normal) < 0.0:
			# Project result onto line i
			var proj_opt: Dictionary = _linear_program_1d(lines, i, max_speed, pref_velocity)
			if not proj_opt["success"]:
				# Return boundary clamped result
				return line.point
			result = proj_opt["result"]
			
	return result

static func _linear_program_1d(lines: Array[OrcaLine], line_idx: int, radius: float, opt_vel: Vector2) -> Dictionary:
	var cur_line: OrcaLine = lines[line_idx]
	var dot_prod: float = cur_line.point.dot(cur_line.direction)
	var discriminant: float = dot_prod * dot_prod - cur_line.point.length_squared() + radius * radius
	
	if discriminant < 0.0:
		return {"success": false, "result": Vector2.ZERO} # Max speed circle does not intersect line
		
	var sqrt_disc: float = sqrt(discriminant)
	var t_left: float = -dot_prod - sqrt_disc
	var t_right: float = -dot_prod + sqrt_disc
	
	for i in range(line_idx):
		var prev_line: OrcaLine = lines[i]
		var denominator: float = cur_line.direction.x * prev_line.direction.y - cur_line.direction.y * prev_line.direction.x
		var numerator: float = (prev_line.point.x - cur_line.point.x) * prev_line.direction.y - (prev_line.point.y - cur_line.point.y) * prev_line.direction.x
		
		if abs(denominator) < 1e-9:
			if numerator < 0.0:
				return {"success": false, "result": Vector2.ZERO}
			continue
			
		var t: float = numerator / denominator
		if denominator > 0.0:
			t_right = minf(t_right, t)
		else:
			t_left = maxf(t_left, t)
			
		if t_left > t_right:
			return {"success": false, "result": Vector2.ZERO}
			
	var t_opt: float = cur_line.direction.dot(opt_vel - cur_line.point)
	t_opt = clampf(t_opt, t_left, t_right)
	return {"success": true, "result": cur_line.point + cur_line.direction * t_opt}
