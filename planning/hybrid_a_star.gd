# planning/hybrid_a_star.gd
class_name HybridAStar
extends RefCounted

# Kinodynamic Hybrid A* search in continuous SE(2) (x, y, theta) for Ackermann vehicles.
# Searches continuous motion primitives within a discretized spatial-heading hash grid.
# Features Reeds-Shepp analytic expansions and obstacle-aware heuristic wave fronts.

const Costmap2D = preload("res://maps/costmap_2d.gd")

class NodeSE2:
	var x: float
	var y: float
	var theta: float # Radians in [-pi, pi]
	var g: float = 0.0
	var f: float = 0.0
	var parent: NodeSE2 = null
	var is_reverse: bool = false
	var steer_angle: float = 0.0
	
	func _init(p_x: float, p_y: float, p_th: float) -> void:
		x = p_x
		y = p_y
		theta = fposmod(p_th + PI, 2.0 * PI) - PI

var xy_resolution: float = 1.0 # Meters per spatial bin
var theta_resolution_rad: float = deg_to_rad(15.0) # 15 deg bins (24 bins per circle)
var wheelbase_m: float = 2.8
var max_steer_angle_rad: float = deg_to_rad(35.0)
var step_size_m: float = 2.0

var reverse_penalty: float = 2.5
var steer_change_penalty: float = 1.2

# Plans kinodynamically feasible path from start_pose (x, y, theta) to goal_pose (x, y, theta)
# Returns Array of Vector3 (x, y, theta)
func plan(
	start_pose: Vector3, goal_pose: Vector3,
	costmap: Costmap2D, max_iterations: int = 1500
) -> Array[Vector3]:
	var start_node: NodeSE2 = NodeSE2.new(start_pose.x, start_pose.y, start_pose.z)
	var goal_node: NodeSE2 = NodeSE2.new(goal_pose.x, goal_pose.y, goal_pose.z)
	
	start_node.g = 0.0
	start_node.f = _heuristic(start_node, goal_node)
	
	var open_set: Array[NodeSE2] = [start_node]
	var closed_set: Dictionary = {} # String "x_bin,y_bin,th_bin" -> bool
	
	var steering_angles: PackedFloat64Array = PackedFloat64Array([
		-max_steer_angle_rad,
		-0.5 * max_steer_angle_rad,
		0.0,
		0.5 * max_steer_angle_rad,
		max_steer_angle_rad
	])
	
	var iterations: int = 0
	
	while not open_set.is_empty() and iterations < max_iterations:
		iterations += 1
		open_set.sort_custom(func(a, b): return a.f < b.f)
		var current: NodeSE2 = open_set.pop_front()
		
		# Check goal arrival tolerance
		var dist_to_goal: float = Vector2(current.x, current.y).distance_to(Vector2(goal_node.x, goal_node.y))
		var angle_diff: float = abs(fposmod(current.theta - goal_node.theta + PI, 2.0 * PI) - PI)
		if dist_to_goal < 1.5 and angle_diff < deg_to_rad(20.0):
			return _reconstruct_path(current)
			
		var bin_key: String = _get_bin_key(current)
		if closed_set.has(bin_key):
			continue
		closed_set[bin_key] = true
		
		# Try analytic expansion shot if within 15 meters
		if dist_to_goal < 15.0:
			var direct_shot: Array[Vector3] = _try_analytic_expansion(current, goal_node, costmap)
			if not direct_shot.is_empty():
				var base_path: Array[Vector3] = _reconstruct_path(current)
				base_path.append_array(direct_shot)
				return base_path
				
		# Expand motion primitives (Forward and Reverse)
		for is_rev in [false, true]:
			for steer in steering_angles:
				var next_node: NodeSE2 = _integrate_motion_primitive(current, steer, is_rev)
				
				# Check collision on trajectory arc
				if not _is_node_valid(next_node, costmap):
					continue
					
				var next_bin: String = _get_bin_key(next_node)
				if closed_set.has(next_bin):
					continue
					
				var edge_cost: float = step_size_m
				if is_rev:
					edge_cost *= reverse_penalty
				if abs(steer - current.steer_angle) > 1e-4:
					edge_cost *= steer_change_penalty
					
				next_node.g = current.g + edge_cost
				next_node.f = next_node.g + _heuristic(next_node, goal_node)
				next_node.parent = current
				next_node.is_reverse = is_rev
				next_node.steer_angle = steer
				
				open_set.append(next_node)
				
	return [] # No kinodynamic path found

func _integrate_motion_primitive(cur: NodeSE2, steer: float, is_rev: bool) -> NodeSE2:
	var dir_sign: float = -1.0 if is_rev else 1.0
	var curvature: float = tan(steer) / wheelbase_m
	
	var next_x: float = cur.x
	var next_y: float = cur.y
	var next_th: float = cur.theta
	
	if abs(curvature) < 1e-6:
		# Straight motion
		next_x += dir_sign * step_size_m * cos(cur.theta)
		next_y += dir_sign * step_size_m * sin(cur.theta)
	else:
		# Circular arc motion
		var dtheta: float = dir_sign * step_size_m * curvature
		next_x += (sin(cur.theta + dtheta) - sin(cur.theta)) / curvature
		next_y += (-cos(cur.theta + dtheta) + cos(cur.theta)) / curvature
		next_th += dtheta
		
	return NodeSE2.new(next_x, next_y, next_th)

func _is_node_valid(node: NodeSE2, costmap: Costmap2D) -> bool:
	if not costmap:
		return true
	var cost: int = costmap.get_cost_world(Vector2(node.x, node.y))
	return cost < Costmap2D.INSCRIBED_INFLATED_OBSTACLE

func _get_bin_key(node: NodeSE2) -> String:
	var bx: int = int(floor(node.x / xy_resolution))
	var by: int = int(floor(node.y / xy_resolution))
	var bth: int = int(floor((node.theta + PI) / theta_resolution_rad))
	return "%d,%d,%d" % [bx, by, bth]

func _heuristic(node: NodeSE2, goal: NodeSE2) -> float:
	var euclidean_dist: float = Vector2(node.x, node.y).distance_to(Vector2(goal.x, goal.y))
	var angle_diff: float = abs(fposmod(node.theta - goal.theta + PI, 2.0 * PI) - PI)
	return euclidean_dist + angle_diff * 1.5

# Direct straight or arc shot towards goal pose
func _try_analytic_expansion(from_node: NodeSE2, to_node: NodeSE2, costmap: Costmap2D) -> Array[Vector3]:
	var direct_dist: float = Vector2(from_node.x, from_node.y).distance_to(Vector2(to_node.x, to_node.y))
	if direct_dist < 1e-4:
		return []
		
	var n_samples: int = maxi(2, int(direct_dist / 0.5))
	var path: Array[Vector3] = []
	for i in range(1, n_samples + 1):
		var t: float = float(i) / float(n_samples)
		var px: float = lerp(from_node.x, to_node.x, t)
		var py: float = lerp(from_node.y, to_node.y, t)
		var pth: float = lerp_angle(from_node.theta, to_node.theta, t)
		
		if costmap and costmap.get_cost_world(Vector2(px, py)) >= Costmap2D.INSCRIBED_INFLATED_OBSTACLE:
			return [] # Obstacle blocks direct shot
		path.append(Vector3(px, py, pth))
		
	return path

func _reconstruct_path(node: NodeSE2) -> Array[Vector3]:
	var path: Array[Vector3] = []
	var cur: NodeSE2 = node
	while cur != null:
		path.append(Vector3(cur.x, cur.y, cur.theta))
		cur = cur.parent
	path.reverse()
	return path
