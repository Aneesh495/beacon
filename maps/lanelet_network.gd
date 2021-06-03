# maps/lanelet_network.gd
class_name LaneletNetwork
extends RefCounted

# Topological and metric road network graph for autonomous vehicle routing.
# Indexes lanelet centerlines, computes optimal topological routes using A*,
# and discovers spatial conflict points at intersections.

const LaneletSegment = preload("res://maps/lanelet_segment.gd")
const KDTree2D = preload("res://core/spatial/kd_tree_2d.gd")
const GeometryUtils = preload("res://core/spatial/geometry_utils.gd")

var segments: Dictionary = {} # int (id) -> LaneletSegment
var spatial_index: KDTree2D = null
var is_indexed: bool = false

func add_segment(segment: LaneletSegment) -> void:
	segments[segment.id] = segment
	is_indexed = false

func get_segment(id: int) -> LaneletSegment:
	return segments.get(id, null)

# Rebuilds spatial KD-tree over all lanelet centerline points for fast spatial lookups
func build_spatial_index() -> void:
	var pts: Array[Vector2] = []
	var ids: PackedInt32Array = PackedInt32Array()
	
	for id in segments.keys():
		var seg: LaneletSegment = segments[id]
		for pt in seg.centerline:
			pts.append(pt)
			ids.append(id)
			
	if pts.size() > 0:
		spatial_index = KDTree2D.build_from_points(pts, ids)
	is_indexed = true

# Finds the closest lanelet segment to a given 2D Cartesian coordinate
func find_closest_lanelet(point: Vector2) -> LaneletSegment:
	if not is_indexed or not spatial_index:
		build_spatial_index()
		
	if not spatial_index or spatial_index.point_count == 0:
		return null
		
	var nearest: Dictionary = spatial_index.find_nearest(point)
	var seg_id: int = nearest["id"]
	return segments.get(seg_id, null)

# A* shortest path search over the topological lanelet graph
# Returns Array of LaneletSegment IDs from start_id to goal_id
func plan_topological_route(start_id: int, goal_id: int) -> PackedInt32Array:
	if not segments.has(start_id) or not segments.has(goal_id):
		return PackedInt32Array()
		
	if start_id == goal_id:
		return PackedInt32Array([start_id])
		
	var goal_seg: LaneletSegment = segments[goal_id]
	var goal_pos: Vector2 = goal_seg.centerline[goal_seg.centerline.size() - 1]
	
	# Priority queue using Array of Dictionaries
	var open_set: Array[Dictionary] = []
	var g_score: Dictionary = {start_id: 0.0}
	var f_score: Dictionary = {start_id: _heuristic(segments[start_id], goal_pos)}
	var came_from: Dictionary = {}
	
	open_set.append({"id": start_id, "f": f_score[start_id]})
	
	while not open_set.is_empty():
		open_set.sort_custom(func(a, b): return a["f"] < b["f"])
		var current_item: Dictionary = open_set.pop_front()
		var current_id: int = current_item["id"]
		
		if current_id == goal_id:
			return _reconstruct_path(came_from, current_id)
			
		var cur_seg: LaneletSegment = segments[current_id]
		var cur_g: float = g_score[current_id]
		
		# Explore successors
		for next_id in cur_seg.successor_ids:
			if not segments.has(next_id):
				continue
			var next_seg: LaneletSegment = segments[next_id]
			var tentative_g: float = cur_g + cur_seg.total_length
			
			if not g_score.has(next_id) or tentative_g < g_score[next_id]:
				came_from[next_id] = current_id
				g_score[next_id] = tentative_g
				var f_val: float = tentative_g + _heuristic(next_seg, goal_pos)
				f_score[next_id] = f_val
				open_set.append({"id": next_id, "f": f_val})
				
		# Explore permissible lane changes
		if cur_seg.can_change_left and cur_seg.left_neighbor_id != -1:
			var left_id: int = cur_seg.left_neighbor_id
			if segments.has(left_id):
				var lane_change_penalty: float = 5.0 # Cost penalty for changing lanes
				var tentative_g: float = cur_g + lane_change_penalty
				if not g_score.has(left_id) or tentative_g < g_score[left_id]:
					came_from[left_id] = current_id
					g_score[left_id] = tentative_g
					var f_val: float = tentative_g + _heuristic(segments[left_id], goal_pos)
					f_score[left_id] = f_val
					open_set.append({"id": left_id, "f": f_val})
					
		if cur_seg.can_change_right and cur_seg.right_neighbor_id != -1:
			var right_id: int = cur_seg.right_neighbor_id
			if segments.has(right_id):
				var lane_change_penalty: float = 5.0
				var tentative_g: float = cur_g + lane_change_penalty
				if not g_score.has(right_id) or tentative_g < g_score[right_id]:
					came_from[right_id] = current_id
					g_score[right_id] = tentative_g
					var f_val: float = tentative_g + _heuristic(segments[right_id], goal_pos)
					f_score[right_id] = f_val
					open_set.append({"id": right_id, "f": f_val})
					
	return PackedInt32Array() # No route found

func _heuristic(seg: LaneletSegment, goal_pos: Vector2) -> float:
	if seg.centerline.is_empty():
		return 0.0
	var seg_end: Vector2 = seg.centerline[seg.centerline.size() - 1]
	return seg_end.distance_to(goal_pos)

func _reconstruct_path(came_from: Dictionary, current_id: int) -> PackedInt32Array:
	var path: Array[int] = [current_id]
	while came_from.has(current_id):
		current_id = came_from[current_id]
		path.append(current_id)
	path.reverse()
	return PackedInt32Array(path)

# Detects potential spatial conflict crossing points between two intersecting lanelets
# Returns Array of Vector2 intersection points
static func find_conflict_points(seg_a: LaneletSegment, seg_b: LaneletSegment) -> Array[Vector2]:
	var conflicts: Array[Vector2] = []
	if seg_a.centerline.size() < 2 or seg_b.centerline.size() < 2:
		return conflicts
		
	for i in range(seg_a.centerline.size() - 1):
		var p1: Vector2 = seg_a.centerline[i]
		var p2: Vector2 = seg_a.centerline[i + 1]
		for j in range(seg_b.centerline.size() - 1):
			var q1: Vector2 = seg_b.centerline[j]
			var q2: Vector2 = seg_b.centerline[j + 1]
			
			var pt: Dictionary = _segment_intersection_2d(p1, p2, q1, q2)
			if pt["intersects"]:
				var is_duplicate: bool = false
				for existing_pt in conflicts:
					if existing_pt.distance_to(pt["point"]) < 1e-3:
						is_duplicate = true
						break
				if not is_duplicate:
					conflicts.append(pt["point"])
				
	return conflicts

static func _segment_intersection_2d(p1: Vector2, p2: Vector2, q1: Vector2, q2: Vector2) -> Dictionary:
	var r: Vector2 = p2 - p1
	var s: Vector2 = q2 - q1
	var r_cross_s: float = r.x * s.y - r.y * s.x
	var q_minus_p: Vector2 = q1 - p1
	var qp_cross_r: float = q_minus_p.x * r.y - q_minus_p.y * r.x
	
	if abs(r_cross_s) < 1e-9:
		return {"intersects": false, "point": Vector2.ZERO} # Parallel or collinear
		
	var t: float = (q_minus_p.x * s.y - q_minus_p.y * s.x) / r_cross_s
	var u: float = qp_cross_r / r_cross_s
	
	if t >= 0.0 and t <= 1.0 and u >= 0.0 and u <= 1.0:
		return {"intersects": true, "point": p1 + r * t}
		
	return {"intersects": false, "point": Vector2.ZERO}
