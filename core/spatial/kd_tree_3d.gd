# core/spatial/kd_tree_3d.gd
class_name KDTree3D
extends RefCounted

# Balanced 3D KD-Tree for spatial indexing of 3D LiDAR point clouds,
# collision obstacles, and volumetric spatial range queries.

class KDNode3D:
	var point: Vector3
	var data_id: int
	var axis: int # 0 for x, 1 for y, 2 for z
	var left: KDNode3D = null
	var right: KDNode3D = null
	
	func _init(p_pt: Vector3, p_id: int, p_axis: int) -> void:
		point = p_pt
		data_id = p_id
		axis = p_axis

var root: KDNode3D = null
var point_count: int = 0

static func build_from_points(points: Array[Vector3], ids: PackedInt32Array = PackedInt32Array()) -> KDTree3D:
	var tree: KDTree3D = KDTree3D.new()
	var n: int = points.size()
	if n == 0:
		return tree
		
	var entries: Array[Dictionary] = []
	entries.resize(n)
	for i in range(n):
		var id_val: int = ids[i] if (ids.size() == n) else i
		entries[i] = {"pt": points[i], "id": id_val}
		
	tree.root = tree._build_recursive(entries, 0)
	tree.point_count = n
	return tree

func _build_recursive(entries: Array[Dictionary], depth: int) -> KDNode3D:
	var n: int = entries.size()
	if n == 0:
		return null
		
	var axis: int = depth % 3
	
	if axis == 0:
		entries.sort_custom(func(a, b): return a["pt"].x < b["pt"].x)
	elif axis == 1:
		entries.sort_custom(func(a, b): return a["pt"].y < b["pt"].y)
	else:
		entries.sort_custom(func(a, b): return a["pt"].z < b["pt"].z)
		
	var median_idx: int = n / 2
	var median_entry: Dictionary = entries[median_idx]
	
	var node: KDNode3D = KDNode3D.new(median_entry["pt"], median_entry["id"], axis)
	
	var left_entries: Array[Dictionary] = entries.slice(0, median_idx)
	var right_entries: Array[Dictionary] = entries.slice(median_idx + 1)
	
	node.left = _build_recursive(left_entries, depth + 1)
	node.right = _build_recursive(right_entries, depth + 1)
	
	return node

func find_nearest(target: Vector3) -> Dictionary:
	if not root:
		return {"point": Vector3.ZERO, "id": -1, "distance": INF}
		
	var best: Dictionary = {"node": root, "dist_sq": INF}
	_nearest_search_recursive(root, target, best)
	
	return {
		"point": best["node"].point,
		"id": best["node"].data_id,
		"distance": sqrt(best["dist_sq"])
	}

func _nearest_search_recursive(node: KDNode3D, target: Vector3, best: Dictionary) -> void:
	if not node:
		return
		
	var cur_dist_sq: float = node.point.distance_squared_to(target)
	if cur_dist_sq < best["dist_sq"]:
		best["dist_sq"] = cur_dist_sq
		best["node"] = node
		
	var axis: int = node.axis
	var delta: float = 0.0
	if axis == 0:
		delta = target.x - node.point.x
	elif axis == 1:
		delta = target.y - node.point.y
	else:
		delta = target.z - node.point.z
		
	var near_child: KDNode3D = node.left if delta < 0.0 else node.right
	var far_child: KDNode3D = node.right if delta < 0.0 else node.left
	
	_nearest_search_recursive(near_child, target, best)
	
	if (delta * delta) < best["dist_sq"]:
		_nearest_search_recursive(far_child, target, best)

func find_in_radius(target: Vector3, radius: float) -> Array[Dictionary]:
	if not root or radius <= 0.0:
		return []
		
	var results: Array[Dictionary] = []
	var radius_sq: float = radius * radius
	_radius_recursive(root, target, radius, radius_sq, results)
	
	results.sort_custom(func(a, b): return a["distance"] < b["distance"])
	return results

func _radius_recursive(node: KDNode3D, target: Vector3, radius: float, radius_sq: float, results: Array[Dictionary]) -> void:
	if not node:
		return
		
	var cur_dist_sq: float = node.point.distance_squared_to(target)
	if cur_dist_sq <= radius_sq:
		results.append({
			"point": node.point,
			"id": node.data_id,
			"distance": sqrt(cur_dist_sq)
		})
		
	var axis: int = node.axis
	var delta: float = 0.0
	if axis == 0:
		delta = target.x - node.point.x
	elif axis == 1:
		delta = target.y - node.point.y
	else:
		delta = target.z - node.point.z
		
	if delta < 0.0:
		_radius_recursive(node.left, target, radius, radius_sq, results)
		if abs(delta) <= radius:
			_radius_recursive(node.right, target, radius, radius_sq, results)
	else:
		_radius_recursive(node.right, target, radius, radius_sq, results)
		if abs(delta) <= radius:
			_radius_recursive(node.left, target, radius, radius_sq, results)
