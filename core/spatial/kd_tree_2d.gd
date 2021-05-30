# core/spatial/kd_tree_2d.gd
class_name KDTree2D
extends RefCounted

# Balanced 2D KD-Tree for fast nearest neighbor, K-nearest neighbor (KNN),
# and circular range queries over 2D spatial points.

class KDNode:
	var point: Vector2
	var data_id: int
	var axis: int # 0 for x-split, 1 for y-split
	var left: KDNode = null
	var right: KDNode = null
	
	func _init(p_pt: Vector2, p_id: int, p_axis: int) -> void:
		point = p_pt
		data_id = p_id
		axis = p_axis

var root: KDNode = null
var point_count: int = 0

# Builds a balanced 2D KD-tree from an array of Vector2 points and optional ID tags.
static func build_from_points(points: Array[Vector2], ids: PackedInt32Array = PackedInt32Array()) -> KDTree2D:
	var tree: KDTree2D = KDTree2D.new()
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

func _build_recursive(entries: Array[Dictionary], depth: int) -> KDNode:
	var n: int = entries.size()
	if n == 0:
		return null
		
	var axis: int = depth % 2
	
	# Sort entries along the current axis
	if axis == 0:
		entries.sort_custom(func(a, b): return a["pt"].x < b["pt"].x)
	else:
		entries.sort_custom(func(a, b): return a["pt"].y < b["pt"].y)
		
	var median_idx: int = n / 2
	var median_entry: Dictionary = entries[median_idx]
	
	var node: KDNode = KDNode.new(median_entry["pt"], median_entry["id"], axis)
	
	var left_entries: Array[Dictionary] = entries.slice(0, median_idx)
	var right_entries: Array[Dictionary] = entries.slice(median_idx + 1)
	
	node.left = _build_recursive(left_entries, depth + 1)
	node.right = _build_recursive(right_entries, depth + 1)
	
	return node

# Finds the single nearest neighbor to target point.
# Returns Dictionary: {"point": Vector2, "id": int, "distance": float}
func find_nearest(target: Vector2) -> Dictionary:
	if not root:
		return {"point": Vector2.ZERO, "id": -1, "distance": INF}
		
	var best: Dictionary = {"node": root, "dist_sq": INF}
	_nearest_search_recursive(root, target, best)
	
	return {
		"point": best["node"].point,
		"id": best["node"].data_id,
		"distance": sqrt(best["dist_sq"])
	}

func _nearest_search_recursive(node: KDNode, target: Vector2, best: Dictionary) -> void:
	if not node:
		return
		
	var cur_dist_sq: float = node.point.distance_squared_to(target)
	if cur_dist_sq < best["dist_sq"]:
		best["dist_sq"] = cur_dist_sq
		best["node"] = node
		
	var axis: int = node.axis
	var delta: float = (target.x - node.point.x) if axis == 0 else (target.y - node.point.y)
	
	var near_child: KDNode = node.left if delta < 0.0 else node.right
	var far_child: KDNode = node.right if delta < 0.0 else node.left
	
	# Search closer subtree first
	_nearest_search_recursive(near_child, target, best)
	
	# Prune farther subtree if hyper-rectangle boundary is further than current best distance
	if (delta * delta) < best["dist_sq"]:
		_nearest_search_recursive(far_child, target, best)

# Finds the k nearest neighbors to target point.
# Returns Array of Dictionaries: [{"point": Vector2, "id": int, "distance": float}]
func find_k_nearest(target: Vector2, k: int) -> Array[Dictionary]:
	if not root or k <= 0:
		return []
		
	var results: Array[Dictionary] = []
	_knn_recursive(root, target, k, results)
	
	# Sort ascending by distance
	results.sort_custom(func(a, b): return a["distance"] < b["distance"])
	return results

func _knn_recursive(node: KDNode, target: Vector2, k: int, results: Array[Dictionary]) -> void:
	if not node:
		return
		
	var dist_sq: float = node.point.distance_squared_to(target)
	var dist: float = sqrt(dist_sq)
	
	# Insert into results maintaining capacity k
	if results.size() < k:
		results.append({"point": node.point, "id": node.data_id, "dist_sq": dist_sq, "distance": dist})
		results.sort_custom(func(a, b): return a["dist_sq"] < b["dist_sq"])
	elif dist_sq < results[results.size() - 1]["dist_sq"]:
		results[results.size() - 1] = {"point": node.point, "id": node.data_id, "dist_sq": dist_sq, "distance": dist}
		results.sort_custom(func(a, b): return a["dist_sq"] < b["dist_sq"])
		
	var max_dist_sq: float = results[results.size() - 1]["dist_sq"] if results.size() == k else INF
	
	var axis: int = node.axis
	var delta: float = (target.x - node.point.x) if axis == 0 else (target.y - node.point.y)
	
	var near_child: KDNode = node.left if delta < 0.0 else node.right
	var far_child: KDNode = node.right if delta < 0.0 else node.left
	
	_knn_recursive(near_child, target, k, results)
	
	if (delta * delta) < max_dist_sq:
		_knn_recursive(far_child, target, k, results)

# Finds all points within radius from target point.
# Returns Array of Dictionaries: [{"point": Vector2, "id": int, "distance": float}]
func find_in_radius(target: Vector2, radius: float) -> Array[Dictionary]:
	if not root or radius <= 0.0:
		return []
		
	var results: Array[Dictionary] = []
	var radius_sq: float = radius * radius
	_radius_recursive(root, target, radius, radius_sq, results)
	
	results.sort_custom(func(a, b): return a["distance"] < b["distance"])
	return results

func _radius_recursive(node: KDNode, target: Vector2, radius: float, radius_sq: float, results: Array[Dictionary]) -> void:
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
	var delta: float = (target.x - node.point.x) if axis == 0 else (target.y - node.point.y)
	
	if delta < 0.0:
		_radius_recursive(node.left, target, radius, radius_sq, results)
		if abs(delta) <= radius:
			_radius_recursive(node.right, target, radius, radius_sq, results)
	else:
		_radius_recursive(node.right, target, radius, radius_sq, results)
		if abs(delta) <= radius:
			_radius_recursive(node.left, target, radius, radius_sq, results)
