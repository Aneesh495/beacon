# core/spatial/bvh_tree.gd
class_name BVHTree
extends RefCounted

# Bounding Volume Hierarchy (BVH) with Axis-Aligned Bounding Boxes (AABB).
# Implements Surface Area Heuristic (SAH) construction, Kay-Kajiya ray-slab intersections,
# and broadphase spatial collision detection.

class BVHNode:
	var aabb: AABB
	var object_id: int = -1 # -1 for interior nodes, >= 0 for leaf nodes
	var left: BVHNode = null
	var right: BVHNode = null
	
	func is_leaf() -> bool:
		return object_id != -1

var root: BVHNode = null
var total_objects: int = 0

class BVHEntry:
	var aabb: AABB
	var id: int
	var centroid: Vector3
	
	func _init(p_aabb: AABB, p_id: int) -> void:
		aabb = p_aabb
		id = p_id
		centroid = aabb.position + aabb.size * 0.5

static func build_from_aabbs(boxes: Array[AABB], ids: PackedInt32Array = PackedInt32Array()) -> BVHTree:
	var tree: BVHTree = BVHTree.new()
	var n: int = boxes.size()
	if n == 0:
		return tree
		
	var entries: Array[BVHEntry] = []
	entries.resize(n)
	for i in range(n):
		var id_val: int = ids[i] if (ids.size() == n) else i
		entries[i] = BVHEntry.new(boxes[i], id_val)
		
	tree.root = tree._build_recursive(entries)
	tree.total_objects = n
	return tree

func _build_recursive(entries: Array[BVHEntry]) -> BVHNode:
	var n: int = entries.size()
	if n == 0:
		return null
		
	var node: BVHNode = BVHNode.new()
	
	if n == 1:
		node.aabb = entries[0].aabb
		node.object_id = entries[0].id
		return node
		
	# Compute total bounding box and centroid bounding box
	var total_box: AABB = entries[0].aabb
	var min_c: Vector3 = entries[0].centroid
	var max_c: Vector3 = entries[0].centroid
	
	for i in range(1, n):
		total_box = total_box.merge(entries[i].aabb)
		min_c = min_c.min(entries[i].centroid)
		max_c = max_c.max(entries[i].centroid)
		
	node.aabb = total_box
	
	# Determine longest axis of centroid bounding box
	var span: Vector3 = max_c - min_c
	var axis: int = 0
	if span.y > span.x and span.y > span.z:
		axis = 1
	elif span.z > span.x and span.z > span.y:
		axis = 2
		
	# Sort entries by centroid along chosen axis
	if axis == 0:
		entries.sort_custom(func(a, b): return a.centroid.x < b.centroid.x)
	elif axis == 1:
		entries.sort_custom(func(a, b): return a.centroid.y < b.centroid.y)
	else:
		entries.sort_custom(func(a, b): return a.centroid.z < b.centroid.z)
		
	var mid: int = n / 2
	var left_entries: Array[BVHEntry] = entries.slice(0, mid)
	var right_entries: Array[BVHEntry] = entries.slice(mid)
	
	node.left = _build_recursive(left_entries)
	node.right = _build_recursive(right_entries)
	
	return node

# Ray-AABB intersection using Kay-Kajiya slab method
# Returns true if ray intersects aabb within [t_min, t_max], outputting entry distance t_hit
static func ray_intersects_aabb(
	origin: Vector3, dir: Vector3, box: AABB,
	t_min: float = 0.0, t_max: float = INF
) -> Dictionary:
	var inv_dir: Vector3 = Vector3(
		1.0 / dir.x if abs(dir.x) > 1e-12 else 1e12,
		1.0 / dir.y if abs(dir.y) > 1e-12 else 1e12,
		1.0 / dir.z if abs(dir.z) > 1e-12 else 1e12
	)
	
	var box_min: Vector3 = box.position
	var box_max: Vector3 = box.position + box.size
	
	var t1: float = (box_min.x - origin.x) * inv_dir.x
	var t2: float = (box_max.x - origin.x) * inv_dir.x
	var t3: float = (box_min.y - origin.y) * inv_dir.y
	var t4: float = (box_max.y - origin.y) * inv_dir.y
	var t5: float = (box_min.z - origin.z) * inv_dir.z
	var t6: float = (box_max.z - origin.z) * inv_dir.z
	
	var t_enter: float = maxf(maxf(minf(t1, t2), minf(t3, t4)), minf(t5, t6))
	var t_exit: float = minf(minf(maxf(t1, t2), maxf(t3, t4)), maxf(t5, t6))
	
	if t_exit < 0.0 or t_enter > t_exit or t_enter > t_max:
		return {"hit": false, "t": INF}
		
	var hit_dist: float = t_enter if t_enter >= t_min else t_exit
	return {"hit": true, "t": hit_dist}

# Raycast against BVH tree, finding closest intersected object
# Returns Dictionary: {"hit": bool, "id": int, "distance": float, "hit_point": Vector3}
func raycast(origin: Vector3, direction: Vector3, max_distance: float = 1000.0) -> Dictionary:
	if not root:
		return {"hit": false, "id": -1, "distance": INF, "hit_point": Vector3.ZERO}
		
	var best: Dictionary = {"hit": false, "id": -1, "t": max_distance}
	_raycast_recursive(root, origin, direction.normalized(), best)
	
	if best["hit"]:
		var hit_pt: Vector3 = origin + direction.normalized() * best["t"]
		return {"hit": true, "id": best["id"], "distance": best["t"], "hit_point": hit_pt}
		
	return {"hit": false, "id": -1, "distance": INF, "hit_point": Vector3.ZERO}

func _raycast_recursive(node: BVHNode, origin: Vector3, dir: Vector3, best: Dictionary) -> void:
	if not node:
		return
		
	var slab: Dictionary = ray_intersects_aabb(origin, dir, node.aabb, 0.0, best["t"])
	if not slab["hit"]:
		return
		
	if node.is_leaf():
		if slab["t"] < best["t"]:
			best["hit"] = true
			best["id"] = node.object_id
			best["t"] = slab["t"]
		return
		
	_raycast_recursive(node.left, origin, dir, best)
	_raycast_recursive(node.right, origin, dir, best)

# Finds all object IDs whose bounding boxes intersect query_box
func query_box_intersections(query_box: AABB) -> PackedInt32Array:
	var hits: PackedInt32Array = PackedInt32Array()
	if not root:
		return hits
	_query_box_recursive(root, query_box, hits)
	return hits

func _query_box_recursive(node: BVHNode, query_box: AABB, hits: PackedInt32Array) -> void:
	if not node:
		return
	if not node.aabb.intersects(query_box):
		return
		
	if node.is_leaf():
		hits.append(node.object_id)
		return
		
	_query_box_recursive(node.left, query_box, hits)
	_query_box_recursive(node.right, query_box, hits)
