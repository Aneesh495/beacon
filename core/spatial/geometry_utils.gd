# core/spatial/geometry_utils.gd
class_name GeometryUtils
extends RefCounted

# Computational geometry algorithms for autonomous vehicle collision checking:
# Monotone chain convex hull, Separating Axis Theorem (SAT) for oriented bounding boxes,
# Minkowski sums for vehicle footprint inflation, and polyline projection routines.

# 2D cross product of vectors OA and OB: (A.x - O.x)*(B.y - O.y) - (A.y - O.y)*(B.x - O.x)
static func cross_2d(o: Vector2, a: Vector2, b: Vector2) -> float:
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

# Computes 2D convex hull of a point set using Andrew's monotone chain algorithm (O(N log N))
# Returns counter-clockwise ordered vertices of convex hull
static func compute_convex_hull(points: Array[Vector2]) -> Array[Vector2]:
	var n: int = points.size()
	if n <= 2:
		return points.duplicate()
		
	# Sort points lexicographically by x, then by y
	var sorted_pts: Array[Vector2] = points.duplicate()
	sorted_pts.sort_custom(func(a, b):
		if abs(a.x - b.x) > 1e-9:
			return a.x < b.x
		return a.y < b.y
	)
	
	# Build lower hull
	var lower: Array[Vector2] = []
	for p in sorted_pts:
		while lower.size() >= 2 and cross_2d(lower[lower.size() - 2], lower[lower.size() - 1], p) <= 0.0:
			lower.pop_back()
		lower.append(p)
		
	# Build upper hull
	var upper: Array[Vector2] = []
	for i in range(n - 1, -1, -1):
		var p: Vector2 = sorted_pts[i]
		while upper.size() >= 2 and cross_2d(upper[upper.size() - 2], upper[upper.size() - 1], p) <= 0.0:
			upper.pop_back()
		upper.append(p)
		
	# Remove last point of each half because it's repeated
	lower.pop_back()
	upper.pop_back()
	
	lower.append_array(upper)
	return lower

# Projects polygon vertices onto an axis vector, returning [min_proj, max_proj]
static func _project_polygon(poly: Array[Vector2], axis: Vector2) -> Vector2:
	var min_p: float = poly[0].dot(axis)
	var max_p: float = min_p
	for i in range(1, poly.size()):
		var p: float = poly[i].dot(axis)
		if p < min_p:
			min_p = p
		elif p > max_p:
			max_p = p
	return Vector2(min_p, max_p)

# Tests intersection between two 2D convex polygons using Separating Axis Theorem (SAT)
# Returns true if polygons overlap
static func sat_polygons_intersect(poly_a: Array[Vector2], poly_b: Array[Vector2]) -> bool:
	if poly_a.size() < 3 or poly_b.size() < 3:
		return false
		
	# Check edge normals of poly_a
	for i in range(poly_a.size()):
		var p1: Vector2 = poly_a[i]
		var p2: Vector2 = poly_a[(i + 1) % poly_a.size()]
		var edge: Vector2 = p2 - p1
		var normal: Vector2 = Vector2(-edge.y, edge.x).normalized()
		
		var range_a: Vector2 = _project_polygon(poly_a, normal)
		var range_b: Vector2 = _project_polygon(poly_b, normal)
		
		# If projections do not overlap, axis separates polygons
		if range_a.y < range_b.x or range_b.y < range_a.x:
			return false
			
	# Check edge normals of poly_b
	for i in range(poly_b.size()):
		var p1: Vector2 = poly_b[i]
		var p2: Vector2 = poly_b[(i + 1) % poly_b.size()]
		var edge: Vector2 = p2 - p1
		var normal: Vector2 = Vector2(-edge.y, edge.x).normalized()
		
		var range_a: Vector2 = _project_polygon(poly_a, normal)
		var range_b: Vector2 = _project_polygon(poly_b, normal)
		
		if range_a.y < range_b.x or range_b.y < range_a.x:
			return false
			
	return true

# Generates 4 corners of an Oriented Bounding Box (OBB) for a vehicle
# center: (x, y), heading: yaw in radians, length: meters, width: meters
static func generate_vehicle_obb(center: Vector2, heading: float, length: float, width: float) -> Array[Vector2]:
	var cos_h: float = cos(heading)
	var sin_h: float = sin(heading)
	var half_l: float = 0.5 * length
	var half_w: float = 0.5 * width
	
	var forward: Vector2 = Vector2(cos_h, sin_h)
	var lateral: Vector2 = Vector2(-sin_h, cos_h)
	
	return [
		center + forward * half_l + lateral * half_w, # Front-Left
		center + forward * half_l - lateral * half_w, # Front-Right
		center - forward * half_l - lateral * half_w, # Rear-Right
		center - forward * half_l + lateral * half_w  # Rear-Left
	]

# Computes Minkowski sum of two convex polygons: A + B = { a + b | a in A, b in B }
# Returns convex hull of pairwise vector additions
static func minkowski_sum_convex(poly_a: Array[Vector2], poly_b: Array[Vector2]) -> Array[Vector2]:
	var sum_points: Array[Vector2] = []
	sum_points.resize(poly_a.size() * poly_b.size())
	var idx: int = 0
	for pa in poly_a:
		for pb in poly_b:
			sum_points[idx] = pa + pb
			idx += 1
	return compute_convex_hull(sum_points)

# Projects point p onto line segment AB, returning the closest point on segment
static func closest_point_on_segment(p: Vector2, a: Vector2, b: Vector2) -> Vector2:
	var ab: Vector2 = b - a
	var ab_len_sq: float = ab.length_squared()
	if ab_len_sq < 1e-12:
		return a
	var t: float = clampf((p - a).dot(ab) / ab_len_sq, 0.0, 1.0)
	return a + ab * t

# Finds closest point and signed distance from point p to a 2D polyline
# Returns Dictionary: {"point": Vector2, "distance": float, "segment_index": int}
static func project_onto_polyline(p: Vector2, polyline: Array[Vector2]) -> Dictionary:
	var n: int = polyline.size()
	if n == 0:
		return {"point": Vector2.ZERO, "distance": INF, "segment_index": -1}
	if n == 1:
		return {"point": polyline[0], "distance": p.distance_to(polyline[0]), "segment_index": 0}
		
	var best_dist: float = INF
	var best_pt: Vector2 = polyline[0]
	var best_seg: int = 0
	
	for i in range(n - 1):
		var pt_on_seg: Vector2 = closest_point_on_segment(p, polyline[i], polyline[i + 1])
		var dist: float = p.distance_to(pt_on_seg)
		if dist < best_dist:
			best_dist = dist
			best_pt = pt_on_seg
			best_seg = i
			
	return {"point": best_pt, "distance": best_dist, "segment_index": best_seg}
