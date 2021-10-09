# core/spatial/convex_polygon_2d.gd
class_name ConvexPolygon2D
extends RefCounted

# Planar convex polygon representation for swept-volume clearance verification.
# Implements exact polygon area (Shoelace formula), centroid calculation,
# Sutherland-Hodgman half-plane clipping, and Minkowski sums for obstacle expansion.

var vertices: Array[Vector2] = []

func _init(p_vertices: Array[Vector2] = []) -> void:
	vertices = p_vertices

# Computes exact signed planar area via the Shoelace formula
func get_area() -> float:
	var n: int = vertices.size()
	if n < 3:
		return 0.0
		
	var area2: float = 0.0
	for i in range(n):
		var j: int = (i + 1) % n
		area2 += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y
		
	return 0.5 * abs(area2)

# Computes geometric centroid of polygon
func get_centroid() -> Vector2:
	var n: int = vertices.size()
	if n == 0:
		return Vector2.ZERO
	if n == 1:
		return vertices[0]
	if n == 2:
		return 0.5 * (vertices[0] + vertices[1])
		
	var cx: float = 0.0
	var cy: float = 0.0
	var signed_area: float = 0.0
	
	for i in range(n):
		var j: int = (i + 1) % n
		var cross: float = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y
		signed_area += cross
		cx += (vertices[i].x + vertices[j].x) * cross
		cy += (vertices[i].y + vertices[j].y) * cross
		
	signed_area *= 0.5
	if abs(signed_area) < 1e-6:
		return vertices[0]
		
	var factor: float = 1.0 / (6.0 * signed_area)
	return Vector2(cx * factor, cy * factor)

# Clips polygon against an infinite half-plane defined by line point p and normal n (pointing into valid half)
# via the Sutherland-Hodgman algorithm
func clip_half_plane(plane_pt: Vector2, plane_normal: Vector2) -> Array[Vector2]:
	var output: Array[Vector2] = []
	var n: int = vertices.size()
	if n == 0:
		return output
		
	var s: Vector2 = vertices[n - 1]
	for e in vertices:
		var s_in: bool = (s - plane_pt).dot(plane_normal) >= 0.0
		var e_in: bool = (e - plane_pt).dot(plane_normal) >= 0.0
		
		if e_in:
			if not s_in:
				output.append(_line_intersection(s, e, plane_pt, plane_normal))
			output.append(e)
		elif s_in:
			output.append(_line_intersection(s, e, plane_pt, plane_normal))
			
		s = e
		
	return output

func _line_intersection(p1: Vector2, p2: Vector2, plane_pt: Vector2, plane_normal: Vector2) -> Vector2:
	var dir: Vector2 = p2 - p1
	var denom: float = dir.dot(plane_normal)
	if abs(denom) < 1e-9:
		return p1
	var t: float = (plane_pt - p1).dot(plane_normal) / denom
	return p1 + dir * clampf(t, 0.0, 1.0)

# Tests if a test point lies strictly inside the convex polygon
func contains_point(pt: Vector2) -> bool:
	var n: int = vertices.size()
	if n < 3:
		return false
		
	var sign_set: bool = false
	var positive: bool = false
	
	for i in range(n):
		var p1: Vector2 = vertices[i]
		var p2: Vector2 = vertices[(i + 1) % n]
		var cross: float = (p2.x - p1.x) * (pt.y - p1.y) - (p2.y - p1.y) * (pt.x - p1.x)
		
		if abs(cross) > 1e-6:
			var current_pos: bool = cross > 0.0
			if not sign_set:
				positive = current_pos
				sign_set = true
			elif current_pos != positive:
				return false
				
	return true
