# tests/unit/test_geometry_utils.gd
class_name TestGeometryUtils
extends "res://tests/test_base.gd"

# Unit verification suite for computational geometry algorithms.

const GeometryUtils = preload("res://core/spatial/geometry_utils.gd")

func before_each() -> void:
	reset_test()

func test_convex_hull_monotone_chain() -> Dictionary:
	var pts: Array[Vector2] = [
		Vector2(0.0, 0.0),
		Vector2(1.0, 0.5), # interior point
		Vector2(2.0, 0.0),
		Vector2(2.0, 2.0),
		Vector2(0.5, 1.5), # interior point
		Vector2(0.0, 2.0)
	]
	var hull: Array[Vector2] = GeometryUtils.compute_convex_hull(pts)
	assert_equal(hull.size(), 4, "Square hull should have 4 corner vertices")
	
	# Verify area of square hull: 2 * 2 = 4
	var area: float = 0.0
	for i in range(hull.size()):
		var p1: Vector2 = hull[i]
		var p2: Vector2 = hull[(i + 1) % hull.size()]
		area += p1.x * p2.y - p2.x * p1.y
	assert_almost_equal(abs(area * 0.5), 4.0, 1e-6, "Convex hull area check")
	
	return get_results()

func test_sat_oriented_bounding_box_collision() -> Dictionary:
	# Vehicle A at origin, length=4, width=2, heading=0 (aligned with X axis)
	var obb_a: Array[Vector2] = GeometryUtils.generate_vehicle_obb(Vector2(0.0, 0.0), 0.0, 4.0, 2.0)
	
	# Vehicle B at (3.0, 0.0) -> Overlaps with A (A extends to x=+2, B extends back to x=+1)
	var obb_b_overlap: Array[Vector2] = GeometryUtils.generate_vehicle_obb(Vector2(3.0, 0.0), 0.0, 4.0, 2.0)
	assert_true(GeometryUtils.sat_polygons_intersect(obb_a, obb_b_overlap), "Vehicles should collide")
	
	# Vehicle C at (6.0, 0.0) -> Clearly separated
	var obb_c_clear: Array[Vector2] = GeometryUtils.generate_vehicle_obb(Vector2(6.0, 0.0), 0.0, 4.0, 2.0)
	assert_false(GeometryUtils.sat_polygons_intersect(obb_a, obb_c_clear), "Vehicles should be separated")
	
	# Vehicle D rotated 45 degrees near corner of A but not intersecting
	var obb_d_rotated: Array[Vector2] = GeometryUtils.generate_vehicle_obb(Vector2(3.5, 3.5), PI * 0.25, 4.0, 2.0)
	assert_false(GeometryUtils.sat_polygons_intersect(obb_a, obb_d_rotated), "Rotated vehicle should not collide")
	
	return get_results()

func test_minkowski_sum_expansion() -> Dictionary:
	# Square A from [-1, 1] x [-1, 1]
	var poly_a: Array[Vector2] = [
		Vector2(-1.0, -1.0), Vector2(1.0, -1.0),
		Vector2(1.0, 1.0), Vector2(-1.0, 1.0)
	]
	# Small circular obstacle approximated by 4-gon radius 0.5
	var poly_b: Array[Vector2] = [
		Vector2(-0.5, 0.0), Vector2(0.0, -0.5),
		Vector2(0.5, 0.0), Vector2(0.0, 0.5)
	]
	var expanded: Array[Vector2] = GeometryUtils.minkowski_sum_convex(poly_a, poly_b)
	assert_true(expanded.size() >= 4, "Minkowski sum should be valid convex polygon")
	
	return get_results()

func test_polyline_projection() -> Dictionary:
	var path: Array[Vector2] = [
		Vector2(0.0, 0.0),
		Vector2(10.0, 0.0),
		Vector2(10.0, 10.0),
		Vector2(20.0, 10.0)
	]
	
	# Query point at (5.0, 2.0) -> closest is on first segment at (5.0, 0.0), distance = 2.0
	var proj1: Dictionary = GeometryUtils.project_onto_polyline(Vector2(5.0, 2.0), path)
	assert_vector2_almost_equal(proj1["point"], Vector2(5.0, 0.0), 1e-4)
	assert_almost_equal(proj1["distance"], 2.0, 1e-4)
	assert_equal(proj1["segment_index"], 0)
	
	# Query point at (11.0, 5.0) -> closest is on second segment at (10.0, 5.0), distance = 1.0
	var proj2: Dictionary = GeometryUtils.project_onto_polyline(Vector2(11.0, 5.0), path)
	assert_vector2_almost_equal(proj2["point"], Vector2(10.0, 5.0), 1e-4)
	assert_almost_equal(proj2["distance"], 1.0, 1e-4)
	assert_equal(proj2["segment_index"], 1)
	
	return get_results()
