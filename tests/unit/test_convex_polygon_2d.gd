# tests/unit/test_convex_polygon_2d.gd
class_name TestConvexPolygon2D
extends "res://tests/test_base.gd"

# Unit verification suite for ConvexPolygon2D geometric operations.

const ConvexPolygon2D = preload("res://core/spatial/convex_polygon_2d.gd")

func before_each() -> void:
	reset_test()

func test_polygon_area_and_centroid() -> Dictionary:
	# Unit square [0, 1] x [0, 1]
	var square_pts: Array[Vector2] = [
		Vector2(0.0, 0.0),
		Vector2(1.0, 0.0),
		Vector2(1.0, 1.0),
		Vector2(0.0, 1.0)
	]
	var poly: ConvexPolygon2D = ConvexPolygon2D.new(square_pts)
	
	assert_almost_equal(poly.get_area(), 1.0, 1e-5, "Shoelace square area")
	var centroid: Vector2 = poly.get_centroid()
	assert_almost_equal(centroid.x, 0.5, 1e-4, "Centroid X coordinate")
	assert_almost_equal(centroid.y, 0.5, 1e-4, "Centroid Y coordinate")
	
	assert_true(poly.contains_point(Vector2(0.5, 0.5)), "Interior point containment")
	assert_true(not poly.contains_point(Vector2(1.5, 0.5)), "Exterior point exclusion")
	
	return get_results()

func test_sutherland_hodgman_half_plane_clipping() -> Dictionary:
	# Square [-1, 1] x [-1, 1], area = 4.0
	var square_pts: Array[Vector2] = [
		Vector2(-1.0, -1.0),
		Vector2(1.0, -1.0),
		Vector2(1.0, 1.0),
		Vector2(-1.0, 1.0)
	]
	var poly: ConvexPolygon2D = ConvexPolygon2D.new(square_pts)
	
	# Clip against half-plane x >= 0 (plane_pt = (0, 0), normal = (1, 0))
	var clipped: Array[Vector2] = poly.clip_half_plane(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
	var clipped_poly: ConvexPolygon2D = ConvexPolygon2D.new(clipped)
	
	# Resulting rectangle [0, 1] x [-1, 1] has area 2.0
	assert_almost_equal(clipped_poly.get_area(), 2.0, 1e-4, "Clipped polygon area")
	assert_equal(clipped.size(), 4, "Clipped rectangle vertex count")
	
	return get_results()
