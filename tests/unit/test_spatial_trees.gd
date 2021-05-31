# tests/unit/test_spatial_trees.gd
class_name TestSpatialTrees
extends "res://tests/test_base.gd"

# Unit verification suite for spatial indexing trees: KDTree2D, KDTree3D, BVHTree, and UniformGrid2D.

const KDTree2D = preload("res://core/spatial/kd_tree_2d.gd")
const KDTree3D = preload("res://core/spatial/kd_tree_3d.gd")
const BVHTree = preload("res://core/spatial/bvh_tree.gd")
const UniformGrid2D = preload("res://core/spatial/uniform_grid.gd")

func before_each() -> void:
	reset_test()

func test_kd_tree_2d_queries() -> Dictionary:
	var pts: Array[Vector2] = [
		Vector2(2.0, 3.0),
		Vector2(5.0, 4.0),
		Vector2(9.0, 6.0),
		Vector2(4.0, 7.0),
		Vector2(8.0, 1.0),
		Vector2(7.0, 2.0)
	]
	var ids: PackedInt32Array = PackedInt32Array([10, 20, 30, 40, 50, 60])
	var tree: KDTree2D = KDTree2D.build_from_points(pts, ids)
	assert_equal(tree.point_count, 6, "Point count check")
	
	# Nearest neighbor to (8.1, 1.9) should be (7.0, 2.0) or (8.0, 1.0)
	var nearest: Dictionary = tree.find_nearest(Vector2(8.1, 1.1))
	assert_equal(nearest["id"], 50, "Nearest neighbor ID check")
	assert_almost_equal(nearest["distance"], 0.141421, 1e-4, "Nearest distance check")
	
	# K-Nearest Neighbors (k=3)
	var knn: Array[Dictionary] = tree.find_k_nearest(Vector2(5.0, 4.0), 3)
	assert_equal(knn.size(), 3, "KNN size")
	assert_equal(knn[0]["id"], 20, "First neighbor is self at distance 0")
	assert_almost_equal(knn[0]["distance"], 0.0, 1e-6)
	
	# Radius query: points within distance 3.5 from (7.0, 2.0)
	var in_radius: Array[Dictionary] = tree.find_in_radius(Vector2(7.0, 2.0), 3.5)
	assert_true(in_radius.size() >= 2, "Radius query should match at least 2 points")
	
	return get_results()

func test_kd_tree_3d_queries() -> Dictionary:
	var pts: Array[Vector3] = [
		Vector3(0.0, 0.0, 0.0),
		Vector3(10.0, 0.0, 0.0),
		Vector3(0.0, 10.0, 0.0),
		Vector3(0.0, 0.0, 10.0),
		Vector3(5.0, 5.0, 5.0)
	]
	var tree: KDTree3D = KDTree3D.build_from_points(pts)
	assert_equal(tree.point_count, 5)
	
	var res: Dictionary = tree.find_nearest(Vector3(0.1, 0.1, 0.1))
	assert_equal(res["id"], 0, "Closest to origin")
	
	var in_rad: Array[Dictionary] = tree.find_in_radius(Vector3(0.0, 0.0, 0.0), 2.0)
	assert_equal(in_rad.size(), 1, "Only origin point within radius 2")
	
	return get_results()

func test_bvh_tree_and_raycasting() -> Dictionary:
	var boxes: Array[AABB] = [
		AABB(Vector3(10.0, -1.0, -1.0), Vector3(2.0, 2.0, 2.0)), # Box 0 at x=10..12
		AABB(Vector3(20.0, -1.0, -1.0), Vector3(2.0, 2.0, 2.0)), # Box 1 at x=20..22
		AABB(Vector3(0.0, 15.0, -1.0), Vector3(2.0, 2.0, 2.0))   # Box 2 at y=15..17
	]
	var tree: BVHTree = BVHTree.build_from_aabbs(boxes)
	assert_equal(tree.total_objects, 3)
	
	# Ray aimed along +X axis from origin
	var ray_hit: Dictionary = tree.raycast(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 100.0)
	assert_true(ray_hit["hit"], "Ray should intersect Box 0")
	assert_equal(ray_hit["id"], 0, "First intersected box should be ID 0")
	assert_almost_equal(ray_hit["distance"], 10.0, 1e-4, "Hit distance should be 10.0")
	
	# Ray aimed away from all boxes
	var ray_miss: Dictionary = tree.raycast(Vector3(0.0, 0.0, 0.0), Vector3(0.0, -1.0, 0.0), 100.0)
	assert_false(ray_miss["hit"], "Ray aimed in -Y should miss all boxes")
	
	# Box intersection query
	var query_box: AABB = AABB(Vector3(9.0, -2.0, -2.0), Vector3(5.0, 4.0, 4.0))
	var hits: PackedInt32Array = tree.query_box_intersections(query_box)
	assert_equal(hits.size(), 1, "Query box should hit exactly Box 0")
	assert_equal(hits[0], 0)
	
	return get_results()

func test_uniform_grid_2d() -> Dictionary:
	var grid: UniformGrid2D = UniformGrid2D.new(10.0)
	grid.insert_object(101, Vector2(5.0, 5.0), {"type": "vehicle"})
	grid.insert_object(102, Vector2(8.0, 8.0), {"type": "vehicle"})
	grid.insert_object(103, Vector2(100.0, 100.0), {"type": "obstacle"})
	
	var neighbors: Array[Dictionary] = grid.query_radius(Vector2(5.0, 5.0), 5.0)
	assert_equal(neighbors.size(), 2, "Should find 101 and 102")
	assert_equal(neighbors[0]["id"], 101)
	assert_equal(neighbors[1]["id"], 102)
	
	# Move 102 far away
	grid.update_object(102, Vector2(200.0, 200.0))
	var neighbors_after_move: Array[Dictionary] = grid.query_radius(Vector2(5.0, 5.0), 5.0)
	assert_equal(neighbors_after_move.size(), 1, "Only 101 should remain in radius")
	
	# Remove 101
	var removed: bool = grid.remove_object(101)
	assert_true(removed, "Removal should succeed")
	var neighbors_empty: Array[Dictionary] = grid.query_radius(Vector2(5.0, 5.0), 5.0)
	assert_equal(neighbors_empty.size(), 0, "No objects left in radius")
	
	return get_results()
