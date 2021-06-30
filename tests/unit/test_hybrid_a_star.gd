# tests/unit/test_hybrid_a_star.gd
class_name TestHybridAStar
extends "res://tests/test_base.gd"

# Unit verification suite for continuous SE(2) Hybrid A* kinodynamic motion planning.

const HybridAStar = preload("res://planning/hybrid_a_star.gd")
const Costmap2D = preload("res://maps/costmap_2d.gd")

func before_each() -> void:
	reset_test()

func test_hybrid_a_star_straight_line() -> Dictionary:
	var planner: HybridAStar = HybridAStar.new()
	var costmap: Costmap2D = Costmap2D.new(50, 50, 1.0, Vector2(0.0, -25.0))
	
	var start_pose: Vector3 = Vector3(2.0, 0.0, 0.0)
	var goal_pose: Vector3 = Vector3(18.0, 0.0, 0.0)
	
	var path: Array[Vector3] = planner.plan(start_pose, goal_pose, costmap, 500)
	assert_true(path.size() >= 2, "Path should contain at least start and goal")
	
	# Verify start and goal poses
	assert_vector2_almost_equal(Vector2(path[0].x, path[0].y), Vector2(start_pose.x, start_pose.y), 1.0)
	var last_pt: Vector3 = path[path.size() - 1]
	assert_vector2_almost_equal(Vector2(last_pt.x, last_pt.y), Vector2(goal_pose.x, goal_pose.y), 1.5)
	
	return get_results()

func test_hybrid_a_star_obstacle_avoidance() -> Dictionary:
	var planner: HybridAStar = HybridAStar.new()
	var costmap: Costmap2D = Costmap2D.new(40, 40, 1.0, Vector2(0.0, -20.0))
	
	# Place lethal obstacle directly on straight line path at (10, 0)
	for oy in range(-2, 3):
		costmap.set_obstacle_world(Vector2(10.0, float(oy)))
	costmap.compute_inflation_layer()
	
	var start_pose: Vector3 = Vector3(2.0, 0.0, 0.0)
	var goal_pose: Vector3 = Vector3(20.0, 0.0, 0.0)
	
	var path: Array[Vector3] = planner.plan(start_pose, goal_pose, costmap, 1000)
	assert_true(path.size() >= 2, "Planner should find path around obstacle")
	
	# Verify that no path point violates the costmap lethal/inscribed obstacles
	for pt in path:
		var cost: int = costmap.get_cost_world(Vector2(pt.x, pt.y))
		assert_true(cost < Costmap2D.INSCRIBED_INFLATED_OBSTACLE, "Path points must be collision free")
		
	return get_results()
