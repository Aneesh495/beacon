# tests/unit/test_costmap_2d.gd
class_name TestCostmap2D
extends "res://tests/test_base.gd"

# Unit verification suite for Costmap2D occupancy grids, inflation layers, and ray clearing.

const Costmap2D = preload("res://maps/costmap_2d.gd")

func before_each() -> void:
	reset_test()

func test_grid_conversions_and_bounds() -> Dictionary:
	var costmap: Costmap2D = Costmap2D.new(50, 50, 1.0, Vector2(0.0, 0.0))
	
	var g0: Vector2i = costmap.world_to_grid(Vector2(0.5, 0.5))
	assert_equal(g0.x, 0)
	assert_equal(g0.y, 0)
	
	var w0: Vector2 = costmap.grid_to_world(Vector2i(0, 0))
	assert_almost_equal(w0.x, 0.5, 1e-4)
	assert_almost_equal(w0.y, 0.5, 1e-4)
	
	assert_true(costmap.is_in_bounds(Vector2i(0, 0)))
	assert_true(costmap.is_in_bounds(Vector2i(49, 49)))
	assert_false(costmap.is_in_bounds(Vector2i(50, 50)))
	assert_false(costmap.is_in_bounds(Vector2i(-1, 0)))
	
	return get_results()

func test_obstacle_inflation_layer() -> Dictionary:
	var costmap: Costmap2D = Costmap2D.new(40, 40, 0.5, Vector2(-10.0, -10.0))
	costmap.robot_radius = 1.0 # 2 cells
	costmap.inflation_radius = 2.0 # 4 cells
	
	# Place lethal obstacle at origin
	costmap.set_obstacle_world(Vector2(0.0, 0.0))
	var lethal_grid: Vector2i = costmap.world_to_grid(Vector2(0.0, 0.0))
	assert_equal(costmap.get_cost(lethal_grid), Costmap2D.LETHAL_OBSTACLE)
	
	# Compute inflation
	costmap.compute_inflation_layer()
	
	# Cell at distance 0.5 meters (within robot_radius = 1.0 m) should be INSCRIBED_INFLATED_OBSTACLE
	var near_grid: Vector2i = costmap.world_to_grid(Vector2(0.5, 0.0))
	assert_equal(costmap.get_cost(near_grid), Costmap2D.INSCRIBED_INFLATED_OBSTACLE)
	
	# Cell at distance 1.5 meters (between robot_radius and inflation_radius) should have positive decay cost
	var mid_grid: Vector2i = costmap.world_to_grid(Vector2(1.5, 0.0))
	var mid_cost: int = costmap.get_cost(mid_grid)
	assert_true(mid_cost > 0 and mid_cost < Costmap2D.INSCRIBED_INFLATED_OBSTACLE, "Mid decay cost check")
	
	# Cell at distance 3.0 meters (beyond inflation_radius = 2.0 m) should remain FREE_SPACE
	var far_grid: Vector2i = costmap.world_to_grid(Vector2(3.0, 0.0))
	assert_equal(costmap.get_cost(far_grid), Costmap2D.FREE_SPACE)
	
	return get_results()

func test_bresenham_ray_clearing() -> Dictionary:
	var costmap: Costmap2D = Costmap2D.new(20, 20, 1.0, Vector2(0.0, 0.0))
	costmap.clear_map(Costmap2D.NO_INFORMATION) # Start unknown
	
	var sensor_origin: Vector2 = Vector2(2.5, 2.5)
	var obstacle_hit: Vector2 = Vector2(8.5, 2.5)
	
	costmap.set_obstacle_world(obstacle_hit)
	costmap.clear_ray_bresenham(sensor_origin, obstacle_hit)
	
	# Cells between sensor and obstacle should be marked FREE_SPACE
	var free_check: Vector2i = costmap.world_to_grid(Vector2(5.5, 2.5))
	assert_equal(costmap.get_cost(free_check), Costmap2D.FREE_SPACE, "Raycleared cell should be FREE")
	
	# Obstacle cell itself should remain LETHAL_OBSTACLE
	var obs_grid: Vector2i = costmap.world_to_grid(obstacle_hit)
	assert_equal(costmap.get_cost(obs_grid), Costmap2D.LETHAL_OBSTACLE, "Obstacle cell must remain LETHAL")
	
	return get_results()
