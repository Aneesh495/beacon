# maps/costmap_2d.gd
class_name Costmap2D
extends RefCounted

# Multi-layered 2D occupancy grid costmap for vehicle navigation and motion planning.
# Implements static map layers, dynamic sensor obstacle updates, Bresenham ray clearing,
# and exponential cost inflation around lethal obstacle boundaries.

const FREE_SPACE: int = 0
const INSCRIBED_INFLATED_OBSTACLE: int = 253
const LETHAL_OBSTACLE: int = 254
const NO_INFORMATION: int = 255

var width: int = 100 # Grid cells
var height: int = 100
var resolution: float = 0.5 # Meters per cell
var inv_resolution: float = 2.0

var origin: Vector2 = Vector2(-25.0, -25.0) # World coordinates of cell (0, 0)
var cost_data: PackedByteArray = PackedByteArray()

# Inflation parameters
var robot_radius: float = 1.0 # Inscribed radius in meters
var inflation_radius: float = 2.5 # Maximum inflation distance in meters
var cost_scaling_factor: float = 3.0 # Alpha in exponential decay

func _init(p_width: int = 100, p_height: int = 100, p_resolution: float = 0.5, p_origin: Vector2 = Vector2(-25.0, -25.0)) -> void:
	width = p_width
	height = p_height
	resolution = p_resolution
	inv_resolution = 1.0 / resolution
	origin = p_origin
	cost_data.resize(width * height)
	clear_map()

func clear_map(default_cost: int = FREE_SPACE) -> void:
	for i in range(width * height):
		cost_data[i] = default_cost

func world_to_grid(world_pos: Vector2) -> Vector2i:
	var gx: int = int(floor((world_pos.x - origin.x) * inv_resolution))
	var gy: int = int(floor((world_pos.y - origin.y) * inv_resolution))
	return Vector2i(gx, gy)

func grid_to_world(grid_pos: Vector2i) -> Vector2:
	return Vector2(
		origin.x + (float(grid_pos.x) + 0.5) * resolution,
		origin.y + (float(grid_pos.y) + 0.5) * resolution
	)

func is_in_bounds(grid_pos: Vector2i) -> bool:
	return grid_pos.x >= 0 and grid_pos.x < width and grid_pos.y >= 0 and grid_pos.y < height

func get_cost(grid_pos: Vector2i) -> int:
	if not is_in_bounds(grid_pos):
		return NO_INFORMATION
	return cost_data[grid_pos.y * width + grid_pos.x]

func set_cost(grid_pos: Vector2i, cost: int) -> void:
	if is_in_bounds(grid_pos):
		cost_data[grid_pos.y * width + grid_pos.x] = clampi(cost, 0, 255)

func get_cost_world(world_pos: Vector2) -> int:
	return get_cost(world_to_grid(world_pos))

# Sets lethal obstacle at world coordinate and triggers local inflation
func set_obstacle_world(world_pos: Vector2) -> void:
	var gp: Vector2i = world_to_grid(world_pos)
	if is_in_bounds(gp):
		set_cost(gp, LETHAL_OBSTACLE)

# Clears free space along a sensor ray from sensor_origin to hit_point using Bresenham's algorithm
func clear_ray_bresenham(sensor_origin_world: Vector2, hit_point_world: Vector2) -> void:
	var start_grid: Vector2i = world_to_grid(sensor_origin_world)
	var end_grid: Vector2i = world_to_grid(hit_point_world)
	
	var x0: int = start_grid.x
	var y0: int = start_grid.y
	var x1: int = end_grid.x
	var y1: int = end_grid.y
	
	var dx: int = abs(x1 - x0)
	var dy: int = abs(y1 - y0)
	var sx: int = 1 if x0 < x1 else -1
	var sy: int = 1 if y0 < y1 else -1
	var err: int = dx - dy
	
	while true:
		var cur_pos: Vector2i = Vector2i(x0, y0)
		if cur_pos == end_grid:
			break # Don't clear the terminal obstacle cell
		if is_in_bounds(cur_pos):
			if get_cost(cur_pos) != LETHAL_OBSTACLE:
				set_cost(cur_pos, FREE_SPACE)
				
		var e2: int = 2 * err
		if e2 > -dy:
			err -= dy
			x0 += sx
		if e2 < dx:
			err += dx
			y0 += sy

# Inflates all lethal obstacles in the costmap according to robot inscribed and inflation radii
func compute_inflation_layer() -> void:
	var lethal_cells: Array[Vector2i] = []
	for y in range(height):
		var row_offset: int = y * width
		for x in range(width):
			if cost_data[row_offset + x] == LETHAL_OBSTACLE:
				lethal_cells.append(Vector2i(x, y))
				
	var cell_inflation_radius: int = int(ceil(inflation_radius * inv_resolution))
	var inscribed_cell_radius: float = robot_radius * inv_resolution
	
	for obstacle in lethal_cells:
		var min_x: int = maxi(0, obstacle.x - cell_inflation_radius)
		var max_x: int = mini(width - 1, obstacle.x + cell_inflation_radius)
		var min_y: int = maxi(0, obstacle.y - cell_inflation_radius)
		var max_y: int = mini(height - 1, obstacle.y + cell_inflation_radius)
		
		for cy in range(min_y, max_y + 1):
			var row_idx: int = cy * width
			for cx in range(min_x, max_x + 1):
				var dx_cells: float = float(cx - obstacle.x)
				var dy_cells: float = float(cy - obstacle.y)
				var dist_cells: float = sqrt(dx_cells * dx_cells + dy_cells * dy_cells)
				var dist_m: float = dist_cells * resolution
				
				if dist_m > inflation_radius:
					continue
					
				var existing_cost: int = cost_data[row_idx + cx]
				if existing_cost == LETHAL_OBSTACLE:
					continue
					
				var new_cost: int = 0
				if dist_m <= robot_radius:
					new_cost = INSCRIBED_INFLATED_OBSTACLE
				else:
					var factor: float = exp(-cost_scaling_factor * (dist_m - robot_radius))
					new_cost = int(round(float(INSCRIBED_INFLATED_OBSTACLE) * factor))
					
				if new_cost > existing_cost:
					cost_data[row_idx + cx] = clampi(new_cost, 0, 255)

# Checks if a given trajectory polyline collides with any lethal or inscribed obstacle
func is_path_collision_free(world_path: Array[Vector2]) -> bool:
	for pt in world_path:
		var cost: int = get_cost_world(pt)
		if cost >= INSCRIBED_INFLATED_OBSTACLE:
			return false
	return true
