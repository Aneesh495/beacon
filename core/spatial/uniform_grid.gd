# core/spatial/uniform_grid.gd
class_name UniformGrid2D
extends RefCounted

# Uniform spatial hash grid for O(1) amortized 2D neighborhood queries,
# multi-agent collision avoidance, and dynamic obstacle proximity detection.

var cell_size: float = 5.0
var inv_cell_size: float = 0.2
var grid_cells: Dictionary = {} # Vector2i -> Array[Dictionary]
var object_locations: Dictionary = {} # int (id) -> Vector2

func _init(p_cell_size: float = 5.0) -> void:
	assert(p_cell_size > 0.0, "Cell size must be strictly positive")
	cell_size = p_cell_size
	inv_cell_size = 1.0 / cell_size

func clear() -> void:
	grid_cells.clear()
	object_locations.clear()

func _world_to_cell(pos: Vector2) -> Vector2i:
	return Vector2i(
		int(floor(pos.x * inv_cell_size)),
		int(floor(pos.y * inv_cell_size))
	)

func insert_object(id: int, pos: Vector2, metadata: Dictionary = {}) -> void:
	var cell: Vector2i = _world_to_cell(pos)
	if not grid_cells.has(cell):
		grid_cells[cell] = []
		
	var entry: Dictionary = {"id": id, "pos": pos, "meta": metadata}
	grid_cells[cell].append(entry)
	object_locations[id] = pos

func update_object(id: int, new_pos: Vector2, metadata: Dictionary = {}) -> void:
	remove_object(id)
	insert_object(id, new_pos, metadata)

func remove_object(id: int) -> bool:
	if not object_locations.has(id):
		return false
	var old_pos: Vector2 = object_locations[id]
	var cell: Vector2i = _world_to_cell(old_pos)
	object_locations.erase(id)
	
	if grid_cells.has(cell):
		var list: Array = grid_cells[cell]
		for i in range(list.size()):
			if list[i]["id"] == id:
				list.remove_at(i)
				break
		if list.is_empty():
			grid_cells.erase(cell)
		return true
	return false

# Queries all objects within radius of center position
# Returns Array of Dictionaries: [{"id": int, "pos": Vector2, "distance": float, "meta": Dictionary}]
func query_radius(center: Vector2, radius: float) -> Array[Dictionary]:
	var results: Array[Dictionary] = []
	if radius <= 0.0:
		return results
		
	var radius_sq: float = radius * radius
	var min_cell: Vector2i = _world_to_cell(center - Vector2(radius, radius))
	var max_cell: Vector2i = _world_to_cell(center + Vector2(radius, radius))
	
	for cx in range(min_cell.x, max_cell.x + 1):
		for cy in range(min_cell.y, max_cell.y + 1):
			var cell_key: Vector2i = Vector2i(cx, cy)
			if grid_cells.has(cell_key):
				var items: Array = grid_cells[cell_key]
				for item in items:
					var dist_sq: float = center.distance_squared_to(item["pos"])
					if dist_sq <= radius_sq:
						results.append({
							"id": item["id"],
							"pos": item["pos"],
							"distance": sqrt(dist_sq),
							"meta": item["meta"]
						})
						
	results.sort_custom(func(a, b): return a["distance"] < b["distance"])
	return results
