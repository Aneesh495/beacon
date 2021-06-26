# planning/space_time_reservation.gd
class_name SpaceTimeReservation
extends RefCounted

# 4D space-time reservation graph for collision-free multi-agent fleet coordination.
# Tracks spatio-temporal occupancy cells (x, y, time_step) and prevents both
# vertex collisions (simultaneous occupancy) and edge collisions (swapping positions).

var spatial_resolution_m: float = 1.0
var inv_spatial_res: float = 1.0
var time_step_sec: float = 0.5
var inv_time_step: float = 2.0

# Table: time_step (int) -> Dictionary of cell_key (Vector2i) -> agent_id (int)
var reservation_table: Dictionary = {}
# Edge table: time_step (int) -> Array of {"from_cell": Vector2i, "to_cell": Vector2i, "agent_id": int}
var edge_reservations: Dictionary = {}

func _init(p_spatial_res: float = 1.0, p_time_step: float = 0.5) -> void:
	spatial_resolution_m = p_spatial_res
	inv_spatial_res = 1.0 / spatial_resolution_m
	time_step_sec = p_time_step
	inv_time_step = 1.0 / time_step_sec

func clear() -> void:
	reservation_table.clear()
	edge_reservations.clear()

func _to_cell(pos: Vector2) -> Vector2i:
	return Vector2i(
		int(floor(pos.x * inv_spatial_res)),
		int(floor(pos.y * inv_spatial_res))
	)

func _to_time_step(time_sec: float) -> int:
	return int(floor(time_sec * inv_time_step))

# Checks whether a spatial position at a given time is available (vertex collision check)
func is_cell_free(pos: Vector2, time_sec: float, ignore_agent_id: int = -1) -> bool:
	var t_step: int = _to_time_step(time_sec)
	if not reservation_table.has(t_step):
		return true
	var cell: Vector2i = _to_cell(pos)
	if reservation_table[t_step].has(cell):
		var occupying_id: int = reservation_table[t_step][cell]
		return occupying_id == ignore_agent_id
	return true

# Checks edge transition between from_pos at t to to_pos at t + dt (swapping collision check)
func is_transition_free(from_pos: Vector2, to_pos: Vector2, from_time_sec: float, ignore_agent_id: int = -1) -> bool:
	var t_step: int = _to_time_step(from_time_sec)
	var cell_from: Vector2i = _to_cell(from_pos)
	var cell_to: Vector2i = _to_cell(to_pos)
	
	if not is_cell_free(to_pos, from_time_sec + time_step_sec, ignore_agent_id):
		return false
		
	# Check if another agent is transitioning from cell_to to cell_from simultaneously
	if edge_reservations.has(t_step):
		for edge in edge_reservations[t_step]:
			if edge["agent_id"] != ignore_agent_id:
				if edge["from_cell"] == cell_to and edge["to_cell"] == cell_from:
					return false # Swapping collision!
					
	return true

# Reserves space-time trajectory for an agent
func reserve_trajectory(agent_id: int, trajectory_points: Array[Vector2], start_time_sec: float) -> void:
	for i in range(trajectory_points.size()):
		var t_sec: float = start_time_sec + float(i) * time_step_sec
		var t_step: int = _to_time_step(t_sec)
		var cell: Vector2i = _to_cell(trajectory_points[i])
		
		if not reservation_table.has(t_step):
			reservation_table[t_step] = {}
		reservation_table[t_step][cell] = agent_id
		
		if i > 0:
			var prev_t_step: int = _to_time_step(start_time_sec + float(i - 1) * time_step_sec)
			var prev_cell: Vector2i = _to_cell(trajectory_points[i - 1])
			if not edge_reservations.has(prev_t_step):
				edge_reservations[prev_t_step] = []
			edge_reservations[prev_t_step].append({
				"from_cell": prev_cell,
				"to_cell": cell,
				"agent_id": agent_id
			})

# Releases all reservations for a given agent
func release_agent(agent_id: int) -> void:
	for t_step in reservation_table.keys():
		var cell_dict: Dictionary = reservation_table[t_step]
		var keys_to_erase: Array = []
		for cell in cell_dict.keys():
			if cell_dict[cell] == agent_id:
				keys_to_erase.append(cell)
		for cell in keys_to_erase:
			cell_dict.erase(cell)
			
	for t_step in edge_reservations.keys():
		var edge_list: Array = edge_reservations[t_step]
		var new_list: Array = []
		for edge in edge_list:
			if edge["agent_id"] != agent_id:
				new_list.append(edge)
		edge_reservations[t_step] = new_list
