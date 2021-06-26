# planning/conflict_based_search.gd
class_name ConflictBasedSearch
extends RefCounted

# Two-level Conflict-Based Search (CBS) for optimal Multi-Agent Path Finding (MAPF).
# High-level search explores a Conflict Tree (CT) branching on spatio-temporal conflicts.
# Low-level search plans individual agent paths in 4D space-time via Space-Time A*.

class CBSConstraint:
	var agent_id: int
	var time_step: int
	var pos: Vector2i # For vertex constraint
	var is_edge: bool = false
	var from_pos: Vector2i = Vector2i.ZERO # For edge constraint
	
	func _init(p_agent: int, p_t: int, p_pos: Vector2i, p_is_edge: bool = false, p_from: Vector2i = Vector2i.ZERO) -> void:
		agent_id = p_agent
		time_step = p_t
		pos = p_pos
		is_edge = p_is_edge
		from_pos = p_from

class CTNode:
	var constraints: Array[CBSConstraint] = []
	var paths: Dictionary = {} # int (agent_id) -> Array[Vector2i]
	var cost: int = 0
	
	func compute_cost() -> void:
		cost = 0
		for aid in paths.keys():
			cost += paths[aid].size()

var grid_width: int = 50
var grid_height: int = 50
var obstacles: Dictionary = {} # Vector2i -> bool

func _init(p_w: int = 50, p_h: int = 50) -> void:
	grid_width = p_w
	grid_height = p_h

func add_obstacle(cell: Vector2i) -> void:
	obstacles[cell] = true

# Solves MAPF for multiple agents:
# agents_starts: Dictionary of agent_id (int) -> Vector2i
# agents_goals: Dictionary of agent_id (int) -> Vector2i
# Returns Dictionary of agent_id -> Array[Vector2i] (collision-free paths)
func solve_cbs(agents_starts: Dictionary, agents_goals: Dictionary, max_ct_nodes: int = 200) -> Dictionary:
	var root: CTNode = CTNode.new()
	
	# Initial individual path planning with zero constraints
	for aid in agents_starts.keys():
		var start_pt: Vector2i = agents_starts[aid]
		var goal_pt: Vector2i = agents_goals[aid]
		var path: Array[Vector2i] = _low_level_space_time_astar(aid, start_pt, goal_pt, [])
		if path.is_empty():
			return {} # Agent path infeasible
		root.paths[aid] = path
		
	root.compute_cost()
	
	var open_set: Array[CTNode] = [root]
	var nodes_expanded: int = 0
	
	while not open_set.is_empty() and nodes_expanded < max_ct_nodes:
		nodes_expanded += 1
		open_set.sort_custom(func(a, b): return a.cost < b.cost)
		var current_node: CTNode = open_set.pop_front()
		
		# Validate paths pairwise for conflicts
		var conflict: Dictionary = _find_first_conflict(current_node.paths)
		if not conflict["has_conflict"]:
			# Conflict-free solution found!
			return current_node.paths
			
		var a1: int = conflict["agent1"]
		var a2: int = conflict["agent2"]
		var t_step: int = conflict["time_step"]
		var is_edge: bool = conflict["is_edge"]
		
		# Branch child 1: constrain agent 1
		var child1: CTNode = CTNode.new()
		child1.constraints = current_node.constraints.duplicate()
		for aid in current_node.paths.keys():
			child1.paths[aid] = current_node.paths[aid].duplicate()
			
		if is_edge:
			child1.constraints.append(CBSConstraint.new(a1, t_step, conflict["pos2"], true, conflict["pos1"]))
		else:
			child1.constraints.append(CBSConstraint.new(a1, t_step, conflict["pos1"]))
			
		var replanned_path1: Array[Vector2i] = _low_level_space_time_astar(
			a1, agents_starts[a1], agents_goals[a1], child1.constraints
		)
		if not replanned_path1.is_empty():
			child1.paths[a1] = replanned_path1
			child1.compute_cost()
			open_set.append(child1)
			
		# Branch child 2: constrain agent 2
		var child2: CTNode = CTNode.new()
		child2.constraints = current_node.constraints.duplicate()
		for aid in current_node.paths.keys():
			child2.paths[aid] = current_node.paths[aid].duplicate()
			
		if is_edge:
			child2.constraints.append(CBSConstraint.new(a2, t_step, conflict["pos1"], true, conflict["pos2"]))
		else:
			child2.constraints.append(CBSConstraint.new(a2, t_step, conflict["pos1"]))
			
		var replanned_path2: Array[Vector2i] = _low_level_space_time_astar(
			a2, agents_starts[a2], agents_goals[a2], child2.constraints
		)
		if not replanned_path2.is_empty():
			child2.paths[a2] = replanned_path2
			child2.compute_cost()
			open_set.append(child2)
			
	return {} # No solution within expansion limit

# Finds first vertex or edge conflict between any two agents
func _find_first_conflict(paths: Dictionary) -> Dictionary:
	var agent_ids: Array = paths.keys()
	var max_t: int = 0
	for aid in agent_ids:
		max_t = maxi(max_t, paths[aid].size())
		
	for t in range(max_t):
		for i in range(agent_ids.size()):
			var a1: int = agent_ids[i]
			var path1: Array[Vector2i] = paths[a1]
			var p1_t: Vector2i = path1[min(t, path1.size() - 1)]
			var p1_prev: Vector2i = path1[min(t - 1, path1.size() - 1)] if t > 0 else p1_t
			
			for j in range(i + 1, agent_ids.size()):
				var a2: int = agent_ids[j]
				var path2: Array[Vector2i] = paths[a2]
				var p2_t: Vector2i = path2[min(t, path2.size() - 1)]
				var p2_prev: Vector2i = path2[min(t - 1, path2.size() - 1)] if t > 0 else p2_t
				
				# Vertex conflict: same position at time t
				if p1_t == p2_t:
					return {
						"has_conflict": true,
						"agent1": a1,
						"agent2": a2,
						"time_step": t,
						"is_edge": false,
						"pos1": p1_t,
						"pos2": p2_t
					}
					
				# Edge conflict: swapping positions between t-1 and t
				if t > 0 and p1_prev == p2_t and p2_prev == p1_t:
					return {
						"has_conflict": true,
						"agent1": a1,
						"agent2": a2,
						"time_step": t,
						"is_edge": true,
						"pos1": p1_prev,
						"pos2": p1_t
					}
					
	return {"has_conflict": false}

# Low-level Space-Time A* for a single agent with constraints
func _low_level_space_time_astar(
	agent_id: int, start: Vector2i, goal: Vector2i,
	constraints: Array[CBSConstraint], max_steps: int = 100
) -> Array[Vector2i]:
	# Constraints filtered for this agent
	var vertex_constraints: Dictionary = {} # int (time) -> Dictionary of Vector2i -> bool
	var edge_constraints: Dictionary = {} # int (time) -> Dictionary of String (from_to) -> bool
	
	for c in constraints:
		if c.agent_id == agent_id:
			if c.is_edge:
				if not edge_constraints.has(c.time_step):
					edge_constraints[c.time_step] = {}
				var key: String = "%d,%d->%d,%d" % [c.from_pos.x, c.from_pos.y, c.pos.x, c.pos.y]
				edge_constraints[c.time_step][key] = true
			else:
				if not vertex_constraints.has(c.time_step):
					vertex_constraints[c.time_step] = {}
				vertex_constraints[c.time_step][c.pos] = true
				
	# State in Space-Time A* is (x, y, t)
	var open_set: Array[Dictionary] = []
	var start_h: int = _manhattan(start, goal)
	open_set.append({"pos": start, "t": 0, "g": 0, "f": start_h, "parent": null})
	
	var closed_set: Dictionary = {} # String "x,y,t" -> bool
	
	# Possible motion primitives: 4 cardinal directions + WAIT in place
	var moves: Array[Vector2i] = [
		Vector2i(0, 0), # Wait action
		Vector2i(1, 0), Vector2i(-1, 0),
		Vector2i(0, 1), Vector2i(0, -1)
	]
	
	while not open_set.is_empty():
		open_set.sort_custom(func(a, b): return a["f"] < b["f"])
		var current: Dictionary = open_set.pop_front()
		var cur_pos: Vector2i = current["pos"]
		var cur_t: int = current["t"]
		
		if cur_pos == goal:
			# Verify no future vertex constraints at goal
			var has_future_conflict: bool = false
			for future_t in range(cur_t + 1, cur_t + 10):
				if vertex_constraints.has(future_t) and vertex_constraints[future_t].has(goal):
					has_future_conflict = true
					break
			if not has_future_conflict:
				return _reconstruct_st_path(current)
				
		var state_key: String = "%d,%d,%d" % [cur_pos.x, cur_pos.y, cur_t]
		if closed_set.has(state_key):
			continue
		closed_set[state_key] = true
		
		if cur_t >= max_steps:
			continue
			
		var next_t: int = cur_t + 1
		for move in moves:
			var next_pos: Vector2i = cur_pos + move
			if next_pos.x < 0 or next_pos.x >= grid_width or next_pos.y < 0 or next_pos.y >= grid_height:
				continue
			if obstacles.has(next_pos):
				continue
				
			# Check vertex constraint at (next_pos, next_t)
			if vertex_constraints.has(next_t) and vertex_constraints[next_t].has(next_pos):
				continue
				
			# Check edge constraint from cur_pos to next_pos at next_t
			if edge_constraints.has(next_t):
				var e_key: String = "%d,%d->%d,%d" % [cur_pos.x, cur_pos.y, next_pos.x, next_pos.y]
				if edge_constraints[next_t].has(e_key):
					continue
					
			var next_key: String = "%d,%d,%d" % [next_pos.x, next_pos.y, next_t]
			if closed_set.has(next_key):
				continue
				
			var g_cost: int = current["g"] + 1
			var h_cost: int = _manhattan(next_pos, goal)
			open_set.append({
				"pos": next_pos,
				"t": next_t,
				"g": g_cost,
				"f": g_cost + h_cost,
				"parent": current
			})
			
	return [] # No path found

func _manhattan(a: Vector2i, b: Vector2i) -> int:
	return abs(a.x - b.x) + abs(a.y - b.y)

func _reconstruct_st_path(target_node: Dictionary) -> Array[Vector2i]:
	var path: Array[Vector2i] = []
	var cur: Variant = target_node
	while cur != null:
		path.append(cur["pos"])
		cur = cur["parent"]
	path.reverse()
	return path
