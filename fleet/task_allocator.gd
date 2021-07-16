# fleet/task_allocator.gd
class_name TaskAllocator
extends RefCounted

# Multi-robot task allocation (MRTA) engine.
# Implements Kuhn-Munkres (Hungarian algorithm) O(N^3) optimal bipartite matching
# and dynamic market auction mechanisms for fleet mission dispatch.

# Solves the linear sum assignment problem using the Kuhn-Munkres (Hungarian) algorithm.
# cost_matrix: Array of PackedFloat64Array or Array of Array[float] representing N x M cost matrix.
# Returns Array of Dictionary: [{"agent_idx": i, "task_idx": j, "cost": c}, ...]
static func solve_hungarian_assignment(cost_matrix: Array) -> Array[Dictionary]:
	var num_rows: int = cost_matrix.size()
	if num_rows == 0:
		return []
	var num_cols: int = cost_matrix[0].size()
	if num_cols == 0:
		return []
		
	# Pad matrix to square K x K
	var k: int = maxi(num_rows, num_cols)
	var matrix: Array = []
	for r in range(k):
		var row: Array[float] = []
		for c in range(k):
			if r < num_rows and c < num_cols:
				row.append(float(cost_matrix[r][c]))
			else:
				row.append(1e8) # Dummy high cost
		matrix.append(row)
		
	# Dual variables (potentials) u for rows, v for columns
	var u: Array[float] = []
	var v: Array[float] = []
	var p: Array[int] = [] # Matching for columns (1-based, 0 = unassigned)
	var way: Array[int] = [] # Predecessors in augmenting path
	
	u.resize(k + 1)
	u.fill(0.0)
	v.resize(k + 1)
	v.fill(0.0)
	p.resize(k + 1)
	p.fill(0)
	way.resize(k + 1)
	way.fill(0)
	
	# Jonker-Volgenant / Hungarian augmenting path routine (1-based indexing for pot arrays)
	for i in range(1, k + 1):
		p[0] = i
		var j0: int = 0
		var minv: Array[float] = []
		minv.resize(k + 1)
		minv.fill(INF)
		var used: Array[bool] = []
		used.resize(k + 1)
		used.fill(false)
		
		while true:
			used[j0] = true
			var i0: int = p[j0]
			var delta: float = INF
			var j1: int = 0
			
			for j in range(1, k + 1):
				if not used[j]:
					var cur: float = matrix[i0 - 1][j - 1] - u[i0] - v[j]
					if cur < minv[j]:
						minv[j] = cur
						way[j] = j0
					if minv[j] < delta:
						delta = minv[j]
						j1 = j
						
			for j in range(0, k + 1):
				if used[j]:
					u[p[j]] += delta
					v[j] -= delta
				else:
					minv[j] -= delta
					
			j0 = j1
			if p[j0] == 0:
				break
				
		while true:
			var j1: int = way[j0]
			p[j0] = p[j1]
			j0 = j1
			if j0 == 0:
				break
				
	# Reconstruct optimal matching
	var assignment: Array[Dictionary] = []
	for j in range(1, k + 1):
		var r_idx: int = p[j] - 1
		var c_idx: int = j - 1
		if r_idx < num_rows and c_idx < num_cols:
			assignment.append({
				"agent_idx": r_idx,
				"task_idx": c_idx,
				"cost": float(cost_matrix[r_idx][c_idx])
			})
			
	# Sort assignment by agent index for deterministic ordering
	assignment.sort_custom(func(a: Dictionary, b: Dictionary) -> bool:
		return a["agent_idx"] < b["agent_idx"]
	)
	
	return assignment

# Dynamic market-based auction for asynchronous single-task dispatch.
# Computes utility bids for candidate agents:
# bid = w_dist * distance + w_soc * (1.0 - soc) * 50.0 + w_urgency * priority
static func conduct_market_auction(
	task: Dictionary, candidate_agents: Array[Dictionary],
	w_dist: float = 1.0, w_soc: float = 20.0
) -> Dictionary:
	if candidate_agents.is_empty():
		return {"success": false, "winner_id": "", "bid": INF}
		
	var task_pos: Vector2 = task.get("position", Vector2.ZERO)
	var best_bid: float = INF
	var winner_id: String = ""
	var winning_agent: Dictionary = {}
	
	for agent in candidate_agents:
		var pos_3d: Vector3 = agent.get("true_position", Vector3.ZERO)
		var agent_pos_2d: Vector2 = Vector2(pos_3d.x, pos_3d.z)
		var dist: float = agent_pos_2d.distance_to(task_pos)
		var soc: float = agent.get("battery_soc", 1.0)
		
		# Low battery penalty
		var soc_penalty: float = (1.0 - soc) * w_soc
		var bid: float = (w_dist * dist) + soc_penalty
		
		# Disqualify if battery is below critical threshold (< 15%)
		if soc < 0.15:
			bid += 10000.0
			
		if bid < best_bid:
			best_bid = bid
			winner_id = agent.get("agent_id", "")
			winning_agent = agent
			
	return {
		"success": not winner_id.is_empty() and best_bid < 9000.0,
		"winner_id": winner_id,
		"bid": best_bid,
		"task_id": task.get("task_id", ""),
		"winning_agent": winning_agent
	}
