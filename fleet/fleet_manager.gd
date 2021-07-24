# fleet/fleet_manager.gd
class_name FleetManager
extends RefCounted

# Supervisory fleet orchestrator coordinating multi-vehicle operations,
# mission task dispatch, intersection reservations, V2X communications,
# and high-level safety monitoring.

const VehicleAgent = preload("res://fleet/vehicle_agent.gd")
const TaskAllocator = preload("res://fleet/task_allocator.gd")
const IntersectionManager = preload("res://fleet/intersection_manager.gd")
const V2xMeshNetwork = preload("res://fleet/v2x_mesh_network.gd")

var agents: Dictionary = {} # agent_id -> VehicleAgent
var v2x_network: V2xMeshNetwork = null
var intersection_manager: IntersectionManager = null

# Fleet performance metrics
var total_fleet_distance_m: float = 0.0
var total_tasks_completed: int = 0
var simulation_time_sec: float = 0.0

func _init() -> void:
	v2x_network = V2xMeshNetwork.new()
	intersection_manager = IntersectionManager.new("central_intersection", Vector2.ZERO, 15.0)

# Registers a vehicle agent into the fleet registry and V2X mesh
func register_vehicle(agent: VehicleAgent) -> void:
	agents[agent.agent_id] = agent
	v2x_network.register_node(agent.agent_id, agent.true_position)

# Spawns a batch of vehicles at specified initial poses
func spawn_batch(count: int, positions: Array[Vector3], headings: Array[float]) -> void:
	for i in range(count):
		var v_id: String = "vehicle_%02d" % i
		var agent: VehicleAgent = VehicleAgent.new(v_id)
		var pos: Vector3 = positions[i] if i < positions.size() else Vector3(float(i) * 5.0, 0.0, 0.0)
		var hdg: float = headings[i] if i < headings.size() else 0.0
		agent.spawn(pos, hdg)
		register_vehicle(agent)

# Batch dispatches tasks to idle vehicles using Kuhn-Munkres optimal assignment
# tasks: Array of Dictionary {"task_id": String, "position": Vector2, "speed_limit": float}
func batch_dispatch(tasks: Array[Dictionary]) -> Array[Dictionary]:
	var idle_agents: Array[VehicleAgent] = []
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		if agent.state == VehicleAgent.AgentState.IDLE and agent.battery_soc > 0.2:
			idle_agents.append(agent)
			
	if idle_agents.is_empty() or tasks.is_empty():
		return []
		
	# Build cost matrix: N agents x M tasks based on Euclidean distance
	var cost_matrix: Array = []
	for agent in idle_agents:
		var row: Array[float] = []
		var agent_pos_2d: Vector2 = Vector2(agent.true_position.x, agent.true_position.z)
		for task in tasks:
			var task_pos: Vector2 = task.get("position", Vector2.ZERO)
			row.append(agent_pos_2d.distance_to(task_pos))
		cost_matrix.append(row)
		
	# Solve optimal bipartite matching
	var assignments: Array[Dictionary] = TaskAllocator.solve_hungarian_assignment(cost_matrix)
	var dispatched: Array[Dictionary] = []
	
	for match_entry in assignments:
		var a_idx: int = match_entry["agent_idx"]
		var t_idx: int = match_entry["task_idx"]
		var assigned_agent: VehicleAgent = idle_agents[a_idx]
		var assigned_task: Dictionary = tasks[t_idx]
		
		var success: bool = assigned_agent.set_destination(
			assigned_task["position"],
			assigned_task.get("speed_limit", 15.0)
		)
		if success:
			dispatched.append({
				"agent_id": assigned_agent.agent_id,
				"task_id": assigned_task.get("task_id", ""),
				"cost": match_entry["cost"]
			})
			
	return dispatched

# Steps the entire fleet simulation forwards by dt seconds
func step_fleet(dt: float) -> Dictionary:
	simulation_time_sec += dt
	
	# 1. Update individual vehicle dynamics and sensors
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		agent.update_simulation(dt)
		v2x_network.update_node_position(v_id, agent.true_position)
		
	# 2. Advance V2X network clock and route packets
	v2x_network.step_network(dt)
	
	# 3. Broadcast 10Hz Basic Safety Messages (BSM) across V2X mesh
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		var bsm_payload: Dictionary = {
			"pos": agent.true_position,
			"vel": agent.true_velocity,
			"speed": agent.true_speed_mps,
			"heading": agent.true_heading_rad,
			"state": agent.state
		}
		v2x_network.broadcast_message(v_id, "BSM", bsm_payload)
		
	# 4. Process received messages and coordinate intersection reservations
	intersection_manager.update_clock(simulation_time_sec)
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		var inbox: Array[Dictionary] = v2x_network.poll_inbox(v_id)
		
		# Check if vehicle is approaching intersection zone
		var dist_to_intersection: float = Vector2(agent.true_position.x, agent.true_position.z).distance_to(intersection_manager.center_position)
		if dist_to_intersection < (intersection_manager.conflict_radius_m + 10.0) and dist_to_intersection > intersection_manager.conflict_radius_m:
			if not intersection_manager.is_agent_reserved(v_id) and agent.state == VehicleAgent.AgentState.EN_ROUTE:
				var res_req: Dictionary = {
					"agent_id": v_id,
					"current_time": simulation_time_sec,
					"eta_to_entry": dist_to_intersection / maxf(1.0, agent.true_speed_mps),
					"trajectory": agent.current_waypoints,
					"target_speed": agent.target_speed_limit_mps,
					"is_emergency": false
				}
				var res_res: Dictionary = intersection_manager.request_reservation(res_req)
				if not res_res["granted"]:
					# Halt at virtual stop line
					agent.state = VehicleAgent.AgentState.INTERSECTION_WAIT
		elif agent.state == VehicleAgent.AgentState.INTERSECTION_WAIT:
			# Retry reservation
			var retry_req: Dictionary = {
				"agent_id": v_id,
				"current_time": simulation_time_sec,
				"eta_to_entry": 0.5,
				"trajectory": agent.current_waypoints,
				"target_speed": agent.target_speed_limit_mps,
				"is_emergency": false
			}
			var retry_res: Dictionary = intersection_manager.request_reservation(retry_req)
			if retry_res["granted"]:
				agent.state = VehicleAgent.AgentState.EN_ROUTE
				
	return get_fleet_telemetry()

# Broadcasts emergency stop command to all vehicles
func broadcast_emergency_stop(reason: String) -> void:
	for v_id in agents:
		agents[v_id].trigger_emergency_stop(reason)

# Clears emergency stop and restores idle readiness
func clear_emergency_stop() -> void:
	for v_id in agents:
		agents[v_id].reset_emergency_stop()

# Aggregates comprehensive fleet telemetry
func get_fleet_telemetry() -> Dictionary:
	var state_counts: Dictionary = {}
	var total_soc: float = 0.0
	var total_dist: float = 0.0
	var agent_telemetry: Array[Dictionary] = []
	
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		var telem: Dictionary = agent.get_telemetry()
		agent_telemetry.append(telem)
		
		var s_name: String = telem["state_string"]
		state_counts[s_name] = state_counts.get(s_name, 0) + 1
		total_soc += agent.battery_soc
		total_dist += agent.distance_travelled_m
		
	var agent_count: int = maxi(1, agents.size())
	return {
		"sim_time_sec": simulation_time_sec,
		"agent_count": agents.size(),
		"state_counts": state_counts,
		"average_battery_soc": total_soc / float(agent_count),
		"total_fleet_distance_m": total_dist,
		"agents": agent_telemetry,
		"network_metrics": v2x_network.get_network_metrics(),
		"intersection_metrics": intersection_manager.get_metrics()
	}
