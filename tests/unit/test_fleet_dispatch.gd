# tests/unit/test_fleet_dispatch.gd
class_name TestFleetDispatch
extends "res://tests/test_base.gd"

# Unit verification suite for fleet management and Kuhn-Munkres task allocation.

const VehicleAgent = preload("res://fleet/vehicle_agent.gd")
const TaskAllocator = preload("res://fleet/task_allocator.gd")
const FleetManager = preload("res://fleet/fleet_manager.gd")

func before_each() -> void:
	reset_test()

func test_hungarian_bipartite_matching() -> Dictionary:
	# Classic 3x3 cost matrix
	# [ 10, 19, 8 ]
	# [ 10, 1,  23]
	# [ 14, 5,  9 ]
	# Optimal assignment:
	# Agent 0 -> Task 2 (cost 8)
	# Agent 1 -> Task 1 (cost 1)
	# Agent 2 -> Task 0 (cost 14) or similar minimum sum = 8 + 1 + 14 = 23? Wait:
	# If Agent 0 -> Task 0 (10), Agent 1 -> Task 1 (1), Agent 2 -> Task 2 (9) => sum = 20!
	# Can we do better?
	# Agent 0 -> Task 2 (8), Agent 1 -> Task 0 (10), Agent 2 -> Task 1 (5) => sum = 23.
	# Min sum is 20!
	var cost_matrix: Array = [
		[10.0, 19.0, 8.0],
		[10.0, 1.0, 23.0],
		[14.0, 5.0, 9.0]
	]
	
	var matching: Array[Dictionary] = TaskAllocator.solve_hungarian_assignment(cost_matrix)
	assert_equal(matching.size(), 3, "Matching size check")
	
	var total_cost: float = 0.0
	for m in matching:
		total_cost += m["cost"]
		
	assert_almost_equal(total_cost, 20.0, 1e-4, "Hungarian minimum assignment total cost")
	
	return get_results()

func test_market_auction_bidding() -> Dictionary:
	var task: Dictionary = {"task_id": "pickup_A", "position": Vector2(100.0, 0.0)}
	
	var candidate_agents: Array[Dictionary] = [
		{
			"agent_id": "v0",
			"true_position": Vector3(0.0, 0.0, 0.0), # dist = 100m
			"battery_soc": 0.95
		},
		{
			"agent_id": "v1",
			"true_position": Vector3(90.0, 0.0, 0.0), # dist = 10m
			"battery_soc": 0.80
		},
		{
			"agent_id": "v2",
			"true_position": Vector3(95.0, 0.0, 0.0), # dist = 5m but low battery
			"battery_soc": 0.10 # Disqualified due to < 15% SoC
		}
	]
	
	var auction_result: Dictionary = TaskAllocator.conduct_market_auction(task, candidate_agents)
	assert_true(auction_result["success"], "Auction should succeed")
	assert_equal(auction_result["winner_id"], "v1", "v1 should win due to proximity and sufficient SoC")
	
	return get_results()

func test_fleet_manager_dispatch_and_step() -> Dictionary:
	var fleet: FleetManager = FleetManager.new()
	
	# Spawn 2 vehicles
	var v0: VehicleAgent = VehicleAgent.new("agent_alpha")
	v0.spawn(Vector3(0.0, 0.0, 0.0), 0.0)
	fleet.register_vehicle(v0)
	
	var v1: VehicleAgent = VehicleAgent.new("agent_beta")
	v1.spawn(Vector3(0.0, 0.0, 20.0), 0.0)
	fleet.register_vehicle(v1)
	
	assert_equal(fleet.agents.size(), 2, "Registered agent count")
	
	# Dispatch destination to v0
	var ok: bool = v0.set_destination(Vector2(50.0, 0.0), 10.0)
	assert_true(ok, "Destination assignment")
	assert_equal(v0.state, VehicleAgent.AgentState.EN_ROUTE, "Vehicle state en-route")
	
	# Step simulation for 1.0 second (20 steps at 0.05s)
	for _i in range(20):
		fleet.step_fleet(0.05)
		
	var telem: Dictionary = fleet.get_fleet_telemetry()
	assert_true(telem["total_fleet_distance_m"] > 0.1, "Fleet should accumulate distance")
	assert_true(v0.true_position.x > 0.5, "v0 should advance forwards along X axis")
	
	# Test emergency stop broadcast
	fleet.broadcast_emergency_stop("Obstacle detected")
	assert_equal(v0.state, VehicleAgent.AgentState.EMERGENCY_STOP, "Emergency stop state")
	
	fleet.clear_emergency_stop()
	assert_equal(v0.state, VehicleAgent.AgentState.IDLE, "Restored to idle")
	
	return get_results()
