# simulation/simulation_engine.gd
class_name SimulationEngine
extends RefCounted

# Master Cyber-Physical Simulation Engine for Beacon.
# Orchestrates deterministic clock stepping, fleet kinematics, multi-agent V2X,
# chaos fault injection, scenario benchmarks, and continuous safety verification.

const DeterministicClock = preload("res://simulation/deterministic_clock.gd")
const FleetManager = preload("res://fleet/fleet_manager.gd")
const VehicleAgent = preload("res://fleet/vehicle_agent.gd")
const ScenarioLoader = preload("res://simulation/scenario_loader.gd")
const FaultInjector = preload("res://simulation/fault_injector.gd")
const TelemetryRecorder = preload("res://simulation/telemetry_recorder.gd")

var clock: DeterministicClock = null
var fleet_manager: FleetManager = null
var scenario_loader: ScenarioLoader = null
var fault_injector: FaultInjector = null
var telemetry_recorder: TelemetryRecorder = null

# Safety verification metrics
var min_inter_vehicle_distance_m: float = INF
var collision_events_detected: int = 0
var near_miss_events_detected: int = 0
const COLLISION_THRESHOLD_M: float = 1.8
const NEAR_MISS_THRESHOLD_M: float = 3.0

func _init() -> void:
	clock = DeterministicClock.new(0.02)
	fleet_manager = FleetManager.new()
	scenario_loader = ScenarioLoader.new()
	fault_injector = FaultInjector.new()
	telemetry_recorder = TelemetryRecorder.new(10000)

# Loads and provisions a scenario by built-in name or JSON path
func load_scenario(scenario_name_or_path: String) -> bool:
	var res: Dictionary = {}
	if scenario_name_or_path.ends_with(".json"):
		res = scenario_loader.load_from_file(scenario_name_or_path)
	else:
		res = scenario_loader.load_builtin(scenario_name_or_path)
		
	if not res.get("success", false):
		return false
		
	# Reset states
	clock.reset()
	telemetry_recorder.clear()
	fault_injector.clear_all_active_faults()
	fleet_manager.agents.clear()
	min_inter_vehicle_distance_m = INF
	collision_events_detected = 0
	near_miss_events_detected = 0
	
	var data: Dictionary = scenario_loader.scenario_metadata
	
	# Configure intersection if defined
	if data.has("intersection"):
		var i_data: Dictionary = data["intersection"]
		var c_arr: Array = i_data.get("center", [0.0, 0.0])
		var c_pos: Vector2 = Vector2(c_arr[0], c_arr[1])
		var c_rad: float = i_data.get("conflict_radius", 15.0)
		fleet_manager.intersection_manager.center_position = c_pos
		fleet_manager.intersection_manager.conflict_radius_m = c_rad
		
	# Provision vehicle agents
	var v_list: Array = data.get("vehicles", [])
	for v_def in v_list:
		var v_id: String = v_def.get("id", "veh")
		var agent: VehicleAgent = VehicleAgent.new(v_id)
		
		var sp: Array = v_def.get("spawn", [0.0, 0.0, 0.0])
		var spawn_pos: Vector3 = Vector3(sp[0], sp[1], sp[2])
		var heading: float = v_def.get("heading", 0.0)
		agent.spawn(spawn_pos, heading)
		
		fleet_manager.register_vehicle(agent)
		
		# Set initial destination if provided
		if v_def.has("destination"):
			var dest_arr: Array = v_def["destination"]
			var dest_2d: Vector2 = Vector2(dest_arr[0], dest_arr[1])
			var spd: float = v_def.get("speed_limit", 12.0)
			agent.set_destination(dest_2d, spd)
			
	return true

# Advances simulation by one discrete tick
func step(dt: float) -> Dictionary:
	var effective_dt: float = clock.advance_tick(dt)
	if effective_dt <= 1e-6:
		return {}
		
	var current_t: float = clock.current_sim_time_sec
	
	# 1. Process scheduled scenario events (fault injections, stops)
	scenario_loader.process_scheduled_events(current_t, self)
	
	# 2. Update chaos fault injection timers
	fault_injector.update_faults(current_t)
	
	# 3. Advance fleet kinematics, sensors, and V2X mesh
	var fleet_telem: Dictionary = fleet_manager.step_fleet(effective_dt)
	
	# 4. Record high-frequency telemetry frame
	telemetry_recorder.record_frame(current_t, fleet_telem)
	
	# 5. Continuous inter-vehicle proximity safety verification
	_verify_fleet_safety()
	
	return fleet_telem

# Executes fixed number of simulation steps
func run_steps(step_count: int, dt: float = 0.02) -> Dictionary:
	for _i in range(step_count):
		step(dt)
	return get_simulation_summary()

# Runs scenario headlessly until scenario duration completes
func run_to_completion(dt: float = 0.02, max_steps: int = 5000) -> Dictionary:
	var steps: int = 0
	while not scenario_loader.is_scenario_finished(clock.current_sim_time_sec) and steps < max_steps:
		step(dt)
		steps += 1
	return get_simulation_summary()

func _verify_fleet_safety() -> void:
	var agent_ids: Array = fleet_manager.agents.keys()
	var n: int = agent_ids.size()
	if n < 2:
		return
		
	for i in range(n):
		var ag_i: VehicleAgent = fleet_manager.agents[agent_ids[i]]
		var pos_i: Vector3 = ag_i.true_position
		for j in range(i + 1, n):
			var ag_j: VehicleAgent = fleet_manager.agents[agent_ids[j]]
			var pos_j: Vector3 = ag_j.true_position
			var dist: float = pos_i.distance_to(pos_j)
			
			if dist < min_inter_vehicle_distance_m:
				min_inter_vehicle_distance_m = dist
				
			if dist < COLLISION_THRESHOLD_M:
				collision_events_detected += 1
			elif dist < NEAR_MISS_THRESHOLD_M:
				near_miss_events_detected += 1

func trigger_agent_emergency_stop(agent_id: String, reason: String) -> void:
	if fleet_manager.agents.has(agent_id):
		fleet_manager.agents[agent_id].trigger_emergency_stop(reason)

func inject_agent_sensor_fault(agent_id: String, sensor_name: String, fault_type: String, duration: float) -> void:
	if fleet_manager.agents.has(agent_id):
		fault_injector.inject_sensor_fault(
			fleet_manager.agents[agent_id],
			sensor_name, fault_type, duration,
			clock.current_sim_time_sec
		)

func inject_network_jamming(drop_rate: float, duration: float) -> void:
	fault_injector.inject_network_jamming(
		fleet_manager.v2x_network,
		drop_rate, duration,
		clock.current_sim_time_sec
	)

func get_simulation_summary() -> Dictionary:
	return {
		"scenario_id": scenario_loader.active_scenario_id,
		"sim_time_sec": clock.current_sim_time_sec,
		"total_ticks": clock.total_tick_count,
		"real_time_factor": clock.get_real_time_factor(),
		"agent_count": fleet_manager.agents.size(),
		"recorded_frames": telemetry_recorder.get_frame_count(),
		"min_inter_vehicle_distance_m": min_inter_vehicle_distance_m,
		"collision_events": collision_events_detected,
		"near_miss_events": near_miss_events_detected,
		"faults_summary": fault_injector.get_fault_summary()
	}
