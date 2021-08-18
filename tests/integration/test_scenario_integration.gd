# tests/integration/test_scenario_integration.gd
class_name TestScenarioIntegration
extends "res://tests/test_base.gd"

# End-to-end integration verification suite executing benchmark scenarios headlessly.

const SimulationEngine = preload("res://simulation/simulation_engine.gd")
const VehicleAgent = preload("res://fleet/vehicle_agent.gd")

func before_each() -> void:
	reset_test()

func test_urban_grid_scenario_execution() -> Dictionary:
	var engine: SimulationEngine = SimulationEngine.new()
	var loaded: bool = engine.load_scenario("urban_grid_4way")
	assert_true(loaded, "Urban grid scenario load check")
	assert_equal(engine.fleet_manager.agents.size(), 12, "12 vehicles should be provisioned")
	
	# Execute 50 simulation steps @ 50Hz (1.0 second sim time)
	engine.run_steps(50, 0.02)
	
	var summary: Dictionary = engine.get_simulation_summary()
	assert_almost_equal(summary["sim_time_sec"], 1.0, 0.05, "Simulation time elapsed")
	assert_equal(summary["collision_events"], 0, "Zero collisions in urban grid intersection")
	assert_true(summary["recorded_frames"] > 40, "Telemetry frames recorded")
	
	return get_results()

func test_sensor_degradation_scenario_execution() -> Dictionary:
	var engine: SimulationEngine = SimulationEngine.new()
	var loaded: bool = engine.load_scenario("sensor_degradation")
	assert_true(loaded, "Sensor degradation scenario load check")
	
	# Run 200 steps (4.0 seconds sim time): scheduled fault triggers at t = 3.0s
	engine.run_steps(200, 0.02)
	
	var summary: Dictionary = engine.get_simulation_summary()
	assert_true(summary["sim_time_sec"] >= 4.0, "Time elapsed past fault trigger")
	assert_true(summary["faults_summary"]["total_injected_faults"] >= 1, "Scheduled GNSS fault was triggered")
	
	var veh: VehicleAgent = engine.fleet_manager.agents["test_vehicle"]
	assert_true(veh.true_position.x > 5.0, "Vehicle navigated forwards despite GPS blackout")
	
	return get_results()

func test_highway_platoon_emergency_braking() -> Dictionary:
	var engine: SimulationEngine = SimulationEngine.new()
	var loaded: bool = engine.load_scenario("highway_platoon")
	assert_true(loaded, "Highway platoon scenario load check")
	assert_equal(engine.fleet_manager.agents.size(), 6, "6 platoon vehicles provisioned")
	
	# Run past 5.0 seconds (450 steps = 9.0 seconds, allowing 4s of 7 m/s^2 braking from 22 m/s)
	engine.run_steps(450, 0.02)
	
	var lead_veh: VehicleAgent = engine.fleet_manager.agents["platoon_lead"]
	assert_equal(lead_veh.state, VehicleAgent.AgentState.EMERGENCY_STOP, "Lead vehicle in emergency stop")
	assert_almost_equal(lead_veh.true_speed_mps, 0.0, 0.5, "Lead vehicle brought to halt")
	
	return get_results()

func test_telemetry_recording_and_export() -> Dictionary:
	var engine: SimulationEngine = SimulationEngine.new()
	engine.load_scenario("sensor_degradation")
	engine.run_steps(50, 0.02)
	
	var recorder = engine.telemetry_recorder
	assert_true(recorder.get_frame_count() >= 50, "At least 50 frames recorded")
	
	# Search frame at timestamp t = 0.5s
	var frame: Dictionary = recorder.find_frame_at_time(0.5)
	assert_almost_equal(frame.get("sim_time", 0.0), 0.5, 0.05, "Temporal seek accuracy")
	
	# Test CSV and JSON export
	var json_ok: bool = recorder.export_to_json("user://test_export.json")
	var csv_ok: bool = recorder.export_to_csv("user://test_export.csv")
	assert_true(json_ok, "JSON telemetry export")
	assert_true(csv_ok, "CSV telemetry export")
	
	return get_results()
