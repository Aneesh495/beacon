# simulation/scenario_loader.gd
class_name ScenarioLoader
extends RefCounted

# Declarative scenario loader and benchmark orchestrator.
# Parses JSON scenario specifications, provisions fleet agents, initializes
# road geometry, configures conflict zones, and schedules dynamic runtime events.

const BUILTIN_SCENARIOS: Array[String] = [
	"urban_grid_4way",
	"warehouse_logistics",
	"highway_platoon",
	"sensor_degradation",
	"dynamic_pedestrian_cross"
]

var active_scenario_id: String = ""
var scenario_metadata: Dictionary = {}
var max_duration_sec: float = 30.0
var scheduled_events: Array[Dictionary] = []
var executed_event_ids: Dictionary = {}

# Loads scenario from file path
func load_from_file(file_path: String) -> Dictionary:
	var file: FileAccess = FileAccess.open(file_path, FileAccess.READ)
	if not file:
		return {"success": false, "error": "Could not open scenario file: " + file_path}
		
	var content: String = file.get_as_text()
	file.close()
	
	var json: JSON = JSON.new()
	var parse_err: Error = json.parse(content)
	if parse_err != OK:
		return {"success": false, "error": "JSON parse error: " + json.get_error_message()}
		
	var data: Dictionary = json.data
	return load_scenario_dictionary(data)

# Loads built-in benchmark scenario by ID name
func load_builtin(scenario_name: String) -> Dictionary:
	var path: String = "res://data/scenarios/%s.json" % scenario_name
	return load_from_file(path)

# Parses scenario dictionary structure
func load_scenario_dictionary(data: Dictionary) -> Dictionary:
	active_scenario_id = data.get("scenario_id", "unnamed_scenario")
	max_duration_sec = data.get("duration_sec", 30.0)
	scenario_metadata = data
	scheduled_events.clear()
	executed_event_ids.clear()
	
	var raw_events: Array = data.get("scheduled_events", [])
	for ev in raw_events:
		scheduled_events.append(ev)
		
	return {
		"success": true,
		"scenario_id": active_scenario_id,
		"duration_sec": max_duration_sec,
		"vehicles_count": data.get("vehicles", []).size(),
		"events_count": scheduled_events.size()
	}

# Evaluates and triggers any scheduled events due at current_time_sec
func process_scheduled_events(current_time_sec: float, engine: Object) -> Array[Dictionary]:
	var triggered: Array[Dictionary] = []
	
	for i in range(scheduled_events.size()):
		var ev: Dictionary = scheduled_events[i]
		var ev_key: String = "ev_%d" % i
		if executed_event_ids.has(ev_key):
			continue
			
		var ev_time: float = ev.get("time_sec", 0.0)
		if current_time_sec >= ev_time:
			executed_event_ids[ev_key] = true
			_execute_event(ev, engine)
			triggered.append(ev)
			
	return triggered

func _execute_event(ev: Dictionary, engine: Object) -> void:
	var ev_type: String = ev.get("event_type", "")
	match ev_type:
		"emergency_stop":
			var target_agent: String = ev.get("target_agent", "")
			var reason: String = ev.get("reason", "Scheduled emergency stop")
			if engine.has_method("trigger_agent_emergency_stop"):
				engine.call("trigger_agent_emergency_stop", target_agent, reason)
		"sensor_fault":
			var target_agent: String = ev.get("target_agent", "")
			var sensor_name: String = ev.get("sensor_name", "gnss")
			var fault_type: String = ev.get("fault_type", "blackout")
			var duration: float = ev.get("duration_sec", 5.0)
			if engine.has_method("inject_agent_sensor_fault"):
				engine.call("inject_agent_sensor_fault", target_agent, sensor_name, fault_type, duration)
		"network_jam":
			var drop_rate: float = ev.get("drop_rate", 1.0)
			var duration: float = ev.get("duration_sec", 5.0)
			if engine.has_method("inject_network_jamming"):
				engine.call("inject_network_jamming", drop_rate, duration)

func is_scenario_finished(current_time_sec: float) -> bool:
	return current_time_sec >= max_duration_sec
