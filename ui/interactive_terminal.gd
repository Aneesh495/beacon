# ui/interactive_terminal.gd
class_name InteractiveTerminal
extends PanelContainer

# Interactive developer cockpit shell for real-time fleet orchestration.
# Parses interactive terminal commands, dispatches simulation actions,
# inspects vehicle telemetry, and triggers chaos fault injections.

var engine_ref: Object = null # Reference to SimulationEngine
var output_log: RichTextLabel = null
var input_field: LineEdit = null
var command_history: Array[String] = []
var history_index: int = -1

func _init(p_engine: Object = null) -> void:
	engine_ref = p_engine
	custom_minimum_size = Vector2(600.0, 320.0)
	_build_terminal_gui()

func _build_terminal_gui() -> void:
	var vbox: VBoxContainer = VBoxContainer.new()
	add_child(vbox)
	
	var title: Label = Label.new()
	title.text = "BEACON INTERACTIVE FLEET COCKPIT // CLI SHELL"
	vbox.add_child(title)
	
	output_log = RichTextLabel.new()
	output_log.custom_minimum_size = Vector2(0.0, 240.0)
	output_log.scroll_following = true
	output_log.bbcode_enabled = true
	output_log.text = "[color=#00ffcc]Beacon OS v1.0.0 Ready. Type 'help' for command manual.[/color]\n"
	vbox.add_child(output_log)
	
	input_field = LineEdit.new()
	input_field.placeholder_text = "beacon> enter command..."
	input_field.text_submitted.connect(_on_command_submitted)
	vbox.add_child(input_field)

func _on_command_submitted(cmd_text: String) -> void:
	if cmd_text.strip_edges().is_empty():
		return
		
	command_history.append(cmd_text)
	history_index = command_history.size()
	
	_print_line("[color=#aaaaaa]beacon>[/color] " + cmd_text)
	var response: String = execute_command(cmd_text)
	if not response.is_empty():
		_print_line(response)
		
	input_field.clear()

func _print_line(msg: String) -> void:
	if output_log:
		output_log.append_text(msg + "\n")

# Parses and executes terminal command string, returning formatted text output
func execute_command(cmd_str: String) -> String:
	var tokens: PackedStringArray = cmd_str.strip_edges().split(" ", false)
	if tokens.is_empty():
		return ""
		
	var verb: String = tokens[0].to_lower()
	
	match verb:
		"help":
			return _cmd_help()
		"status":
			return _cmd_status()
		"fleet":
			return _cmd_fleet(tokens)
		"scenario":
			return _cmd_scenario(tokens)
		"fault":
			return _cmd_fault(tokens)
		"clock":
			return _cmd_clock(tokens)
		"telemetry":
			return _cmd_telemetry(tokens)
		"clear":
			if output_log:
				output_log.clear()
			return ""
		_:
			return "[color=#ff5555]Unknown command: '%s'. Type 'help' for manual.[/color]" % verb

func _cmd_help() -> String:
	return """[color=#ffff55]AVAILABLE COMMANDS:[/color]
  status                                 Display master simulation engine state & metrics
  fleet list                             List all registered vehicle agents and coordinates
  fleet dispatch <id> <x> <z> [speed]    Assign destination waypoint to vehicle
  fleet estop [id]                       Trigger emergency braking (for target or all)
  fleet resume [id]                      Clear emergency stop
  scenario list                          List pre-built benchmark scenarios
  scenario load <name>                   Load and provision benchmark scenario
  fault inject <id> <sensor> <type> <t>  Inject sensor chaos fault (e.g. blackout, bias_jump)
  fault clear                            Clear all active faults
  clock pause | resume | step            Control deterministic simulation clock
  clock speed <scale>                    Set simulation time scale (0.1x to 10.0x)
  telemetry export <csv|json> <path>     Dump flight data recorder telemetry
  clear                                  Clear terminal scrollback buffer"""

func _cmd_status() -> String:
	if not engine_ref:
		return "[color=#ff5555]Engine reference not attached.[/color]"
	var summary: Dictionary = engine_ref.get_simulation_summary()
	return """[color=#55ff55]=== BEACON ENGINE STATUS ===[/color]
  Scenario ID:        %s
  Simulation Time:    %.2f sec
  Total Physics Ticks: %d
  Real-Time Factor:   %.2fx
  Active Fleet Count: %d
  Recorded Frames:    %d
  Min Inter-Vehicle:  %.2f m
  Collisions:         %d
  Near-Misses:        %d""" % [
		summary.get("scenario_id", "none"),
		summary.get("sim_time_sec", 0.0),
		summary.get("total_ticks", 0),
		summary.get("real_time_factor", 1.0),
		summary.get("agent_count", 0),
		summary.get("recorded_frames", 0),
		summary.get("min_inter_vehicle_distance_m", 0.0),
		summary.get("collision_events", 0),
		summary.get("near_miss_events", 0)
	]

func _cmd_fleet(tokens: PackedStringArray) -> String:
	if tokens.size() < 2:
		return "Usage: fleet list | dispatch | estop | resume"
		
	var sub: String = tokens[1].to_lower()
	if not engine_ref:
		return "[color=#ff5555]Engine reference not attached.[/color]"
		
	var fm = engine_ref.fleet_manager
	match sub:
		"list":
			var lines: String = "[color=#00ffff]AGENT_ID         STATUS         SPEED        BATTERY    POSITION[/color]\n"
			for v_id in fm.agents:
				var ag = fm.agents[v_id]
				var p: Vector3 = ag.true_position
				lines += "%-16s %-14s %-12.2f %-10.1f (%.1f, %.1f)\n" % [
					ag.agent_id,
					ag.get_telemetry()["state_string"],
					ag.true_speed_mps,
					ag.battery_soc * 100.0,
					p.x, p.z
				]
			return lines
		"dispatch":
			if tokens.size() < 5:
				return "Usage: fleet dispatch <agent_id> <x> <z> [speed]"
			var target_id: String = tokens[2]
			var x: float = tokens[3].to_float()
			var z: float = tokens[4].to_float()
			var spd: float = tokens[5].to_float() if tokens.size() > 5 else 12.0
			if fm.agents.has(target_id):
				fm.agents[target_id].set_destination(Vector2(x, z), spd)
				return "[color=#55ff55]Dispatched %s to (%.1f, %.1f) at %.1f m/s[/color]" % [target_id, x, z, spd]
			return "[color=#ff5555]Vehicle '%s' not found.[/color]" % target_id
		"estop":
			var target_id: String = tokens[2] if tokens.size() > 2 else ""
			if target_id.is_empty():
				fm.broadcast_emergency_stop("Manual operator override")
				return "[color=#ff5555]Broadcast emergency stop to entire fleet.[/color]"
			elif fm.agents.has(target_id):
				fm.agents[target_id].trigger_emergency_stop("Manual operator override")
				return "[color=#ff5555]Emergency stop triggered on %s[/color]" % target_id
			return "[color=#ff5555]Vehicle '%s' not found.[/color]" % target_id
		"resume":
			var target_id: String = tokens[2] if tokens.size() > 2 else ""
			if target_id.is_empty():
				fm.clear_emergency_stop()
				return "[color=#55ff55]Cleared fleet emergency stop.[/color]"
			elif fm.agents.has(target_id):
				fm.agents[target_id].reset_emergency_stop()
				return "[color=#55ff55]Cleared emergency stop on %s[/color]" % target_id
			return "[color=#ff5555]Vehicle '%s' not found.[/color]" % target_id
		_:
			return "Unknown fleet subcommand: " + sub

func _cmd_scenario(tokens: PackedStringArray) -> String:
	if tokens.size() < 2:
		return "Usage: scenario list | load <name>"
		
	var sub: String = tokens[1].to_lower()
	match sub:
		"list":
			return "[color=#00ffff]Available Scenarios:[/color]\n  * urban_grid_4way\n  * warehouse_logistics\n  * highway_platoon\n  * sensor_degradation\n  * dynamic_pedestrian_cross"
		"load":
			if tokens.size() < 3:
				return "Usage: scenario load <name>"
			var s_name: String = tokens[2]
			if engine_ref:
				var ok: bool = engine_ref.load_scenario(s_name)
				if ok:
					return "[color=#55ff55]Scenario '%s' loaded successfully.[/color]" % s_name
				return "[color=#ff5555]Failed to load scenario: %s[/color]" % s_name
			return "[color=#ff5555]Engine not attached.[/color]"
		_:
			return "Unknown scenario subcommand: " + sub

func _cmd_fault(tokens: PackedStringArray) -> String:
	if tokens.size() < 2:
		return "Usage: fault inject <id> <sensor> <type> <duration_sec> | fault clear"
		
	var sub: String = tokens[1].to_lower()
	if sub == "clear":
		if engine_ref:
			engine_ref.fault_injector.clear_all_active_faults()
			return "[color=#55ff55]Cleared all active faults.[/color]"
		return "Engine not attached."
	elif sub == "inject":
		if tokens.size() < 6:
			return "Usage: fault inject <agent_id> <sensor> <type> <sec>"
		var a_id: String = tokens[2]
		var s_name: String = tokens[3]
		var f_type: String = tokens[4]
		var dur: float = tokens[5].to_float()
		if engine_ref:
			engine_ref.inject_agent_sensor_fault(a_id, s_name, f_type, dur)
			return "[color=#ffaa00]Injected %s fault '%s' into %s for %.1fs[/color]" % [s_name, f_type, a_id, dur]
	return "Unknown fault command."

func _cmd_clock(tokens: PackedStringArray) -> String:
	if tokens.size() < 2 or not engine_ref:
		return "Usage: clock pause | resume | step | speed <scale>"
	var clk = engine_ref.clock
	var sub: String = tokens[1].to_lower()
	match sub:
		"pause":
			clk.pause()
			return "Clock paused."
		"resume":
			clk.resume()
			return "Clock resumed."
		"step":
			clk.step_single_frame()
			return "Stepped single frame."
		"speed":
			if tokens.size() > 2:
				clk.time_scale = tokens[2].to_float()
				return "Time scale set to %.2fx" % clk.time_scale
	return "Unknown clock command."

func _cmd_telemetry(tokens: PackedStringArray) -> String:
	if tokens.size() < 4 or not engine_ref:
		return "Usage: telemetry export <csv|json> <path>"
	var fmt: String = tokens[2].to_lower()
	var path: String = tokens[3]
	var rec = engine_ref.telemetry_recorder
	if fmt == "csv":
		var ok: bool = rec.export_to_csv(path)
		return "[color=#55ff55]Exported CSV to %s[/color]" % path if ok else "[color=#ff5555]CSV export failed[/color]"
	elif fmt == "json":
		var ok: bool = rec.export_to_json(path)
		return "[color=#55ff55]Exported JSON to %s[/color]" % path if ok else "[color=#ff5555]JSON export failed[/color]"
	return "Unknown format: " + fmt
