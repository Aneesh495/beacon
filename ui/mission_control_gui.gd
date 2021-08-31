# ui/mission_control_gui.gd
class_name MissionControlGui
extends Control

# High-fidelity Mission Control Heads-Up Display (HUD).
# Displays real-time digital twin telemetry, vehicle cockpit instruments,
# V2X network metrics, intersection reservations, and fleet agent state tables.

# UI element references
var label_header: Label = null
var label_sim_time: Label = null
var label_rtf: Label = null

var label_speed_digital: Label = null
var label_steering_digital: Label = null
var label_g_force: Label = null
var progress_battery: ProgressBar = null
var label_rmse: Label = null

var label_v2x_stats: Label = null
var label_intersection_stats: Label = null
var label_fleet_table: Label = null

func _init() -> void:
	anchor_right = 1.0
	anchor_bottom = 1.0
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_build_ui_layout()

func _build_ui_layout() -> void:
	# Top bar container
	var top_panel: PanelContainer = PanelContainer.new()
	top_panel.anchor_right = 1.0
	top_panel.custom_minimum_size = Vector2(0.0, 48.0)
	add_child(top_panel)
	
	var top_hbox: HBoxContainer = HBoxContainer.new()
	top_panel.add_child(top_hbox)
	
	label_header = Label.new()
	label_header.text = "  BEACON // AUTONOMOUS FLEET DIGITAL TWIN  "
	top_hbox.add_child(label_header)
	
	label_sim_time = Label.new()
	label_sim_time.text = "T: 0.00s | Ticks: 0"
	top_hbox.add_child(label_sim_time)
	
	label_rtf = Label.new()
	label_rtf.text = " | RTF: 1.00x | Mode: FIXED_STEP"
	top_hbox.add_child(label_rtf)
	
	# Left telemetry instrumentation panel
	var left_panel: PanelContainer = PanelContainer.new()
	left_panel.position = Vector2(16.0, 64.0)
	left_panel.custom_minimum_size = Vector2(280.0, 320.0)
	add_child(left_panel)
	
	var left_vbox: VBoxContainer = VBoxContainer.new()
	left_panel.add_child(left_vbox)
	
	var left_title: Label = Label.new()
	left_title.text = "PRIMARY VEHICLE COCKPIT"
	left_vbox.add_child(left_title)
	
	label_speed_digital = Label.new()
	label_speed_digital.text = "Speed: 0.00 m/s (0.0 km/h)"
	left_vbox.add_child(label_speed_digital)
	
	label_steering_digital = Label.new()
	label_steering_digital.text = "Steering: 0.0 deg"
	left_vbox.add_child(label_steering_digital)
	
	label_g_force = Label.new()
	label_g_force.text = "G-Force: 0.00 G (Lat: 0.00 G)"
	left_vbox.add_child(label_g_force)
	
	label_rmse = Label.new()
	label_rmse.text = "Tracking RMSE: 0.000 m"
	left_vbox.add_child(label_rmse)
	
	var bat_lbl: Label = Label.new()
	bat_lbl.text = "Battery State of Charge (SoC):"
	left_vbox.add_child(bat_lbl)
	
	progress_battery = ProgressBar.new()
	progress_battery.min_value = 0.0
	progress_battery.max_value = 100.0
	progress_battery.value = 100.0
	left_vbox.add_child(progress_battery)
	
	# Right diagnostics panel (V2X & Intersection)
	var right_panel: PanelContainer = PanelContainer.new()
	right_panel.anchor_left = 1.0
	right_panel.anchor_right = 1.0
	right_panel.offset_left = -320.0
	right_panel.offset_top = 64.0
	right_panel.custom_minimum_size = Vector2(300.0, 320.0)
	add_child(right_panel)
	
	var right_vbox: VBoxContainer = VBoxContainer.new()
	right_panel.add_child(right_vbox)
	
	var right_title: Label = Label.new()
	right_title.text = "V2X SIDE-LINK & AIM PROTOCOL"
	right_vbox.add_child(right_title)
	
	label_v2x_stats = Label.new()
	label_v2x_stats.text = "V2X Mesh:\n  Nodes: 0\n  Sent: 0\n  Delivered: 0\n  Drop Rate: 0.0%\n  Avg Latency: 0.0 ms"
	right_vbox.add_child(label_v2x_stats)
	
	label_intersection_stats = Label.new()
	label_intersection_stats.text = "\nIntersection Manager:\n  Active Slots: 0\n  Completed: 0\n  Conflicts: 0"
	right_vbox.add_child(label_intersection_stats)
	
	# Bottom active fleet status table
	var bottom_panel: PanelContainer = PanelContainer.new()
	bottom_panel.anchor_top = 1.0
	bottom_panel.anchor_right = 1.0
	bottom_panel.anchor_bottom = 1.0
	bottom_panel.offset_top = -180.0
	bottom_panel.offset_left = 16.0
	bottom_panel.offset_right = -16.0
	bottom_panel.offset_bottom = -16.0
	add_child(bottom_panel)
	
	var bottom_vbox: VBoxContainer = VBoxContainer.new()
	bottom_panel.add_child(bottom_vbox)
	
	var b_title: Label = Label.new()
	b_title.text = "FLEET STATUS ROSTER"
	bottom_vbox.add_child(b_title)
	
	label_fleet_table = Label.new()
	label_fleet_table.text = "AGENT_ID         STATUS         SPEED (m/s)    SOC (%)    DIST (m)     POS (X, Z)"
	bottom_vbox.add_child(label_fleet_table)

# Updates HUD telemetry in real-time
func update_telemetry(fleet_telem: Dictionary, active_agent_id: String = "") -> void:
	var sim_t: float = fleet_telem.get("sim_time_sec", 0.0)
	var agent_count: int = fleet_telem.get("agent_count", 0)
	label_sim_time.text = "T: %.2fs | Active Agents: %d" % [sim_t, agent_count]
	
	# Primary cockpit update
	var agents: Array = fleet_telem.get("agents", [])
	var target_agent: Dictionary = {}
	for ag in agents:
		if active_agent_id.is_empty() or ag.get("agent_id", "") == active_agent_id:
			target_agent = ag
			break
			
	if not target_agent.is_empty():
		var spd: float = target_agent.get("true_speed_mps", 0.0)
		var spd_kmh: float = spd * 3.6
		label_speed_digital.text = "Speed: %.2f m/s (%.1f km/h)" % [spd, spd_kmh]
		
		var steer_deg: float = rad_to_deg(target_agent.get("true_steering_rad", 0.0))
		label_steering_digital.text = "Steering: %.1f deg" % steer_deg
		
		var rmse: float = target_agent.get("tracking_rmse", 0.0)
		label_rmse.text = "Tracking RMSE: %.3f m" % rmse
		
		var soc: float = target_agent.get("battery_soc", 1.0) * 100.0
		progress_battery.value = soc
		
	# V2X & Intersection updates
	var net: Dictionary = fleet_telem.get("network_metrics", {})
	label_v2x_stats.text = "V2X Mesh:\n  Nodes: %d\n  Sent: %d\n  Delivered: %d\n  Drop Rate: %.1f%%\n  Avg Latency: %.2f ms" % [
		net.get("active_nodes", 0),
		net.get("total_sent", 0),
		net.get("total_delivered", 0),
		net.get("packet_drop_rate", 0.0) * 100.0,
		net.get("average_latency_ms", 0.0)
	]
	
	var im: Dictionary = fleet_telem.get("intersection_metrics", {})
	label_intersection_stats.text = "\nIntersection Manager:\n  Active Slots: %d\n  Completed: %d\n  Rejected: %d" % [
		im.get("active_reservations", 0),
		im.get("completed_reservations", 0),
		im.get("rejected_requests", 0)
	]
	
	# Fleet roster update
	var roster_text: String = "AGENT_ID         STATUS         SPEED (m/s)    SOC (%)    DIST (m)     POS (X, Z)\n"
	for ag in agents:
		var p: Vector3 = ag.get("true_position", Vector3.ZERO)
		var line: String = "%-16s %-14s %-14.2f %-10.1f %-12.1f (%.1f, %.1f)\n" % [
			ag.get("agent_id", ""),
			ag.get("state_string", "IDLE"),
			ag.get("true_speed_mps", 0.0),
			ag.get("battery_soc", 1.0) * 100.0,
			ag.get("distance_travelled_m", 0.0),
			p.x, p.z
		]
		roster_text += line
	label_fleet_table.text = roster_text
