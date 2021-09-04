# scenes/main.gd
class_name MainSimulationScene
extends Node3D

# Primary 3D simulation stage for Beacon digital twin.
# Integrates 3D graphics, physical vehicle nodes, tactical camera,
# trajectory visualizer ribbons, mission control HUD, and developer CLI shell.

const SimulationEngine = preload("res://simulation/simulation_engine.gd")
const VehicleAgent = preload("res://fleet/vehicle_agent.gd")
const TacticalCamera = preload("res://ui/tactical_camera.gd")
const TrajectoryVisualizer = preload("res://ui/trajectory_visualizer.gd")
const LidarVisualizer = preload("res://ui/lidar_visualizer.gd")
const MissionControlGui = preload("res://ui/mission_control_gui.gd")
const InteractiveTerminal = preload("res://ui/interactive_terminal.gd")

var engine_instance: SimulationEngine = null

# 3D and UI Nodes
var tactical_camera: TacticalCamera = null
var trajectory_viz: TrajectoryVisualizer = null
var lidar_viz: LidarVisualizer = null
var hud_gui: MissionControlGui = null
var terminal_gui: InteractiveTerminal = null

# Vehicle visual node registry: agent_id -> Node3D
var vehicle_mesh_nodes: Dictionary = {}
var vehicle_body_material: StandardMaterial3D = null
var selected_agent_id: String = ""

func _ready() -> void:
	_setup_environment()
	
	engine_instance = SimulationEngine.new()
	
	# Visualizers
	trajectory_viz = TrajectoryVisualizer.new()
	add_child(trajectory_viz)
	
	lidar_viz = LidarVisualizer.new()
	add_child(lidar_viz)
	
	# Tactical Camera
	tactical_camera = TacticalCamera.new()
	add_child(tactical_camera)
	
	# GUI & Terminal Overlay
	hud_gui = MissionControlGui.new()
	add_child(hud_gui)
	
	terminal_gui = InteractiveTerminal.new(engine_instance)
	terminal_gui.position = Vector2(300.0, 100.0)
	terminal_gui.visible = false
	add_child(terminal_gui)
	
	# Load default showcase scenario
	engine_instance.load_scenario("urban_grid_4way")
	_rebuild_vehicle_meshes()

func _setup_environment() -> void:
	# Directional Sun Light
	var sun: DirectionalLight3D = DirectionalLight3D.new()
	sun.rotation = Vector3(-deg_to_rad(45.0), deg_to_rad(30.0), 0.0)
	sun.light_color = Color(1.0, 0.98, 0.92)
	sun.shadow_enabled = true
	add_child(sun)
	
	# Ground Plane
	var ground: MeshInstance3D = MeshInstance3D.new()
	var plane_mesh: PlaneMesh = PlaneMesh.new()
	plane_mesh.size = Vector2(400.0, 400.0)
	ground.mesh = plane_mesh
	
	var ground_mat: StandardMaterial3D = StandardMaterial3D.new()
	ground_mat.albedo_color = Color(0.12, 0.14, 0.16) # Dark asphalt asphalt tone
	ground_mat.roughness = 0.8
	ground.material_override = ground_mat
	add_child(ground)
	
	# Vehicle chassis material
	vehicle_body_material = StandardMaterial3D.new()
	vehicle_body_material.albedo_color = Color(0.15, 0.55, 0.95)
	vehicle_body_material.metallic = 0.6
	vehicle_body_material.roughness = 0.3

func _rebuild_vehicle_meshes() -> void:
	for n in vehicle_mesh_nodes.values():
		n.queue_free()
	vehicle_mesh_nodes.clear()
	
	var agents: Dictionary = engine_instance.fleet_manager.agents
	for v_id in agents:
		var node: Node3D = _create_vehicle_chassis_node(v_id)
		add_child(node)
		vehicle_mesh_nodes[v_id] = node
		if selected_agent_id.is_empty():
			selected_agent_id = v_id

func _create_vehicle_chassis_node(v_id: String) -> Node3D:
	var root: Node3D = Node3D.new()
	root.name = "Mesh_" + v_id
	
	# Main body box
	var body: MeshInstance3D = MeshInstance3D.new()
	var box: BoxMesh = BoxMesh.new()
	box.size = Vector3(1.9, 1.4, 4.6)
	body.mesh = box
	body.position = Vector3(0.0, 0.7, 0.0)
	body.material_override = vehicle_body_material
	root.add_child(body)
	
	# Rooftop LiDAR puck cylinder
	var lidar_puck: MeshInstance3D = MeshInstance3D.new()
	var cyl: CylinderMesh = CylinderMesh.new()
	cyl.top_radius = 0.2
	cyl.bottom_radius = 0.2
	cyl.height = 0.25
	lidar_puck.mesh = cyl
	lidar_puck.position = Vector3(0.0, 1.55, 0.0)
	var puck_mat: StandardMaterial3D = StandardMaterial3D.new()
	puck_mat.albedo_color = Color(0.05, 0.05, 0.05)
	lidar_puck.material_override = puck_mat
	root.add_child(lidar_puck)
	
	return root

func _process(delta: float) -> void:
	# Advance simulation engine
	var telem: Dictionary = engine_instance.step(delta)
	
	# Sync 3D vehicle visual node transforms with true physical poses
	var agents: Dictionary = engine_instance.fleet_manager.agents
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		if vehicle_mesh_nodes.has(v_id):
			var v_node: Node3D = vehicle_mesh_nodes[v_id]
			v_node.position = agent.true_position
			v_node.rotation = Vector3(0.0, -agent.true_heading_rad + (PI * 0.5), 0.0)
			
	# Update tactical camera
	if vehicle_mesh_nodes.has(selected_agent_id):
		tactical_camera.target_node = vehicle_mesh_nodes[selected_agent_id]
	tactical_camera.update_camera(delta)
	
	# Redraw active trajectory ribbons
	_update_trajectory_visuals()
	
	# Update GUI telemetry
	hud_gui.update_telemetry(telem, selected_agent_id)

func _update_trajectory_visuals() -> void:
	trajectory_viz.clear_canvas()
	
	# Draw active waypoints for each vehicle
	var agents: Dictionary = engine_instance.fleet_manager.agents
	for v_id in agents:
		var agent: VehicleAgent = agents[v_id]
		if agent.current_waypoints.size() >= 2:
			var ribbon_color: Color = Color(0.2, 0.8, 1.0, 0.6) if v_id == selected_agent_id else Color(0.8, 0.8, 0.2, 0.3)
			trajectory_viz.draw_ribbon_2d(agent.current_waypoints, 0.08, 0.9, ribbon_color)

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_QUOTELEFT:
				terminal_gui.visible = not terminal_gui.visible
			KEY_1:
				tactical_camera.set_mode(TacticalCamera.CameraMode.TOP_DOWN)
			KEY_2:
				tactical_camera.set_mode(TacticalCamera.CameraMode.VEHICLE_FOLLOW)
			KEY_3:
				tactical_camera.set_mode(TacticalCamera.CameraMode.FREE_FLY)
			KEY_4:
				tactical_camera.set_mode(TacticalCamera.CameraMode.ORBIT)
			KEY_SPACE:
				var clk = engine_instance.clock
				if clk.mode == clk.ClockMode.PAUSED:
					clk.resume()
				else:
					clk.pause()
			KEY_TAB:
				_cycle_selected_vehicle()

func _cycle_selected_vehicle() -> void:
	var agent_ids: Array = engine_instance.fleet_manager.agents.keys()
	if agent_ids.is_empty():
		return
	var idx: int = agent_ids.find(selected_agent_id)
	var next_idx: int = (idx + 1) % agent_ids.size()
	selected_agent_id = agent_ids[next_idx]
