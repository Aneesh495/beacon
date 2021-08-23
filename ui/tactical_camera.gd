# ui/tactical_camera.gd
class_name TacticalCamera
extends Camera3D

# Advanced 3D tactical camera controller for digital twin inspection.
# Supports 4 distinct viewing modes:
# 1. FREE_FLY: WASD keyboard panning + mouse yaw/pitch inspection
# 2. VEHICLE_FOLLOW: Third-person chase camera locked behind active agent
# 3. TOP_DOWN: Orthographic / high-altitude aerial tactical overview
# 4. ORBIT: Orbital rotational inspection around selected point of interest

enum CameraMode {
	FREE_FLY,
	VEHICLE_FOLLOW,
	TOP_DOWN,
	ORBIT
}

var mode: CameraMode = CameraMode.TOP_DOWN

# Target tracking parameters
var target_node: Node3D = null
var target_position: Vector3 = Vector3.ZERO
var follow_distance_m: float = 18.0
var follow_height_m: float = 8.0
var follow_smoothing: float = 6.0

# Free fly motion
var fly_speed_mps: float = 30.0
var mouse_sensitivity: float = 0.003
var yaw_rad: float = 0.0
var pitch_rad: float = -0.7

# Orbit mode
var orbit_angle_rad: float = 0.0
var orbit_speed_radps: float = 0.2
var orbit_radius_m: float = 25.0
var orbit_height_m: float = 12.0

func _ready() -> void:
	current = true
	_apply_mode_perspective()

func set_mode(new_mode: CameraMode) -> void:
	mode = new_mode
	_apply_mode_perspective()

func _apply_mode_perspective() -> void:
	match mode:
		CameraMode.TOP_DOWN:
			position = Vector3(0.0, 75.0, 0.0)
			rotation = Vector3(-PI * 0.5, 0.0, 0.0)
		CameraMode.VEHICLE_FOLLOW:
			pass
		CameraMode.FREE_FLY:
			pass
		CameraMode.ORBIT:
			pass

func update_camera(dt: float) -> void:
	match mode:
		CameraMode.TOP_DOWN:
			# Gentle keyboard panning in top-down view
			var pan: Vector3 = Vector3.ZERO
			if Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP):
				pan.z -= 1.0
			if Input.is_key_pressed(KEY_S) or Input.is_key_pressed(KEY_DOWN):
				pan.z += 1.0
			if Input.is_key_pressed(KEY_A) or Input.is_key_pressed(KEY_LEFT):
				pan.x -= 1.0
			if Input.is_key_pressed(KEY_D) or Input.is_key_pressed(KEY_RIGHT):
				pan.x += 1.0
			position += pan.normalized() * fly_speed_mps * dt
			
		CameraMode.VEHICLE_FOLLOW:
			if target_node:
				var t_pos: Vector3 = target_node.global_position
				var t_forward: Vector3 = -target_node.global_transform.basis.z
				var desired_pos: Vector3 = t_pos - (t_forward * follow_distance_m) + Vector3(0.0, follow_height_m, 0.0)
				global_position = global_position.lerp(desired_pos, clampf(follow_smoothing * dt, 0.0, 1.0))
				look_at(t_pos + Vector3(0.0, 1.5, 0.0), Vector3.UP)
			elif target_position != Vector3.ZERO:
				var desired: Vector3 = target_position + Vector3(0.0, follow_height_m, follow_distance_m)
				global_position = global_position.lerp(desired, clampf(follow_smoothing * dt, 0.0, 1.0))
				look_at(target_position, Vector3.UP)
				
		CameraMode.ORBIT:
			orbit_angle_rad += orbit_speed_radps * dt
			var center: Vector3 = target_node.global_position if target_node else target_position
			var orbit_pos: Vector3 = center + Vector3(
				cos(orbit_angle_rad) * orbit_radius_m,
				orbit_height_m,
				sin(orbit_angle_rad) * orbit_radius_m
			)
			global_position = orbit_pos
			look_at(center + Vector3(0.0, 1.0, 0.0), Vector3.UP)
			
		CameraMode.FREE_FLY:
			var move_dir: Vector3 = Vector3.ZERO
			if Input.is_key_pressed(KEY_W):
				move_dir -= transform.basis.z
			if Input.is_key_pressed(KEY_S):
				move_dir += transform.basis.z
			if Input.is_key_pressed(KEY_A):
				move_dir -= transform.basis.x
			if Input.is_key_pressed(KEY_D):
				move_dir += transform.basis.x
			if Input.is_key_pressed(KEY_E):
				move_dir += Vector3.UP
			if Input.is_key_pressed(KEY_Q):
				move_dir -= Vector3.UP
				
			position += move_dir.normalized() * fly_speed_mps * dt
