# sensors/sensor_base.gd
class_name SensorBase
extends RefCounted

# Abstract base class for cyber-physical sensors in Beacon.
# Implements hardware lifecycle states, update frequency timing,
# diagnostic telemetry, and fault injection hooks.

enum SensorState {
	UNINITIALIZED = 0,
	WARMUP = 1,
	HEALTHY = 2,
	DEGRADED = 3,
	FAILED = 4,
	OFFLINE = 5
}

var sensor_name: String = "generic_sensor"
var state: SensorState = SensorState.UNINITIALIZED

var update_rate_hz: float = 10.0 # Nominal rate
var dt_nominal: float = 0.1
var time_since_last_update: float = 0.0
var total_measurements_produced: int = 0
var dropped_frames_count: int = 0

# Noise and fault simulation flags
var noise_enabled: bool = true
var simulated_fault_active: bool = false
var fault_type: String = "none"

func _init(p_name: String = "generic_sensor", p_rate_hz: float = 10.0) -> void:
	sensor_name = p_name
	set_update_rate(p_rate_hz)
	state = SensorState.HEALTHY

func set_update_rate(hz: float) -> void:
	assert(hz > 0.0, "Update rate must be positive")
	update_rate_hz = hz
	dt_nominal = 1.0 / hz

func is_ready_to_update(delta_time: float) -> bool:
	if state == SensorState.OFFLINE or state == SensorState.FAILED:
		return false
	time_since_last_update += delta_time
	return time_since_last_update >= dt_nominal

func mark_updated() -> void:
	time_since_last_update = fmod(time_since_last_update, dt_nominal)
	total_measurements_produced += 1

func inject_fault(p_fault_type: String) -> void:
	simulated_fault_active = true
	fault_type = p_fault_type
	if p_fault_type == "blackout" or p_fault_type == "power_loss":
		state = SensorState.FAILED
	elif p_fault_type == "high_noise" or p_fault_type == "bias_jump":
		state = SensorState.DEGRADED

func clear_faults() -> void:
	simulated_fault_active = false
	fault_type = "none"
	state = SensorState.HEALTHY

func get_diagnostics() -> Dictionary:
	return {
		"name": sensor_name,
		"state": state,
		"rate_hz": update_rate_hz,
		"measurements": total_measurements_produced,
		"dropped_frames": dropped_frames_count,
		"fault_active": simulated_fault_active,
		"fault_type": fault_type
	}
