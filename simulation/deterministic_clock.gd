# simulation/deterministic_clock.gd
class_name DeterministicClock
extends RefCounted

# Microsecond-accurate deterministic simulation clock.
# Enforces exact fixed-step numerical integration independent of host frame rates,
# supporting pause, single-step debugging, and variable time dilation.

enum ClockMode {
	REAL_TIME,
	FIXED_STEP,
	FAST_FORWARD,
	PAUSED
}

var mode: ClockMode = ClockMode.FIXED_STEP
var fixed_delta_sec: float = 0.02 # 50 Hz default physics tick rate
var time_scale: float = 1.0 # 1.0 = real-time, 2.0 = 2x speed, etc.

var current_sim_time_sec: float = 0.0
var total_tick_count: int = 0
var wall_clock_start_ms: int = 0
var accumulated_wall_time_sec: float = 0.0

func _init(p_fixed_delta: float = 0.02) -> void:
	fixed_delta_sec = p_fixed_delta
	wall_clock_start_ms = Time.get_ticks_msec()

# Advances clock by one tick if running, returning the effective delta time
func advance_tick(wall_dt_sec: float = 0.0) -> float:
	match mode:
		ClockMode.PAUSED:
			return 0.0
		ClockMode.FIXED_STEP:
			current_sim_time_sec += fixed_delta_sec
			total_tick_count += 1
			accumulated_wall_time_sec += wall_dt_sec
			return fixed_delta_sec
		ClockMode.REAL_TIME:
			var effective_dt: float = minf(wall_dt_sec * time_scale, fixed_delta_sec * 2.0)
			current_sim_time_sec += effective_dt
			total_tick_count += 1
			accumulated_wall_time_sec += wall_dt_sec
			return effective_dt
		ClockMode.FAST_FORWARD:
			var effective_step: float = fixed_delta_sec * time_scale
			current_sim_time_sec += effective_step
			total_tick_count += 1
			accumulated_wall_time_sec += wall_dt_sec
			return effective_step
			
	return fixed_delta_sec

# Manually steps forward exactly one fixed tick (useful for frame-by-frame analysis)
func step_single_frame() -> float:
	current_sim_time_sec += fixed_delta_sec
	total_tick_count += 1
	return fixed_delta_sec

func pause() -> void:
	mode = ClockMode.PAUSED

func resume() -> void:
	mode = ClockMode.FIXED_STEP

func reset() -> void:
	current_sim_time_sec = 0.0
	total_tick_count = 0
	wall_clock_start_ms = Time.get_ticks_msec()
	accumulated_wall_time_sec = 0.0

# Computes the Real-Time Factor (RTF = sim_time / wall_time)
func get_real_time_factor() -> float:
	if accumulated_wall_time_sec < 1e-4:
		return 1.0
	return current_sim_time_sec / accumulated_wall_time_sec

func get_clock_info() -> Dictionary:
	return {
		"sim_time_sec": current_sim_time_sec,
		"total_ticks": total_tick_count,
		"fixed_delta_sec": fixed_delta_sec,
		"time_scale": time_scale,
		"mode": ClockMode.keys()[mode],
		"real_time_factor": get_real_time_factor()
	}
