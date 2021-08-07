# simulation/telemetry_recorder.gd
class_name TelemetryRecorder
extends RefCounted

# High-frequency flight data recorder for cyber-physical simulation analysis.
# Captures full-state time-series telemetry into a memory-efficient ring buffer,
# enabling timeline scrubbing, post-mortem safety verification, and JSON/CSV export.

var buffer_capacity: int = 5000 # 5000 frames @ 50Hz = 100 seconds of continuous telemetry
var frames: Array[Dictionary] = []
var is_recording: bool = true

func _init(p_capacity: int = 5000) -> void:
	buffer_capacity = p_capacity

# Records a simulation snapshot frame
func record_frame(sim_time_sec: float, fleet_telemetry: Dictionary) -> void:
	if not is_recording:
		return
		
	var frame: Dictionary = {
		"sim_time": sim_time_sec,
		"agent_count": fleet_telemetry.get("agent_count", 0),
		"state_counts": fleet_telemetry.get("state_counts", {}).duplicate(),
		"avg_battery_soc": fleet_telemetry.get("average_battery_soc", 1.0),
		"total_fleet_dist": fleet_telemetry.get("total_fleet_distance_m", 0.0),
		"agents": fleet_telemetry.get("agents", []).duplicate(true),
		"network": fleet_telemetry.get("network_metrics", {}).duplicate(),
		"intersection": fleet_telemetry.get("intersection_metrics", {}).duplicate()
	}
	
	frames.append(frame)
	if frames.size() > buffer_capacity:
		frames.pop_front()

# Retrieves frame at specific index
func get_frame(index: int) -> Dictionary:
	if index < 0 or index >= frames.size():
		return {}
	return frames[index]

# Retrieves the latest recorded frame
func get_latest_frame() -> Dictionary:
	if frames.is_empty():
		return {}
	return frames[frames.size() - 1]

# Binary search to find recorded frame closest to target timestamp
func find_frame_at_time(target_time_sec: float) -> Dictionary:
	if frames.is_empty():
		return {}
	if target_time_sec <= frames[0]["sim_time"]:
		return frames[0]
	if target_time_sec >= frames[frames.size() - 1]["sim_time"]:
		return frames[frames.size() - 1]
		
	var low: int = 0
	var high: int = frames.size() - 1
	while low <= high:
		var mid: int = (low + high) / 2
		var t_mid: float = frames[mid]["sim_time"]
		if abs(t_mid - target_time_sec) < 0.01:
			return frames[mid]
		elif t_mid < target_time_sec:
			low = mid + 1
		else:
			high = mid - 1
			
	return frames[clampi(low, 0, frames.size() - 1)]

func clear() -> void:
	frames.clear()

func get_frame_count() -> int:
	return frames.size()

# Exports recorded telemetry frames to structured JSON
func export_to_json(file_path: String) -> bool:
	var export_dict: Dictionary = {
		"version": "1.0.0",
		"frame_count": frames.size(),
		"frames": frames
	}
	var json_str: String = JSON.stringify(export_dict, "  ")
	var file: FileAccess = FileAccess.open(file_path, FileAccess.WRITE)
	if not file:
		return false
	file.store_string(json_str)
	file.close()
	return true

# Exports high-signal vehicle trajectories to CSV
func export_to_csv(file_path: String) -> bool:
	var file: FileAccess = FileAccess.open(file_path, FileAccess.WRITE)
	if not file:
		return false
		
	# Header line
	file.store_line("sim_time,agent_id,pos_x,pos_y,pos_z,speed_mps,heading_rad,battery_soc,rmse")
	
	for frame in frames:
		var t: float = frame["sim_time"]
		var agents_list: Array = frame.get("agents", [])
		for ag in agents_list:
			var p: Vector3 = ag.get("true_position", Vector3.ZERO)
			var row: String = "%.3f,%s,%.3f,%.3f,%.3f,%.2f,%.4f,%.3f,%.4f" % [
				t,
				ag.get("agent_id", ""),
				p.x, p.y, p.z,
				ag.get("true_speed_mps", 0.0),
				ag.get("true_heading_rad", 0.0),
				ag.get("battery_soc", 1.0),
				ag.get("tracking_rmse", 0.0)
			]
			file.store_line(row)
			
	file.close()
	return true
