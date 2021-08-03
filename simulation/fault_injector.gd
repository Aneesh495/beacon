# simulation/fault_injector.gd
class_name FaultInjector
extends RefCounted

# Chaos engineering fault injection substrate for autonomous cyber-physical fleets.
# Injects controlled sensor anomalies, actuator degradations, network jamming,
# and unexpected environmental obstacles to verify system fault tolerance.

const VehicleAgent = preload("res://fleet/vehicle_agent.gd")
const V2xMeshNetwork = preload("res://fleet/v2x_mesh_network.gd")

# Active scheduled faults: Array of Dictionary
# {fault_id, target_type, target_id, fault_type, end_time_sec, original_params}
var active_faults: Array[Dictionary] = []
var fault_history_log: Array[Dictionary] = []

# Injects a sensor anomaly into a target vehicle agent
func inject_sensor_fault(
	agent: VehicleAgent,
	sensor_name: String, # "gnss", "imu", "lidar", "wheel_odom"
	fault_type: String,  # "blackout", "multipath", "bias_jump", "high_noise"
	duration_sec: float,
	current_time_sec: float
) -> String:
	var fault_id: String = "fault_sensor_%s_%s_%d" % [agent.agent_id, sensor_name, int(current_time_sec * 100)]
	
	match sensor_name:
		"gnss":
			if agent.sensor_gnss:
				agent.sensor_gnss.inject_fault(fault_type)
		"imu":
			if agent.sensor_imu:
				agent.sensor_imu.inject_fault(fault_type)
		"lidar":
			if agent.sensor_lidar:
				agent.sensor_lidar.inject_fault(fault_type)
		"wheel_odom":
			if agent.sensor_wheel_odom:
				agent.sensor_wheel_odom.inject_fault(fault_type)
				
	var entry: Dictionary = {
		"fault_id": fault_id,
		"category": "sensor",
		"target_agent": agent.agent_id,
		"sensor_name": sensor_name,
		"fault_type": fault_type,
		"start_time_sec": current_time_sec,
		"end_time_sec": current_time_sec + duration_sec,
		"agent_ref": agent
	}
	
	active_faults.append(entry)
	fault_history_log.append(entry)
	return fault_id

# Injects RF mesh channel degradation (e.g. urban jamming or severe multipath storm)
func inject_network_jamming(
	network: V2xMeshNetwork,
	drop_rate: float,
	duration_sec: float,
	current_time_sec: float
) -> String:
	var fault_id: String = "fault_net_jam_%d" % int(current_time_sec * 100)
	var prev_drop: float = network.packet_drop_rate_override
	network.packet_drop_rate_override = drop_rate
	
	var entry: Dictionary = {
		"fault_id": fault_id,
		"category": "network",
		"fault_type": "jamming",
		"drop_rate": drop_rate,
		"start_time_sec": current_time_sec,
		"end_time_sec": current_time_sec + duration_sec,
		"network_ref": network,
		"original_drop_rate": prev_drop
	}
	
	active_faults.append(entry)
	fault_history_log.append(entry)
	return fault_id

# Advances fault timers and automatically clears expired anomalies
func update_faults(current_time_sec: float) -> int:
	var remaining_faults: Array[Dictionary] = []
	var cleared_count: int = 0
	
	for fault in active_faults:
		if current_time_sec >= fault["end_time_sec"]:
			_clear_fault(fault)
			cleared_count += 1
		else:
			remaining_faults.append(fault)
			
	active_faults = remaining_faults
	return cleared_count

func _clear_fault(fault: Dictionary) -> void:
	match fault.get("category", ""):
		"sensor":
			var agent: VehicleAgent = fault.get("agent_ref", null)
			if agent:
				var s_name: String = fault.get("sensor_name", "")
				match s_name:
					"gnss":
						if agent.sensor_gnss:
							agent.sensor_gnss.clear_fault()
					"imu":
						if agent.sensor_imu:
							agent.sensor_imu.clear_fault()
					"lidar":
						if agent.sensor_lidar:
							agent.sensor_lidar.clear_fault()
					"wheel_odom":
						if agent.sensor_wheel_odom:
							agent.sensor_wheel_odom.clear_fault()
		"network":
			var network: V2xMeshNetwork = fault.get("network_ref", null)
			if network:
				network.packet_drop_rate_override = fault.get("original_drop_rate", -1.0)

# Clears all currently active faults immediately
func clear_all_active_faults() -> void:
	for fault in active_faults:
		_clear_fault(fault)
	active_faults.clear()

func get_active_fault_count() -> int:
	return active_faults.size()

func get_fault_summary() -> Dictionary:
	return {
		"active_faults_count": active_faults.size(),
		"total_injected_faults": fault_history_log.size(),
		"active_faults": active_faults,
		"history": fault_history_log
	}
