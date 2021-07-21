# fleet/v2x_mesh_network.gd
class_name V2xMeshNetwork
extends RefCounted

# Ad-hoc Vehicle-to-Everything (V2X / C-V2X Sidelink) mesh communication simulator.
# Models physical RF path loss, multipath fading, stochastic packet drops,
# transmission queue latency, and multi-hop safety message dissemination.

# RF channel parameters
var max_radio_range_m: float = 250.0
var reference_distance_m: float = 100.0 # d0 for exponential drop curve
var path_loss_exponent: float = 2.8 # Urban vehicular environment
var base_latency_sec: float = 0.010 # 10 ms nominal base latency
var packet_drop_rate_override: float = -1.0 # Force fault drop rate if >= 0.0

# Registered transceiver nodes: node_id -> Vector3 position
var active_nodes: Dictionary = {}

# In-flight packet queue: Array of Dictionary
# {sender_id, target_id (empty for broadcast), msg_type, payload, delivery_time_sec}
var in_flight_packets: Array[Dictionary] = []

# Node mailboxes: node_id -> Array of Dictionary
var mailboxes: Dictionary = {}

# Network telemetry statistics
var total_packets_sent: int = 0
var total_packets_delivered: int = 0
var total_packets_dropped_range: int = 0
var total_packets_dropped_fading: int = 0
var accumulated_latency_sec: float = 0.0

var current_sim_time_sec: float = 0.0

func _init(p_range: float = 250.0, p_base_latency: float = 0.010) -> void:
	max_radio_range_m = p_range
	base_latency_sec = p_base_latency

# Registers a new transceiver node in the network mesh
func register_node(node_id: String, initial_pos: Vector3 = Vector3.ZERO) -> void:
	active_nodes[node_id] = initial_pos
	mailboxes[node_id] = []

# Unregisters a node when it disconnects or shuts down
func unregister_node(node_id: String) -> void:
	active_nodes.erase(node_id)
	mailboxes.erase(node_id)

# Updates a node's physical spatial position for RF link calculations
func update_node_position(node_id: String, pos_3d: Vector3) -> void:
	if active_nodes.has(node_id):
		active_nodes[node_id] = pos_3d

# Broadcasts a safety message to all nodes within communication range
func broadcast_message(sender_id: String, msg_type: String, payload: Dictionary) -> int:
	if not active_nodes.has(sender_id):
		return 0
		
	var sender_pos: Vector3 = active_nodes[sender_id]
	var scheduled_count: int = 0
	total_packets_sent += 1
	
	for target_id in active_nodes:
		if target_id == sender_id:
			continue
			
		var target_pos: Vector3 = active_nodes[target_id]
		var dist: float = sender_pos.distance_to(target_pos)
		
		if dist > max_radio_range_m:
			total_packets_dropped_range += 1
			continue
			
		# Calculate packet loss probability using log-distance Rayleigh fading model
		var drop_prob: float = 0.0
		if packet_drop_rate_override >= 0.0:
			drop_prob = packet_drop_rate_override
		else:
			var normalized_dist: float = dist / reference_distance_m
			drop_prob = clampf(1.0 - exp(-pow(normalized_dist, path_loss_exponent) * 0.15), 0.0, 0.95)
			
		if randf() < drop_prob:
			total_packets_dropped_fading += 1
			continue
			
		# Stochastic latency jitter ~ [base - 2ms, base + 8ms]
		var jitter: float = randf_range(-0.002, 0.008)
		var latency: float = maxf(0.001, base_latency_sec + jitter + (dist / 300000000.0))
		var delivery_time: float = current_sim_time_sec + latency
		
		in_flight_packets.append({
			"sender_id": sender_id,
			"target_id": target_id,
			"msg_type": msg_type,
			"payload": payload,
			"delivery_time_sec": delivery_time,
			"latency_sec": latency
		})
		scheduled_count += 1
		
	return scheduled_count

# Sends a targeted unicast message to a specific recipient node
func send_unicast(sender_id: String, target_id: String, msg_type: String, payload: Dictionary) -> bool:
	if not active_nodes.has(sender_id) or not active_nodes.has(target_id):
		return false
		
	total_packets_sent += 1
	var sender_pos: Vector3 = active_nodes[sender_id]
	var target_pos: Vector3 = active_nodes[target_id]
	var dist: float = sender_pos.distance_to(target_pos)
	
	if dist > max_radio_range_m:
		total_packets_dropped_range += 1
		return false
		
	var drop_prob: float = packet_drop_rate_override if packet_drop_rate_override >= 0.0 else (
		clampf(1.0 - exp(-pow(dist / reference_distance_m, path_loss_exponent) * 0.1), 0.0, 0.9)
	)
	
	if randf() < drop_prob:
		total_packets_dropped_fading += 1
		return false
		
	var latency: float = maxf(0.001, base_latency_sec + randf_range(-0.002, 0.005))
	in_flight_packets.append({
		"sender_id": sender_id,
		"target_id": target_id,
		"msg_type": msg_type,
		"payload": payload,
		"delivery_time_sec": current_sim_time_sec + latency,
		"latency_sec": latency
	})
	return true

# Advances network clock and routes arrived packets to recipient mailboxes
func step_network(dt: float) -> void:
	current_sim_time_sec += dt
	var remaining_packets: Array[Dictionary] = []
	
	for pkt in in_flight_packets:
		if current_sim_time_sec >= pkt["delivery_time_sec"]:
			var target_id: String = pkt["target_id"]
			if mailboxes.has(target_id):
				mailboxes[target_id].append({
					"sender_id": pkt["sender_id"],
					"msg_type": pkt["msg_type"],
					"payload": pkt["payload"],
					"received_time_sec": current_sim_time_sec,
					"latency_sec": pkt["latency_sec"]
				})
				total_packets_delivered += 1
				accumulated_latency_sec += pkt["latency_sec"]
		else:
			remaining_packets.append(pkt)
			
	in_flight_packets = remaining_packets

func poll_inbox(node_id: String) -> Array[Dictionary]:
	var result: Array[Dictionary] = []
	if not mailboxes.has(node_id):
		return result
	var raw_list: Array = mailboxes[node_id]
	for item in raw_list:
		result.append(item)
	raw_list.clear()
	return result

# Diagnostics and performance metrics
func get_network_metrics() -> Dictionary:
	var avg_latency: float = 0.0
	if total_packets_delivered > 0:
		avg_latency = (accumulated_latency_sec / float(total_packets_delivered)) * 1000.0 # ms
		
	var total_drops: int = total_packets_dropped_range + total_packets_dropped_fading
	var drop_rate: float = float(total_drops) / float(maxi(1, total_packets_sent))
	
	return {
		"active_nodes": active_nodes.size(),
		"packets_in_flight": in_flight_packets.size(),
		"total_sent": total_packets_sent,
		"total_delivered": total_packets_delivered,
		"total_dropped_range": total_packets_dropped_range,
		"total_dropped_fading": total_packets_dropped_fading,
		"average_latency_ms": avg_latency,
		"packet_drop_rate": drop_rate
	}
