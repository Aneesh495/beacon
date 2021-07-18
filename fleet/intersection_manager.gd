# fleet/intersection_manager.gd
class_name IntersectionManager
extends RefCounted

# Autonomous Intersection Management (AIM) spatio-temporal reservation protocol.
# Coordinates unsignalized intersection crossings via V2X reservation tokens,
# guaranteeing conflict-free vehicle trajectories without physical traffic lights.

const GeometryUtils = preload("res://core/spatial/geometry_utils.gd")

# Intersection geometry attributes
var intersection_id: String = "intersection_0"
var center_position: Vector2 = Vector2.ZERO
var conflict_radius_m: float = 12.0 # Radius of active intersection conflict zone
var safety_time_buffer_sec: float = 0.8 # Buffer between consecutive crossing vehicles

# Internal sub-tiles or crossing trajectories
# Key: reservation_id -> Dictionary {agent_id, entry_time, exit_time, trajectory_path, speed}
var active_reservations: Dictionary = {}
var completed_reservation_count: int = 0
var rejected_request_count: int = 0

func _init(p_id: String = "intersection_0", p_center: Vector2 = Vector2.ZERO, p_radius: float = 12.0) -> void:
	intersection_id = p_id
	center_position = p_center
	conflict_radius_m = p_radius

# Requests a reservation to cross the intersection.
# request: Dictionary {
#   "agent_id": String,
#   "current_time": float,
#   "eta_to_entry": float,
#   "trajectory": Array[Vector2], # Planned path through intersection
#   "target_speed": float,
#   "is_emergency": bool
# }
# Returns Dictionary: {
#   "granted": bool,
#   "reservation_id": String,
#   "entry_time": float,
#   "exit_time": float,
#   "suggested_delay_sec": float
# }
func request_reservation(request: Dictionary) -> Dictionary:
	var agent_id: String = request.get("agent_id", "")
	var current_time: float = request.get("current_time", 0.0)
	var eta: float = request.get("eta_to_entry", 2.0)
	var trajectory: Array[Vector2] = request.get("trajectory", [])
	var target_speed: float = maxf(1.0, request.get("target_speed", 10.0))
	var is_emergency: bool = request.get("is_emergency", false)
	
	if trajectory.size() < 2:
		return {"granted": false, "reservation_id": "", "reason": "Empty trajectory"}
		
	# Compute path length inside conflict radius
	var path_in_zone_length: float = 0.0
	for i in range(trajectory.size() - 1):
		path_in_zone_length += trajectory[i].distance_to(trajectory[i + 1])
		
	var crossing_duration: float = path_in_zone_length / target_speed
	var requested_entry_time: float = current_time + eta
	var requested_exit_time: float = requested_entry_time + crossing_duration
	
	# Verify space-time conflicts with all currently active reservations
	var conflict_found: bool = false
	var earliest_available_time: float = requested_entry_time
	
	for res_id in active_reservations:
		var existing: Dictionary = active_reservations[res_id]
		if existing["agent_id"] == agent_id:
			continue
			
		var ex_entry: float = existing["entry_time"] - safety_time_buffer_sec
		var ex_exit: float = existing["exit_time"] + safety_time_buffer_sec
		
		# Check time interval overlap
		if requested_entry_time < ex_exit and requested_exit_time > ex_entry:
			# Check spatial trajectory conflict between paths
			if _trajectories_conflict(trajectory, existing["trajectory"], 3.5):
				conflict_found = true
				earliest_available_time = maxf(earliest_available_time, existing["exit_time"] + safety_time_buffer_sec)
				
	if not conflict_found:
		var res_id: String = "res_%s_%d" % [agent_id, int(requested_entry_time * 100)]
		active_reservations[res_id] = {
			"reservation_id": res_id,
			"agent_id": agent_id,
			"entry_time": requested_entry_time,
			"exit_time": requested_exit_time,
			"trajectory": trajectory,
			"target_speed": target_speed,
			"is_emergency": is_emergency
		}
		return {
			"granted": true,
			"reservation_id": res_id,
			"entry_time": requested_entry_time,
			"exit_time": requested_exit_time,
			"suggested_delay_sec": 0.0
		}
	else:
		rejected_request_count += 1
		var suggested_delay: float = maxf(0.1, earliest_available_time - requested_entry_time)
		return {
			"granted": false,
			"reservation_id": "",
			"suggested_delay_sec": suggested_delay,
			"earliest_entry_time": earliest_available_time,
			"reason": "Trajectory conflict detected"
		}

# Checks if two spatial trajectories come within min_dist of each other
func _trajectories_conflict(path_a: Array[Vector2], path_b: Array[Vector2], min_dist: float) -> bool:
	var min_dist_sq: float = min_dist * min_dist
	for pt_a in path_a:
		for pt_b in path_b:
			if pt_a.distance_squared_to(pt_b) < min_dist_sq:
				return true
	return false

# Updates simulation clock and purges completed reservations
func update_clock(sim_time_sec: float) -> void:
	var to_remove: Array[String] = []
	for res_id in active_reservations:
		var res: Dictionary = active_reservations[res_id]
		if sim_time_sec > (res["exit_time"] + 1.0):
			to_remove.append(res_id)
			
	for res_id in to_remove:
		active_reservations.erase(res_id)
		completed_reservation_count += 1

# Cancels an existing reservation for an agent
func cancel_reservation(reservation_id: String) -> bool:
	if active_reservations.has(reservation_id):
		active_reservations.erase(reservation_id)
		return true
	return false

# Queries status of an agent reservation
func is_agent_reserved(p_agent_id: String) -> bool:
	for res_id in active_reservations:
		if active_reservations[res_id]["agent_id"] == p_agent_id:
			return true
	return false

func get_metrics() -> Dictionary:
	return {
		"intersection_id": intersection_id,
		"active_reservations": active_reservations.size(),
		"completed_reservations": completed_reservation_count,
		"rejected_requests": rejected_request_count
	}
