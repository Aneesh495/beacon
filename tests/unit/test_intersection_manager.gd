# tests/unit/test_intersection_manager.gd
class_name TestIntersectionManager
extends "res://tests/test_base.gd"

# Unit verification suite for V2X Autonomous Intersection Management (AIM) protocol.

const IntersectionManager = preload("res://fleet/intersection_manager.gd")

func before_each() -> void:
	reset_test()

func test_intersection_conflict_free_reservations() -> Dictionary:
	var im: IntersectionManager = IntersectionManager.new("im_test", Vector2.ZERO, 15.0)
	
	# Agent 1 approaches from South traveling North (X = 0, Y = -20 -> +20)
	var path_a: Array[Vector2] = [Vector2(0.0, -15.0), Vector2(0.0, 0.0), Vector2(0.0, 15.0)]
	var req_a: Dictionary = {
		"agent_id": "car_north",
		"current_time": 0.0,
		"eta_to_entry": 2.0,
		"trajectory": path_a,
		"target_speed": 10.0,
		"is_emergency": false
	}
	
	var res_a: Dictionary = im.request_reservation(req_a)
	assert_true(res_a["granted"], "First reservation should be granted")
	assert_equal(im.active_reservations.size(), 1, "One active reservation")
	
	# Agent 2 approaches from West traveling East (X = -20 -> +20, Y = 0) at the EXACT SAME TIME
	var path_b: Array[Vector2] = [Vector2(-15.0, 0.0), Vector2(0.0, 0.0), Vector2(15.0, 0.0)]
	var req_b: Dictionary = {
		"agent_id": "car_east",
		"current_time": 0.0,
		"eta_to_entry": 2.0, # Same arrival time -> conflict at (0, 0)
		"trajectory": path_b,
		"target_speed": 10.0,
		"is_emergency": false
	}
	
	var res_b: Dictionary = im.request_reservation(req_b)
	assert_true(not res_b["granted"], "Conflicting reservation should be rejected")
	assert_true(res_b["suggested_delay_sec"] > 0.0, "Should suggest delay to avoid collision")
	
	# Now update clock past Agent 1's exit time
	im.update_clock(res_a["exit_time"] + 2.0)
	assert_equal(im.active_reservations.size(), 0, "Reservation should be cleared after vehicle exits")
	
	# Retrying Agent 2 should now succeed
	var req_b2: Dictionary = {
		"agent_id": "car_east",
		"current_time": res_a["exit_time"] + 2.0,
		"eta_to_entry": 1.0,
		"trajectory": path_b,
		"target_speed": 10.0,
		"is_emergency": false
	}
	var res_b2: Dictionary = im.request_reservation(req_b2)
	assert_true(res_b2["granted"], "Subsequent reservation should be granted after clearing")
	
	return get_results()

func test_non_conflicting_parallel_paths() -> Dictionary:
	var im: IntersectionManager = IntersectionManager.new("im_test", Vector2.ZERO, 15.0)
	
	# Northbound lane at X = 2.0
	var path_north: Array[Vector2] = [Vector2(2.0, -15.0), Vector2(2.0, 15.0)]
	# Southbound lane at X = -2.0 (parallel, separated by 4m > 3.5m min threshold)
	var path_south: Array[Vector2] = [Vector2(-2.0, 15.0), Vector2(-2.0, -15.0)]
	
	var res_1: Dictionary = im.request_reservation({
		"agent_id": "v_north", "current_time": 0.0, "eta_to_entry": 1.0,
		"trajectory": path_north, "target_speed": 10.0
	})
	var res_2: Dictionary = im.request_reservation({
		"agent_id": "v_south", "current_time": 0.0, "eta_to_entry": 1.0,
		"trajectory": path_south, "target_speed": 10.0
	})
	
	assert_true(res_1["granted"], "Northbound granted")
	assert_true(res_2["granted"], "Parallel Southbound should also be granted concurrently")
	assert_equal(im.active_reservations.size(), 2, "Both active concurrently")
	
	return get_results()
