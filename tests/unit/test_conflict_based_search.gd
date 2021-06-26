# tests/unit/test_conflict_based_search.gd
class_name TestConflictBasedSearch
extends "res://tests/test_base.gd"

# Unit verification suite for Conflict-Based Search (CBS) multi-agent path finding.

const ConflictBasedSearch = preload("res://planning/conflict_based_search.gd")

func before_each() -> void:
	reset_test()

func test_cbs_two_agent_crossing() -> Dictionary:
	var cbs: ConflictBasedSearch = ConflictBasedSearch.new(20, 20)
	
	# Agent 1 starts at (0, 5) heading to (8, 5)
	# Agent 2 starts at (8, 5) heading to (0, 5) - head-on collision on narrow corridor!
	# Give them room to step sideways into y=4 or y=6
	var starts: Dictionary = {
		1: Vector2i(0, 5),
		2: Vector2i(8, 5)
	}
	var goals: Dictionary = {
		1: Vector2i(8, 5),
		2: Vector2i(0, 5)
	}
	
	var solution: Dictionary = cbs.solve_cbs(starts, goals, 150)
	assert_true(solution.has(1) and solution.has(2), "CBS should find collision-free multi-agent solution")
	
	var path1: Array[Vector2i] = solution[1]
	var path2: Array[Vector2i] = solution[2]
	
	# Verify start and goal arrival
	assert_equal(path1[0], Vector2i(0, 5))
	assert_equal(path1[path1.size() - 1], Vector2i(8, 5))
	
	assert_equal(path2[0], Vector2i(8, 5))
	assert_equal(path2[path2.size() - 1], Vector2i(0, 5))
	
	# Verify that no vertex collision occurred at any time step t
	var max_len: int = maxi(path1.size(), path2.size())
	for t in range(max_len):
		var pos1: Vector2i = path1[min(t, path1.size() - 1)]
		var pos2: Vector2i = path2[min(t, path2.size() - 1)]
		assert_false(pos1 == pos2, "Zero vertex collisions guaranteed at t=%d" % t)
		
	# Verify that no edge swapping collision occurred
	for t in range(1, max_len):
		var p1_prev: Vector2i = path1[min(t - 1, path1.size() - 1)]
		var p1_cur: Vector2i = path1[min(t, path1.size() - 1)]
		var p2_prev: Vector2i = path2[min(t - 1, path2.size() - 1)]
		var p2_cur: Vector2i = path2[min(t, path2.size() - 1)]
		
		assert_false(p1_prev == p2_cur and p2_prev == p1_cur, "Zero edge swapping collision at t=%d" % t)
		
	return get_results()

func test_cbs_orthogonal_intersection() -> Dictionary:
	var cbs: ConflictBasedSearch = ConflictBasedSearch.new(20, 20)
	
	# Agent 1 crosses East-West: (2, 10) -> (18, 10)
	# Agent 2 crosses North-South: (10, 2) -> (10, 18)
	var starts: Dictionary = {
		1: Vector2i(2, 10),
		2: Vector2i(10, 2)
	}
	var goals: Dictionary = {
		1: Vector2i(18, 10),
		2: Vector2i(10, 18)
	}
	
	var solution: Dictionary = cbs.solve_cbs(starts, goals, 100)
	assert_true(solution.has(1) and solution.has(2), "Intersection CBS solution found")
	
	return get_results()
