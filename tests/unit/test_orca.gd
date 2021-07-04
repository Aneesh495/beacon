# tests/unit/test_orca.gd
class_name TestOrca
extends "res://tests/test_base.gd"

# Unit verification suite for ORCA / RVO2 reciprocal collision avoidance.

const OrcaCollisionAvoidance = preload("res://planning/orca_collision_avoidance.gd")

func before_each() -> void:
	reset_test()

func test_orca_head_on_deflection() -> Dictionary:
	# Agent A at (-5, 0) moving East at 2 m/s
	var pos_a: Vector2 = Vector2(-5.0, 0.0)
	var vel_a: Vector2 = Vector2(2.0, 0.0)
	var radius_a: float = 1.0
	
	# Agent B at (5, 0) moving West at 2 m/s (direct head-on collision course)
	var pos_b: Vector2 = Vector2(5.0, 0.0)
	var vel_b: Vector2 = Vector2(-2.0, 0.0)
	var radius_b: float = 1.0
	
	var line: OrcaCollisionAvoidance.OrcaLine = OrcaCollisionAvoidance.compute_orca_halfplane(
		pos_a, vel_a, radius_a, pos_b, vel_b, radius_b, 4.0
	)
	
	assert_not_null(line)
	assert_true(line.direction.length() > 0.9, "Direction should be unit vector")
	
	# Solve LP for optimal deflection velocity away from straight ahead
	var safe_vel: Vector2 = OrcaCollisionAvoidance.solve_2d_lp([line], 3.0, vel_a)
	
	# Safe velocity must deviate laterally (non-zero Y velocity) to avoid head-on crash
	assert_true(abs(safe_vel.y) > 0.1, "ORCA must deflect velocity laterally to avoid head-on crash")
	
	return get_results()
