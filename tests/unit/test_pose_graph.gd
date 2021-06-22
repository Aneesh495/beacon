# tests/unit/test_pose_graph.gd
class_name TestPoseGraph
extends "res://tests/test_base.gd"

# Unit verification suite for PoseGraph SE(2) non-linear least squares optimization.

const PoseGraph = preload("res://estimation/pose_graph.gd")

func before_each() -> void:
	reset_test()

func test_pose_graph_square_loop_closure() -> Dictionary:
	var graph: PoseGraph = PoseGraph.new()
	graph.fixed_node_id = 0
	
	# Synthetic ground truth square loop: (0,0) -> (10,0) -> (10,10) -> (0,10) -> (0,0)
	# Inject initial dead reckoning drift on nodes
	graph.add_node(0, Vector2(0.0, 0.0), 0.0)
	graph.add_node(1, Vector2(10.2, 0.1), 0.02)
	graph.add_node(2, Vector2(10.4, 10.3), 1.59)
	graph.add_node(3, Vector2(-0.3, 10.5), 3.16)
	
	# Add between-factors (relative odometry of 10m segments with 90 deg turns)
	graph.add_factor(0, 1, Vector2(10.0, 0.0), 0.0, 1.0)
	graph.add_factor(1, 2, Vector2(10.0, 0.0), PI * 0.5, 1.0)
	graph.add_factor(2, 3, Vector2(10.0, 0.0), PI * 0.5, 1.0)
	# Loop closure factor between node 3 and node 0: closing the loop back to origin!
	graph.add_factor(3, 0, Vector2(10.0, 0.0), PI * 0.5, 1.0)
	
	var initial_error: float = graph.compute_total_error()
	assert_true(initial_error > 0.0, "Initial graph should have positive error")
	
	# Run Gauss-Newton optimization
	var final_error: float = graph.optimize_gauss_newton(6)
	
	# Final error must decrease significantly
	assert_true(final_error < initial_error, "Optimization must reduce total graph error")
	
	# Verify that node 1 converged close to (10, 0)
	assert_almost_equal(graph.nodes[1].pos.x, 10.0, 0.2, "Node 1 X convergence")
	assert_almost_equal(graph.nodes[1].pos.y, 0.0, 0.2, "Node 1 Y convergence")
	
	return get_results()
