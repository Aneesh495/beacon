# tests/unit/test_lanelet_network.gd
class_name TestLaneletNetwork
extends "res://tests/test_base.gd"

# Unit verification suite for LaneletSegment, LaneletNetwork, Frenet transformations, and A* route planning.

const LaneletSegment = preload("res://maps/lanelet_segment.gd")
const LaneletNetwork = preload("res://maps/lanelet_network.gd")

func before_each() -> void:
	reset_test()

func test_frenet_coordinate_conversions() -> Dictionary:
	var seg: LaneletSegment = LaneletSegment.new(1, 15.0)
	# 50-meter straight lane along X-axis, width = 4 meters (y from -2 to +2)
	seg.left_boundary = [Vector2(0.0, 2.0), Vector2(50.0, 2.0)]
	seg.right_boundary = [Vector2(0.0, -2.0), Vector2(50.0, -2.0)]
	seg.compute_centerline(10)
	
	assert_almost_equal(seg.total_length, 50.0, 1e-4, "Segment total length check")
	
	# Point at (25.0, 1.5): s should be 25.0, d should be +1.5 (left of centerline)
	var frenet1: Vector2 = seg.cartesian_to_frenet(Vector2(25.0, 1.5))
	assert_almost_equal(frenet1.x, 25.0, 1e-4, "Frenet s longitudinal check")
	assert_almost_equal(frenet1.y, 1.5, 1e-4, "Frenet d lateral check")
	
	# Roundtrip back to Cartesian
	var cart1: Vector2 = seg.frenet_to_cartesian(frenet1.x, frenet1.y)
	assert_vector2_almost_equal(cart1, Vector2(25.0, 1.5), 1e-4, "Frenet roundtrip check")
	
	# Point at (10.0, -1.0): s should be 10.0, d should be -1.0 (right of centerline)
	var frenet2: Vector2 = seg.cartesian_to_frenet(Vector2(10.0, -1.0))
	assert_almost_equal(frenet2.x, 10.0, 1e-4)
	assert_almost_equal(frenet2.y, -1.0, 1e-4)
	
	return get_results()

func test_topological_network_and_routing() -> Dictionary:
	var network: LaneletNetwork = LaneletNetwork.new()
	
	# Create segment 1: [0, 0] -> [50, 0]
	var seg1: LaneletSegment = LaneletSegment.new(1)
	seg1.left_boundary = [Vector2(0.0, 2.0), Vector2(50.0, 2.0)]
	seg1.right_boundary = [Vector2(0.0, -2.0), Vector2(50.0, -2.0)]
	seg1.compute_centerline(5)
	seg1.successor_ids = PackedInt32Array([2])
	network.add_segment(seg1)
	
	# Create segment 2: [50, 0] -> [100, 0]
	var seg2: LaneletSegment = LaneletSegment.new(2)
	seg2.left_boundary = [Vector2(50.0, 2.0), Vector2(100.0, 2.0)]
	seg2.right_boundary = [Vector2(50.0, -2.0), Vector2(100.0, -2.0)]
	seg2.compute_centerline(5)
	seg2.predecessor_ids = PackedInt32Array([1])
	seg2.successor_ids = PackedInt32Array([3])
	network.add_segment(seg2)
	
	# Create segment 3: [100, 0] -> [150, 0]
	var seg3: LaneletSegment = LaneletSegment.new(3)
	seg3.left_boundary = [Vector2(100.0, 2.0), Vector2(150.0, 2.0)]
	seg3.right_boundary = [Vector2(100.0, -2.0), Vector2(150.0, -2.0)]
	seg3.compute_centerline(5)
	seg3.predecessor_ids = PackedInt32Array([2])
	network.add_segment(seg3)
	
	# Spatial index lookup
	var found_seg: LaneletSegment = network.find_closest_lanelet(Vector2(75.0, 0.5))
	assert_not_null(found_seg, "Closest lanelet must be found")
	assert_equal(found_seg.id, 2, "Expected segment 2 at x=75")
	
	# A* route planning from 1 to 3
	var route: PackedInt32Array = network.plan_topological_route(1, 3)
	assert_equal(route.size(), 3, "Route should contain 3 segments")
	assert_equal(route[0], 1)
	assert_equal(route[1], 2)
	assert_equal(route[2], 3)
	
	return get_results()

func test_intersection_conflict_detection() -> Dictionary:
	# North-South segment intersecting East-West segment at origin
	var seg_ew: LaneletSegment = LaneletSegment.new(10)
	seg_ew.left_boundary = [Vector2(-20.0, 2.0), Vector2(20.0, 2.0)]
	seg_ew.right_boundary = [Vector2(-20.0, -2.0), Vector2(20.0, -2.0)]
	seg_ew.compute_centerline(5)
	
	var seg_ns: LaneletSegment = LaneletSegment.new(20)
	seg_ns.left_boundary = [Vector2(-2.0, -20.0), Vector2(-2.0, 20.0)]
	seg_ns.right_boundary = [Vector2(2.0, -20.0), Vector2(2.0, 20.0)]
	seg_ns.compute_centerline(5)
	
	var conflicts: Array[Vector2] = LaneletNetwork.find_conflict_points(seg_ew, seg_ns)
	assert_equal(conflicts.size(), 1, "Expected exactly 1 conflict point at intersection")
	assert_vector2_almost_equal(conflicts[0], Vector2(0.0, 0.0), 1e-4, "Conflict should be at origin")
	
	return get_results()
