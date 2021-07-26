# tests/unit/test_v2x_mesh.gd
class_name TestV2xMesh
extends "res://tests/test_base.gd"

# Unit verification suite for V2X ad-hoc mesh networking and RF channel simulation.

const V2xMeshNetwork = preload("res://fleet/v2x_mesh_network.gd")

func before_each() -> void:
	reset_test()

func test_v2x_broadcast_and_delivery() -> Dictionary:
	var net: V2xMeshNetwork = V2xMeshNetwork.new(200.0, 0.010)
	net.packet_drop_rate_override = 0.0 # Zero drops for deterministic test
	
	# Register 3 nodes
	net.register_node("node_A", Vector3(0.0, 0.0, 0.0))
	net.register_node("node_B", Vector3(30.0, 0.0, 0.0)) # 30m away
	net.register_node("node_C", Vector3(500.0, 0.0, 0.0)) # 500m away (exceeds 200m range)
	
	# Node A broadcasts
	var scheduled: int = net.broadcast_message("node_A", "BSM", {"speed": 12.5})
	assert_equal(scheduled, 1, "Only node_B should be in range to receive")
	
	# Check before latency expires
	net.step_network(0.002) # Advance 2ms (less than 10ms latency)
	var inbox_early: Array[Dictionary] = net.poll_inbox("node_B")
	assert_equal(inbox_early.size(), 0, "Packet should still be in-flight before delivery time")
	
	# Step past delivery time
	net.step_network(0.025) # Total > 25ms
	var inbox_delivered: Array[Dictionary] = net.poll_inbox("node_B")
	assert_equal(inbox_delivered.size(), 1, "Packet should arrive in node_B mailbox")
	assert_equal(inbox_delivered[0]["msg_type"], "BSM", "Message type check")
	assert_almost_equal(inbox_delivered[0]["payload"]["speed"], 12.5, 1e-4, "Payload integrity")
	
	# Node C inbox should be empty
	var inbox_c: Array[Dictionary] = net.poll_inbox("node_C")
	assert_equal(inbox_c.size(), 0, "Out of range node should receive nothing")
	
	return get_results()

func test_v2x_unicast_communication() -> Dictionary:
	var net: V2xMeshNetwork = V2xMeshNetwork.new(100.0, 0.005)
	net.packet_drop_rate_override = 0.0
	
	net.register_node("leader", Vector3(0.0, 0.0, 0.0))
	net.register_node("follower", Vector3(10.0, 0.0, 0.0))
	
	var sent: bool = net.send_unicast("leader", "follower", "PLATOON_ACCEL", {"accel": 1.8})
	assert_true(sent, "Unicast dispatch should succeed within range")
	
	net.step_network(0.020)
	var follower_inbox: Array[Dictionary] = net.poll_inbox("follower")
	assert_equal(follower_inbox.size(), 1, "Follower receives unicast")
	assert_almost_equal(follower_inbox[0]["payload"]["accel"], 1.8, 1e-4, "Unicast content check")
	
	return get_results()
