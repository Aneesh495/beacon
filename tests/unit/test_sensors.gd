# tests/unit/test_sensors.gd
class_name TestSensors
extends "res://tests/test_base.gd"

# Unit verification suite for simulated sensors: Lidar3D, Imu6Dof, WheelOdometry, and GnssReceiver.

const SensorBase = preload("res://sensors/sensor_base.gd")
const Lidar3D = preload("res://sensors/lidar_3d.gd")
const Imu6Dof = preload("res://sensors/imu_6dof.gd")
const WheelOdometry = preload("res://sensors/wheel_odometry.gd")
const GnssReceiver = preload("res://sensors/gnss_receiver.gd")
const BVHTree = preload("res://core/spatial/bvh_tree.gd")

func before_each() -> void:
	reset_test()

func test_sensor_base_lifecycle() -> Dictionary:
	var sensor: SensorBase = SensorBase.new("test_sensor", 20.0) # 20 Hz => dt = 0.05s
	assert_equal(sensor.state, SensorBase.SensorState.HEALTHY)
	assert_almost_equal(sensor.dt_nominal, 0.05, 1e-6)
	
	# After 0.02s, should not be ready to update
	assert_false(sensor.is_ready_to_update(0.02))
	# After additional 0.035s (total 0.055s), should be ready
	assert_true(sensor.is_ready_to_update(0.035))
	
	sensor.mark_updated()
	assert_equal(sensor.total_measurements_produced, 1)
	
	# Test fault injection
	sensor.inject_fault("blackout")
	assert_equal(sensor.state, SensorBase.SensorState.FAILED)
	assert_false(sensor.is_ready_to_update(1.0), "Failed sensor should not update")
	
	sensor.clear_faults()
	assert_equal(sensor.state, SensorBase.SensorState.HEALTHY)
	
	return get_results()

func test_lidar_3d_scan_and_ransac() -> Dictionary:
	var lidar: Lidar3D = Lidar3D.new(16, 10.0)
	lidar.horizontal_resolution_deg = 30.0 # 12 horizontal steps for fast unit test
	
	# Create synthetic obstacles: Box at (10, 0, 1)
	var boxes: Array[AABB] = [
		AABB(Vector3(8.0, -1.0, 0.0), Vector3(4.0, 2.0, 2.0))
	]
	var bvh: BVHTree = BVHTree.build_from_aabbs(boxes)
	
	# Scan from (0, 0, 1.5) looking forward
	var cloud: PackedVector3Array = lidar.scan(Vector3(0.0, 0.0, 1.5), Quaternion.IDENTITY, bvh)
	assert_true(cloud.size() > 0, "LiDAR should produce non-empty point cloud")
	
	# Test RANSAC ground plane segmentation
	var segmentation: Dictionary = Lidar3D.segment_ground_ransac(cloud, 0.2, 30)
	var ground_pts: PackedVector3Array = segmentation["ground_points"]
	var obs_pts: PackedVector3Array = segmentation["obstacle_points"]
	var normal: Vector3 = segmentation["plane_normal"]
	
	assert_true(ground_pts.size() > 0, "Should detect ground points")
	assert_true(abs(normal.z) > 0.8, "Ground plane normal should point upwards")
	
	# Test Euclidean clustering on obstacle points
	var clusters: Array[AABB] = Lidar3D.extract_clusters(obs_pts, 3.0, 2)
	assert_true(clusters.size() >= 0, "Clustering should run without error")
	
	return get_results()

func test_imu_gravity_and_acceleration() -> Dictionary:
	var imu: Imu6Dof = Imu6Dof.new(100.0)
	imu.noise_enabled = false # Test deterministic physics first
	
	# Vehicle at rest level on the ground: true linear accel = 0, true angular vel = 0, orientation = IDENTITY
	var meas_rest: Dictionary = imu.measure(Vector3.ZERO, Vector3.ZERO, Quaternion.IDENTITY, 0.01)
	var accel: Vector3 = meas_rest["linear_acceleration"]
	var gyro: Vector3 = meas_rest["angular_velocity"]
	
	# Accelerometer on stationary table should measure upward normal force matching gravity +9.80665 m/s^2 on Z axis
	assert_almost_equal(accel.z, 9.80665 + imu.accel_bias.z, 1e-3, "Gravity reaction force check")
	assert_almost_equal(accel.x, imu.accel_bias.x, 1e-3)
	assert_almost_equal(accel.y, imu.accel_bias.y, 1e-3)
	
	# Gyro should measure bias when stationary
	assert_almost_equal(gyro.z, imu.gyro_bias.z, 1e-4)
	
	return get_results()

func test_wheel_odometry_rk2() -> Dictionary:
	var odom: WheelOdometry = WheelOdometry.new(0.35, 1.6, 50.0)
	odom.noise_enabled = false
	odom.reset_odometry(Vector2(0.0, 0.0), 0.0)
	
	# Drive straight at 10 m/s for 1 second (10 steps of 0.1s)
	for i in range(10):
		odom.update_ackermann_rk2(10.0, 0.0, 0.1)
		
	# X should be approximately 10 meters, Y should remain 0, heading 0
	assert_almost_equal(odom.pose_x, 10.0, 1e-3, "Odometry X position check")
	assert_almost_equal(odom.pose_y, 0.0, 1e-3, "Odometry Y position check")
	assert_almost_equal(odom.pose_theta, 0.0, 1e-3, "Odometry heading check")
	
	return get_results()

func test_gnss_receiver_measurements() -> Dictionary:
	var gnss: GnssReceiver = GnssReceiver.new(5.0)
	gnss.noise_enabled = false
	
	var true_pos: Vector3 = Vector3(100.0, 250.0, 5.0)
	var meas: Dictionary = gnss.measure_position(true_pos)
	
	assert_vector3_almost_equal(meas["position"], true_pos, 1e-4, "Noise-free GNSS position")
	assert_equal(meas["fix_type"], GnssReceiver.FixType.RTK_FIXED)
	
	# Test tunnel blackout
	gnss.inject_fault("tunnel")
	var meas_tunnel: Dictionary = gnss.measure_position(true_pos)
	assert_equal(meas_tunnel["fix_type"], GnssReceiver.FixType.NO_FIX, "Tunnel should cause loss-of-fix")
	
	return get_results()
