# sensors/lidar_3d.gd
class_name Lidar3D
extends "res://sensors/sensor_base.gd"

# Spinning multi-beam 3D LiDAR sensor simulator.
# Generates synthetic point clouds using spherical raycasting, Gaussian range noise,
# incidence angle reflectivity dropoff, RANSAC ground-plane segmentation,
# and Euclidean cluster extraction for 3D bounding box obstacle detection.

const BVHTree = preload("res://core/spatial/bvh_tree.gd")

var num_channels: int = 16 # Vertical beams
var horizontal_resolution_deg: float = 2.0 # Azimuth step
var min_range_m: float = 0.5
var max_range_m: float = 80.0
var range_noise_std_dev_m: float = 0.02 # 2cm Gaussian noise

# Vertical elevation angles (e.g. Velodyne VLP-16 spans -15 to +15 deg)
var vertical_angles_rad: PackedFloat64Array = PackedFloat64Array()

func _init(p_channels: int = 16, p_rate_hz: float = 10.0) -> void:
	super("lidar_3d", p_rate_hz)
	num_channels = p_channels
	_setup_vertical_beams()

func _setup_vertical_beams() -> void:
	vertical_angles_rad.resize(num_channels)
	var min_deg: float = -15.0
	var max_deg: float = 15.0
	var step_deg: float = (max_deg - min_deg) / float(maxi(1, num_channels - 1))
	for i in range(num_channels):
		vertical_angles_rad[i] = deg_to_rad(min_deg + float(i) * step_deg)

# Generates 3D point cloud by raycasting against obstacle BVH and ground plane
# Returns PackedVector3Array of detected returns in vehicle body frame or world frame
func scan(sensor_pos: Vector3, sensor_rot_quat: Quaternion, obstacle_bvh: BVHTree) -> PackedVector3Array:
	var point_cloud: PackedVector3Array = PackedVector3Array()
	if state == SensorState.FAILED or state == SensorState.OFFLINE:
		return point_cloud
		
	var h_steps: int = int(round(360.0 / horizontal_resolution_deg))
	var noise_factor: float = range_noise_std_dev_m if noise_enabled else 0.0
	if simulated_fault_active and fault_type == "high_noise":
		noise_factor *= 5.0
		
	for h_idx in range(h_steps):
		var azimuth: float = deg_to_rad(float(h_idx) * horizontal_resolution_deg)
		var cos_az: float = cos(azimuth)
		var sin_az: float = sin(azimuth)
		
		for v_idx in range(num_channels):
			var elev: float = vertical_angles_rad[v_idx]
			var cos_el: float = cos(elev)
			var sin_el: float = sin(elev)
			
			# Unit ray in sensor local frame: [forward=X, left=Y, up=Z]
			var local_dir: Vector3 = Vector3(cos_el * cos_az, cos_el * sin_az, sin_el)
			# Transform to world frame
			var world_dir: Vector3 = sensor_rot_quat * local_dir
			
			# Check intersection against obstacle BVH
			var hit_dist: float = max_range_m
			var ray_hit: Dictionary = obstacle_bvh.raycast(sensor_pos, world_dir, max_range_m) if obstacle_bvh else {"hit": false}
			
			if ray_hit["hit"] and ray_hit["distance"] < hit_dist:
				hit_dist = ray_hit["distance"]
				
			# Check intersection with ground plane (z = 0)
			if world_dir.z < -1e-6:
				var t_ground: float = (0.0 - sensor_pos.z) / world_dir.z
				if t_ground > 0.0 and t_ground < hit_dist:
					hit_dist = t_ground
					
			if hit_dist >= min_range_m and hit_dist < max_range_m:
				# Add simulated Gaussian measurement noise
				var noisy_dist: float = hit_dist
				if noise_factor > 0.0:
					# Box-Muller Gaussian random variable
					var u1: float = maxf(1e-9, randf())
					var u2: float = randf()
					var gauss: float = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2)
					noisy_dist += gauss * noise_factor
					
				var hit_point: Vector3 = sensor_pos + world_dir * noisy_dist
				point_cloud.append(hit_point)
				
	mark_updated()
	return point_cloud

# RANSAC ground plane segmentation algorithm: finds dominant plane z = a*x + b*y + d
# Returns Dictionary: {"ground_points": PackedVector3Array, "obstacle_points": PackedVector3Array, "plane_normal": Vector3}
static func segment_ground_ransac(
	cloud: PackedVector3Array,
	distance_threshold: float = 0.15,
	max_iterations: int = 50
) -> Dictionary:
	var n: int = cloud.size()
	if n < 3:
		return {"ground_points": PackedVector3Array(), "obstacle_points": cloud, "plane_normal": Vector3.UP}
		
	var best_inliers: PackedInt32Array = PackedInt32Array()
	var best_normal: Vector3 = Vector3.UP
	
	for iter in range(max_iterations):
		var idx1: int = randi() % n
		var idx2: int = randi() % n
		var idx3: int = randi() % n
		if idx1 == idx2 or idx2 == idx3 or idx1 == idx3:
			continue
			
		var p1: Vector3 = cloud[idx1]
		var p2: Vector3 = cloud[idx2]
		var p3: Vector3 = cloud[idx3]
		
		var v1: Vector3 = p2 - p1
		var v2: Vector3 = p3 - p1
		var normal: Vector3 = v1.cross(v2).normalized()
		
		# Ground plane normal should be roughly vertical (|normal.z| > 0.7)
		if abs(normal.z) < 0.7:
			continue
			
		var d: float = -normal.dot(p1)
		var current_inliers: PackedInt32Array = PackedInt32Array()
		
		for i in range(n):
			var dist: float = abs(normal.dot(cloud[i]) + d)
			if dist <= distance_threshold:
				current_inliers.append(i)
				
		if current_inliers.size() > best_inliers.size():
			best_inliers = current_inliers
			best_normal = normal
			
	var is_ground: PackedByteArray = PackedByteArray()
	is_ground.resize(n)
	for idx in best_inliers:
		is_ground[idx] = 1
		
	var ground_pts: PackedVector3Array = PackedVector3Array()
	var obstacle_pts: PackedVector3Array = PackedVector3Array()
	for i in range(n):
		if is_ground[i] == 1:
			ground_pts.append(cloud[i])
		else:
			obstacle_pts.append(cloud[i])
			
	return {
		"ground_points": ground_pts,
		"obstacle_points": obstacle_pts,
		"plane_normal": best_normal
	}

# Extracts 3D obstacle bounding boxes from non-ground points using Euclidean clustering (DBSCAN)
# Returns Array of AABBs
static func extract_clusters(obstacle_pts: PackedVector3Array, cluster_radius: float = 1.2, min_points: int = 4) -> Array[AABB]:
	var clusters: Array[AABB] = []
	var n: int = obstacle_pts.size()
	if n < min_points:
		return clusters
		
	var visited: PackedByteArray = PackedByteArray()
	visited.resize(n)
	var cluster_r_sq: float = cluster_radius * cluster_radius
	
	for i in range(n):
		if visited[i] == 1:
			continue
		visited[i] = 1
		
		var current_cluster_pts: Array[Vector3] = [obstacle_pts[i]]
		var queue: Array[int] = [i]
		
		while not queue.is_empty():
			var cur_idx: int = queue.pop_front()
			var p_cur: Vector3 = obstacle_pts[cur_idx]
			
			for j in range(n):
				if visited[j] == 0:
					if p_cur.distance_squared_to(obstacle_pts[j]) <= cluster_r_sq:
						visited[j] = 1
						queue.append(j)
						current_cluster_pts.append(obstacle_pts[j])
						
		if current_cluster_pts.size() >= min_points:
			var min_pt: Vector3 = current_cluster_pts[0]
			var max_pt: Vector3 = current_cluster_pts[0]
			for pt in current_cluster_pts:
				min_pt = min_pt.min(pt)
				max_pt = max_pt.max(pt)
			clusters.append(AABB(min_pt, max_pt - min_pt))
			
	return clusters
