# Core API Reference Manual

This document provides class specifications, method signatures, and parameter documentation across Beacon's primary modules.

---

## 1. Core Mathematics (`core/math/`)

### `MatrixMN`
Arbitrary $M \times N$ dense linear algebra substrate with row-major caching.
- `static func identity(n: int) -> MatrixMN`: Returns $n \times n$ identity matrix.
- `static func zeros(rows: int, cols: int) -> MatrixMN`: Returns zero-initialized matrix.
- `func multiply(other: MatrixMN) -> MatrixMN`: Matrix multiplication $A \times B$.
- `func transpose() -> MatrixMN`: Returns matrix transpose $A^T$.
- `func lu_decompose() -> Dictionary`: Computes PA = LU decomposition with partial pivoting. Returns `{"L": MatrixMN, "U": MatrixMN, "P": MatrixMN, "det": float}`.
- `func cholesky_decompose() -> MatrixMN`: Computes lower triangular Cholesky factor $L$ such that $A = L L^T$.
- `func solve_linear_system(b: PackedFloat64Array) -> PackedFloat64Array`: Solves $Ax = b$ via Gaussian elimination.
- `func inverse() -> MatrixMN`: Computes matrix inverse $A^{-1}$ via Gauss-Jordan full pivoting.
- `func regularize_covariance(epsilon: float = 1e-6) -> void`: Enforces symmetry $P = \frac{1}{2}(P + P^T)$ and positive semi-definiteness.

### `QuaternionMath`
Lie algebra $\mathfrak{so}(3)$ and unit quaternion kinematics.
- `static func skew_symmetric(v: Vector3) -> MatrixMN`: Computes hat operator $[v]_\times \in \mathfrak{so}(3)$.
- `static func unskew_symmetric(m: MatrixMN) -> Vector3`: Computes vee operator $([v]_\times)^\vee \in \mathbb{R}^3$.
- `static func so3_exp(theta_vec: Vector3) -> MatrixMN`: Computes matrix exponential $\exp([\theta]_\times) \in SO(3)$ via Rodrigues formula.
- `static func so3_log(r: MatrixMN) -> Vector3`: Computes matrix logarithm $\log(R)^\vee \in \mathbb{R}^3$.
- `static func quat_from_rotation_matrix(r: MatrixMN) -> Quaternion`: Converts rotation matrix to unit quaternion.
- `static func euler_to_quaternion(yaw: float, pitch: float, roll: float) -> Quaternion`: ZYX aerospace Euler to quaternion.

---

## 2. Spatial Indexing & Geometry (`core/spatial/`)

### `KDTree2D`
Balanced 2D KD-Tree with median-split construction.
- `static func build_from_points(points: Array[Vector2], ids: PackedInt32Array = PackedInt32Array()) -> KDTree2D`: Constructs balanced tree in $O(N \log N)$.
- `func find_nearest(target: Vector2) -> Dictionary`: Finds single nearest neighbor `{"point": Vector2, "distance": float, "id": int}` in $O(\log N)$.
- `func find_k_nearest(target: Vector2, k: int) -> Array[Dictionary]`: Finds $K$ nearest neighbors.
- `func find_in_radius(target: Vector2, radius: float) -> Array[Dictionary]`: Circular range query.

### `GeometryUtils`
Planar computational geometry routines.
- `static func convex_hull_2d(points: Array[Vector2]) -> Array[Vector2]`: Computes convex hull via Monotone Chain algorithm in $O(N \log N)$.
- `static func sat_obb_collision(box_a: Dictionary, box_b: Dictionary) -> bool`: Evaluates Oriented Bounding Box collision via Separating Axis Theorem.
- `static func project_onto_polyline(point: Vector2, polyline: Array[Vector2]) -> Dictionary`: Computes orthogonal projection returning `{"point": Vector2, "distance": float, "segment_index": int, "t": float}`.

---

## 3. Road Topologies & Costmaps (`maps/`)

### `LaneletSegment`
Curvilinear road segment with Frenet coordinate mapping.
- `func cartesian_to_frenet(pt: Vector2) -> Vector2`: Transforms $(X, Y) \to (s, d)$ where $s$ is arc-length and $d$ is lateral displacement.
- `func frenet_to_cartesian(frenet_sd: Vector2) -> Vector2`: Transforms $(s, d) \to (X, Y)$.

### `Costmap2D`
Multi-layer 2D occupancy grid with Bresenham clearing and exponential obstacle inflation.
- `func set_obstacle(world_pos: Vector2, cost: float = 1.0) -> void`: Marks an obstacle cell.
- `func update_inflation_layer(radius_cells: int = 5, decay_factor: float = 0.5) -> void`: Inflates lethal obstacles exponentially outward.
- `func is_traversable(world_pos: Vector2, footprint_radius_m: float = 1.0) -> bool`: Checks if cell is collision-free.

---

## 4. State Estimation Substrate (`estimation/`)

### `ExtendedKalmanFilter`
15-state Error-State Extended Kalman Filter (ES-EKF).
- `func reset_state(init_pos: Vector3, init_vel: Vector3, init_quat: Quaternion) -> void`: Initializes state and prior covariance.
- `func predict_imu(accel_m: Vector3, gyro_m: Vector3, dt: float) -> void`: Propagates nominal state and covariance via high-rate IMU specific force and angular velocity.
- `func update_gnss_position(gnss_pos: Vector3, accuracy_m: float) -> bool`: Updates position with Mahalanobis outlier gating.
- `func update_wheel_speed(measured_speed: float, accuracy_mps: float) -> bool`: Updates vehicle forward speed.

---

## 5. Kinodynamic Motion Planning (`planning/`)

### `HybridAStar`
Kinodynamic path planner in continuous $SE(2)$ with Reed-Shepp / bicycle primitives.
- `func plan_path(start: Vector3, goal: Vector3) -> Array[Dictionary]`: Plans collision-free trajectory returning continuous waypoints and steering angles.

### `ConflictBasedSearch`
Two-level multi-agent path deconfliction algorithm.
- `func plan_multi_agent_paths(agents_start_goal: Array[Dictionary]) -> Array[Array]`: Computes provably conflict-free space-time paths for multiple vehicles simultaneously.

---

## 6. Vehicle Trajectory Tracking (`control/`)

### `ModelPredictiveController`
Linear Time-Varying Model Predictive Controller (LTV-MPC).
- `func compute_control(current_state: Vector4, ref_trajectory: Array[Dictionary]) -> Dictionary`: Solves finite-horizon constrained optimization returning `{"accel_cmd": float, "steering_cmd": float, "predicted_states": Array}`.

### `StanleyController`
Front-axle tracking controller with non-linear cross-track error damping.
- `func compute_steering(rear_x: float, rear_y: float, heading_rad: float, speed_mps: float, path: Array[Vector2], dt: float) -> float`: Computes steering angle command.

---

## 7. Fleet Orchestration (`fleet/`)

### `VehicleAgent`
Encapsulates vehicle physical dynamics, sensor suite, state estimation, and path execution.
- `func spawn(p_pos: Vector3, p_heading: float) -> void`: Spawns vehicle at designated pose.
- `func set_destination(destination_2d: Vector2, speed_limit_mps: float = 15.0) -> bool`: Plans and profiles trajectory to target.
- `func update_simulation(dt: float) -> Dictionary`: Advances vehicle simulation step.
- `func trigger_emergency_stop(reason: String) -> void`: Commands immediate maximum deceleration.

### `FleetManager`
Supervisory coordinator managing vehicles, V2X communications, and intersection reservations.
- `func register_vehicle(agent: VehicleAgent) -> void`: Registers agent into fleet.
- `func batch_dispatch(tasks: Array[Dictionary]) -> Array[Dictionary]`: Assigns tasks optimally via Kuhn-Munkres matching.
- `func step_fleet(dt: float) -> Dictionary`: Steps entire fleet, V2X mesh, and intersection reservations.
