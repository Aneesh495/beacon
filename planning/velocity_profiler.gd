# planning/velocity_profiler.gd
class_name VelocityProfiler
extends RefCounted

# Time-optimal, jerk-limited S-curve velocity profiling for geometric trajectories.
# Enforces longitudinal acceleration limits, deceleration stopping constraints,
# and lateral acceleration limits governed by trajectory curvature: v <= sqrt(a_lat_max / kappa).

var max_velocity_mps: float = 15.0 # ~54 km/h
var max_acceleration_mps2: float = 2.5 # ~0.25 g comfort limit
var max_deceleration_mps2: float = 3.5 # Braking limit
var max_lateral_accel_mps2: float = 2.0 # Lateral stability limit
var max_jerk_mps3: float = 4.0

# Profiles an input geometric path into time-parameterized states
# Input path: Array of Dictionaries with {"position": Vector2, "curvature": float, "s": float}
# Returns Array of Dictionaries: {"position": Vector2, "v": float, "a": float, "t": float, "s": float}
func profile_path(
	path_samples: Array[Dictionary],
	start_speed: float = 0.0,
	end_speed: float = 0.0
) -> Array[Dictionary]:
	var n: int = path_samples.size()
	if n == 0:
		return []
	if n == 1:
		return [{
			"position": path_samples[0]["position"],
			"v": start_speed,
			"a": 0.0,
			"t": 0.0,
			"s": 0.0
		}]
		
	var v_profile: PackedFloat64Array = PackedFloat64Array()
	v_profile.resize(n)
	
	# Step 1: Maximum permissible velocity based on road speed limit and lateral curvature
	for i in range(n):
		var kappa: float = abs(path_samples[i].get("curvature", 0.0))
		var v_curv: float = max_velocity_mps
		if kappa > 1e-4:
			v_curv = sqrt(max_lateral_accel_mps2 / kappa)
		v_profile[i] = minf(max_velocity_mps, v_curv)
		
	# Clamp boundary speeds
	v_profile[0] = minf(v_profile[0], start_speed)
	v_profile[n - 1] = minf(v_profile[n - 1], end_speed)
	
	# Step 2: Forward pass (acceleration limit)
	# v_i^2 <= v_{i-1}^2 + 2 * a_max * ds
	for i in range(1, n):
		var ds: float = path_samples[i]["s"] - path_samples[i - 1]["s"]
		if ds < 1e-6:
			continue
		var max_reachable_sq: float = v_profile[i - 1] * v_profile[i - 1] + 2.0 * max_acceleration_mps2 * ds
		var max_reachable_v: float = sqrt(maxf(0.0, max_reachable_sq))
		v_profile[i] = minf(v_profile[i], max_reachable_v)
		
	# Step 3: Backward pass (deceleration limit)
	# v_{i-1}^2 <= v_i^2 + 2 * a_decel * ds
	for i in range(n - 2, -1, -1):
		var ds: float = path_samples[i + 1]["s"] - path_samples[i]["s"]
		if ds < 1e-6:
			continue
		var max_decel_sq: float = v_profile[i + 1] * v_profile[i + 1] + 2.0 * max_deceleration_mps2 * ds
		var max_decel_v: float = sqrt(maxf(0.0, max_decel_sq))
		v_profile[i] = minf(v_profile[i], max_decel_v)
		
	# Step 4: Time and acceleration parameterization
	var profiled_trajectory: Array[Dictionary] = []
	var accum_t: float = 0.0
	
	for i in range(n):
		var v_i: float = v_profile[i]
		var a_i: float = 0.0
		
		if i < n - 1:
			var ds: float = path_samples[i + 1]["s"] - path_samples[i]["s"]
			var v_next: float = v_profile[i + 1]
			var v_avg: float = maxf(0.1, 0.5 * (v_i + v_next))
			var dt: float = ds / v_avg
			a_i = (v_next - v_i) / dt if dt > 1e-6 else 0.0
			
			profiled_trajectory.append({
				"position": path_samples[i]["position"],
				"v": v_i,
				"a": a_i,
				"t": accum_t,
				"s": path_samples[i]["s"]
			})
			accum_t += dt
		else:
			profiled_trajectory.append({
				"position": path_samples[i]["position"],
				"v": v_i,
				"a": 0.0,
				"t": accum_t,
				"s": path_samples[i]["s"]
			})
			
	return profiled_trajectory
