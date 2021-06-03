# maps/lanelet_segment.gd
class_name LaneletSegment
extends RefCounted

# Vector road network segment inspired by Lanelet2 / OpenDRIVE specifications.
# Defined by left boundary polyline, right boundary polyline, and derived centerline.
# Supports Frenet frame coordinate conversions (s: longitudinal, d: lateral cross-track).

const GeometryUtils = preload("res://core/spatial/geometry_utils.gd")

var id: int = -1
var speed_limit_mps: float = 13.88 # Default 50 km/h = ~13.88 m/s
var is_one_way: bool = true

var left_boundary: Array[Vector2] = []
var right_boundary: Array[Vector2] = []
var centerline: Array[Vector2] = []

var predecessor_ids: PackedInt32Array = PackedInt32Array()
var successor_ids: PackedInt32Array = PackedInt32Array()
var left_neighbor_id: int = -1
var right_neighbor_id: int = -1

var can_change_left: bool = false
var can_change_right: bool = false

var regulatory_elements: Array[Dictionary] = [] # {"type": "stop_sign"|"traffic_signal"|"yield", "s": float}
var total_length: float = 0.0

func _init(p_id: int = -1, p_speed_limit: float = 13.88) -> void:
	id = p_id
	speed_limit_mps = p_speed_limit

# Computes centerline from average of left and right boundary polylines
func compute_centerline(samples: int = 20) -> void:
	assert(left_boundary.size() >= 2 and right_boundary.size() >= 2, "Boundaries must have at least 2 points")
	centerline.clear()
	
	# Sample left and right boundaries proportionally
	var left_len: float = _compute_polyline_length(left_boundary)
	var right_len: float = _compute_polyline_length(right_boundary)
	
	for i in range(samples):
		var frac: float = float(i) / float(samples - 1)
		var p_left: Vector2 = _sample_polyline_at_fraction(left_boundary, left_len, frac)
		var p_right: Vector2 = _sample_polyline_at_fraction(right_boundary, right_len, frac)
		centerline.append((p_left + p_right) * 0.5)
		
	total_length = _compute_polyline_length(centerline)

func _compute_polyline_length(polyline: Array[Vector2]) -> float:
	var l: float = 0.0
	for i in range(polyline.size() - 1):
		l += polyline[i].distance_to(polyline[i + 1])
	return l

func _sample_polyline_at_fraction(polyline: Array[Vector2], total_len: float, frac: float) -> Vector2:
	if frac <= 0.0 or polyline.size() == 0:
		return polyline[0]
	if frac >= 1.0:
		return polyline[polyline.size() - 1]
		
	var target_dist: float = frac * total_len
	var accum: float = 0.0
	
	for i in range(polyline.size() - 1):
		var seg_len: float = polyline[i].distance_to(polyline[i + 1])
		if accum + seg_len >= target_dist:
			var remaining: float = target_dist - accum
			var t: float = remaining / seg_len if seg_len > 1e-9 else 0.0
			return polyline[i].lerp(polyline[i + 1], t)
		accum += seg_len
		
	return polyline[polyline.size() - 1]

# Converts Cartesian coordinate (x, y) to Frenet coordinates (s: longitudinal along centerline, d: lateral)
# Positive d is to the left of the centerline; negative d is to the right.
func cartesian_to_frenet(point: Vector2) -> Vector2:
	if centerline.size() < 2:
		return Vector2.ZERO
		
	var proj: Dictionary = GeometryUtils.project_onto_polyline(point, centerline)
	var seg_idx: int = proj["segment_index"]
	var pt_on_center: Vector2 = proj["point"]
	
	# Compute s coordinate (accumulated distance up to segment + distance to pt_on_center)
	var s: float = 0.0
	for i in range(seg_idx):
		s += centerline[i].distance_to(centerline[i + 1])
	s += centerline[seg_idx].distance_to(pt_on_center)
	
	# Compute lateral d using cross product to determine sign (left vs right)
	var seg_dir: Vector2 = (centerline[seg_idx + 1] - centerline[seg_idx]).normalized()
	var to_pt: Vector2 = point - pt_on_center
	var signed_cross: float = seg_dir.x * to_pt.y - seg_dir.y * to_pt.x
	var d: float = signf(signed_cross) * to_pt.length()
	
	return Vector2(s, d)

# Converts Frenet coordinates (s, d) back to Cartesian coordinate (x, y)
func frenet_to_cartesian(s: float, d: float) -> Vector2:
	if centerline.size() < 2:
		return Vector2.ZERO
		
	s = clampf(s, 0.0, total_length)
	var accum: float = 0.0
	var seg_idx: int = 0
	var pt_on_line: Vector2 = centerline[0]
	var seg_dir: Vector2 = (centerline[1] - centerline[0]).normalized()
	
	for i in range(centerline.size() - 1):
		var seg_len: float = centerline[i].distance_to(centerline[i + 1])
		if accum + seg_len >= s:
			var remaining: float = s - accum
			var t: float = remaining / seg_len if seg_len > 1e-9 else 0.0
			pt_on_line = centerline[i].lerp(centerline[i + 1], t)
			seg_dir = (centerline[i + 1] - centerline[i]).normalized()
			break
		accum += seg_len
		
	# Left normal vector: (-seg_dir.y, seg_dir.x)
	var normal: Vector2 = Vector2(-seg_dir.y, seg_dir.x)
	return pt_on_line + normal * d
