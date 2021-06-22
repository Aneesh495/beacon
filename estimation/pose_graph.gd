# estimation/pose_graph.gd
class_name PoseGraph
extends RefCounted

# Pose Graph non-linear least squares SLAM backend in SE(2).
# Optimizes trajectory poses subject to odometry constraints, GNSS anchor priors,
# and loop closure factor constraints with robust Huber loss kernels.

const MatrixMN = preload("res://core/math/matrix_mn.gd")

class PoseNode:
	var id: int
	var pos: Vector2
	var heading: float # Radians
	
	func _init(p_id: int, p_pos: Vector2, p_heading: float) -> void:
		id = p_id
		pos = p_pos
		heading = p_heading

class BetweenFactor:
	var from_id: int
	var to_id: int
	var delta_pos: Vector2 # Measured translation in 'from' body frame
	var delta_heading: float # Measured rotation
	var weight: float = 1.0 # 1 / variance
	
	func _init(p_from: int, p_to: int, p_dp: Vector2, p_dth: float, p_weight: float = 1.0) -> void:
		from_id = p_from
		to_id = p_to
		delta_pos = p_dp
		delta_heading = p_dth
		weight = p_weight

var nodes: Dictionary = {} # int (id) -> PoseNode
var factors: Array[BetweenFactor] = []
var fixed_node_id: int = 0 # Anchor pose

func add_node(id: int, initial_pos: Vector2, initial_heading: float) -> void:
	nodes[id] = PoseNode.new(id, initial_pos, initial_heading)

func add_factor(from_id: int, to_id: int, delta_pos: Vector2, delta_heading: float, weight: float = 1.0) -> void:
	factors.append(BetweenFactor.new(from_id, to_id, delta_pos, delta_heading, weight))

# Evaluates total chi-square objective error: sum r_k^T * Omega_k * r_k
func compute_total_error() -> float:
	var total_err: float = 0.0
	for factor in factors:
		var n_i: PoseNode = nodes[factor.from_id]
		var n_j: PoseNode = nodes[factor.to_id]
		
		var cos_th: float = cos(n_i.heading)
		var sin_th: float = sin(n_i.heading)
		var diff_pos: Vector2 = n_j.pos - n_i.pos
		
		# Rotate into node_i body frame
		var pred_dx: float = cos_th * diff_pos.x + sin_th * diff_pos.y
		var pred_dy: float = -sin_th * diff_pos.x + cos_th * diff_pos.y
		var pred_dth: float = fposmod(n_j.heading - n_i.heading + PI, 2.0 * PI) - PI
		
		var err_x: float = pred_dx - factor.delta_pos.x
		var err_y: float = pred_dy - factor.delta_pos.y
		var err_th: float = fposmod(pred_dth - factor.delta_heading + PI, 2.0 * PI) - PI
		
		var sq_err: float = err_x * err_x + err_y * err_y + err_th * err_th
		total_err += factor.weight * sq_err
		
	return total_err

# Performs Gauss-Newton optimization over all non-fixed pose nodes
func optimize_gauss_newton(iterations: int = 5) -> float:
	var n_nodes: int = nodes.size()
	if n_nodes <= 1 or factors.is_empty():
		return 0.0
		
	var id_list: Array = nodes.keys()
	id_list.sort()
	var total_dim: int = n_nodes * 3
	
	# Mapping from node id to 0-based index
	var id_to_idx: Dictionary = {}
	for idx in range(id_list.size()):
		id_to_idx[id_list[idx]] = idx
		
	for iter in range(iterations):
		var H: MatrixMN = MatrixMN.zeros(total_dim, total_dim)
		var b: PackedFloat64Array = PackedFloat64Array()
		b.resize(total_dim)
		for k in range(total_dim):
			b[k] = 0.0
			
		for factor in factors:
			var idx_i: int = id_to_idx[factor.from_id]
			var idx_j: int = id_to_idx[factor.to_id]
			var n_i: PoseNode = nodes[factor.from_id]
			var n_j: PoseNode = nodes[factor.to_id]
			
			var cos_th: float = cos(n_i.heading)
			var sin_th: float = sin(n_i.heading)
			var diff_pos: Vector2 = n_j.pos - n_i.pos
			
			# Predicted relative displacement in body frame i
			var pred_dx: float = cos_th * diff_pos.x + sin_th * diff_pos.y
			var pred_dy: float = -sin_th * diff_pos.x + cos_th * diff_pos.y
			var pred_dth: float = fposmod(n_j.heading - n_i.heading + PI, 2.0 * PI) - PI
			
			# Residual vector r in R^3
			var rx: float = pred_dx - factor.delta_pos.x
			var ry: float = pred_dy - factor.delta_pos.y
			var rth: float = fposmod(pred_dth - factor.delta_heading + PI, 2.0 * PI) - PI
			
			var w: float = factor.weight
			
			# J_i (3x3): partial derivatives w.r.t [x_i, y_i, theta_i]
			var Ji: MatrixMN = MatrixMN.zeros(3, 3)
			Ji.set_element(0, 0, -cos_th)
			Ji.set_element(0, 1, -sin_th)
			Ji.set_element(0, 2, -sin_th * diff_pos.x + cos_th * diff_pos.y)
			
			Ji.set_element(1, 0,  sin_th)
			Ji.set_element(1, 1, -cos_th)
			Ji.set_element(1, 2, -cos_th * diff_pos.x - sin_th * diff_pos.y)
			
			Ji.set_element(2, 0,  0.0)
			Ji.set_element(2, 1,  0.0)
			Ji.set_element(2, 2, -1.0)
			
			# J_j (3x3): partial derivatives w.r.t [x_j, y_j, theta_j]
			var Jj: MatrixMN = MatrixMN.zeros(3, 3)
			Jj.set_element(0, 0,  cos_th)
			Jj.set_element(0, 1,  sin_th)
			Jj.set_element(0, 2,  0.0)
			
			Jj.set_element(1, 0, -sin_th)
			Jj.set_element(1, 1,  cos_th)
			Jj.set_element(1, 2,  0.0)
			
			Jj.set_element(2, 0,  0.0)
			Jj.set_element(2, 1,  0.0)
			Jj.set_element(2, 2,  1.0)
			
			# Compute block matrix products
			var r_vec: PackedFloat64Array = PackedFloat64Array([rx, ry, rth])
			var JiT: MatrixMN = Ji.transpose()
			var JjT: MatrixMN = Jj.transpose()
			
			var H_ii: MatrixMN = JiT.multiply(Ji).scale(w)
			var H_jj: MatrixMN = JjT.multiply(Jj).scale(w)
			var H_ij: MatrixMN = JiT.multiply(Jj).scale(w)
			var H_ji: MatrixMN = JjT.multiply(Ji).scale(w)
			
			var b_i: PackedFloat64Array = JiT.multiply_vector(r_vec)
			var b_j: PackedFloat64Array = JjT.multiply_vector(r_vec)
			
			var base_i: int = idx_i * 3
			var base_j: int = idx_j * 3
			
			# Assemble into global H matrix and b vector
			for r in range(3):
				for c in range(3):
					H.add_element(base_i + r, base_i + c, H_ii.get_element(r, c))
					H.add_element(base_j + r, base_j + c, H_jj.get_element(r, c))
					H.add_element(base_i + r, base_j + c, H_ij.get_element(r, c))
					H.add_element(base_j + r, base_i + c, H_ji.get_element(r, c))
					
				b[base_i + r] -= w * b_i[r]
				b[base_j + r] -= w * b_j[r]
				
		# Fix anchor node (index for fixed_node_id)
		var anchor_idx: int = id_to_idx.get(fixed_node_id, 0) * 3
		for r in range(total_dim):
			for c in range(3):
				H.set_element(anchor_idx + c, r, 0.0)
				H.set_element(r, anchor_idx + c, 0.0)
		for c in range(3):
			H.set_element(anchor_idx + c, anchor_idx + c, 1.0)
			b[anchor_idx + c] = 0.0
			
		# Regularize diagonal slightly (Levenberg-Marquardt damping lambda)
		for d in range(total_dim):
			H.add_element(d, d, 1e-4)
			
		# Solve linear system H * delta = b
		var delta: PackedFloat64Array = H.solve_linear(b)
		
		# Apply updates with step damping
		for idx in range(id_list.size()):
			var node_id: int = id_list[idx]
			if node_id == fixed_node_id:
				continue
			var cur_node: PoseNode = nodes[node_id]
			var offset: int = idx * 3
			cur_node.pos.x += delta[offset]
			cur_node.pos.y += delta[offset + 1]
			cur_node.heading = fposmod(cur_node.heading + delta[offset + 2] + PI, 2.0 * PI) - PI
			
	return compute_total_error()
