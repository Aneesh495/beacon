# core/math/matrix_mn.gd
class_name MatrixMN
extends RefCounted

# General M x N floating-point matrix with numerical linear algebra routines.
# Provides dense matrix operations, LU decomposition with partial pivoting,
# Cholesky factorization, QR decomposition via Householder reflections,
# Gauss-Jordan full-pivoting inversion, and linear system solvers.

var rows: int = 0
var cols: int = 0
var data: PackedFloat64Array = PackedFloat64Array()

func _init(p_rows: int = 0, p_cols: int = 0, initial_value: float = 0.0) -> void:
	rows = p_rows
	cols = p_cols
	var size: int = rows * cols
	data.resize(size)
	if initial_value != 0.0:
		for i in range(size):
			data[i] = initial_value

static func zeros(p_rows: int, p_cols: int) -> MatrixMN:
	return MatrixMN.new(p_rows, p_cols, 0.0)

static func identity(n: int) -> MatrixMN:
	var m: MatrixMN = MatrixMN.new(n, n, 0.0)
	for i in range(n):
		m.set_element(i, i, 1.0)
	return m

static func diag(elements: PackedFloat64Array) -> MatrixMN:
	var n: int = elements.size()
	var m: MatrixMN = MatrixMN.new(n, n, 0.0)
	for i in range(n):
		m.set_element(i, i, elements[i])
	return m

static func from_2d_array(arr: Array) -> MatrixMN:
	var r: int = arr.size()
	if r == 0:
		return MatrixMN.new(0, 0)
	var c: int = arr[0].size()
	var m: MatrixMN = MatrixMN.new(r, c)
	for i in range(r):
		for j in range(c):
			m.set_element(i, j, float(arr[i][j]))
	return m

static func from_vector(vec: PackedFloat64Array, as_column: bool = true) -> MatrixMN:
	var n: int = vec.size()
	if as_column:
		var m: MatrixMN = MatrixMN.new(n, 1)
		for i in range(n):
			m.set_element(i, 0, vec[i])
		return m
	else:
		var m: MatrixMN = MatrixMN.new(1, n)
		for i in range(n):
			m.set_element(0, i, vec[i])
		return m

func clone() -> MatrixMN:
	var c: MatrixMN = MatrixMN.new(rows, cols)
	c.data = data.duplicate()
	return c

func get_element(r: int, c: int) -> float:
	return data[r * cols + c]

func set_element(r: int, c: int, val: float) -> void:
	data[r * cols + c] = val

func add_element(r: int, c: int, delta: float) -> void:
	data[r * cols + c] += delta

func to_packed_column(c: int) -> PackedFloat64Array:
	var col_data: PackedFloat64Array = PackedFloat64Array()
	col_data.resize(rows)
	for i in range(rows):
		col_data[i] = get_element(i, c)
	return col_data

func to_packed_row(r: int) -> PackedFloat64Array:
	var row_data: PackedFloat64Array = PackedFloat64Array()
	row_data.resize(cols)
	for j in range(cols):
		row_data[j] = get_element(r, j)
	return row_data

func add(other: MatrixMN) -> MatrixMN:
	assert(rows == other.rows and cols == other.cols, "Matrix dimension mismatch in add")
	var result: MatrixMN = MatrixMN.new(rows, cols)
	var size: int = rows * cols
	for i in range(size):
		result.data[i] = data[i] + other.data[i]
	return result

func subtract(other: MatrixMN) -> MatrixMN:
	assert(rows == other.rows and cols == other.cols, "Matrix dimension mismatch in subtract")
	var result: MatrixMN = MatrixMN.new(rows, cols)
	var size: int = rows * cols
	for i in range(size):
		result.data[i] = data[i] - other.data[i]
	return result

func scale(scalar: float) -> MatrixMN:
	var result: MatrixMN = MatrixMN.new(rows, cols)
	var size: int = rows * cols
	for i in range(size):
		result.data[i] = data[i] * scalar
	return result

func multiply(other: MatrixMN) -> MatrixMN:
	assert(cols == other.rows, "Inner dimension mismatch in matrix multiplication (%d != %d)" % [cols, other.rows])
	var result: MatrixMN = MatrixMN.new(rows, other.cols, 0.0)
	var m: int = rows
	var k_dim: int = cols
	var n: int = other.cols
	
	# Cache-friendly row-oriented multiplication
	for i in range(m):
		var i_offset: int = i * k_dim
		var res_offset: int = i * n
		for k in range(k_dim):
			var a_ik: float = data[i_offset + k]
			if a_ik == 0.0:
				continue
			var other_offset: int = k * n
			for j in range(n):
				result.data[res_offset + j] += a_ik * other.data[other_offset + j]
	return result

func multiply_vector(vec: PackedFloat64Array) -> PackedFloat64Array:
	assert(cols == vec.size(), "Dimension mismatch in matrix-vector product")
	var result: PackedFloat64Array = PackedFloat64Array()
	result.resize(rows)
	for i in range(rows):
		var sum: float = 0.0
		var row_offset: int = i * cols
		for j in range(cols):
			sum += data[row_offset + j] * vec[j]
		result[i] = sum
	return result

func transpose() -> MatrixMN:
	var result: MatrixMN = MatrixMN.new(cols, rows)
	for i in range(rows):
		for j in range(cols):
			result.set_element(j, i, get_element(i, j))
	return result

func trace() -> float:
	assert(rows == cols, "Trace only defined for square matrices")
	var tr: float = 0.0
	for i in range(rows):
		tr += get_element(i, i)
	return tr

func frobenius_norm() -> float:
	var sum_sq: float = 0.0
	var size: int = rows * cols
	for i in range(size):
		sum_sq += data[i] * data[i]
	return sqrt(sum_sq)

# Computes LU decomposition with partial pivoting: P * A = L * U
# Returns dictionary: {"L": MatrixMN, "U": MatrixMN, "P": MatrixMN, "perm": PackedInt32Array, "det": float}
func decompose_lu() -> Dictionary:
	assert(rows == cols, "LU decomposition requires square matrix")
	var n: int = rows
	var L: MatrixMN = MatrixMN.identity(n)
	var U: MatrixMN = clone()
	var perm: PackedInt32Array = PackedInt32Array()
	perm.resize(n)
	for i in range(n):
		perm[i] = i
		
	var det_sign: float = 1.0
	
	for col in range(n):
		# Find maximum pivot in column
		var max_val: float = abs(U.get_element(col, col))
		var max_row: int = col
		for r in range(col + 1, n):
			var val: float = abs(U.get_element(r, col))
			if val > max_val:
				max_val = val
				max_row = r
				
		if max_val < 1e-15:
			# Singular or near-singular column
			continue
			
		if max_row != col:
			# Swap rows in U
			for j in range(n):
				var tmp_u: float = U.get_element(col, j)
				U.set_element(col, j, U.get_element(max_row, j))
				U.set_element(max_row, j, tmp_u)
			# Swap row prefixes in L
			for j in range(col):
				var tmp_l: float = L.get_element(col, j)
				L.set_element(col, j, L.get_element(max_row, j))
				L.set_element(max_row, j, tmp_l)
			# Update permutation
			var tmp_p: int = perm[col]
			perm[col] = perm[max_row]
			perm[max_row] = tmp_p
			det_sign = -det_sign
			
		var pivot: float = U.get_element(col, col)
		for r in range(col + 1, n):
			var factor: float = U.get_element(r, col) / pivot
			L.set_element(r, col, factor)
			U.set_element(r, col, 0.0)
			for c in range(col + 1, n):
				U.add_element(r, c, -factor * U.get_element(col, c))
				
	var P: MatrixMN = MatrixMN.zeros(n, n)
	for i in range(n):
		P.set_element(i, perm[i], 1.0)
		
	var det: float = det_sign
	for i in range(n):
		det *= U.get_element(i, i)
		
	return {"L": L, "U": U, "P": P, "perm": perm, "det": det}

# Computes Cholesky factorization: A = L * L^T for symmetric positive-definite matrix.
# Returns lower triangular matrix L.
func decompose_cholesky() -> MatrixMN:
	assert(rows == cols, "Cholesky requires square matrix")
	var n: int = rows
	var L: MatrixMN = MatrixMN.zeros(n, n)
	
	for i in range(n):
		for j in range(i + 1):
			var sum: float = 0.0
			for k in range(j):
				sum += L.get_element(i, k) * L.get_element(j, k)
				
			if i == j:
				var diag_val: float = get_element(i, i) - sum
				if diag_val <= 0.0:
					# Add small numerical regularization jitter if slightly indefinite
					diag_val = 1e-12
				L.set_element(i, j, sqrt(diag_val))
			else:
				var l_jj: float = L.get_element(j, j)
				if abs(l_jj) < 1e-15:
					l_jj = 1e-12
				L.set_element(i, j, (get_element(i, j) - sum) / l_jj)
	return L

# Computes QR decomposition via Householder reflectors: A = Q * R
# Returns dictionary: {"Q": MatrixMN, "R": MatrixMN}
func decompose_qr() -> Dictionary:
	var m: int = rows
	var n: int = cols
	var Q: MatrixMN = MatrixMN.identity(m)
	var R: MatrixMN = clone()
	
	var min_dim: int = mini(m - 1, n)
	for k in range(min_dim):
		# Compute Householder vector for column k from row k to m-1
		var x_norm_sq: float = 0.0
		for i in range(k, m):
			var val: float = R.get_element(i, k)
			x_norm_sq += val * val
		var x_norm: float = sqrt(x_norm_sq)
		if x_norm < 1e-15:
			continue
			
		var alpha: float = -signf(R.get_element(k, k)) * x_norm
		var v: PackedFloat64Array = PackedFloat64Array()
		v.resize(m)
		for i in range(m):
			v[i] = 0.0
		v[k] = R.get_element(k, k) - alpha
		for i in range(k + 1, m):
			v[i] = R.get_element(i, k)
			
		var v_norm_sq: float = 0.0
		for i in range(k, m):
			v_norm_sq += v[i] * v[i]
		if v_norm_sq < 1e-15:
			continue
			
		var beta: float = 2.0 / v_norm_sq
		
		# Apply Householder reflection to R: R = (I - beta * v * v^T) * R
		for j in range(k, n):
			var v_dot_col: float = 0.0
			for i in range(k, m):
				v_dot_col += v[i] * R.get_element(i, j)
			for i in range(k, m):
				R.add_element(i, j, -beta * v[i] * v_dot_col)
				
		# Apply Householder reflection to Q: Q = Q * (I - beta * v * v^T)
		for i in range(m):
			var q_dot_v: float = 0.0
			for j in range(k, m):
				q_dot_v += Q.get_element(i, j) * v[j]
			for j in range(k, m):
				Q.add_element(i, j, -beta * q_dot_v * v[j])
				
	return {"Q": Q, "R": R}

# Inverts square matrix using Gauss-Jordan elimination with full row pivoting.
# Returns inverted MatrixMN.
func inverse() -> MatrixMN:
	assert(rows == cols, "Inverse only defined for square matrices")
	var n: int = rows
	var aug: MatrixMN = MatrixMN.new(n, 2 * n, 0.0)
	
	# Build augmented matrix [A | I]
	for i in range(n):
		for j in range(n):
			aug.set_element(i, j, get_element(i, j))
		aug.set_element(i, n + i, 1.0)
		
	for col in range(n):
		# Locate pivot
		var max_val: float = abs(aug.get_element(col, col))
		var pivot_row: int = col
		for r in range(col + 1, n):
			var val: float = abs(aug.get_element(r, col))
			if val > max_val:
				max_val = val
				pivot_row = r
				
		assert(max_val > 1e-15, "Matrix is singular and cannot be inverted")
		
		# Swap rows if necessary
		if pivot_row != col:
			for j in range(2 * n):
				var tmp: float = aug.get_element(col, j)
				aug.set_element(col, j, aug.get_element(pivot_row, j))
				aug.set_element(pivot_row, j, tmp)
				
		# Scale pivot row to 1
		var pivot: float = aug.get_element(col, col)
		var inv_pivot: float = 1.0 / pivot
		for j in range(col, 2 * n):
			aug.set_element(col, j, aug.get_element(col, j) * inv_pivot)
			
		# Eliminate other rows
		for r in range(n):
			if r != col:
				var factor: float = aug.get_element(r, col)
				if abs(factor) > 1e-15:
					for j in range(col, 2 * n):
						aug.add_element(r, j, -factor * aug.get_element(col, j))
						
	# Extract inverted matrix
	var inv_m: MatrixMN = MatrixMN.new(n, n)
	for i in range(n):
		for j in range(n):
			inv_m.set_element(i, j, aug.get_element(i, n + j))
	return inv_m

# Solves linear system A * x = b for square matrix A using LU decomposition.
func solve_linear(b: PackedFloat64Array) -> PackedFloat64Array:
	assert(rows == cols and rows == b.size(), "Dimension mismatch in solve_linear")
	var lu: Dictionary = decompose_lu()
	var L: MatrixMN = lu["L"]
	var U: MatrixMN = lu["U"]
	var perm: PackedInt32Array = lu["perm"]
	var n: int = rows
	
	# Permute b: b_perm = P * b
	var y: PackedFloat64Array = PackedFloat64Array()
	y.resize(n)
	for i in range(n):
		var sum: float = b[perm[i]]
		for j in range(i):
			sum -= L.get_element(i, j) * y[j]
		y[i] = sum
		
	# Back substitution: U * x = y
	var x: PackedFloat64Array = PackedFloat64Array()
	x.resize(n)
	for i in range(n - 1, -1, -1):
		var sum: float = y[i]
		for j in range(i + 1, n):
			sum -= U.get_element(i, j) * x[j]
		var diag_u: float = U.get_element(i, i)
		assert(abs(diag_u) > 1e-15, "Singular matrix encountered during back substitution")
		x[i] = sum / diag_u
		
	return x

# Enforces symmetric positive-semi-definite structure (e.g. For Kalman covariance).
# Adds regularizing diagonal jitter if any diagonal term falls below floor_val.
func regularize_covariance(floor_val: float = 1e-9) -> MatrixMN:
	assert(rows == cols, "Covariance regularization requires square matrix")
	var n: int = rows
	var res: MatrixMN = MatrixMN.new(n, n)
	for i in range(n):
		for j in range(i, n):
			var avg: float = 0.5 * (get_element(i, j) + get_element(j, i))
			res.set_element(i, j, avg)
			res.set_element(j, i, avg)
		if res.get_element(i, i) < floor_val:
			res.set_element(i, i, floor_val)
	return res

func to_string_formatted() -> String:
	var s: String = "MatrixMN(%dx%d):\n" % [rows, cols]
	for i in range(rows):
		s += "  ["
		for j in range(cols):
			s += "%9.4f " % get_element(i, j)
		s += "]\n"
	return s
