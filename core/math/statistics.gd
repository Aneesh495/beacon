# core/math/statistics.gd
class_name Statistics
extends RefCounted

# Multivariate statistical methods, Mahalanobis distance calculation,
# Gaussian likelihoods, and Chi-square cumulative distribution lookup tables
# for Kalman filter validation gates and sensor fault detection.

const MatrixMN = preload("res://core/math/matrix_mn.gd")

# Chi-squared critical values table: degrees of freedom k in [1, 16]
# Keys: degrees of freedom (int) -> Dictionary: {"p05": float, "p01": float, "p001": float}
const CHI2_TABLE: Dictionary = {
	1:  {"p05": 3.841,  "p01": 6.635,  "p001": 10.828},
	2:  {"p05": 5.991,  "p01": 9.210,  "p001": 13.816},
	3:  {"p05": 7.815,  "p01": 11.345, "p001": 16.266},
	4:  {"p05": 9.488,  "p01": 13.277, "p001": 18.467},
	5:  {"p05": 11.070, "p01": 15.086, "p001": 20.515},
	6:  {"p05": 12.592, "p01": 16.812, "p001": 22.458},
	7:  {"p05": 14.067, "p01": 18.475, "p001": 24.322},
	8:  {"p05": 15.507, "p01": 20.090, "p001": 26.124},
	9:  {"p05": 16.919, "p01": 21.666, "p001": 27.877},
	10: {"p05": 18.307, "p01": 23.209, "p001": 29.588},
	11: {"p05": 19.675, "p01": 24.725, "p001": 31.264},
	12: {"p05": 21.026, "p01": 26.217, "p001": 32.909},
	13: {"p05": 22.362, "p01": 27.688, "p001": 34.528},
	14: {"p05": 23.685, "p01": 29.141, "p001": 36.123},
	15: {"p05": 24.996, "p01": 30.578, "p001": 37.697},
	16: {"p05": 26.296, "p01": 32.000, "p001": 39.252}
}

# Computes squared Mahalanobis distance: d_M^2 = (y - y_hat)^T * S^{-1} * (y - y_hat)
# where y is measurement residual, and S is innovation covariance matrix.
static func mahalanobis_distance_sq(residual: PackedFloat64Array, innovation_cov: MatrixMN) -> float:
	var n: int = residual.size()
	assert(innovation_cov.rows == n and innovation_cov.cols == n, "Residual and covariance dimension mismatch")
	
	# Solve S * x = residual => x = S^{-1} * residual
	var inv_s_residual: PackedFloat64Array = innovation_cov.solve_linear(residual)
	
	# Compute inner product: residual^T * (S^{-1} * residual)
	var d_sq: float = 0.0
	for i in range(n):
		d_sq += residual[i] * inv_s_residual[i]
		
	return maxf(0.0, d_sq)

# Validates whether a measurement innovation residual falls within the Chi-square acceptance gate
static func validate_mahalanobis_gate(residual: PackedFloat64Array, innovation_cov: MatrixMN, p_level: String = "p01") -> bool:
	var dof: int = residual.size()
	if not CHI2_TABLE.has(dof):
		# Fallback approximation for high degrees of freedom: Wilson-Hilferty transformation
		var z: float = 2.576 # 99% confidence z-score
		var threshold: float = dof * pow(1.0 - 2.0 / (9.0 * float(dof)) + z * sqrt(2.0 / (9.0 * float(dof))), 3)
		var d_sq: float = mahalanobis_distance_sq(residual, innovation_cov)
		return d_sq <= threshold
		
	var gate_threshold: float = float(CHI2_TABLE[dof].get(p_level, 9.210))
	var d_sq_calc: float = mahalanobis_distance_sq(residual, innovation_cov)
	return d_sq_calc <= gate_threshold

# Multivariate Gaussian log-likelihood: log p(x) = -0.5 * [ k*log(2*pi) + log|Sigma| + (x-mu)^T * Sigma^{-1} * (x-mu) ]
static func log_gaussian_likelihood(x: PackedFloat64Array, mean: PackedFloat64Array, cov: MatrixMN) -> float:
	var k: int = x.size()
	assert(mean.size() == k and cov.rows == k, "Dimension mismatch in Gaussian likelihood")
	
	var residual: PackedFloat64Array = PackedFloat64Array()
	residual.resize(k)
	for i in range(k):
		residual[i] = x[i] - mean[i]
		
	var d_sq: float = mahalanobis_distance_sq(residual, cov)
	var lu: Dictionary = cov.decompose_lu()
	var det: float = lu["det"]
	assert(det > 0.0, "Covariance determinant must be strictly positive")
	
	var log_det: float = log(det)
	var const_term: float = float(k) * log(2.0 * PI)
	
	return -0.5 * (const_term + log_det + d_sq)
