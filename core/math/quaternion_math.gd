# core/math/quaternion_math.gd
class_name QuaternionMath
extends RefCounted

# Lie group SO(3) and unit quaternion kinematic calculations.
# Implements exponential/logarithmic maps, skew-symmetric matrices,
# Euler transformations, and strapdown inertial integration for attitude estimation.

const EPSILON: float = 1e-12

# Converts 3-vector into 3x3 skew-symmetric matrix (hat operator)
# [v]^ = [  0  -v3   v2]
#        [ v3    0  -v1]
#        [-v2   v1    0]
static func skew_symmetric(v: Vector3) -> MatrixMN:
	var m: MatrixMN = MatrixMN.zeros(3, 3)
	m.set_element(0, 1, -v.z)
	m.set_element(0, 2,  v.y)
	m.set_element(1, 0,  v.z)
	m.set_element(1, 2, -v.x)
	m.set_element(2, 0, -v.y)
	m.set_element(2, 1,  v.x)
	return m

# Extracts 3-vector from 3x3 skew-symmetric matrix (vee operator)
static func unskew(m: MatrixMN) -> Vector3:
	return Vector3(
		m.get_element(2, 1),
		m.get_element(0, 2),
		m.get_element(1, 0)
	)

# SO(3) Exponential Map: Maps rotation vector omega in so(3) to 3x3 rotation matrix
# Uses Rodrigues' rotation formula: R = I + (sin(theta)/theta) * [w]^ + ((1 - cos(theta))/theta^2) * [w]^^2
static func exp_so3(w: Vector3) -> MatrixMN:
	var theta: float = w.length()
	var eye: MatrixMN = MatrixMN.identity(3)
	if theta < EPSILON:
		return eye
		
	var w_hat: MatrixMN = skew_symmetric(w)
	var w_hat_sq: MatrixMN = w_hat.multiply(w_hat)
	
	var a: float = sin(theta) / theta
	var b: float = (1.0 - cos(theta)) / (theta * theta)
	
	var term1: MatrixMN = w_hat.scale(a)
	var term2: MatrixMN = w_hat_sq.scale(b)
	
	return eye.add(term1).add(term2)

# SO(3) Logarithmic Map: Maps 3x3 rotation matrix R to rotation vector omega in so(3)
static func log_so3(R: MatrixMN) -> Vector3:
	var tr: float = R.trace()
	var cos_theta: float = clampf(0.5 * (tr - 1.0), -1.0, 1.0)
	var theta: float = acos(cos_theta)
	
	if theta < EPSILON:
		return Vector3.ZERO
		
	var factor: float = theta / (2.0 * sin(theta))
	var diff: MatrixMN = R.subtract(R.transpose())
	var w_scaled: Vector3 = unskew(diff)
	return w_scaled * factor

# Maps rotation vector omega in R^3 to unit Quaternion
static func exp_quaternion(w: Vector3) -> Quaternion:
	var theta: float = w.length()
	if theta < EPSILON:
		return Quaternion.IDENTITY
	var half_theta: float = 0.5 * theta
	var axis: Vector3 = w / theta
	var s: float = sin(half_theta)
	return Quaternion(axis.x * s, axis.y * s, axis.z * s, cos(half_theta)).normalized()

# Maps unit Quaternion to rotation vector omega in R^3 (Logarithmic map)
static func log_quaternion(q: Quaternion) -> Vector3:
	var q_norm: Quaternion = q.normalized()
	var v: Vector3 = Vector3(q_norm.x, q_norm.y, q_norm.z)
	var v_len: float = v.length()
	if v_len < EPSILON:
		return Vector3.ZERO
	var theta: float = 2.0 * atan2(v_len, q_norm.w)
	return (v / v_len) * theta

# Converts unit Quaternion to 3x3 rotation matrix
static func quaternion_to_matrix3(q: Quaternion) -> MatrixMN:
	var qn: Quaternion = q.normalized()
	var x: float = qn.x
	var y: float = qn.y
	var z: float = qn.z
	var w: float = qn.w
	
	var m: MatrixMN = MatrixMN.zeros(3, 3)
	m.set_element(0, 0, 1.0 - 2.0 * (y * y + z * z))
	m.set_element(0, 1, 2.0 * (x * y - z * w))
	m.set_element(0, 2, 2.0 * (x * z + y * w))
	
	m.set_element(1, 0, 2.0 * (x * y + z * w))
	m.set_element(1, 1, 1.0 - 2.0 * (x * x + z * z))
	m.set_element(1, 2, 2.0 * (y * z - x * w))
	
	m.set_element(2, 0, 2.0 * (x * z - y * w))
	m.set_element(2, 1, 2.0 * (y * z + x * w))
	m.set_element(2, 2, 1.0 - 2.0 * (x * x + y * y))
	return m

# Converts 3x3 rotation matrix to unit Quaternion using Shepperd's algorithm
static func matrix3_to_quaternion(R: MatrixMN) -> Quaternion:
	var tr: float = R.trace()
	var q: Quaternion = Quaternion.IDENTITY
	
	if tr > 0.0:
		var s: float = sqrt(tr + 1.0) * 2.0 # s = 4 * w
		q.w = 0.25 * s
		q.x = (R.get_element(2, 1) - R.get_element(1, 2)) / s
		q.y = (R.get_element(0, 2) - R.get_element(2, 0)) / s
		q.z = (R.get_element(1, 0) - R.get_element(0, 1)) / s
	elif (R.get_element(0, 0) > R.get_element(1, 1)) and (R.get_element(0, 0) > R.get_element(2, 2)):
		var s: float = sqrt(1.0 + R.get_element(0, 0) - R.get_element(1, 1) - R.get_element(2, 2)) * 2.0 # s = 4 * x
		q.w = (R.get_element(2, 1) - R.get_element(1, 2)) / s
		q.x = 0.25 * s
		q.y = (R.get_element(0, 1) + R.get_element(1, 0)) / s
		q.z = (R.get_element(0, 2) + R.get_element(2, 0)) / s
	elif R.get_element(1, 1) > R.get_element(2, 2):
		var s: float = sqrt(1.0 + R.get_element(1, 1) - R.get_element(0, 0) - R.get_element(2, 2)) * 2.0 # s = 4 * y
		q.w = (R.get_element(0, 2) - R.get_element(2, 0)) / s
		q.x = (R.get_element(0, 1) + R.get_element(1, 0)) / s
		q.y = 0.25 * s
		q.z = (R.get_element(1, 2) + R.get_element(2, 1)) / s
	else:
		var s: float = sqrt(1.0 + R.get_element(2, 2) - R.get_element(0, 0) - R.get_element(1, 1)) * 2.0 # s = 4 * z
		q.w = (R.get_element(1, 0) - R.get_element(0, 1)) / s
		q.x = (R.get_element(0, 2) + R.get_element(2, 0)) / s
		q.y = (R.get_element(1, 2) + R.get_element(2, 1)) / s
		q.z = 0.25 * s
		
	return q.normalized()

# Converts Euler angles (roll = x, pitch = y, yaw = z in radians) to Quaternion (ZYX aerospace convention)
static func from_euler_rpy(rpy: Vector3) -> Quaternion:
	var roll: float = rpy.x
	var pitch: float = rpy.y
	var yaw: float = rpy.z
	
	var cr: float = cos(roll * 0.5)
	var sr: float = sin(roll * 0.5)
	var cp: float = cos(pitch * 0.5)
	var sp: float = sin(pitch * 0.5)
	var cy: float = cos(yaw * 0.5)
	var sy: float = sin(yaw * 0.5)
	
	var q: Quaternion = Quaternion(
		sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy,
		cr * cp * sy - sr * sp * cy,
		cr * cp * cy + sr * sp * sy
	)
	return q.normalized()

# Converts Quaternion to Euler angles (roll = x, pitch = y, yaw = z in radians)
static func to_euler_rpy(q: Quaternion) -> Vector3:
	var qn: Quaternion = q.normalized()
	# Roll (x-axis rotation)
	var sinr_cosp: float = 2.0 * (qn.w * qn.x + qn.y * qn.z)
	var cosr_cosp: float = 1.0 - 2.0 * (qn.x * qn.x + qn.y * qn.y)
	var roll: float = atan2(sinr_cosp, cosr_cosp)
	
	# Pitch (y-axis rotation)
	var sinp: float = 2.0 * (qn.w * qn.y - qn.z * qn.x)
	var pitch: float = 0.0
	if abs(sinp) >= 1.0:
		pitch = signf(sinp) * (PI * 0.5) # Gimbal lock 90 degrees
	else:
		pitch = asin(sinp)
		
	# Yaw (z-axis rotation)
	var siny_cosp: float = 2.0 * (qn.w * qn.z + qn.x * qn.y)
	var cosy_cosp: float = 1.0 - 2.0 * (qn.y * qn.y + qn.z * qn.z)
	var yaw: float = atan2(siny_cosp, cosy_cosp)
	
	return Vector3(roll, pitch, yaw)

# Discrete quaternion strapdown integration using zero-order hold on angular rate omega
# q_{k+1} = q_k * exp(0.5 * omega * dt)
static func integrate_quaternion(q_current: Quaternion, omega: Vector3, dt: float) -> Quaternion:
	var delta_rot: Quaternion = exp_quaternion(omega * dt)
	return (q_current * delta_rot).normalized()

# Computes 4x4 quaternion transition matrix Omega(omega) such that d/dt(q) = 0.5 * Omega(omega) * q
# Vector ordering: [qx, qy, qz, qw]^T
static func quaternion_kinematic_matrix(omega: Vector3) -> MatrixMN:
	var m: MatrixMN = MatrixMN.zeros(4, 4)
	var wx: float = omega.x
	var wy: float = omega.y
	var wz: float = omega.z
	
	m.set_element(0, 1,  wz)
	m.set_element(0, 2, -wy)
	m.set_element(0, 3,  wx)
	
	m.set_element(1, 0, -wz)
	m.set_element(1, 2,  wx)
	m.set_element(1, 3,  wy)
	
	m.set_element(2, 0,  wy)
	m.set_element(2, 1, -wx)
	m.set_element(2, 3,  wz)
	
	m.set_element(3, 0, -wx)
	m.set_element(3, 1, -wy)
	m.set_element(3, 2, -wz)
	
	return m.scale(0.5)
