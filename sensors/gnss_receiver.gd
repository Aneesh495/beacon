# sensors/gnss_receiver.gd
class_name GnssReceiver
extends "res://sensors/sensor_base.gd"

# High-precision RTK GNSS receiver simulator.
# Generates local East-North-Up (ENU) coordinates with Dilution of Precision (DOP)
# covariance scaling, multipath urban canyon reflection errors, and loss-of-lock blackout.

enum FixType {
	NO_FIX = 0,
	AUTONOMOUS_2D = 1,
	AUTONOMOUS_3D = 2,
	DGPS = 3,
	RTK_FLOAT = 4,
	RTK_FIXED = 5
}

var fix_type: FixType = FixType.RTK_FIXED
var horizontal_accuracy_m: float = 0.02 # 2cm RTK fixed
var vertical_accuracy_m: float = 0.04 # 4cm RTK fixed
var hdop: float = 0.8
var vdop: float = 1.2

func _init(p_rate_hz: float = 5.0) -> void:
	super("gnss_receiver", p_rate_hz)

# Measures 3D position in local Cartesian ENU coordinates: [East=X, North=Y, Up=Z]
# Returns Dictionary: {"position": Vector3, "fix_type": FixType, "hdop": float, "accuracy_m": float}
func measure_position(true_pos_enu: Vector3) -> Dictionary:
	if state == SensorState.FAILED or state == SensorState.OFFLINE or fix_type == FixType.NO_FIX:
		return {"position": Vector3.ZERO, "fix_type": FixType.NO_FIX, "hdop": 99.9, "accuracy_m": INF}
		
	var noise_factor_h: float = horizontal_accuracy_m * hdop
	var noise_factor_v: float = vertical_accuracy_m * vdop
	
	if simulated_fault_active:
		if fault_type == "blackout" or fault_type == "tunnel":
			fix_type = FixType.NO_FIX
			return {"position": Vector3.ZERO, "fix_type": FixType.NO_FIX, "hdop": 99.9, "accuracy_m": INF}
		elif fault_type == "multipath":
			# Multipath step error (e.g. urban canyon signal bounce)
			noise_factor_h += 3.5
			noise_factor_v += 7.0
			
	var meas_x: float = true_pos_enu.x
	var meas_y: float = true_pos_enu.y
	var meas_z: float = true_pos_enu.z
	
	if noise_enabled:
		meas_x += _box_muller(noise_factor_h)
		meas_y += _box_muller(noise_factor_h)
		meas_z += _box_muller(noise_factor_v)
		
	mark_updated()
	return {
		"position": Vector3(meas_x, meas_y, meas_z),
		"fix_type": fix_type,
		"hdop": hdop,
		"accuracy_m": noise_factor_h
	}

func _box_muller(std_dev: float) -> float:
	var u1: float = maxf(1e-9, randf())
	var u2: float = randf()
	return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2) * std_dev
