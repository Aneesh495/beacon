# control/cascaded_pid_controller.gd
class_name CascadedPidController
extends "res://control/controller_base.gd"

# Cascaded PID longitudinal velocity and acceleration tracking controller.
# Includes physical aerodynamic drag, rolling resistance, and anti-windup clamping.
# Outer loop: velocity error -> target acceleration
# Inner loop: target acceleration + resistance compensation -> throttle/brake actuation.

# Vehicle physical parameters (typical autonomous electric sedan)
var vehicle_mass_kg: float = 1800.0
var frontal_area_m2: float = 2.4
var drag_coefficient: float = 0.28
var air_density_kgm3: float = 1.225
var rolling_resistance_coeff: float = 0.015
var gravity_mss: float = 9.80665

# Maximum powertrain actuator limits
var max_drive_force_n: float = 6000.0 # ~6 kN peak motor thrust
var max_brake_force_n: float = 12000.0 # ~12 kN hydraulic emergency braking
var max_brake_pressure_kpa: float = 8000.0 # 80 bar

# Outer loop velocity PID gains
var kp_vel: float = 1.2
var ki_vel: float = 0.15
var kd_vel: float = 0.05
var vel_integral_clamp: float = 2.0

var vel_integral_error: float = 0.0
var prev_vel_error: float = 0.0

func _init(p_mass: float = 1800.0) -> void:
	super("cascaded_pid", 2.8)
	vehicle_mass_kg = p_mass

# Computes throttle ratio [0, 1] and brake pressure (kPa) given current speed and target speed
func compute_longitudinal_control(
	current_speed_mps: float,
	target_speed_mps: float,
	dt: float = 0.05
) -> Dictionary:
	var vel_error: float = target_speed_mps - current_speed_mps
	
	# Integral with anti-windup clamping
	vel_integral_error += vel_error * dt
	vel_integral_error = clampf(vel_integral_error, -vel_integral_clamp, vel_integral_clamp)
	
	var d_vel_error: float = (vel_error - prev_vel_error) / dt if dt > 1e-6 else 0.0
	prev_vel_error = vel_error
	
	# Desired longitudinal acceleration command from outer loop
	var target_accel: float = kp_vel * vel_error + ki_vel * vel_integral_error + kd_vel * d_vel_error
	target_accel = clampf(target_accel, -5.0, 3.5) # Physical acceleration limits
	
	# Aerodynamic drag resistance force: F_aero = 0.5 * rho * Cd * A * v^2
	var f_aero: float = 0.5 * air_density_kgm3 * drag_coefficient * frontal_area_m2 * current_speed_mps * current_speed_mps
	# Rolling resistance force: F_roll = Crr * m * g
	var f_roll: float = rolling_resistance_coeff * vehicle_mass_kg * gravity_mss
	
	# Total required traction / braking force: F_req = m * a_des + F_aero + F_roll
	var required_force: float = vehicle_mass_kg * target_accel + f_aero + f_roll
	
	var throttle: float = 0.0
	var brake_kpa: float = 0.0
	
	if required_force > 0.0:
		# Driving thrust
		throttle = clampf(required_force / max_drive_force_n, 0.0, 1.0)
		brake_kpa = 0.0
	else:
		# Braking deceleration
		throttle = 0.0
		var brake_ratio: float = clampf(abs(required_force) / max_brake_force_n, 0.0, 1.0)
		brake_kpa = brake_ratio * max_brake_pressure_kpa
		
	current_throttle_ratio = throttle
	current_brake_pressure_kpa = brake_kpa
	
	return {
		"target_accel": target_accel,
		"throttle_ratio": throttle,
		"brake_pressure_kpa": brake_kpa,
		"required_force_n": required_force
	}
