# tests/test_base.gd
class_name TestBase
extends RefCounted

# Base harness for test suites in Beacon.
# Provides robust numeric, matrix, vector, and structural assertion helpers.

var assertion_count: int = 0
var failure_messages: Array[String] = []

func reset_test() -> void:
	assertion_count = 0
	failure_messages.clear()

func assert_true(condition: bool, message: String = "Expected condition to be true") -> void:
	assertion_count += 1
	if not condition:
		failure_messages.append(message)

func assert_false(condition: bool, message: String = "Expected condition to be false") -> void:
	assertion_count += 1
	if condition:
		failure_messages.append(message)

func assert_equal(actual: Variant, expected: Variant, message: String = "") -> void:
	assertion_count += 1
	if actual != expected:
		var desc: String = message if message != "" else "Expected %s, received %s" % [str(expected), str(actual)]
		failure_messages.append(desc)

func assert_not_equal(actual: Variant, expected: Variant, message: String = "") -> void:
	assertion_count += 1
	if actual == expected:
		var desc: String = message if message != "" else "Expected value not equal to %s" % [str(expected)]
		failure_messages.append(desc)

func assert_null(actual: Variant, message: String = "Expected value to be null") -> void:
	assertion_count += 1
	if actual != null:
		failure_messages.append(message + " (received: %s)" % str(actual))

func assert_not_null(actual: Variant, message: String = "Expected value to not be null") -> void:
	assertion_count += 1
	if actual == null:
		failure_messages.append(message)

func assert_almost_equal(actual: float, expected: float, epsilon: float = 1e-4, message: String = "") -> void:
	assertion_count += 1
	var diff: float = abs(actual - expected)
	if diff > epsilon:
		var desc: String = message if message != "" else "Expected %.6f, received %.6f (diff: %.6e, eps: %.6e)" % [expected, actual, diff, epsilon]
		failure_messages.append(desc)

func assert_vector2_almost_equal(actual: Vector2, expected: Vector2, epsilon: float = 1e-4, message: String = "") -> void:
	assertion_count += 1
	var dist: float = actual.distance_to(expected)
	if dist > epsilon:
		var desc: String = message if message != "" else "Vector2 mismatch: expected %s, got %s (dist: %.6e)" % [str(expected), str(actual), dist]
		failure_messages.append(desc)

func assert_vector3_almost_equal(actual: Vector3, expected: Vector3, epsilon: float = 1e-4, message: String = "") -> void:
	assertion_count += 1
	var dist: float = actual.distance_to(expected)
	if dist > epsilon:
		var desc: String = message if message != "" else "Vector3 mismatch: expected %s, got %s (dist: %.6e)" % [str(expected), str(actual), dist]
		failure_messages.append(desc)

func assert_in_range(val: float, min_val: float, max_val: float, message: String = "") -> void:
	assertion_count += 1
	if val < min_val or val > max_val:
		var desc: String = message if message != "" else "Value %.6f out of range [%.6f, %.6f]" % [val, min_val, max_val]
		failure_messages.append(desc)

func get_results() -> Dictionary:
	return {
		"passed": failure_messages.is_empty(),
		"assertions": assertion_count,
		"failures": failure_messages.duplicate()
	}
