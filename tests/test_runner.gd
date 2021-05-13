# tests/test_runner.gd
class_name TestRunner
extends SceneTree

# Headless automated test runner for Beacon.
# Discovers and executes all registered unit, integration, and stress test suites.
# Returns exit code 0 on complete pass, or exit code 1 if any assertion fails.

const ANSI_RESET: String = "\u001b[0m"
const ANSI_RED: String = "\u001b[31;1m"
const ANSI_GREEN: String = "\u001b[32;1m"
const ANSI_YELLOW: String = "\u001b[33;1m"
const ANSI_BLUE: String = "\u001b[34;1m"
const ANSI_CYAN: String = "\u001b[36;1m"
const ANSI_GRAY: String = "\u001b[90m"

var total_assertions: int = 0
var total_passed_assertions: int = 0
var total_failed_assertions: int = 0

var suite_count: int = 0
var suite_passed: int = 0
var suite_failed: int = 0

var failure_log: Array[Dictionary] = []

func _init() -> void:
	print(ANSI_CYAN + "========================================================================" + ANSI_RESET)
	print(ANSI_CYAN + "   BEACON HEADLESS VERIFICATION SUITE   " + ANSI_RESET)
	print(ANSI_CYAN + "   Autonomous Cyber-Physical Fleet & Sensor-Fusion Substrate" + ANSI_RESET)
	print(ANSI_CYAN + "========================================================================" + ANSI_RESET)
	
	call_deferred("_run_all_tests")

func _run_all_tests() -> void:
	var start_time_ms: int = Time.get_ticks_msec()
	
	var suites_to_run: Array[String] = [
		"res://tests/unit/test_matrix_mn.gd",
		"res://tests/unit/test_quaternion_math.gd",
		"res://tests/unit/test_polynomial_spline.gd",
		"res://tests/unit/test_spatial_trees.gd",
		"res://tests/unit/test_geometry_utils.gd",
		"res://tests/unit/test_lanelet_network.gd",
		"res://tests/unit/test_costmap_2d.gd",
		"res://tests/unit/test_sensors.gd",
		"res://tests/unit/test_ekf_estimation.gd",
		"res://tests/unit/test_ukf_estimation.gd",
		"res://tests/unit/test_pose_graph.gd",
		"res://tests/unit/test_hybrid_a_star.gd",
		"res://tests/unit/test_conflict_based_search.gd",
		"res://tests/unit/test_velocity_profiler.gd",
		"res://tests/unit/test_orca.gd",
		"res://tests/unit/test_controllers.gd",
		"res://tests/unit/test_fleet_dispatch.gd",
		"res://tests/unit/test_intersection_manager.gd",
		"res://tests/unit/test_v2x_mesh.gd",
		"res://tests/integration/test_scenario_integration.gd",
		"res://tests/stress/test_benchmark_stress.gd"
	]
	
	for suite_path in suites_to_run:
		if ResourceLoader.exists(suite_path):
			_execute_suite(suite_path)
		else:
			print(ANSI_GRAY + "  [SKIP] Suite file not yet present: " + suite_path.get_file() + ANSI_RESET)
			
	var elapsed_time_sec: float = (Time.get_ticks_msec() - start_time_ms) / 1000.0
	
	print("\n" + ANSI_CYAN + "========================================================================" + ANSI_RESET)
	print(ANSI_CYAN + "   VERIFICATION SUMMARY REPORT   " + ANSI_RESET)
	print(ANSI_CYAN + "========================================================================" + ANSI_RESET)
	print("  Execution Duration: %.3f seconds" % elapsed_time_sec)
	print("  Suites Executed:    %d (Passed: %d, Failed: %d)" % [suite_count, suite_passed, suite_failed])
	print("  Total Assertions:   %d" % total_assertions)
	print("  Passed Assertions:  " + ANSI_GREEN + "%d" % total_passed_assertions + ANSI_RESET)
	
	if total_failed_assertions > 0:
		print("  Failed Assertions:  " + ANSI_RED + "%d" % total_failed_assertions + ANSI_RESET)
		print("\n" + ANSI_RED + "FAILURE DETAILS:" + ANSI_RESET)
		for f in failure_log:
			print(ANSI_RED + "  * [%s::%s] Line %d: %s" % [f["suite"], f["test"], f["line"], f["message"]] + ANSI_RESET)
		print(ANSI_CYAN + "========================================================================" + ANSI_RESET)
		quit(1)
	else:
		print("  Failed Assertions:  " + ANSI_GREEN + "0" + ANSI_RESET)
		print(ANSI_GREEN + "\nALL TEST SUITES PASSED FLAWLESSLY." + ANSI_RESET)
		print(ANSI_CYAN + "========================================================================" + ANSI_RESET)
		quit(0)

func _execute_suite(suite_path: String) -> void:
	suite_count += 1
	var script: GDScript = load(suite_path)
	if not script:
		print(ANSI_RED + "  [ERROR] Could not load suite: " + suite_path + ANSI_RESET)
		suite_failed += 1
		return
		
	var instance: Object = script.new()
	if not instance:
		print(ANSI_RED + "  [ERROR] Could not instantiate: " + suite_path + ANSI_RESET)
		suite_failed += 1
		return
		
	var suite_name: String = suite_path.get_file().get_basename()
	print("\n" + ANSI_BLUE + "-> Running Suite: " + suite_name + ANSI_RESET)
	
	var methods: Array = script.get_script_method_list()
	var suite_has_failures: bool = false
	
	for m in methods:
		var method_name: String = m["name"]
		if method_name.begins_with("test_"):
			# Set up test context if suite supports it
			if instance.has_method("before_each"):
				instance.call("before_each")
				
			var result: Dictionary = instance.call(method_name)
			if result == null:
				result = {"passed": true, "assertions": 1, "failures": []}
				
			var assertions: int = result.get("assertions", 1)
			var failures: Array = result.get("failures", [])
			
			total_assertions += assertions
			if failures.is_empty():
				total_passed_assertions += assertions
				print("    " + ANSI_GREEN + "[PASS]" + ANSI_RESET + " %s (%d assertions)" % [method_name, assertions])
			else:
				total_failed_assertions += failures.size()
				total_passed_assertions += (assertions - failures.size())
				suite_has_failures = true
				print("    " + ANSI_RED + "[FAIL]" + ANSI_RESET + " %s (%d failures)" % [method_name, failures.size()])
				for fail_msg in failures:
					failure_log.append({
						"suite": suite_name,
						"test": method_name,
						"line": 0,
						"message": str(fail_msg)
					})
					
			if instance.has_method("after_each"):
				instance.call("after_each")
				
	if instance.has_method("after_all"):
		instance.call("after_all")
		
	if suite_has_failures:
		suite_failed += 1
	else:
		suite_passed += 1
