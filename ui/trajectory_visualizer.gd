# ui/trajectory_visualizer.gd
class_name TrajectoryVisualizer
extends MeshInstance3D

# 3D immediate-mode renderer for vehicle trajectory ribbons, planned path polylines,
# road lanelet boundaries, and spatio-temporal reservation envelopes.

var immediate_mesh: ImmediateMesh = null
var default_material: StandardMaterial3D = null

func _init() -> void:
	immediate_mesh = ImmediateMesh.new()
	mesh = immediate_mesh
	
	default_material = StandardMaterial3D.new()
	default_material.shading_mode = StandardMaterial3D.SHADING_MODE_UNSHADED
	default_material.vertex_color_use_as_albedo = true
	material_override = default_material

# Renders a 3D line strip
func draw_polyline_3d(points: Array[Vector3], color: Color) -> void:
	if points.size() < 2:
		return
		
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for pt in points:
		immediate_mesh.surface_set_color(color)
		immediate_mesh.surface_add_vertex(pt)
	immediate_mesh.surface_end()

# Renders a 2D polyline elevated at y_elevation
func draw_polyline_2d(points: Array[Vector2], y_elevation: float, color: Color) -> void:
	if points.size() < 2:
		return
		
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for pt in points:
		immediate_mesh.surface_set_color(color)
		immediate_mesh.surface_add_vertex(Vector3(pt.x, y_elevation, pt.y))
	immediate_mesh.surface_end()

# Renders a continuous 3D ribbon along a 2D path
func draw_ribbon_2d(points: Array[Vector2], y_elevation: float, half_width: float, color: Color) -> void:
	var n: int = points.size()
	if n < 2:
		return
		
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP)
	for i in range(n):
		var p: Vector2 = points[i]
		var normal: Vector2 = Vector2.ZERO
		if i < n - 1:
			var tangent: Vector2 = (points[i + 1] - p).normalized()
			normal = Vector2(-tangent.y, tangent.x)
		else:
			var tangent: Vector2 = (p - points[i - 1]).normalized()
			normal = Vector2(-tangent.y, tangent.x)
			
		var left: Vector2 = p + normal * half_width
		var right: Vector2 = p - normal * half_width
		
		immediate_mesh.surface_set_color(color)
		immediate_mesh.surface_add_vertex(Vector3(left.x, y_elevation, left.y))
		immediate_mesh.surface_set_color(color)
		immediate_mesh.surface_add_vertex(Vector3(right.x, y_elevation, right.y))
	immediate_mesh.surface_end()

func clear_canvas() -> void:
	immediate_mesh.clear_surfaces()
