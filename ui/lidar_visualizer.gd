# ui/lidar_visualizer.gd
class_name LidarVisualizer
extends MeshInstance3D

# 3D point cloud visualizer for multi-channel spinning LiDAR returns.
# Renders synthetic raycast point clouds with elevation/intensity color ramps
# and draws 3D bounding boxes around segmented obstacle clusters.

var immediate_mesh: ImmediateMesh = null
var point_material: StandardMaterial3D = null

func _init() -> void:
	immediate_mesh = ImmediateMesh.new()
	mesh = immediate_mesh
	
	point_material = StandardMaterial3D.new()
	point_material.shading_mode = StandardMaterial3D.SHADING_MODE_UNSHADED
	point_material.vertex_color_use_as_albedo = true
	point_material.use_point_size = true
	point_material.point_size = 3.0
	material_override = point_material

# Renders a 3D point cloud colored by elevation (Z/Y)
func render_point_cloud(points: PackedVector3Array, min_y: float = -0.5, max_y: float = 3.0) -> void:
	if points.is_empty():
		return
		
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_POINTS)
	var y_range: float = maxf(0.1, max_y - min_y)
	
	for i in range(points.size()):
		var pt: Vector3 = points[i]
		var norm_y: float = clampf((pt.y - min_y) / y_range, 0.0, 1.0)
		
		# Turbo/rainbow color map: Blue (ground) -> Green -> Yellow -> Red (high obstacles)
		var c: Color = Color.from_hsv(lerp(0.66, 0.0, norm_y), 0.9, 1.0)
		immediate_mesh.surface_set_color(c)
		immediate_mesh.surface_add_vertex(pt)
		
	immediate_mesh.surface_end()

# Renders 3D wireframe bounding boxes around obstacle clusters
func render_bounding_boxes(boxes: Array[AABB], box_color: Color = Color.RED) -> void:
	if boxes.is_empty():
		return
		
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	for b in boxes:
		var p0: Vector3 = b.position
		var p1: Vector3 = b.position + b.size
		
		# 8 corners
		var c000: Vector3 = Vector3(p0.x, p0.y, p0.z)
		var c001: Vector3 = Vector3(p0.x, p0.y, p1.z)
		var c010: Vector3 = Vector3(p0.x, p1.y, p0.z)
		var c011: Vector3 = Vector3(p0.x, p1.y, p1.z)
		var c100: Vector3 = Vector3(p1.x, p0.y, p0.z)
		var c101: Vector3 = Vector3(p1.x, p0.y, p1.z)
		var c110: Vector3 = Vector3(p1.x, p1.y, p0.z)
		var c111: Vector3 = Vector3(p1.x, p1.y, p1.z)
		
		# 12 edges
		_draw_edge(c000, c001, box_color)
		_draw_edge(c001, c101, box_color)
		_draw_edge(c101, c100, box_color)
		_draw_edge(c100, c000, box_color)
		
		_draw_edge(c010, c011, box_color)
		_draw_edge(c011, c111, box_color)
		_draw_edge(c111, c110, box_color)
		_draw_edge(c110, c010, box_color)
		
		_draw_edge(c000, c010, box_color)
		_draw_edge(c001, c011, box_color)
		_draw_edge(c100, c110, box_color)
		_draw_edge(c101, c111, box_color)
		
	immediate_mesh.surface_end()

func _draw_edge(v1: Vector3, v2: Vector3, col: Color) -> void:
	immediate_mesh.surface_set_color(col)
	immediate_mesh.surface_add_vertex(v1)
	immediate_mesh.surface_set_color(col)
	immediate_mesh.surface_add_vertex(v2)

func clear_points() -> void:
	immediate_mesh.clear_surfaces()
