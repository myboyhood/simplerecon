import open3d as o3d
print(o3d.__version__)
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_3.0_open3d_color/city_sim_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/mountains_snow_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/medical_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/paris_AB.ply")
mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/Evalley_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/vdr/dense/meshes/0.04_3.0_open3d_color/house.ply")
mesh.compute_vertex_normals()

# 创建可视化窗口并设置背景颜色
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)

# 设置背景为黑色
opt = vis.get_render_option()
opt.background_color = [0, 0, 0]  # 黑色背景 RGB

vis.run()
vis.destroy_window()