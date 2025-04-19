import open3d as o3d

mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/vdr/dense/meshes/0.04_3.0_open3d_color/house.ply")
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])