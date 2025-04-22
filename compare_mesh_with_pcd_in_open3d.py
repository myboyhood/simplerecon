import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
print(o3d.__version__)

def transform_mesh(mesh, translation, rpy_deg, scale=1.0):
    """
    对 mesh 应用缩放、平移和欧拉角旋转变换
    :param mesh: Open3D TriangleMesh
    :param translation: list [x, y, z]
    :param rpy_deg: list [roll, pitch, yaw] in degrees
    :param scale: float or list [sx, sy, sz]
    :return: transformed mesh
    """
    # 缩放
    mesh.scale(scale, center=(0, 0, 0))  # 支持 float 或 list

    # 将角度转为弧度
    rpy_rad = np.radians(rpy_deg)

    # 创建旋转矩阵（从 RPY）
    rotation_matrix = R.from_euler('xyz', rpy_rad).as_matrix()

    # 应用旋转和平移
    mesh.rotate(rotation_matrix, center=(0, 0, 0))
    mesh.translate(translation)
    return mesh

# 读取 mesh
mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/city_sim_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/mountains_snow_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/medical_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/paris_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/custom/dense/meshes/0.04_50.0_open3d_color/Evalley_AB.ply")
# mesh = o3d.io.read_triangle_mesh("/home/zph/projects/simplerecon/OUTPUT_PATH/HERO_MODEL/vdr/dense/meshes/0.04_3.0_open3d_color/house.ply")
mesh.compute_vertex_normals()

# ==== 加载点云 ====
# pcd = o3d.io.read_point_cloud("/home/zph/hard_disk/rosbag/lidar/SJTU_campus/outdoor-SJTU-campus-202412/E-vallley-V3-rgb_pt.pcd")
# pcd = o3d.io.read_point_cloud("/home/zph/hard_disk/rosbag/lidar/SJTU_campus/outdoor-SJTU-campus-202412/Paris-v2-rgb_pt.pcd")
# pcd = o3d.io.read_point_cloud("/home/zph/hard_disk/rosbag/lidar/SJTU_campus/outdoor-SJTU-campus-202412/medical-V3-rgb_pt.pcd")
pcd = o3d.io.read_point_cloud("/home/zph/hard_disk/rosbag/two_uavs_fly_outdoor/GazeboSim/city-rtx/depthAlign/single_image/lidar_ground_truth/lidar_gt_GazeboSim_cityrtx_259_324000.pcd")
# pcd = o3d.io.read_point_cloud("/home/zph/hard_disk/rosbag/two_uavs_fly_outdoor/GazeboSim/mountains_snow/depthAlign/single_image/lidar_ground_truth/lidar_gt_GazeboSim_mountains_snow_62_508000.pcd")

# # ==== 为点云添加红→青渐变颜色 ====
# points = np.asarray(pcd.points)
# x_vals = points[:, 0]
# x_min, x_max = x_vals.min(), x_vals.max()
# x_norm = (x_vals - x_min) / (x_max - x_min + 1e-8)  # Normalize to [0,1]
#
# # 红 -> 青渐变（红→绿→蓝）
# colors = np.zeros_like(points)
# colors[:, 0] = 1 - x_norm           # R: 从1到0
# colors[:, 1] = x_norm * (1 - x_norm) * 4  # G: 中间区域高
# colors[:, 2] = x_norm              # B: 从0到1
# pcd.colors = o3d.utility.Vector3dVector(colors)


# ==== Z 轴渐变颜色编码 ====
points = np.asarray(pcd.points)
z_vals = points[:, 2]
z_min, z_max = z_vals.min(), z_vals.max()
z_max = 25
z_norm = (z_vals - z_min) / (z_max - z_min + 1e-8)  # Normalize to [0,1]

# 红 → 青渐变（红->绿->蓝）
colors = np.zeros_like(points)
colors[:, 0] = 1 - z_norm           # R: 从1到0
colors[:, 1] = z_norm * (1 - z_norm) * 4  # G: 中间高亮
colors[:, 2] = z_norm              # B: 从0到1
pcd.colors = o3d.utility.Vector3dVector(colors)


# ====== 手动调节这些参数来对齐 mesh 到点云 ======
# ================= Evalley_AB =================
# translation = [-3.0, 14.0, -2.0]      # 单位：米
# rpy_deg = [0.0, 0.0, -90]          # 单位：度
# scale = 5.0                        # 缩放比例（可以是 float 或 [sx, sy, sz]）

# ================= paris_AB =================
# translation = [0.0, 0.0, 0.0]      # 单位：米
# rpy_deg = [0.0, 0.0, -90]          # 单位：度
# scale = 5.0                        # 缩放比例（可以是 float 或 [sx, sy, sz]）

# ================= medical_AB =================
# translation = [-5.0, -1.0, -2.0]      # 单位：米
# rpy_deg = [0.0, 0.0, -90]          # 单位：度
# scale = 3.0                        # 缩放比例（可以是 float 或 [sx, sy, sz]）

# ================= city_sim_AB =================
translation = [-3.0, 11.0, -5.00]      # 单位：米
rpy_deg = [0.0, 0.0, -90]          # 单位：度
scale = 5.0                        # 缩放比例（可以是 float 或 [sx, sy, sz]）

# ================= mountain_snow_AB =================
# translation = [-1280.0, 100.0, -455.0]      # 单位：米
# rpy_deg = [0.0, 0.0, -90]          # 单位：度
# scale = 20.0                        # 缩放比例（可以是 float 或 [sx, sy, sz]）

# 应用变换
mesh = transform_mesh(mesh, translation, rpy_deg, scale)

# ==== 显示结果 ====
# o3d.visualization.draw_geometries([mesh, pcd],
#                                   window_name="Align Mesh with GT PointCloud",
#                                   width=1280,
#                                   height=1280,
#                                   mesh_show_back_face=True)

# ==== 创建可视化窗口并设置黑色背景 ====
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Final Output", width=1580, height=1280)
vis.add_geometry(mesh)
vis.add_geometry(pcd)

# 设置背景颜色为黑色
opt = vis.get_render_option()
opt.background_color = np.array([0, 0, 0])

vis.run()
vis.destroy_window()
