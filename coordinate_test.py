import numpy as np

# ENU to ARKit 坐标转换矩阵
enu_to_arkit = np.array([
    [ 0, -1,  0],
    [ 0,  0,  1],
    [-1,  0,  0]
])

# 假设 ENU 系下的旋转矩阵（示意）
R_enu = np.eye(3)  # 即单位矩阵，对应无旋转
t_enu = np.array([1, 2, 3])

# 转换位置和姿态
R_arkit = enu_to_arkit @ R_enu @ enu_to_arkit.T
t_arkit = enu_to_arkit @ t_enu

print(f"t_arkit = {t_arkit}")
print(f"R_arkit = {R_arkit}")