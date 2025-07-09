import numpy as np
from scipy.spatial.transform import Rotation as R

def rpy_from_fixed_xyz(x_proj, y_proj, z_proj):
    # """
    # 给定新坐标系 B 的三个轴在旧坐标系 A 中的投影，计算外旋XYZ顺序的欧拉角（RPY）
    # 返回值为：弧度 + 角度（°）
    # """
    # 构造旋转矩阵 R：列是 B 坐标轴在 A 系中的投影
    R_mat = np.column_stack((x_proj, y_proj, z_proj))

    # 外旋 XYZ 相当于内旋 ZYX 的逆顺序
    rot = R.from_matrix(R_mat)
    rpy_rad = rot.as_euler('ZYX', degrees=False)[::-1]  # 外旋 XYZ
    rpy_deg = np.degrees(rpy_rad)

    return rpy_rad, rpy_deg

# 示例输入：B 系三个轴在 A 系下的投影向量
x_proj = np.array([-1.0,0.0, 0.0])
y_proj = np.array([0.0 ,-1.0, 0.0])
z_proj = np.array([0.0,0.0, 1.0])

# 计算外旋 RPY
rpy_rad, rpy_deg = rpy_from_fixed_xyz(x_proj, y_proj, z_proj)

# 输出结果
print("=== 外旋X→Y→Z ===")
print(f"Roll  = {rpy_rad[0]:.6f} rad ({rpy_deg[0]:.2f}°)")
print(f"Pitch = {rpy_rad[1]:.6f} rad ({rpy_deg[1]:.2f}°)")
print(f"Yaw   = {rpy_rad[2]:.6f} rad ({rpy_deg[2]:.2f}°)")
