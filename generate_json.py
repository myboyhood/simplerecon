# -*- coding: utf-8 -*-
import os
import json
import shutil

from glob import glob
import numpy as np

# ===== 参数设置 =====
base_path = "/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/custom-raw-data/city_sim_raw/"
pose_txt = base_path + "time_camA_pose.txt"
image_folder = base_path + "image_raw"
output_image_folder = base_path + "renamed_image"
output_json_path = base_path + "capture.json"
intrinsics = [320.0, 320.0, 320.0, 240.0, 0.0]  # fx, fy, cx, cy, k1（畸变），仿真 Iris相机
# intrinsics = [383.949, 383.567, 316.075, 245.676, 0.0]  # fx, fy, cx, cy, k1（畸变），kun1 D455相机
resolution = [640, 480]

# ARkit x right, y up, z back
enu_to_arkit = np.array([
    [0, -1, 0],
    [0, 0, 1],
    [-1, 0, 0]
])

def read_pose_file(txt_file):
    poses = []
    with open(txt_file, 'r') as f:
        for line in f:
            if line.startswith("#") or line.strip() == "":
                continue
            parts = line.strip().split()
            timestamp = float(parts[0])
            t = list(map(float, parts[1:4]))
            q = list(map(float, parts[4:]))
            poses.append((timestamp, t, q))
    poses.sort(key=lambda x: x[0])
    return poses


def load_images(image_dir):
    images = glob(os.path.join(image_dir, "*.png"))
    timestamp_map = {}
    for img in images:
        basename = os.path.basename(img)
        ts_str = basename.replace('_', '.').replace('.png', '')
        try:
            ts_float = float(ts_str)
            timestamp_map[ts_float] = img
        except:
            print(f"Warning: Cannot parse timestamp from {basename}")
    return timestamp_map


def check_mismatches(pose_list, image_map):
    pose_ts_set = set(ts for ts, _, _ in pose_list)
    image_ts_set = set(image_map.keys())

    unmatched_poses = sorted(list(pose_ts_set - image_ts_set))
    unmatched_images = sorted(list(image_ts_set - pose_ts_set))

    if unmatched_poses:
        print("\n 以下 pose 有时间戳，但没有对应图像：")
        for ts in unmatched_poses:
            print(f" - pose timestamp: {ts}")
    if unmatched_images:
        print("\n 以下图像有时间戳，但没有对应 pose：")
        for ts in unmatched_images:
            print(f" - image timestamp: {ts}")
    if not unmatched_poses and not unmatched_images:
        print(" 所有图像和位姿匹配成功。")

    # 返回只保留匹配的
    matched = [(ts, t, q, image_map[ts]) for ts, t, q in pose_list if ts in image_map]
    return matched


def pose_to_matrix(t, q):
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    rot = R.from_quat(q).as_matrix()
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = t
    return mat
    # return mat.tolist()


def generate_capture_json(matched_entries, intrinsics, resolution):
    frames = []
    for seq, (ts, t, q, img_path) in enumerate(matched_entries):
        pose_mat = pose_to_matrix(t, q)
        #转乘ARKit的形式再存储写入,而且再转置一下
        rot_arkit = enu_to_arkit @ pose_mat[:3, :3] @ enu_to_arkit.T
        trans_arkit = enu_to_arkit @ pose_mat[:3, 3]
        arkit_mat = np.eye(4)
        arkit_mat[:3, :3] = rot_arkit
        arkit_mat[:3, 3] = trans_arkit

        arkit_mat_T = arkit_mat.T

        pose = t + q  # 7D pose [tx, ty, tz, qx, qy, qz, qw]
        frame_name = f"frame_{seq}.png"

        frame = {
            "image": frame_name,
            "timestamp": ts,
            "resolution": resolution,
            "sequence": seq,
            "intrinsics": intrinsics,
            "projection": [],  # Optional: fill if you have projection matrix
            "pose4x4": arkit_mat_T.tolist(),  # 4x4 pose matrix
            "pose": pose,
        }
        frames.append(frame)
    return {"frames": frames}


def rename_images(matched_entries, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    for i, (_, _, _, img_path) in enumerate(matched_entries):
        new_name = os.path.join(output_folder, f"frame_{i}.png")
        shutil.copy(img_path, new_name)


def main():
    pose_list = read_pose_file(pose_txt)
    image_map = load_images(image_folder)
    matched_entries = check_mismatches(pose_list, image_map)

    # 1. 重命名图像为 frame_0.jpg ...
    rename_images(matched_entries, output_image_folder)

    # 2. 生成 JSON 文件
    json_data = generate_capture_json(matched_entries, intrinsics, resolution)
    with open(output_json_path, 'w') as f:
        json.dump(json_data, f, indent=2)
    print(f"\n JSON 文件已保存至: {output_json_path}")
    print(f" 图像已复制并重命名至: {output_image_folder}/")


if __name__ == "__main__":
    main()
