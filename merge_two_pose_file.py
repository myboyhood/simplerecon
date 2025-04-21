# -*- coding: utf-8 -*-
import os
base_path = "/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/custom-raw-data/mountains_sim_raw_AB/"
file_a_path = base_path + 'matched_UAV_A_odom_poses.txt'
file_b_path = base_path + 'matched_UAV_B_odom_poses.txt'
output_path = base_path + 'merged_AB_pose.txt'

def load_pose_file(path):
    pose_list = []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#') or line.strip() == "":
                continue
            tokens = line.strip().split()
            timestamp = float(tokens[0])
            pose = [float(x) for x in tokens[1:]]
            pose_list.append((timestamp, pose))
    return pose_list

# 读取两个文件
poses_a = load_pose_file(file_a_path)
poses_b = load_pose_file(file_b_path)

# 合并并排序
merged = poses_a + poses_b
merged.sort(key=lambda x: x[0])  # 按时间戳排序

# 写入新文件
with open(output_path, 'w') as f:
    f.write("# timestamp cam_pose_t cam_pose_q\n")
    for timestamp, pose in merged:
        line = "{:.3f} {}\n".format(timestamp, ' '.join(['{:.3f}'.format(x) for x in pose]))
        f.write(line)

print("合并完成，输出文件为:", output_path)

