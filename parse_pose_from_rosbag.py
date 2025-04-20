#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import os
import sys
import math
from nav_msgs.msg import Odometry

# 用户输入：修改这些路径
bag_path = "/home/zph/hard_disk/rosbag/match_test/indoor/B201_lab/indoor-B201-solo-kun1_image_imu_odom_2023-06-07-10-49-44.bag"  # 替换为你的 bag 路径
image_dir = "/home/zph/hard_disk/rosbag/match_test/indoor/B201_lab/solo-kun1/_kun1_D455_camera_color_image_raw_compressed"  # 替换为你的图像文件夹
output_txt_path = "/home/zph/hard_disk/rosbag/match_test/indoor/B201_lab/matched_odom_poses.txt"
topic_name = "/kun1/kun1_camera/odom/sample"
t_start = 1686106200.0
t_end = 1686106222.0

def parse_image_timestamps(image_dir):
    timestamps = []
    for fname in os.listdir(image_dir):
        if fname.endswith(".png"):
            name = os.path.splitext(fname)[0]  # 去掉 .png
            name = name.replace("_", ".")  # 把下划线还原为小数点
            try:
                ts = float(name)
                timestamps.append((ts, fname))
            except:
                continue
    timestamps.sort()  # 按时间戳排序
    return timestamps

def main():
    if not os.path.exists(bag_path):
        print("Bag file not found: {}".format(bag_path))
        return

    if not os.path.exists(image_dir):
        print("Image folder not found: {}".format(image_dir))
        return

    print("提取图像时间戳...")
    image_ts_list = parse_image_timestamps(image_dir)

    print("读取 rosbag 位姿...")
    pose_list = []
    bag = rosbag.Bag(bag_path, 'r')
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        timestamp = t.to_sec()
        if timestamp < t_start:
            continue
        if timestamp > t_end:
            break

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        pose_list.append((timestamp, pos, ori))
    bag.close()

    if len(pose_list) == 0:
        print("未读取到任何位姿信息")
        return

    print("为图像匹配最近位姿...")
    with open(output_txt_path, 'w') as f_out:
        f_out.write("# timestamp cam_pose_t cam_pose_q\n")
        for ts_img, fname in image_ts_list:
            min_dt = float("inf")
            best_pose = None
            for ts_pose, pos, ori in pose_list:
                dt = abs(ts_pose - ts_img)
                if dt < min_dt:
                    min_dt = dt
                    best_pose = (pos, ori)
            if best_pose is not None:
                pos, ori = best_pose
                line = "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n" % (
                    ts_img, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w
                )
                f_out.write(line)

    print("匹配完成，写入文件 {}".format(output_txt_path))

if __name__ == "__main__":
    main()
