# -*- coding: utf-8 -*-
import os
import shutil

image_folder = '/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/custom/city-sim/images'
image_files = [f for f in os.listdir(image_folder) if f.endswith('.png')]
output_folder = os.path.join(image_folder, "renamed")
os.makedirs(output_folder, exist_ok=True)

# 提取原始时间戳并排序
def filename_to_timestamp(name):
    # 例如 1734857224_698.png → 1734857224.698
    name = name.replace(".png", "")
    return float(name.replace("_", "."))

image_files.sort(key=lambda f: filename_to_timestamp(f))

# 重命名为 frame_0.png, frame_1.png, ...
for i, old_name in enumerate(image_files):
    old_path = os.path.join(image_folder, old_name)
    new_name = f"frame_{i}.png"
    # new_path = os.path.join(image_folder, new_name)
    shutil.copy(old_path, os.path.join(output_folder, new_name)) # 用 copy 保留原图，如果不需要原图可以用 os.rename
    print(f"{old_name} -> {new_name}")

print("图像重命名完成")
