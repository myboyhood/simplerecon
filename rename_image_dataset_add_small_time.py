# -*- coding: utf-8 -*-
import os
import shutil

image_folder = '/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/custom-raw-data/medical_raw_AB/_kun1_D455_camera_color_image_raw_compressed'
image_files = [f for f in os.listdir(image_folder) if f.endswith('.png')]
output_folder = os.path.join(image_folder, "renamed")
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 提取原始时间戳并排序
def filename_to_timestamp(name):
    # 例如 1734857224_698.png → 1734857224.698
    name = name.replace(".png", "")
    return float(name.replace("_", "."))

# 排序
image_files.sort(key=lambda f: filename_to_timestamp(f))

# 重命名并复制到新目录
for f in image_files:
    timestamp = filename_to_timestamp(f)
    new_timestamp = timestamp + 0.001
    # 保留3位小数，重新构造文件名，注意替换小数点为下划线
    new_name = "{:.3f}".format(new_timestamp).replace(".", "_") + ".png"
    src_path = os.path.join(image_folder, f)
    dst_path = os.path.join(output_folder, new_name)
    shutil.copyfile(src_path, dst_path)

print("图像重命名完成")
