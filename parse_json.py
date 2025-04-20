import json
import numpy as np
from scipy.spatial.transform import Rotation as R

json_path = '/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/vdr/scans/living_room/capture.json'
output_txt = '/home/zph/hard_disk/rosbag/MVS_work/simpleRecon/vdr/scans/living_room/poses_from_json.txt'

with open(json_path, 'r') as f:
    data = json.load(f)

frames = data['frames']
frames.sort(key=lambda x: x['sequence'])  # 确保按序号顺序

with open(output_txt, 'w') as f:
    f.write("# sequence timestamp tx ty tz qx qy qz qw\n")
    for frame in frames:
        seq = frame['sequence']
        ts = frame['timestamp']
        pose_mat = np.array(frame['pose4x4']).reshape(4, 4).T

        t = pose_mat[:3, 3]
        R_mat = pose_mat[:3, :3]
        q = R.from_matrix(R_mat).as_quat()  # [qx, qy, qz, qw]

        line = f"{seq} {ts:.9f} " + " ".join(f"{v:.10f}" for v in t) + " " + " ".join(f"{v:.10f}" for v in q) + "\n"
        f.write(line)

print(f"写入完成，共 {len(frames)} 帧，输出文件: {output_txt}")
