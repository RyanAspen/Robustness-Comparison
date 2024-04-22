import numpy as np
from scipy.spatial.transform import Rotation as R

# NOTE: Change file paths as appropriate

kitti_ground_truth_file = open('00.txt', 'r')
# print(len(kitti_ground_truth_file.readlines()))
kitti_timestamp_file = open('/home/kgabel/collab_orb_slam2/dataset/sequences/00/times.txt', 'r')
# print(len(kitti_timestamp_file.readlines()))
kitti_ground_truth_converted_file = open('converted00.txt', 'w')
kitti_ground_truth_raw_lines = kitti_ground_truth_file.readlines()
kitti_timestamp_raw_lines = kitti_timestamp_file.readlines()
converted_lines = []
for i in range(len(kitti_ground_truth_raw_lines)):
    timestamp = float(kitti_timestamp_raw_lines[i])
    split_line = kitti_ground_truth_raw_lines[i].split()
    rotation_matrix = np.array([
        [float(split_line[0]), float(split_line[1]), float(split_line[2])],
        [float(split_line[4]), float(split_line[5]), float(split_line[6])],
        [float(split_line[8]), float(split_line[9]), float(split_line[10])]]
    )
    r = R.from_dcm(rotation_matrix)
    qx, qy, qz, qw = r.as_quat()
    tx, ty, tz = [float(split_line[3]), float(split_line[7]), float(split_line[11])]
    converted_line = [timestamp, tx, ty, tz, qx, qy, qz, qw]
    converted_line_str = ' '.join(str(x) for x in converted_line) + "\n"
    converted_lines.append(converted_line_str)
kitti_ground_truth_converted_file.writelines(converted_lines)
kitti_ground_truth_file.close()
kitti_timestamp_file.close()
kitti_ground_truth_converted_file.close()