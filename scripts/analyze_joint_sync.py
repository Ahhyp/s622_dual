#!/usr/bin/env python3
"""分析双臂 j2 完成时间差。
CSV 格式: wall_time, ros_time, left_j1, left_j3, right_finger2, right_finger1,
           right_j5, left_j2, right_j4, right_j3, right_j2, left_j6, right_j1,
           left_j5, right_j6, left_finger2, left_j4, left_finger1
"""
import csv
import sys


def main(csv_path: str):
    TARGET = -1.5708  # -90° home
    TOLERANCE = 0.02  # 归位判定阈值

    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader, None)  # 跳过表头

        l_done_wall = l_done_ros = 0.0
        r_done_wall = r_done_ros = 0.0

        for row in reader:
            wall_t = float(row[0])
            ros_t = float(row[1])
            l_j2 = float(row[6])
            r_j2 = float(row[9])

            if not l_done_wall and abs(l_j2 - TARGET) < TOLERANCE:
                l_done_wall = wall_t
                l_done_ros = ros_t

            if not r_done_wall and abs(r_j2 - TARGET) < TOLERANCE:
                r_done_wall = wall_t
                r_done_ros = ros_t

        print(f'left_j2  最后归位 (wall): {l_done_wall:.3f}')
        print(f'right_j2 最后归位 (wall): {r_done_wall:.3f}')
        print(f'left_j2  最后归位 (ros):  {l_done_ros:.3f}')
        print(f'right_j2 最后归位 (ros):  {r_done_ros:.3f}')
        print()
        wall_diff = abs(r_done_wall - l_done_wall)
        ros_diff = abs(r_done_ros - l_done_ros)
        print(f'完成时间差 (wall): {wall_diff:.3f}s ({wall_diff*1000:.0f}ms)')
        print(f'完成时间差 (ros):  {ros_diff:.3f}s ({ros_diff*1000:.0f}ms)')


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dual_arm_joint_states2.csv'
    main(path)
