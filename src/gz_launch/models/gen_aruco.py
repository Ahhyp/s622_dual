# # gen_aruco.py
# import cv2
# import cv2.aruco as aruco
# import os

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)

# # ID 0: 5cm 测试 marker (放世界里)
# # ID 1: 4cm 夹爪 marker
# for marker_id in [0, 1]:
#     img = aruco.drawMarker(aruco_dict, marker_id, 1000)
#     bordered = cv2.copyMakeBorder(
#         img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
#     out_dir = f'aruco_marker_{marker_id}/materials/textures'
#     os.makedirs(out_dir, exist_ok=True)
#     cv2.imwrite(f'{out_dir}/aruco_id{marker_id}.png', bordered)

# print('generated aruco_marker_0/, aruco_marker_1/')

import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)

for marker_id in [0, 1, 2]:
    # OpenCV >= 4.7
    img = aruco.drawMarker(aruco_dict, marker_id, 1000)
    # 加白边 (quiet zone) — 必须有,否则检测困难
    border = 100
    bordered = cv2.copyMakeBorder(
        img, border, border, border, border,
        cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f'/home/yep/my_S622/src/gz_launch/models/aruco_id{marker_id}.png', bordered)
print('done')