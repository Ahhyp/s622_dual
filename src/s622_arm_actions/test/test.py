# import cv2, cv2.aruco as aruco, numpy as np

# img = cv2.imread('/tmp/aruco_frame.png')
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# # 试多个字典
# for dict_name in ['DICT_5X5_100', 'DICT_5X5_50', 'DICT_4X4_50']:
#     d = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
#     params = aruco.DetectorParameters_create()
#     corners, ids, rejected = aruco.detectMarkers(gray, d, parameters=params)
#     print(f'{dict_name}: ids={ids}, rejected={len(rejected)} candidates')

# # 特别看 rejected: 有候选说明 detector 找到了边框但没通过 ID 解码
import cv2, cv2.aruco as aruco, sys, os

path = os.path.expanduser('~/my_S622/src/gz_launch/models/aruco_marker_1/materials/textures/aruco_id1.png')
img = cv2.imread(path)
if img is None:
    print(f'ERROR: {path} not found')
    sys.exit(1)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
d = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
params = aruco.DetectorParameters_create()
result = aruco.detectMarkers(gray, d, parameters=params)
corners, ids = result[0], result[1] if len(result) >= 2 else None
print('detected in source PNG:', ids)   # 应该输出 [[1]]

