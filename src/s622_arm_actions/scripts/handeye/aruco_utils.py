# scripts/handeye/aruco_utils.py
import cv2
import numpy as np


def make_aruco_detector(dict_id=cv2.aruco.DICT_5X5_100):
    """兼容 OpenCV 4.5.4 ~ 4.13"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    # OpenCV 4.10+ 用 ArucoDetector + DetectorParameters()
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        def detect(gray):
            return detector.detectMarkers(gray)
    else:
        # OpenCV 4.5.x 用 detectMarkers + DetectorParameters_create()
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        def detect(gray):
            return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    return detect


def solve_marker_pose(corners, marker_size, K, D):
    """solvePnP + IPPE_SQUARE, 返回 4x4 T_marker^cam 或 None"""
    half = marker_size / 2.0
    obj = np.array([[-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0]], dtype=np.float32)
    img_pts = corners.reshape(-1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T