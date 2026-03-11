"""
Usage:
python pose_estimation_multi_targets.py --type DICT_5X5_100
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from utils import ARUCO_DICT

# ==========================================
# [사용자 정의 상수]
# ==========================================
# 마커 중심 좌표 (이전과 동일)
MARKER_CENTERS = {
    4: [-0.035, 0.035, 0.0],       # TL
    1: [0.255, 0.03, 0.0],         # TR
    2: [0.255, 0.135, 0.025],      # BR (Z=2.5cm)
    3: [-0.035, 0.135, 0.025]      # BL (Z=2.5cm)
}

# --- 타겟 좌표 정의 (로컬 좌표계) ---
# Y=14.7cm, Z=5.5cm (바닥) 공통
# 중앙: X=11cm, 가이드: X=3cm, 19cm
COMMON_Y = 0.147
DEPTH_LV1 = 0.025 # 1단 깊이 (Slot/Guide 시작)
DEPTH_LV2 = 0.055 # 2단 깊이 (Real Hole, 2.5 + 3.0)

TARGETS = {
    "CENTER": {
        "pos": [0.11, COMMON_Y, DEPTH_LV2], 
        "shape": "slot", 
        "size_lv1": (0.072, 0.04), # 긴지름, 짧은지름
        "size_lv2": 0.026          # 바닥 구멍 지름
    },
    "GUIDE_L": {
        "pos": [0.03, COMMON_Y, DEPTH_LV2], 
        "shape": "circle", 
        "size_lv1": 0.04,          # 가이드 지름
        "size_lv2": 0.026          # 바닥 구멍 지름
    },
    "GUIDE_R": {
        "pos": [0.19, COMMON_Y, DEPTH_LV2], 
        "shape": "circle", 
        "size_lv1": 0.04, 
        "size_lv2": 0.026
    }
}

MIN_MARKERS_REQUIRED = 3
# ==========================================

def get_intrinsics_from_stream(profile):
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream.get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=float)
    D = np.array(intr.coeffs, dtype=float)
    return K, D

def draw_projected_shape(img, K, D, rvec, tvec, target_name, info):
    """3D 형상을 2D 이미지에 투영하여 그리기"""
    
    # 1. 중심 좌표 계산 (로컬 -> 카메라)
    pos_local = np.array(info["pos"], dtype=np.float32).reshape(3, 1)
    
    # 바닥 구멍 (Level 2, Z=5.5cm) 위치 변환
    R, _ = cv2.Rodrigues(rvec)
    t_target_cam = R @ pos_local + tvec
    
    # 투영 (Project Points)
    imgpts, _ = cv2.projectPoints(t_target_cam, np.zeros(3), np.zeros(3), K, D)
    cx, cy = imgpts[0].ravel().astype(int)
    
    # 2. 시각화: 바닥 구멍 (Real Hole, 지름 2.6cm) - 빨간색 점
    cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
    cv2.putText(img, target_name, (cx-10, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    # 3. 시각화: 상단 윤곽선 (Level 1, Z=2.5cm) 그리기
    # 로컬 좌표계에서 윤곽선 점들을 생성한 뒤 투영
    pos_lv1 = pos_local.copy()
    pos_lv1[2] = DEPTH_LV1 # 깊이를 2.5cm로 변경
    
    contour_points = []
    steps = 20
    
    if info["shape"] == "circle":
        radius = info["size_lv1"] / 2.0
        for i in range(steps):
            theta = 2 * np.pi * i / steps
            x = pos_lv1[0] + radius * np.cos(theta)
            y = pos_lv1[1] + radius * np.sin(theta)
            contour_points.append([x[0], y[0], pos_lv1[2][0]])
            
    elif info["shape"] == "slot":
        rx = info["size_lv1"][0] / 2.0
        ry = info["size_lv1"][1] / 2.0
        for i in range(steps):
            theta = 2 * np.pi * i / steps
            x = pos_lv1[0] + rx * np.cos(theta)
            y = pos_lv1[1] + ry * np.sin(theta)
            contour_points.append([x[0], y[0], pos_lv1[2][0]])

    # 윤곽선 투영 및 그리기 (초록색 실선)
    if contour_points:
        contour_3d = np.array(contour_points, dtype=np.float32)
        # R, tvec을 적용하여 카메라 좌표계로 변환
        contour_cam = (R @ contour_3d.T).T + tvec.reshape(1, 3)
        
        imgpts_cont, _ = cv2.projectPoints(contour_cam, np.zeros(3), np.zeros(3), K, D)
        imgpts_cont = imgpts_cont.reshape(-1, 2).astype(np.int32)
        
        cv2.polylines(img, [imgpts_cont], True, (0, 255, 0), 1)

    return t_target_cam

def run_estimation(args):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    profile = pipeline.start(config)
    K, D = get_intrinsics_from_stream(profile)

    print("--- Multi-Target Pose Estimation Started ---")
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue
            
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 마커 검출
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

            detected_count = 0
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                image_points = []
                object_points = []
                
                for i, marker_id in enumerate(ids.flatten()):
                    mid = int(marker_id)
                    if mid in MARKER_CENTERS:
                        c = corners[i][0]
                        cx, cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                        image_points.append([cx, cy])
                        object_points.append(MARKER_CENTERS[mid])
                        detected_count += 1

                # solvePnP 실행
                if detected_count >= MIN_MARKERS_REQUIRED:
                    success, rvec, tvec = cv2.solvePnP(
                        np.array(object_points, dtype=np.float32), 
                        np.array(image_points, dtype=np.float32), 
                        K, D, flags=cv2.SOLVEPNP_SQPNP
                    )
                    
                    if success:
                        # 물체 원점 축 그리기
                        cv2.drawFrameAxes(frame, K, D, rvec, tvec, 0.05)
                        
                        # 모든 타겟(구멍) 그리기 및 좌표 표시
                        for name, info in TARGETS.items():
                            t_target = draw_projected_shape(frame, K, D, rvec, tvec, name, info)
                            
                            # 거리 정보 표시 (중앙 홀만)
                            if name == "CENTER":
                                dist = np.linalg.norm(t_target)
                                cv2.putText(frame, f"Center Dist: {dist*100:.1f}cm", (20, 50), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, f"Markers: {detected_count}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Multi-Target Pose", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", type=str, default="DICT_4X4_50")
    args = vars(parser.parse_args())
    run_estimation(args)