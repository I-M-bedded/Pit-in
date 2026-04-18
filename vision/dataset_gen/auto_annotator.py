"""
ArUco 마커 기반 이중 구멍 자동 어노테이션 도구 (YOLO Pose 버전)
================================================
RealSense 카메라 + ArUco 마커로 구멍 위치를 자동 계산하여
YOLO Pose 형식(.txt)으로 저장합니다.

YOLO Pose 포맷:
  <class_id> <x_center> <y_center> <width> <height> <px1> <py1> <v1> ... <px9> <py9> <v9>
  (모든 좌표는 이미지 너비/높이로 정규화된 0~1 범위)

클래스 정의:
  0 : center_hole_outer  - CENTER 슬롯 바깥쪽 윤곽 (LV1, Z=2.5cm)
  1 : center_hole_inner  - CENTER 슬롯 바닥 구멍   (LV2, Z=5.5cm)
  2 : guide_l_outer      - GUIDE_L 바깥쪽 윤곽
  3 : guide_l_inner      - GUIDE_L 바닥 구멍
  4 : guide_r_outer      - GUIDE_R 바깥쪽 윤곽
  5 : guide_r_inner      - GUIDE_R 바닥 구멍

사용법:
  python auto_annotator.py --type DICT_5X5_100 --save_dir ./dataset
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os
import time
import json
from pathlib import Path

try:
    from .utils import ARUCO_DICT
except ImportError:
    from utils import ARUCO_DICT

# ==========================================
# [사용자 정의 상수]
# ==========================================
MARKER_CENTERS = {
    4: [-0.035, 0.035,  0.0  ],   # TL
    1: [ 0.255, 0.03,   0.0  ],   # TR
    2: [ 0.255, 0.135,  0.025],   # BR
    3: [-0.035, 0.135,  0.025],   # BL
}

COMMON_Y  = 0.147
DEPTH_LV1 = 0.025   # 상단 윤곽 깊이 (2.5cm)
DEPTH_LV2 = 0.055   # 바닥 구멍 깊이 (5.5cm)

TARGETS = {
    "CENTER": {
        "pos"     : [0.11, COMMON_Y, DEPTH_LV2],
        "shape"   : "slot",
        "size_lv1": (0.072, 0.04),   # (긴지름, 짧은지름)
        "size_lv2": 0.026,
        # Class IDs — suffix "_B" = outer, no suffix = inner
        "class_inner": 0,   # CenterHole
        "class_outer": 1,   # CenterHole_B
    },
    "GUIDE_L": {
        "pos"     : [0.03, COMMON_Y, DEPTH_LV2],
        "shape"   : "circle",
        "size_lv1": 0.04,
        "size_lv2": 0.026,
        "class_inner": 2,   # Hole
        "class_outer": 3,   # Hole_B
    },
    "GUIDE_R": {
        "pos"     : [0.19, COMMON_Y, DEPTH_LV2],
        "shape"   : "circle",
        "size_lv1": 0.04,
        "size_lv2": 0.026,
        "class_inner": 2,   # Hole
        "class_outer": 3,   # Hole_B
    },
}

CLASS_NAMES = [
    "center_hole_inner",   # 0: CenterHole
    "center_hole_outer",   # 1: CenterHole_B
    "guide_hole_inner",    # 2: Hole
    "guide_hole_outer",    # 3: Hole_B
]

MIN_MARKERS_REQUIRED = 3
AUTO_SAVE_INTERVAL   = 30
CONTOUR_STEPS        = 9   # YOLO Pose: 9 Keypoints
# ==========================================

# 클래스별 색상 (BGR)
CLASS_COLORS = [
    (0,   0,   255),   # 0: center inner  - 빨강
    (0,   200, 255),   # 1: center outer  - 하늘색
    (50,  200, 0  ),   # 2: guide inner   - 초록 (L/R 공통)
    (0,   255, 100),   # 3: guide outer   - 연두 (L/R 공통)
]

class AutoAnnotator:
    def __init__(self, K, D, img_w=640, img_h=480):
        self.K = K
        self.D = D
        self.img_w = img_w
        self.img_h = img_h
        
    def build_keypoints_3d(self, pos_local: np.ndarray, shape: str, size, depth: float) -> np.ndarray:
        """
        9개의 키포인트 생성 (1: 중심, 2~9: 45도 간격의 외곽 점)
        """
        cx, cy = pos_local[0], pos_local[1]
        points = [[cx, cy, depth]] # 1. Center

        if shape == "circle":
            r = size / 2.0
            rx, ry = r, r
        else: # slot
            rx, ry = size[0] / 2.0, size[1] / 2.0

        for i in range(8):
            theta = 2 * np.pi * i / 8
            points.append([cx + rx * np.cos(theta),
                           cy + ry * np.sin(theta),
                           depth])

        return np.array(points, dtype=np.float32)

    def project_points(self, points_3d: np.ndarray, R: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        pts_cam = (R @ points_3d.T).T + tvec.reshape(1, 3)
        imgpts, _ = cv2.projectPoints(
            pts_cam.reshape(-1, 1, 3).astype(np.float32),
            np.zeros(3), np.zeros(3),
            self.K, self.D
        )
        return imgpts.reshape(-1, 2)

    def get_pose_annotation(self, imgpts: np.ndarray):
        """
        imgpts (9, 2) -> [x_c, y_c, w, h, k1_x, k1_y, v1, ...]
        """
        # Bounding Box from 8 perimeter points (exclude center at index 0)
        perim = imgpts[1:]
        x_min, y_min = np.min(perim, axis=0)
        x_max, y_max = np.max(perim, axis=0)
        
        w = x_max - x_min
        h = y_max - y_min
        xc = (x_min + x_max) / 2.0
        yc = (y_min + y_max) / 2.0
        
        # Normalize
        xc /= self.img_w
        yc /= self.img_h
        w /= self.img_w
        h /= self.img_h
        
        data = [xc, yc, w, h]
        for pt in imgpts:
            px = float(np.clip(pt[0] / self.img_w, 0.0, 1.0))
            py = float(np.clip(pt[1] / self.img_h, 0.0, 1.0))
            v = 2 # Visible and labeled
            data.extend([px, py, v])
            
        return data

    def compute(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        annotations = []
        imgpts_dict = {}

        for name, info in TARGETS.items():
            pos = np.array(info["pos"], dtype=np.float32)

            # Outer
            pts3d_o = self.build_keypoints_3d(pos, info["shape"], info["size_lv1"], DEPTH_LV1)
            imgpts_o = self.project_points(pts3d_o, R, tvec)
            if self.is_in_frame(imgpts_o):
                data = self.get_pose_annotation(imgpts_o)
                annotations.append((info["class_outer"], data))
                imgpts_dict[f"{name}_outer"] = imgpts_o

            # Inner
            pts3d_i = self.build_keypoints_3d(pos, "circle", info["size_lv2"], DEPTH_LV2)
            imgpts_i = self.project_points(pts3d_i, R, tvec)
            if self.is_in_frame(imgpts_i):
                data = self.get_pose_annotation(imgpts_i)
                annotations.append((info["class_inner"], data))
                imgpts_dict[f"{name}_inner"] = imgpts_i

        return annotations, imgpts_dict

    def is_in_frame(self, imgpts):
        in_frame = ((imgpts[:, 0] >= 0) & (imgpts[:, 0] < self.img_w) &
                    (imgpts[:, 1] >= 0) & (imgpts[:, 1] < self.img_h))
        return in_frame.sum() / len(imgpts) >= 0.5

def write_yolo_pose_label(label_path, annotations):
    with open(label_path, "w") as f:
        for class_id, data in annotations:
            data_str = " ".join(f"{v:.6f}" for v in data)
            f.write(f"{class_id} {data_str}\n")

def draw_pose_annotations(frame, imgpts_dict, annotations):
    for ann_idx, (class_id, data) in enumerate(annotations):
        color = CLASS_COLORS[class_id]
        key = list(imgpts_dict.keys())[ann_idx]
        pts = imgpts_dict[key].astype(np.int32)
        
        # BBox
        xc, yc, w, h = data[:4]
        x1 = int((xc - w/2) * frame.shape[1])
        y1 = int((yc - h/2) * frame.shape[0])
        x2 = int((xc + w/2) * frame.shape[1])
        y2 = int((yc + h/2) * frame.shape[0])
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
        
        # Keypoints
        for i in range(9):
            px = int(pts[i, 0])
            py = int(pts[i, 1])
            cv2.circle(frame, (px, py), 3, color, -1)
            if i == 0: # Center
                cv2.circle(frame, (px, py), 5, (255, 255, 255), 1)
    return frame

def setup_dataset_dirs(save_dir: str):
    base = Path(save_dir)
    dirs = {
        "images" : base / "images" / "train",
        "labels" : base / "labels" / "train",
        "preview": base / "preview",
    }
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)
    return dirs

def draw_hud(frame: np.ndarray, detected_count: int, saved_count: int, auto_mode: bool, pose_ok: bool):
    cv2.rectangle(frame, (0, 0), (320, 120), (20, 20, 20), -1)
    status_color = (0, 255, 80) if pose_ok else (0, 80, 255)
    status_text  = "POSE OK" if pose_ok else "SEARCHING..."
    cv2.putText(frame, f"STATUS: {status_text}",  (10, 22),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1)
    cv2.putText(frame, f"MARKERS: {detected_count}/{MIN_MARKERS_REQUIRED}", (10, 44),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, f"SAVED:   {saved_count} frames", (10, 66),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    auto_col = (0, 255, 200) if auto_mode else (100, 100, 100)
    cv2.putText(frame, f"AUTO:    {'ON' if auto_mode else 'OFF'}", (10, 88),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, auto_col, 1)
    cv2.putText(frame, "[S]save [A]auto [Q]quit", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 150, 150), 1)

def run(args):
    dirs = setup_dataset_dirs(args["save_dir"])
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    params     = cv2.aruco.DetectorParameters_create()
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile  = pipeline.start(config)
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream.get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    D = np.array(intr.coeffs, dtype=np.float64)
    img_h, img_w = 480, 640
    annotator = AutoAnnotator(K, D, img_w, img_h)
    saved_count, auto_mode, frame_count, pose_ok = 0, False, 0, False
    last_valid = {"frame": None, "annotations": None, "imgpts_dict": None}

    def save_sample(frame, annotations, imgpts_dict, stem):
        nonlocal saved_count
        cv2.imwrite(str(dirs["images"] / f"{stem}.jpg"), frame)
        write_yolo_pose_label(str(dirs["labels"] / f"{stem}.txt"), annotations)
        preview = frame.copy()
        draw_pose_annotations(preview, imgpts_dict, annotations)
        cv2.imwrite(str(dirs["preview"] / f"{stem}_preview.jpg"), preview)
        saved_count += 1
        print(f"[SAVE] {stem}.jpg | 어노테이션 {len(annotations)}개 | 누적 {saved_count}장")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue
            frame = np.asanyarray(color_frame.get_data())
            display, gray = frame.copy(), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_count += 1
            if hasattr(cv2.aruco, "ArucoDetector"):
                corners, ids, _ = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            detected_count, pose_ok, annotations, imgpts_dict = 0, False, [], {}
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                image_points, object_points = [], []
                for i, marker_id in enumerate(ids.flatten()):
                    mid = int(marker_id)
                    if mid in MARKER_CENTERS:
                        c = corners[i][0]
                        image_points.append([np.mean(c[:, 0]), np.mean(c[:, 1])])
                        object_points.append(MARKER_CENTERS[mid])
                        detected_count += 1
                if detected_count >= MIN_MARKERS_REQUIRED:
                    success, rvec, tvec = cv2.solvePnP(np.array(object_points, dtype=np.float32), np.array(image_points, dtype=np.float32), K, D, flags=cv2.SOLVEPNP_SQPNP)
                    if success:
                        cv2.drawFrameAxes(display, K, D, rvec, tvec, 0.05)
                        annotations, imgpts_dict = annotator.compute(rvec, tvec)
                        pose_ok = len(annotations) > 0
                        if pose_ok:
                            draw_pose_annotations(display, imgpts_dict, annotations)
                            last_valid = {"frame": frame.copy(), "annotations": annotations, "imgpts_dict": imgpts_dict}
                            if auto_mode and (frame_count % AUTO_SAVE_INTERVAL == 0):
                                save_sample(last_valid["frame"], last_valid["annotations"], last_valid["imgpts_dict"], f"frame_{int(time.time()*1000)}")
            draw_hud(display, detected_count, saved_count, auto_mode, pose_ok)
            cv2.imshow("Auto Annotator YOLO Pose", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s') and last_valid["frame"] is not None:
                save_sample(last_valid["frame"], last_valid["annotations"], last_valid["imgpts_dict"], f"frame_{int(time.time()*1000)}")
            elif key == ord('a'): auto_mode = not auto_mode
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        meta = {"total_frames": saved_count, "class_names": CLASS_NAMES, "keypoints": 9, "targets": {k: {kk: vv for kk, vv in v.items() if kk not in ("class_outer", "class_inner")} for k, v in TARGETS.items()}}
        with open(Path(args["save_dir"]) / "collection_meta.json", "w", encoding="utf-8") as f: json.dump(meta, f, indent=2, ensure_ascii=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco 기반 YOLO Pose 자동 어노테이션")
    parser.add_argument("--type", type=str, default="DICT_4X4_50")
    parser.add_argument("--save_dir", type=str, default="./dataset")
    args = vars(parser.parse_args())
    run(args)
