"""
ArUco 마커 기반 이중 구멍 자동 어노테이션 도구
================================================
RealSense 카메라 + ArUco 마커로 구멍 위치를 자동 계산하여
YOLO Segmentation 형식(.txt)으로 저장합니다.

YOLO Segmentation 포맷:
  <class_id> <x1> <y1> <x2> <y2> ... <xn> <yn>
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
  
  [s] 키 : 현재 프레임 저장 (이미지 + 어노테이션)
  [a] 키 : 자동 저장 모드 토글 (N프레임마다 자동 캡처)
  [q] 키 : 종료
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os
import time
import json
from pathlib import Path
from utils import ARUCO_DICT

# ==========================================
# [사용자 정의 상수] - 기존 코드와 동일
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
        "class_outer": 0,
        "class_inner": 1,
    },
    "GUIDE_L": {
        "pos"     : [0.03, COMMON_Y, DEPTH_LV2],
        "shape"   : "circle",
        "size_lv1": 0.04,
        "size_lv2": 0.026,
        "class_outer": 2,
        "class_inner": 3,
    },
    "GUIDE_R": {
        "pos"     : [0.19, COMMON_Y, DEPTH_LV2],
        "shape"   : "circle",
        "size_lv1": 0.04,
        "size_lv2": 0.026,
        "class_outer": 2,   # GUIDE_L과 동일 클래스
        "class_inner": 3,
    },
}

CLASS_NAMES = [
    "center_hole_outer",   # 0
    "center_hole_inner",   # 1
    "guide_hole_outer",    # 2 (GUIDE_L, GUIDE_R 공통)
    "guide_hole_inner",    # 3 (GUIDE_L, GUIDE_R 공통)
]

MIN_MARKERS_REQUIRED = 3
CONTOUR_STEPS        = 36   # 윤곽 다각형 꼭짓점 수 (많을수록 정밀)
AUTO_SAVE_INTERVAL   = 30   # 자동 저장 모드: N프레임마다 저장
# ==========================================


# ─────────────────────────────────────────
# 유틸리티
# ─────────────────────────────────────────

def get_intrinsics(profile):
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr   = stream.get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx],
                  [0, intr.fy, intr.ppy],
                  [0, 0,       1       ]], dtype=np.float64)
    D = np.array(intr.coeffs, dtype=np.float64)
    return K, D


def setup_dataset_dirs(save_dir: str):
    """YOLO 학습용 디렉토리 구조 생성"""
    base = Path(save_dir)
    dirs = {
        "images" : base / "images" / "train",
        "labels" : base / "labels" / "train",
        "preview": base / "preview",          # 어노테이션 시각화 저장
    }
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)

    # dataset.yaml 생성
    yaml_path = base / "dataset.yaml"
    if not yaml_path.exists():
        yaml_content = f"""# TD-DRP v3+ 학습용 데이터셋
path: {base.resolve()}
train: images/train
val:   images/val

nc: {len(CLASS_NAMES)}
names: {CLASS_NAMES}
"""
        yaml_path.write_text(yaml_content)
        print(f"[INFO] dataset.yaml 생성: {yaml_path}")

    return dirs


# ─────────────────────────────────────────
# 3D → 2D 마스크 포인트 생성
# ─────────────────────────────────────────

def build_contour_3d(pos_local: np.ndarray, shape: str, size, depth: float) -> np.ndarray:
    """
    로컬 좌표계에서 윤곽 다각형 3D 포인트 생성

    Args:
        pos_local : 중심 위치 (3,)
        shape     : "circle" or "slot"
        size      : circle -> float(지름), slot -> (긴지름, 짧은지름)
        depth     : 해당 레벨의 Z 깊이

    Returns:
        points_3d : (CONTOUR_STEPS, 3) ndarray
    """
    cx, cy = pos_local[0], pos_local[1]
    points = []

    if shape == "circle":
        r = size / 2.0
        for i in range(CONTOUR_STEPS):
            theta = 2 * np.pi * i / CONTOUR_STEPS
            points.append([cx + r * np.cos(theta),
                           cy + r * np.sin(theta),
                           depth])

    elif shape == "slot":
        rx, ry = size[0] / 2.0, size[1] / 2.0
        for i in range(CONTOUR_STEPS):
            theta = 2 * np.pi * i / CONTOUR_STEPS
            points.append([cx + rx * np.cos(theta),
                           cy + ry * np.sin(theta),
                           depth])

    return np.array(points, dtype=np.float32)  # (N, 3)


def project_contour(points_3d: np.ndarray,
                    R: np.ndarray,
                    tvec: np.ndarray,
                    K: np.ndarray,
                    D: np.ndarray) -> np.ndarray:
    """
    로컬 3D 포인트 → 카메라 좌표 변환 → 이미지 투영

    Returns:
        imgpts : (N, 2) int32 픽셀 좌표
    """
    # 로컬 → 카메라 좌표계
    pts_cam = (R @ points_3d.T).T + tvec.reshape(1, 3)  # (N, 3)

    # 카메라 좌표 → 이미지 좌표 투영
    # projectPoints는 (N,1,3) 형태를 기대
    imgpts, _ = cv2.projectPoints(
        pts_cam.reshape(-1, 1, 3).astype(np.float32),
        np.zeros(3), np.zeros(3),   # 이미 카메라 좌표이므로 변환 없음
        K, D
    )
    return imgpts.reshape(-1, 2).astype(np.int32)


# ─────────────────────────────────────────
# YOLO 어노테이션 생성
# ─────────────────────────────────────────

def imgpts_to_yolo_segment(imgpts: np.ndarray,
                           img_w: int,
                           img_h: int) -> list[float]:
    """
    픽셀 좌표 배열 → YOLO Segmentation 정규화 좌표 리스트

    Returns:
        [x1, y1, x2, y2, ..., xn, yn]  (0~1 범위)
    """
    coords = []
    for pt in imgpts:
        x_norm = float(np.clip(pt[0] / img_w, 0.0, 1.0))
        y_norm = float(np.clip(pt[1] / img_h, 0.0, 1.0))
        coords.extend([x_norm, y_norm])
    return coords


def is_contour_in_frame(imgpts: np.ndarray, img_w: int, img_h: int,
                         min_ratio: float = 0.5) -> bool:
    """
    윤곽선 포인트 중 최소 비율이 이미지 안에 있는지 확인
    (프레임 밖으로 크게 벗어난 경우 저장 제외)
    """
    in_frame = ((imgpts[:, 0] >= 0) & (imgpts[:, 0] < img_w) &
                (imgpts[:, 1] >= 0) & (imgpts[:, 1] < img_h))
    return in_frame.sum() / len(imgpts) >= min_ratio


def compute_annotations(rvec, tvec, K, D, img_w, img_h):
    """
    PnP 결과(rvec, tvec)로부터 모든 타겟의 YOLO 어노테이션 계산

    Returns:
        annotations : list of (class_id, [x1,y1,...])
        imgpts_dict : 시각화용 픽셀 좌표 딕셔너리
        valid       : 어노테이션이 최소 1개 이상인지 여부
    """
    R, _ = cv2.Rodrigues(rvec)
    annotations = []
    imgpts_dict  = {}

    for name, info in TARGETS.items():
        pos = np.array(info["pos"], dtype=np.float32)

        # ── Outer 윤곽 (LV1)
        pts3d_outer = build_contour_3d(pos, info["shape"], info["size_lv1"], DEPTH_LV1)
        imgpts_outer = project_contour(pts3d_outer, R, tvec, K, D)

        if is_contour_in_frame(imgpts_outer, img_w, img_h):
            coords = imgpts_to_yolo_segment(imgpts_outer, img_w, img_h)
            annotations.append((info["class_outer"], coords))
            imgpts_dict[f"{name}_outer"] = imgpts_outer

        # ── Inner 구멍 (LV2) - 항상 원형
        pts3d_inner = build_contour_3d(pos, "circle", info["size_lv2"], DEPTH_LV2)
        imgpts_inner = project_contour(pts3d_inner, R, tvec, K, D)

        if is_contour_in_frame(imgpts_inner, img_w, img_h):
            coords = imgpts_to_yolo_segment(imgpts_inner, img_w, img_h)
            annotations.append((info["class_inner"], coords))
            imgpts_dict[f"{name}_inner"] = imgpts_inner

    return annotations, imgpts_dict, len(annotations) > 0


def write_yolo_label(label_path: str, annotations: list):
    """
    YOLO Segmentation .txt 파일 저장

    형식 (한 줄 = 하나의 인스턴스):
      <class_id> <x1> <y1> <x2> <y2> ... <xn> <yn>
    """
    with open(label_path, "w") as f:
        for class_id, coords in annotations:
            coord_str = " ".join(f"{v:.6f}" for v in coords)
            f.write(f"{class_id} {coord_str}\n")


# ─────────────────────────────────────────
# 시각화
# ─────────────────────────────────────────

# 클래스별 색상 (BGR)
CLASS_COLORS = [
    (0,   200, 255),   # 0: center outer  - 하늘색
    (0,   0,   255),   # 1: center inner  - 빨강
    (0,   255, 100),   # 2: guide outer   - 연두 (L/R 공통)
    (50,  200, 0  ),   # 3: guide inner   - 초록 (L/R 공통)
]


def draw_annotations(frame: np.ndarray,
                     imgpts_dict: dict,
                     annotations: list,
                     show_label: bool = True) -> np.ndarray:
    """어노테이션 마스크 + 윤곽선을 프레임에 오버레이"""
    overlay = frame.copy()

    for ann_idx, (class_id, _) in enumerate(annotations):
        color = CLASS_COLORS[class_id]

        # imgpts_dict에서 대응 키 찾기
        key = list(imgpts_dict.keys())[ann_idx] if ann_idx < len(imgpts_dict) else None
        if key is None:
            continue
        pts = imgpts_dict[key].reshape(-1, 1, 2)

        # 반투명 채우기
        cv2.fillPoly(overlay, [pts], color)
        # 윤곽선
        cv2.polylines(frame, [pts], True, color, 2)

        if show_label:
            M = cv2.moments(pts)
            if M["m00"] != 0:
                lx = int(M["m10"] / M["m00"])
                ly = int(M["m01"] / M["m00"])
                cv2.putText(frame, CLASS_NAMES[class_id],
                            (lx - 30, ly),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            (255, 255, 255), 1)

    # 반투명 블렌딩
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
    return frame


def draw_hud(frame: np.ndarray,
             detected_count: int,
             saved_count: int,
             auto_mode: bool,
             pose_ok: bool):
    """상태 정보 HUD 오버레이"""
    h, w = frame.shape[:2]

    # 배경 반투명 패널
    panel = frame.copy()
    cv2.rectangle(panel, (0, 0), (320, 120), (20, 20, 20), -1)
    cv2.addWeighted(panel, 0.6, frame, 0.4, 0, frame)

    status_color = (0, 255, 80) if pose_ok else (0, 80, 255)
    status_text  = "POSE OK" if pose_ok else "SEARCHING..."

    cv2.putText(frame, f"STATUS: {status_text}",  (10, 22),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 1)
    cv2.putText(frame, f"MARKERS: {detected_count}/{MIN_MARKERS_REQUIRED}", (10, 44),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    cv2.putText(frame, f"SAVED:   {saved_count} frames", (10, 66),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    auto_col = (0, 255, 200) if auto_mode else (100, 100, 100)
    cv2.putText(frame, f"AUTO:    {'ON' if auto_mode else 'OFF'}", (10, 88),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, auto_col, 1)
    cv2.putText(frame, "[S]save [A]auto [Q]quit", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 150, 150), 1)


# ─────────────────────────────────────────
# 메인 루프
# ─────────────────────────────────────────

def run(args):
    # ── 디렉토리 준비
    dirs = setup_dataset_dirs(args["save_dir"])
    print(f"[INFO] 저장 경로: {args['save_dir']}")
    print(f"[INFO] 클래스: {CLASS_NAMES}")

    # ── ArUco 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    params     = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # ── RealSense 파이프라인
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile  = pipeline.start(config)
    K, D     = get_intrinsics(profile)

    img_h, img_w = 480, 640

    # ── 상태 변수
    saved_count  = 0
    auto_mode    = False
    frame_count  = 0
    pose_ok      = False

    # 최근 유효 어노테이션 캐시 (자동 저장용)
    last_valid = {"frame": None, "annotations": None, "imgpts_dict": None}

    print("\n[INFO] 어노테이션 수집 시작")
    print("  [S] 수동 저장 | [A] 자동 저장 토글 | [Q] 종료\n")

    def save_sample(frame, annotations, imgpts_dict, stem):
        """이미지 + 라벨 + 프리뷰 저장"""
        nonlocal saved_count

        img_path     = str(dirs["images"] / f"{stem}.jpg")
        label_path   = str(dirs["labels"] / f"{stem}.txt")
        preview_path = str(dirs["preview"] / f"{stem}_preview.jpg")

        # 원본 이미지 저장
        cv2.imwrite(img_path, frame)

        # YOLO 라벨 저장
        write_yolo_label(label_path, annotations)

        # 어노테이션 시각화 프리뷰 저장
        preview = frame.copy()
        draw_annotations(preview, imgpts_dict, annotations, show_label=True)
        cv2.imwrite(preview_path, preview)

        saved_count += 1
        print(f"[SAVE] {stem}.jpg | 어노테이션 {len(annotations)}개 | 누적 {saved_count}장")

    try:
        while True:
            frames      = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame      = np.asanyarray(color_frame.get_data())
            display    = frame.copy()
            gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_count += 1

            # ── 마커 검출
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

            detected_count = 0
            pose_ok        = False
            annotations    = []
            imgpts_dict    = {}

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                image_points  = []
                object_points = []

                for i, marker_id in enumerate(ids.flatten()):
                    mid = int(marker_id)
                    if mid in MARKER_CENTERS:
                        c          = corners[i][0]
                        cx_m, cy_m = np.mean(c[:, 0]), np.mean(c[:, 1])
                        image_points.append([cx_m, cy_m])
                        object_points.append(MARKER_CENTERS[mid])
                        detected_count += 1

                # ── PnP 포즈 추정
                if detected_count >= MIN_MARKERS_REQUIRED:
                    success, rvec, tvec = cv2.solvePnP(
                        np.array(object_points, dtype=np.float32),
                        np.array(image_points,  dtype=np.float32),
                        K, D,
                        flags=cv2.SOLVEPNP_SQPNP,
                    )

                    if success:
                        cv2.drawFrameAxes(display, K, D, rvec, tvec, 0.05)

                        # ── 어노테이션 계산
                        annotations, imgpts_dict, pose_ok = compute_annotations(
                            rvec, tvec, K, D, img_w, img_h
                        )

                        if pose_ok:
                            # 시각화 오버레이
                            draw_annotations(display, imgpts_dict, annotations)
                            last_valid = {
                                "frame"       : frame.copy(),
                                "annotations" : annotations,
                                "imgpts_dict" : imgpts_dict,
                            }

                            # 자동 저장
                            if auto_mode and (frame_count % AUTO_SAVE_INTERVAL == 0):
                                stem = f"frame_{int(time.time()*1000)}"
                                save_sample(
                                    last_valid["frame"],
                                    last_valid["annotations"],
                                    last_valid["imgpts_dict"],
                                    stem,
                                )

            # ── HUD 표시
            draw_hud(display, detected_count, saved_count, auto_mode, pose_ok)
            cv2.imshow("Auto Annotator - TD-DRP v3+", display)

            # ── 키 입력 처리
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("[INFO] 종료")
                break

            elif key == ord('s'):
                # 수동 저장
                if last_valid["frame"] is not None:
                    stem = f"frame_{int(time.time()*1000)}"
                    save_sample(
                        last_valid["frame"],
                        last_valid["annotations"],
                        last_valid["imgpts_dict"],
                        stem,
                    )
                else:
                    print("[WARN] 유효한 포즈가 없습니다. 마커를 더 보이게 조정하세요.")

            elif key == ord('a'):
                auto_mode = not auto_mode
                print(f"[INFO] 자동 저장 모드: {'ON' if auto_mode else 'OFF'} "
                      f"(매 {AUTO_SAVE_INTERVAL}프레임마다 저장)")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

        # ── 수집 완료 요약
        print("\n" + "=" * 50)
        print(f"  수집 완료: {saved_count}장")
        print(f"  저장 위치: {args['save_dir']}")
        print(f"  클래스 수: {len(CLASS_NAMES)}")
        print("=" * 50)

        # 수집 메타데이터 저장
        meta = {
            "total_frames" : saved_count,
            "class_names"  : CLASS_NAMES,
            "contour_steps": CONTOUR_STEPS,
            "targets"      : {k: {kk: vv for kk, vv in v.items()
                                  if kk not in ("class_outer", "class_inner")}
                              for k, v in TARGETS.items()},
        }
        meta_path = Path(args["save_dir"]) / "collection_meta.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        print(f"  메타데이터: {meta_path}")


# ─────────────────────────────────────────
# 진입점
# ─────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ArUco 기반 이중 구멍 YOLO 자동 어노테이션 도구"
    )
    parser.add_argument(
        "--type",
        type=str,
        default="DICT_4X4_50",
        help="ArUco 딕셔너리 타입 (예: DICT_4X4_50, DICT_5X5_100)",
    )
    parser.add_argument(
        "--save_dir",
        type=str,
        default="./dataset",
        help="데이터셋 저장 루트 경로",
    )
    args = vars(parser.parse_args())
    run(args)
