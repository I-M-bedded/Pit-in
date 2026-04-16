#!/usr/bin/env python
"""
Estimate hole center pose from YOLO-Pose keypoints.

This script supports:
- plain YOLO-Pose weights
- COIN-patched YOLO-Pose weights
- raw / CLAHE / MSRCR preprocessing

Geometry is mirrored from `vision/dataset_gen/auto_annotator.py`.
For each detection it:
1. collects 9 keypoints (kp_0 center + 8 boundary points),
2. fits an ellipse on the 8 boundary points,
3. optionally replaces kp_0 with the fitted ellipse center,
4. solves camera-frame pose with solvePnP using the known 3D template.

Notes on geometry:
- The projection of a 3D planar circle is generally an ellipse.
- The fitted image ellipse center is not, in general, identical to the
  perspective projection of the true 3D circle center.
- They coincide only in affine / weak-perspective conditions, or when the
  circle plane is close to fronto-parallel.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from train_clahe_baseline import CLAHE_CLIP, CLAHE_GRID, apply_clahe
from train_coin_pose import patch_yolo_with_coin
from train_retinex_baseline import (
    COLOR_RESTORE_ALPHA,
    COLOR_RESTORE_BETA,
    RETINEX_GAIN,
    RETINEX_OFFSET,
    RETINEX_SIGMAS,
    apply_msrcr,
)


CLASS_ID_TO_NAME = {
    0: "center_hole_outer",
    1: "center_hole_inner",
    2: "guide_hole_outer",
    3: "guide_hole_inner",
}

# Mirrored from vision/dataset_gen/auto_annotator.py.
# The center outer target is modeled as an ellipse-like slot in that file.
GEOMETRY_SPECS = {
    "center_hole_outer": {"shape": "slot", "size": (0.072, 0.040)},
    "center_hole_inner": {"shape": "circle", "size": 0.026},
    "guide_hole_outer": {"shape": "circle", "size": 0.040},
    "guide_hole_inner": {"shape": "circle", "size": 0.026},
}

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
PNP_METHOD_NAMES = {
    int(cv2.SOLVEPNP_ITERATIVE): "ITERATIVE",
    int(cv2.SOLVEPNP_EPNP): "EPNP",
    int(cv2.SOLVEPNP_P3P): "P3P",
    int(cv2.SOLVEPNP_AP3P): "AP3P",
    int(cv2.SOLVEPNP_IPPE): "IPPE",
    int(cv2.SOLVEPNP_IPPE_SQUARE): "IPPE_SQUARE",
    int(cv2.SOLVEPNP_SQPNP): "SQPNP",
}


def load_intrinsics(path: Optional[Path], fx: float, fy: float, cx: float, cy: float,
                    dist: list[float]) -> tuple[np.ndarray, np.ndarray]:
    if path is None:
        if min(fx, fy) <= 0:
            raise ValueError("Either --intrinsics or positive --fx/--fy/--cx/--cy must be provided.")
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist_coeffs = np.array(dist, dtype=np.float64)
        return camera_matrix, dist_coeffs

    suffix = path.suffix.lower()
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f) if suffix == ".json" else yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Unsupported intrinsics payload in {path}")

    if "camera_matrix" in data:
        cm = data["camera_matrix"]
        if isinstance(cm, dict) and "data" in cm:
            camera_matrix = np.array(cm["data"], dtype=np.float64).reshape(3, 3)
        else:
            camera_matrix = np.array(cm, dtype=np.float64).reshape(3, 3)
    elif "K" in data:
        camera_matrix = np.array(data["K"], dtype=np.float64).reshape(3, 3)
    elif all(k in data for k in ("fx", "fy", "cx", "cy")):
        camera_matrix = np.array(
            [[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    else:
        raise ValueError(f"Could not parse camera matrix from {path}")

    if "dist_coeffs" in data:
        dc = data["dist_coeffs"]
        if isinstance(dc, dict) and "data" in dc:
            dist_coeffs = np.array(dc["data"], dtype=np.float64).reshape(-1)
        else:
            dist_coeffs = np.array(dc, dtype=np.float64).reshape(-1)
    elif "D" in data:
        dist_coeffs = np.array(data["D"], dtype=np.float64).reshape(-1)
    elif "distortion_coefficients" in data:
        dc = data["distortion_coefficients"]
        if isinstance(dc, dict) and "data" in dc:
            dist_coeffs = np.array(dc["data"], dtype=np.float64).reshape(-1)
        else:
            dist_coeffs = np.array(dc, dtype=np.float64).reshape(-1)
    else:
        dist_coeffs = np.zeros(5, dtype=np.float64)

    return camera_matrix, dist_coeffs


def build_object_keypoints(class_id: int) -> np.ndarray:
    class_name = CLASS_ID_TO_NAME[int(class_id)]
    spec = GEOMETRY_SPECS[class_name]
    points = [[0.0, 0.0, 0.0]]

    if spec["shape"] == "circle":
        radius = float(spec["size"]) / 2.0
        rx = radius
        ry = radius
    else:
        rx = float(spec["size"][0]) / 2.0
        ry = float(spec["size"][1]) / 2.0

    for idx in range(8):
        theta = 2.0 * np.pi * idx / 8.0
        points.append([rx * np.cos(theta), ry * np.sin(theta), 0.0])

    return np.array(points, dtype=np.float32)


def preprocess_image(img_bgr: np.ndarray, mode: str) -> np.ndarray:
    if mode == "raw":
        return img_bgr
    if mode == "clahe":
        return apply_clahe(img_bgr, CLAHE_CLIP, CLAHE_GRID)
    if mode == "msrcr":
        return apply_msrcr(
            img_bgr,
            sigmas=RETINEX_SIGMAS,
            gain=RETINEX_GAIN,
            offset=RETINEX_OFFSET,
            alpha=COLOR_RESTORE_ALPHA,
            beta=COLOR_RESTORE_BETA,
        )
    raise ValueError(f"Unsupported preprocessing mode: {mode}")


def iter_images(source: Path) -> Iterable[Path]:
    if source.is_file():
        yield source
        return
    if not source.is_dir():
        raise FileNotFoundError(f"Source not found: {source}")
    for path in sorted(source.iterdir()):
        if path.suffix.lower() in IMAGE_EXTS:
            yield path


def fit_boundary_ellipse(kpts_xy: np.ndarray, kpts_conf: np.ndarray, kp_conf: float) -> Optional[dict]:
    keep = kpts_conf >= kp_conf
    points = kpts_xy[keep]
    if len(points) < 5:
        return None
    ellipse = cv2.fitEllipse(points.astype(np.float32).reshape(-1, 1, 2))
    center_xy = np.array(ellipse[0], dtype=np.float64)
    axes = np.array(ellipse[1], dtype=np.float64)
    angle_deg = float(ellipse[2])
    return {
        "center_xy": center_xy,
        "axes_px": axes,
        "angle_deg": angle_deg,
        "num_points": int(len(points)),
    }


def solve_pose(object_points: np.ndarray, image_points: np.ndarray,
               camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Optional[dict]:
    methods = []
    if len(object_points) >= 4 and np.allclose(object_points[:, 2], object_points[0, 2]):
        methods.append(cv2.SOLVEPNP_IPPE)
    methods.extend([cv2.SOLVEPNP_ITERATIVE, cv2.SOLVEPNP_SQPNP])

    obj = np.ascontiguousarray(object_points.astype(np.float32))
    img = np.ascontiguousarray(image_points.astype(np.float32))

    for method in methods:
        try:
            ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs, flags=method)
        except cv2.error:
            continue
        if not ok:
            continue

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        reproj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        reproj = reproj.reshape(-1, 2)
        reproj_err = float(np.sqrt(np.mean(np.sum((reproj - img) ** 2, axis=1))))

        return {
            "solver": int(method),
            "solver_name": PNP_METHOD_NAMES.get(int(method), f"METHOD_{int(method)}"),
            "rvec": rvec.reshape(-1).astype(float).tolist(),
            "tvec_m": tvec.reshape(-1).astype(float).tolist(),
            "xyz_m": tvec.reshape(-1).astype(float).tolist(),
            "normal_camera": rotation_matrix[:, 2].astype(float).tolist(),
            "reprojection_error_px": reproj_err,
            "num_points": int(len(obj)),
        }

    return None


def estimate_detection(
    class_id: int,
    box_conf: float,
    bbox_xyxy: np.ndarray,
    keypoints: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    center_source: str,
    kp_conf: float,
) -> dict:
    class_name = CLASS_ID_TO_NAME.get(int(class_id), f"class_{int(class_id)}")
    object_points = build_object_keypoints(int(class_id))

    kpts_xy = keypoints[:, :2].astype(np.float64)
    kpts_conf = keypoints[:, 2].astype(np.float64)

    ellipse = fit_boundary_ellipse(kpts_xy[1:], kpts_conf[1:], kp_conf)
    kp0_xy = kpts_xy[0]
    kp0_conf = float(kpts_conf[0])

    chosen_center_xy = kp0_xy.copy()
    chosen_center_source = "kp0"
    if center_source == "ellipse" and ellipse is not None:
        chosen_center_xy = ellipse["center_xy"].copy()
        chosen_center_source = "ellipse"

    image_points = kpts_xy.copy()
    image_points[0] = chosen_center_xy

    valid_mask = kpts_conf >= kp_conf
    if chosen_center_source == "ellipse":
        valid_mask[0] = True
    if valid_mask.sum() < 4:
        pose = None
    else:
        pose = solve_pose(
            object_points=object_points[valid_mask],
            image_points=image_points[valid_mask],
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )

    result = {
        "class_id": int(class_id),
        "class_name": class_name,
        "box_conf": float(box_conf),
        "bbox_xyxy": bbox_xyxy.astype(float).tolist(),
        "kp0_xy": kp0_xy.astype(float).tolist(),
        "kp0_conf": kp0_conf,
        "center_xy": chosen_center_xy.astype(float).tolist(),
        "center_source": chosen_center_source,
        "num_valid_keypoints": int(valid_mask.sum()),
        "pose": pose,
    }

    if ellipse is not None:
        result["ellipse"] = {
            "center_xy": ellipse["center_xy"].astype(float).tolist(),
            "axes_px": ellipse["axes_px"].astype(float).tolist(),
            "angle_deg": float(ellipse["angle_deg"]),
            "num_points": int(ellipse["num_points"]),
        }
        result["kp0_vs_ellipse_px"] = float(np.linalg.norm(kp0_xy - ellipse["center_xy"]))
    else:
        result["ellipse"] = None
        result["kp0_vs_ellipse_px"] = None

    return result


def draw_detection(image: np.ndarray, detection: dict) -> np.ndarray:
    out = image.copy()
    x1, y1, x2, y2 = [int(round(v)) for v in detection["bbox_xyxy"]]
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 180), 2)

    center_xy = tuple(int(round(v)) for v in detection["center_xy"])
    kp0_xy = tuple(int(round(v)) for v in detection["kp0_xy"])

    if detection["ellipse"] is not None:
        ecx, ecy = detection["ellipse"]["center_xy"]
        ew, eh = detection["ellipse"]["axes_px"]
        angle = detection["ellipse"]["angle_deg"]
        cv2.ellipse(
            out,
            (int(round(ecx)), int(round(ecy))),
            (max(1, int(round(ew / 2.0))), max(1, int(round(eh / 2.0)))),
            angle,
            0,
            360,
            (255, 180, 0),
            1,
        )
        cv2.circle(out, (int(round(ecx)), int(round(ecy))), 4, (255, 180, 0), -1)

    cv2.circle(out, kp0_xy, 4, (0, 0, 255), -1)
    cv2.circle(out, center_xy, 5, (255, 255, 255), 1)

    label = f"{detection['class_name']} {detection['box_conf']:.2f}"
    cv2.putText(out, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 180), 2)

    pose = detection.get("pose")
    if pose is not None:
        x_m, y_m, z_m = pose["xyz_m"]
        pose_text = f"x={x_m:.4f} y={y_m:.4f} z={z_m:.4f} m"
        cv2.putText(out, pose_text, (x1, min(out.shape[0] - 10, y2 + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1)

    return out


def load_model(yolo_weights: Path, device: str, coin_weights: Optional[Path]) -> YOLO:
    model = YOLO(str(yolo_weights))
    if coin_weights is not None:
        patch_yolo_with_coin(model, coin_ckpt=str(coin_weights), anchor_weight=0.0, aux_tv_weight=0.0)
    return model


def run_on_image(
    model: YOLO,
    image_path: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    preproc: str,
    conf: float,
    device: str,
    center_source: str,
    kp_conf: float,
) -> tuple[dict, np.ndarray]:
    img_bgr = cv2.imread(str(image_path))
    if img_bgr is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    input_bgr = preprocess_image(img_bgr, preproc)
    results = model.predict(source=input_bgr, conf=conf, device=device, verbose=False)
    result = results[0]

    detections = []
    visual = input_bgr.copy()

    if result.boxes is not None and result.keypoints is not None and len(result.boxes) > 0:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        keypoints = result.keypoints.data.cpu().numpy()

        for det_idx in range(len(boxes)):
            detection = estimate_detection(
                class_id=int(classes[det_idx]),
                box_conf=float(scores[det_idx]),
                bbox_xyxy=boxes[det_idx],
                keypoints=keypoints[det_idx],
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                center_source=center_source,
                kp_conf=kp_conf,
            )
            detections.append(detection)
            visual = draw_detection(visual, detection)

    payload = {
        "image_path": str(image_path.resolve()),
        "preproc": preproc,
        "center_source": center_source,
        "detections": detections,
    }
    return payload, visual


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Estimate hole camera-frame pose from YOLO-Pose keypoints")
    ap.add_argument("--source", type=Path, required=True, help="Image file or directory")
    ap.add_argument("--yolo-weights", type=Path, required=True, help="YOLO pose checkpoint")
    ap.add_argument("--coin-weights", type=Path, default=None, help="COIN module checkpoint")
    ap.add_argument("--intrinsics", type=Path, default=None, help="JSON/YAML with K and D")
    ap.add_argument("--fx", type=float, default=-1.0)
    ap.add_argument("--fy", type=float, default=-1.0)
    ap.add_argument("--cx", type=float, default=-1.0)
    ap.add_argument("--cy", type=float, default=-1.0)
    ap.add_argument("--dist", nargs="*", type=float, default=[0.0, 0.0, 0.0, 0.0, 0.0])
    ap.add_argument("--preproc", choices=["raw", "clahe", "msrcr"], default="raw")
    ap.add_argument("--conf", type=float, default=0.25, help="Box confidence threshold")
    ap.add_argument("--kp-conf", type=float, default=0.15, help="Keypoint confidence threshold for ellipse/PnP")
    ap.add_argument("--center-source", choices=["kp0", "ellipse"], default="ellipse")
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--out-dir", type=Path, default=Path("vision/result/hole_pose_estimator"))
    ap.add_argument("--save-vis", action="store_true", help="Save visualization images")
    ap.add_argument("--save-json", action="store_true", help="Save per-image JSON outputs")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    camera_matrix, dist_coeffs = load_intrinsics(args.intrinsics, args.fx, args.fy, args.cx, args.cy, args.dist)
    model = load_model(args.yolo_weights, args.device, args.coin_weights)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    all_results = []

    for image_path in iter_images(args.source):
        payload, visual = run_on_image(
            model=model,
            image_path=image_path,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            preproc=args.preproc,
            conf=args.conf,
            device=args.device,
            center_source=args.center_source,
            kp_conf=args.kp_conf,
        )
        all_results.append(payload)

        if args.save_json:
            out_json = args.out_dir / f"{image_path.stem}.json"
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)

        if args.save_vis:
            out_img = args.out_dir / f"{image_path.stem}_pose.png"
            cv2.imwrite(str(out_img), visual)

        print(json.dumps(payload, ensure_ascii=False, indent=2))

    if len(all_results) > 1:
        summary_path = args.out_dir / "summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(all_results, f, indent=2)


if __name__ == "__main__":
    main()
