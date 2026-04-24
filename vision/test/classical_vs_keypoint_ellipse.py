"""
Unified paper-style comparison for classical ellipse localization and
learning-based baseline models on the baseline test split.

Outputs per-method:
- bbox localization metrics
- ellipse localization metrics
- one example visualization image
- summary.txt / summary.json
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
from phasepack import phasecong
from ultralytics import YOLO

YOLO_DIR = Path(__file__).resolve().parents[1] / "yolo"
if str(YOLO_DIR) not in sys.path:
    sys.path.insert(0, str(YOLO_DIR))

from train_clahe_baseline import apply_clahe  # noqa: E402
from train_retinex_baseline import apply_msrcr  # noqa: E402


DEFAULT_DATASET_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_OUT_DIR = Path("vision/result/classical_vs_kpt")
DEFAULT_WEIGHTS = [
    "yolo_n=vision/result/weight/yolo_baseline_n.pt",
    "clahe=vision/result/weight/clahe_baseline.pt",
    "retinex=vision/result/weight/retinex_baseline.pt",
]
NUM_KPTS = 9


def _bbox_from_yolo(cx, cy, w, h, img_w, img_h):
    cx_px = cx * img_w
    cy_px = cy * img_h
    w_px = w * img_w
    h_px = h * img_h
    x1 = cx_px - 0.5 * w_px
    y1 = cy_px - 0.5 * h_px
    x2 = cx_px + 0.5 * w_px
    y2 = cy_px + 0.5 * h_px
    return np.array([x1, y1, x2, y2], dtype=np.float32)


def _ellipse_to_bbox(ellipse):
    (cx, cy), (w, h), _ = ellipse
    return np.array([cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h], dtype=np.float32)


def _fit_ellipse_from_points(points: np.ndarray):
    if points.shape[0] < 5:
        return None
    return cv2.fitEllipse(points.astype(np.float32))


def parse_label_objects(label_path: Path, img_w: int, img_h: int):
    """Parse YOLO-pose labels into bbox + contour-ellipse GT objects."""
    objects = []
    if not label_path.exists():
        return objects

    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) < 5 + NUM_KPTS * 3:
                continue

            cls_id, cx, cy, bw, bh = vals[:5]
            bbox = _bbox_from_yolo(cx, cy, bw, bh, img_w, img_h)

            contour_pts = []
            all_kpts = []
            kpts_raw = vals[5:5 + NUM_KPTS * 3]
            for k in range(NUM_KPTS):
                kx, ky, kv = kpts_raw[k * 3], kpts_raw[k * 3 + 1], kpts_raw[k * 3 + 2]
                kp = [kx * img_w, ky * img_h, kv]
                all_kpts.append(kp)
                if k > 0 and kv > 0:
                    contour_pts.append([kp[0], kp[1]])

            ellipse = _fit_ellipse_from_points(np.array(contour_pts, dtype=np.float32)) if contour_pts else None
            objects.append(
                {
                    "class_id": int(cls_id),
                    "bbox": bbox,
                    "ellipse": ellipse,
                    "keypoints": np.array(all_kpts, dtype=np.float32),
                }
            )
    return objects


def parse_weight_specs(weight_specs: List[str]) -> Dict[str, Path]:
    parsed = {}
    for spec in weight_specs:
        if "=" in spec:
            name, raw_path = spec.split("=", 1)
            path = Path(raw_path.strip())
            parsed[name.strip()] = path
        else:
            path = Path(spec.strip())
            parsed[path.stem] = path
    for name, path in parsed.items():
        if not path.exists():
            raise FileNotFoundError(f"Missing weight for '{name}': {path}")
    return parsed


def load_models(weight_specs: List[str]) -> Dict[str, YOLO]:
    models = {}
    for name, path in parse_weight_specs(weight_specs).items():
        models[name] = YOLO(str(path))
    return models


def preprocess_for_method(method: str, img_bgr: np.ndarray) -> np.ndarray:
    if method == "clahe":
        return apply_clahe(img_bgr)
    if method == "retinex":
        return apply_msrcr(img_bgr)
    return img_bgr


def extract_model_predictions(result, kp_conf: float):
    boxes = result.boxes
    preds = []
    if boxes is None or len(boxes) == 0:
        return preds

    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(np.int32)
    keypoints = None
    if result.keypoints is not None and result.keypoints.data is not None:
        keypoints = result.keypoints.data.cpu().numpy()

    for idx in range(len(xyxy)):
        kpts = np.zeros((0, 3), dtype=np.float32)
        contour_pts = []
        ellipse = None
        if keypoints is not None and idx < len(keypoints):
            kpts = keypoints[idx].astype(np.float32)
            for kp_idx in range(1, min(len(kpts), NUM_KPTS)):
                if kpts[kp_idx, 2] >= kp_conf:
                    contour_pts.append(kpts[kp_idx, :2])
            if len(contour_pts) >= 5:
                ellipse = _fit_ellipse_from_points(np.array(contour_pts, dtype=np.float32))

        preds.append(
            {
                "class_id": int(classes[idx]),
                "conf": float(confs[idx]),
                "bbox": xyxy[idx].astype(np.float32),
                "ellipse": ellipse,
                "keypoints": kpts,
            }
        )
    return preds


def _create_edge_drawing():
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "createEdgeDrawing"):
        ed = cv2.ximgproc.createEdgeDrawing()
        params = cv2.ximgproc.EdgeDrawing.Params()
        params.EdgeDetectionOperator = cv2.ximgproc.EDGE_DRAWING_SOBEL
        params.MinPathLength = 15
        ed.setParams(params)
        return ed
    return None


def _ellipse_axes_ratio(ellipse):
    (_, _), (a, b), _ = ellipse
    major = max(float(a), float(b))
    minor = min(float(a), float(b))
    ratio = major / max(minor, 1e-6)
    return major, minor, ratio


def _shape_score(ratio: float, expected_ratios: List[float], rel_tol: float) -> float:
    errs = [abs(ratio - target) / max(target, 1e-6) for target in expected_ratios]
    rel_err = min(errs) if errs else 1.0
    if rel_err > rel_tol:
        return 0.0
    return 1.0 - rel_err / max(rel_tol, 1e-6)


def _ellipse_masks(shape, ellipse, ring_scale: float = 1.35):
    h, w = shape[:2]
    inner = np.zeros((h, w), dtype=np.uint8)
    outer = np.zeros((h, w), dtype=np.uint8)
    c = (int(round(ellipse[0][0])), int(round(ellipse[0][1])))
    axes = (max(1, int(round(ellipse[1][0] / 2.0))), max(1, int(round(ellipse[1][1] / 2.0))))
    outer_axes = (
        max(1, int(round(axes[0] * ring_scale))),
        max(1, int(round(axes[1] * ring_scale))),
    )
    cv2.ellipse(inner, c, axes, ellipse[2], 0, 360, 255, -1)
    cv2.ellipse(outer, c, outer_axes, ellipse[2], 0, 360, 255, -1)
    ring = cv2.subtract(outer, inner)
    return inner, ring


def _contrast_score(gray: np.ndarray, ellipse, min_contrast: float):
    inner_mask, ring_mask = _ellipse_masks(gray.shape, ellipse)
    inner_vals = gray[inner_mask > 0]
    ring_vals = gray[ring_mask > 0]
    if inner_vals.size == 0 or ring_vals.size == 0:
        return 0.0, 0.0
    contrast = float(ring_vals.mean() - inner_vals.mean())
    if contrast <= min_contrast:
        return 0.0, contrast
    return min(1.0, (contrast - min_contrast) / 40.0), contrast


def _nms_ellipses(candidates: List[dict], iou_thresh: float, max_keep: int):
    kept = []
    for cand in sorted(candidates, key=lambda item: item["score"], reverse=True):
        cand_box = _ellipse_to_bbox(cand["ellipse"])
        if any(bbox_iou(cand_box, _ellipse_to_bbox(prev["ellipse"])) >= iou_thresh for prev in kept):
            continue
        kept.append(cand)
        if len(kept) >= max_keep:
            break
    return kept


def _detect_ellipses_from_contours(
    gray: np.ndarray,
    edge_img: np.ndarray,
    expected_ratios: List[float],
    ratio_tol: float,
    min_contrast: float,
    min_major_px: float,
    min_minor_px: float,
    max_major_px: float,
    max_minor_px: float,
    nms_iou: float,
    max_keep: int,
):
    candidates = []
    contours, _ = cv2.findContours(edge_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        if len(cnt) < 20:
            continue
        area = cv2.contourArea(cnt)
        if area < 25.0:
            continue
        ellipse = cv2.fitEllipse(cnt)
        major, minor, ratio = _ellipse_axes_ratio(ellipse)
        if major < min_major_px or minor < min_minor_px:
            continue
        if major > max_major_px or minor > max_minor_px:
            continue
        shape_score = _shape_score(ratio, expected_ratios, ratio_tol)
        if shape_score <= 0.0:
            continue
        contrast_score, contrast = _contrast_score(gray, ellipse, min_contrast=min_contrast)
        if contrast_score <= 0.0:
            continue
        score = 1.5 * shape_score + contrast_score
        candidates.append(
            {
                "ellipse": ellipse,
                "score": float(score),
                "contrast": float(contrast),
                "ratio": float(ratio),
            }
        )
    return [item["ellipse"] for item in _nms_ellipses(candidates, iou_thresh=nms_iou, max_keep=max_keep)]


def detect_ellipses_phase_cong(
    gray: np.ndarray,
    expected_ratios: List[float],
    ratio_tol: float,
    min_contrast: float,
    min_major_px: float,
    min_minor_px: float,
    max_major_px: float,
    max_minor_px: float,
    nms_iou: float,
    max_keep: int,
):
    result = phasecong(gray.astype(np.float64), nscale=4, norient=6, minWaveLength=6)
    m = result[0]
    edge_map = np.clip(m * 255.0, 0, 255).astype(np.uint8)
    _, edge_bin = cv2.threshold(edge_map, 30, 255, cv2.THRESH_BINARY)

    ed = _create_edge_drawing()
    if ed is not None:
        ed.detectEdges(edge_bin)
        detected = ed.detectEllipses()
        ellipses = _parse_ed_ellipses(detected)
    else:
        ellipses = _detect_ellipses_from_contours(
            gray=gray,
            edge_img=edge_bin,
            expected_ratios=expected_ratios,
            ratio_tol=ratio_tol,
            min_contrast=min_contrast,
            min_major_px=min_major_px,
            min_minor_px=min_minor_px,
            max_major_px=max_major_px,
            max_minor_px=max_minor_px,
            nms_iou=nms_iou,
            max_keep=max_keep,
        )
    return ellipses, edge_map


def detect_ellipses_clahe_canny(
    gray: np.ndarray,
    expected_ratios: List[float],
    ratio_tol: float,
    min_contrast: float,
    min_major_px: float,
    min_minor_px: float,
    max_major_px: float,
    max_minor_px: float,
    nms_iou: float,
    max_keep: int,
):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    edges = cv2.Canny(enhanced, 50, 150)

    ed = _create_edge_drawing()
    if ed is not None:
        ed.detectEdges(edges)
        detected = ed.detectEllipses()
        ellipses = _parse_ed_ellipses(detected)
    else:
        ellipses = _detect_ellipses_from_contours(
            gray=gray,
            edge_img=edges,
            expected_ratios=expected_ratios,
            ratio_tol=ratio_tol,
            min_contrast=min_contrast,
            min_major_px=min_major_px,
            min_minor_px=min_minor_px,
            max_major_px=max_major_px,
            max_minor_px=max_minor_px,
            nms_iou=nms_iou,
            max_keep=max_keep,
        )
    return ellipses, edges


def _parse_ed_ellipses(detected):
    ellipses = []
    if detected is None:
        return ellipses
    for row in detected.reshape(-1, 6):
        cx, cy = float(row[0]), float(row[1])
        semi_a, semi_b = float(row[3]), float(row[4])
        angle = float(row[5])
        if semi_a > 0 and semi_b > 0:
            ellipses.append(((cx, cy), (semi_a * 2.0, semi_b * 2.0), angle))
    return ellipses


def ellipse_iou_mask(e1, e2, img_shape):
    h, w = img_shape[:2]
    mask1 = np.zeros((h, w), dtype=np.uint8)
    mask2 = np.zeros((h, w), dtype=np.uint8)
    c1 = (int(round(e1[0][0])), int(round(e1[0][1])))
    a1 = (int(round(e1[1][0] / 2.0)), int(round(e1[1][1] / 2.0)))
    c2 = (int(round(e2[0][0])), int(round(e2[0][1])))
    a2 = (int(round(e2[1][0] / 2.0)), int(round(e2[1][1] / 2.0)))
    if a1[0] <= 0 or a1[1] <= 0 or a2[0] <= 0 or a2[1] <= 0:
        return 0.0
    cv2.ellipse(mask1, c1, a1, e1[2], 0, 360, 255, -1)
    cv2.ellipse(mask2, c2, a2, e2[2], 0, 360, 255, -1)
    inter = np.logical_and(mask1 > 0, mask2 > 0).sum()
    union = np.logical_or(mask1 > 0, mask2 > 0).sum()
    return float(inter / union) if union > 0 else 0.0


def center_error(e1, e2):
    return float(np.hypot(e1[0][0] - e2[0][0], e1[0][1] - e2[0][1]))


def axis_error(e1, e2):
    a1 = sorted(e1[1], reverse=True)
    a2 = sorted(e2[1], reverse=True)
    if a1[0] == 0 or a1[1] == 0:
        return 100.0
    err_major = abs(a1[0] - a2[0]) / a1[0] * 100.0
    err_minor = abs(a1[1] - a2[1]) / a1[1] * 100.0
    return float((err_major + err_minor) * 0.5)


def bbox_iou(box1, box2):
    x1 = max(float(box1[0]), float(box2[0]))
    y1 = max(float(box1[1]), float(box2[1]))
    x2 = min(float(box1[2]), float(box2[2]))
    y2 = min(float(box1[3]), float(box2[3]))
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    area1 = max(0.0, float(box1[2] - box1[0])) * max(0.0, float(box1[3] - box1[1]))
    area2 = max(0.0, float(box2[2] - box2[0])) * max(0.0, float(box2[3] - box2[1]))
    union = area1 + area2 - inter
    return float(inter / union) if union > 0 else 0.0


def bbox_center_error(box1, box2):
    c1x = 0.5 * (float(box1[0]) + float(box1[2]))
    c1y = 0.5 * (float(box1[1]) + float(box1[3]))
    c2x = 0.5 * (float(box2[0]) + float(box2[2]))
    c2y = 0.5 * (float(box2[1]) + float(box2[3]))
    return float(np.hypot(c1x - c2x, c1y - c2y))


def match_bboxes(gt_boxes, pred_boxes):
    results = []
    used = set()
    for gt_box in gt_boxes:
        best_iou = 0.0
        best_idx = -1
        best_metrics = None
        for idx, pred_box in enumerate(pred_boxes):
            if idx in used:
                continue
            iou = bbox_iou(gt_box, pred_box)
            if iou > best_iou:
                best_iou = iou
                best_idx = idx
                best_metrics = {"iou": iou, "center_err": bbox_center_error(gt_box, pred_box)}
        if best_idx >= 0 and best_iou > 0.01:
            used.add(best_idx)
            results.append(best_metrics)
        else:
            results.append(None)
    return results


def match_ellipses(gt_ellipses, pred_ellipses, img_shape):
    results = []
    used = set()
    for gt in gt_ellipses:
        best_iou = 0.0
        best_idx = -1
        best_metrics = None
        for idx, pred in enumerate(pred_ellipses):
            if idx in used:
                continue
            iou = ellipse_iou_mask(gt, pred, img_shape)
            if iou > best_iou:
                best_iou = iou
                best_idx = idx
                best_metrics = {
                    "iou": iou,
                    "center_err": center_error(gt, pred),
                    "axis_err": axis_error(gt, pred),
                }
        if best_idx >= 0 and best_iou > 0.01:
            used.add(best_idx)
            results.append(best_metrics)
        else:
            results.append(None)
    return results


def draw_example(img_bgr: np.ndarray, gt_objects: List[dict], pred_objects: List[dict], method_name: str, out_path: Path):
    vis = img_bgr.copy()
    for obj in gt_objects:
        if obj["ellipse"] is not None:
            e = obj["ellipse"]
            c = (int(round(e[0][0])), int(round(e[0][1])))
            a = (int(round(e[1][0] / 2.0)), int(round(e[1][1] / 2.0)))
            if a[0] > 0 and a[1] > 0:
                cv2.ellipse(vis, c, a, e[2], 0, 360, (0, 255, 0), 1)
    for pred in pred_objects:
        bbox = pred["bbox"]
        cv2.rectangle(
            vis,
            (int(round(bbox[0])), int(round(bbox[1]))),
            (int(round(bbox[2])), int(round(bbox[3]))),
            (255, 255, 0),
            1,
        )
        if pred["ellipse"] is not None:
            e = pred["ellipse"]
            c = (int(round(e[0][0])), int(round(e[0][1])))
            a = (int(round(e[1][0] / 2.0)), int(round(e[1][1] / 2.0)))
            if a[0] > 0 and a[1] > 0:
                cv2.ellipse(vis, c, a, e[2], 0, 360, (0, 0, 255), 2)
    cv2.putText(vis, method_name, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 220, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_path), vis)


def _stats_or_none(values):
    if not values:
        return None
    arr = np.asarray(values, dtype=np.float32)
    return {
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "median": float(np.median(arr)),
    }


def method_label(method: str, classical_backend: str):
    if method == "phase_cong":
        return "Phase Congruency + contour-fit ellipse" if classical_backend == "contour_fit" else "Phase Congruency + AAMED"
    if method == "clahe_canny":
        return "CLAHE + Canny + contour-fit ellipse" if classical_backend == "contour_fit" else "CLAHE + Canny + AAMED"
    if method == "yolo_n":
        return "YOLO baseline (n)"
    if method == "clahe":
        return "CLAHE + YOLO baseline"
    if method == "retinex":
        return "Retinex + YOLO baseline"
    return method


def _serializable(value):
    if isinstance(value, dict):
        return {k: _serializable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_serializable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def build_argparser():
    ap = argparse.ArgumentParser(description="Unified classical vs baseline-model localization evaluation")
    ap.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    ap.add_argument("--split", type=str, default="test", choices=["train", "val", "test"])
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--weights", nargs="+", default=DEFAULT_WEIGHTS)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--kp-conf", type=float, default=0.15)
    ap.add_argument("--device", type=str, default="cpu")
    ap.add_argument("--max-samples", type=int, default=0)
    ap.add_argument("--shape-ratios", type=str, default="1.0,1.5385")
    ap.add_argument("--ratio-tol", type=float, default=0.35)
    ap.add_argument("--min-contrast", type=float, default=0.0)
    ap.add_argument("--min-major-px", type=float, default=6.0)
    ap.add_argument("--min-minor-px", type=float, default=6.0)
    ap.add_argument("--max-major-px", type=float, default=120.0)
    ap.add_argument("--max-minor-px", type=float, default=90.0)
    ap.add_argument("--nms-iou", type=float, default=0.45)
    ap.add_argument("--max-keep", type=int, default=16)
    ap.add_argument("--example-stem", type=str, default="001917_b")
    return ap


def main():
    args = build_argparser().parse_args()
    expected_ratios = [float(v.strip()) for v in args.shape_ratios.split(",") if v.strip()]

    img_dir = args.dataset_root / "images" / args.split
    label_dir = args.dataset_root / "labels" / args.split
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    examples_dir = out_dir / "examples"
    examples_dir.mkdir(parents=True, exist_ok=True)

    image_paths = sorted(glob.glob(str(img_dir / "*.png")) + glob.glob(str(img_dir / "*.jpg")))
    if not image_paths:
        raise FileNotFoundError(f"No images found under {img_dir}")
    if args.max_samples > 0:
        step = max(len(image_paths) // args.max_samples, 1)
        image_paths = image_paths[::step][:args.max_samples]

    models = load_models(args.weights)
    classical_backend = "edge_drawing" if _create_edge_drawing() is not None else "contour_fit"
    methods = ["phase_cong", "clahe_canny", "yolo_n", "clahe", "retinex"]

    stats = {
        method: {
            "detected": 0,
            "bbox_matched": 0,
            "ellipse_detected": 0,
            "ellipse_matched": 0,
            "bbox_iou": [],
            "bbox_center_err": [],
            "ellipse_iou": [],
            "ellipse_center_err": [],
            "ellipse_axis_err": [],
        }
        for method in methods
    }
    sample_saved = {method: False for method in methods}
    example_found = False
    gt_box_total = 0
    gt_ellipse_total = 0

    print(f"Selected {len(image_paths)} images from split '{args.split}'")

    for idx, image_path_str in enumerate(image_paths):
        image_path = Path(image_path_str)
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gt_objects = parse_label_objects(label_dir / f"{image_path.stem}.txt", w, h)
        gt_boxes = [obj["bbox"] for obj in gt_objects]
        gt_ellipses = [obj["ellipse"] for obj in gt_objects if obj["ellipse"] is not None]
        gt_box_total += len(gt_boxes)
        gt_ellipse_total += len(gt_ellipses)

        classical_preds = {}
        pc_ellipses, _ = detect_ellipses_phase_cong(
            gray=gray,
            expected_ratios=expected_ratios,
            ratio_tol=args.ratio_tol,
            min_contrast=args.min_contrast,
            min_major_px=args.min_major_px,
            min_minor_px=args.min_minor_px,
            max_major_px=args.max_major_px,
            max_minor_px=args.max_minor_px,
            nms_iou=args.nms_iou,
            max_keep=args.max_keep,
        )
        classical_preds["phase_cong"] = [{"bbox": _ellipse_to_bbox(e), "ellipse": e} for e in pc_ellipses]

        cc_ellipses, _ = detect_ellipses_clahe_canny(
            gray=gray,
            expected_ratios=expected_ratios,
            ratio_tol=args.ratio_tol,
            min_contrast=args.min_contrast,
            min_major_px=args.min_major_px,
            min_minor_px=args.min_minor_px,
            max_major_px=args.max_major_px,
            max_minor_px=args.max_minor_px,
            nms_iou=args.nms_iou,
            max_keep=args.max_keep,
        )
        classical_preds["clahe_canny"] = [{"bbox": _ellipse_to_bbox(e), "ellipse": e} for e in cc_ellipses]

        model_preds = {}
        for method_name in ["yolo_n", "clahe", "retinex"]:
            proc = preprocess_for_method(method_name, img)
            results = models[method_name].predict(
                source=proc,
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                verbose=False,
            )
            model_preds[method_name] = extract_model_predictions(results[0], kp_conf=args.kp_conf)

        for method_name in methods:
            if method_name in classical_preds:
                preds = classical_preds[method_name]
            else:
                preds = model_preds[method_name]

            pred_boxes = [pred["bbox"] for pred in preds]
            pred_ellipses = [pred["ellipse"] for pred in preds if pred["ellipse"] is not None]
            stats[method_name]["detected"] += len(pred_boxes)
            stats[method_name]["ellipse_detected"] += len(pred_ellipses)

            for match in match_bboxes(gt_boxes, pred_boxes):
                if match is not None:
                    stats[method_name]["bbox_matched"] += 1
                    stats[method_name]["bbox_iou"].append(match["iou"])
                    stats[method_name]["bbox_center_err"].append(match["center_err"])

            for match in match_ellipses(gt_ellipses, pred_ellipses, img.shape):
                if match is not None:
                    stats[method_name]["ellipse_matched"] += 1
                    stats[method_name]["ellipse_iou"].append(match["iou"])
                    stats[method_name]["ellipse_center_err"].append(match["center_err"])
                    stats[method_name]["ellipse_axis_err"].append(match["axis_err"])

            if image_path.stem == args.example_stem and gt_objects:
                draw_example(
                    img_bgr=img,
                    gt_objects=gt_objects,
                    pred_objects=preds,
                    method_name=method_label(method_name, classical_backend),
                    out_path=examples_dir / f"{method_name}.png",
                )
                sample_saved[method_name] = True
                example_found = True

        if (idx + 1) % 25 == 0:
            print(f"  Processed {idx + 1}/{len(image_paths)}")

    summary = {
        "dataset_root": str(args.dataset_root.resolve()),
        "split": args.split,
        "image_count": len(image_paths),
        "gt_boxes_total": gt_box_total,
        "gt_ellipses_total": gt_ellipse_total,
        "classical_backend": classical_backend,
        "example_stem": args.example_stem,
        "classical_shape_prior": {
            "expected_ratios": expected_ratios,
            "ratio_tol": args.ratio_tol,
            "min_contrast": args.min_contrast,
            "min_major_px": args.min_major_px,
            "min_minor_px": args.min_minor_px,
            "max_major_px": args.max_major_px,
            "max_minor_px": args.max_minor_px,
            "nms_iou": args.nms_iou,
            "max_keep": args.max_keep,
        },
        "methods": {},
    }

    print("\n" + "=" * 72)
    print(f"GT boxes total   : {gt_box_total}")
    print(f"GT ellipses total: {gt_ellipse_total}")
    print("=" * 72)

    if not example_found:
        print(f"[WARN] Example stem '{args.example_stem}' was not found or had no GT objects.")

    lines = [
        f"Dataset root: {args.dataset_root.resolve()}",
        f"Split: {args.split}",
        f"Images evaluated: {len(image_paths)}",
        f"GT boxes total: {gt_box_total}",
        f"GT ellipses total: {gt_ellipse_total}",
        f"Classical backend: {classical_backend}",
        f"Classical shape prior ratios: {expected_ratios}",
        "",
    ]

    for method_name in methods:
        label = method_label(method_name, classical_backend)
        st = stats[method_name]
        bbox_precision = st["bbox_matched"] / st["detected"] if st["detected"] else 0.0
        bbox_recall = st["bbox_matched"] / gt_box_total if gt_box_total else 0.0
        ellipse_precision = st["ellipse_matched"] / st["ellipse_detected"] if st["ellipse_detected"] else 0.0
        ellipse_recall = st["ellipse_matched"] / gt_ellipse_total if gt_ellipse_total else 0.0
        bbox_iou_stats = _stats_or_none(st["bbox_iou"])
        bbox_center_stats = _stats_or_none(st["bbox_center_err"])
        ellipse_iou_stats = _stats_or_none(st["ellipse_iou"])
        ellipse_center_stats = _stats_or_none(st["ellipse_center_err"])
        ellipse_axis_stats = _stats_or_none(st["ellipse_axis_err"])

        method_summary = {
            "label": label,
            "detected": st["detected"],
            "bbox_matched": st["bbox_matched"],
            "bbox_precision": bbox_precision,
            "bbox_recall": bbox_recall,
            "ellipse_detected": st["ellipse_detected"],
            "ellipse_matched": st["ellipse_matched"],
            "ellipse_precision": ellipse_precision,
            "ellipse_recall": ellipse_recall,
            "bbox_iou": bbox_iou_stats,
            "bbox_center_err_px": bbox_center_stats,
            "ellipse_iou": ellipse_iou_stats,
            "ellipse_center_err_px": ellipse_center_stats,
            "ellipse_axis_err_pct": ellipse_axis_stats,
            "bbox_iou_ge_0_5": int(sum(v >= 0.5 for v in st["bbox_iou"])),
            "ellipse_iou_ge_0_5": int(sum(v >= 0.5 for v in st["ellipse_iou"])),
            "ellipse_iou_ge_0_7": int(sum(v >= 0.7 for v in st["ellipse_iou"])),
            "example_path": str((examples_dir / f"{method_name}.png").resolve()),
        }
        summary["methods"][method_name] = method_summary

        print(f"\n[{label}]")
        print(f"  Detected boxes    : {st['detected']}")
        print(f"  BBox matched GT   : {st['bbox_matched']}/{gt_box_total} (P={bbox_precision*100:.1f}%  R={bbox_recall*100:.1f}%)")
        if bbox_iou_stats is not None:
            print(
                f"  BBox IoU         : mean={bbox_iou_stats['mean']:.3f} "
                f"std={bbox_iou_stats['std']:.3f} median={bbox_iou_stats['median']:.3f}"
            )
            print(
                f"  BBox center err  : mean={bbox_center_stats['mean']:.1f}px "
                f"std={bbox_center_stats['std']:.1f}px median={bbox_center_stats['median']:.1f}px"
            )
        print(f"  Detected ellipses : {st['ellipse_detected']}")
        print(
            f"  Ellipse matched   : {st['ellipse_matched']}/{gt_ellipse_total} "
            f"(P={ellipse_precision*100:.1f}%  R={ellipse_recall*100:.1f}%)"
        )
        if ellipse_iou_stats is not None:
            print(
                f"  Ellipse IoU      : mean={ellipse_iou_stats['mean']:.3f} "
                f"std={ellipse_iou_stats['std']:.3f} median={ellipse_iou_stats['median']:.3f}"
            )
            print(
                f"  Center err       : mean={ellipse_center_stats['mean']:.1f}px "
                f"std={ellipse_center_stats['std']:.1f}px median={ellipse_center_stats['median']:.1f}px"
            )
            print(
                f"  Axis err         : mean={ellipse_axis_stats['mean']:.1f}% "
                f"std={ellipse_axis_stats['std']:.1f}% median={ellipse_axis_stats['median']:.1f}%"
            )

        lines.extend(
            [
                f"[{label}]",
                f"  Detected boxes: {st['detected']}",
                f"  BBox matched: {st['bbox_matched']}/{gt_box_total} (Precision: {bbox_precision*100:.1f}%  Recall: {bbox_recall*100:.1f}%)",
            ]
        )
        if bbox_iou_stats is not None:
            lines.append(
                f"  BBox IoU: mean={bbox_iou_stats['mean']:.3f} std={bbox_iou_stats['std']:.3f} median={bbox_iou_stats['median']:.3f}"
            )
            lines.append(
                f"  BBox center err: mean={bbox_center_stats['mean']:.1f}px std={bbox_center_stats['std']:.1f}px median={bbox_center_stats['median']:.1f}px"
            )
        lines.append(f"  Detected ellipses: {st['ellipse_detected']}")
        lines.append(
            f"  Ellipse matched: {st['ellipse_matched']}/{gt_ellipse_total} (Precision: {ellipse_precision*100:.1f}%  Recall: {ellipse_recall*100:.1f}%)"
        )
        if ellipse_iou_stats is not None:
            lines.append(
                f"  Ellipse IoU: mean={ellipse_iou_stats['mean']:.3f} std={ellipse_iou_stats['std']:.3f} median={ellipse_iou_stats['median']:.3f}"
            )
            lines.append(
                f"  Center err: mean={ellipse_center_stats['mean']:.1f}px std={ellipse_center_stats['std']:.1f}px median={ellipse_center_stats['median']:.1f}px"
            )
            lines.append(
                f"  Axis err: mean={ellipse_axis_stats['mean']:.1f}% std={ellipse_axis_stats['std']:.1f}% median={ellipse_axis_stats['median']:.1f}%"
            )
        lines.append(f"  Example: {(examples_dir / f'{method_name}.png').resolve()}")
        lines.append("")

    summary_txt = out_dir / "summary.txt"
    summary_json = out_dir / "summary.json"
    with open(summary_txt, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(_serializable(summary), f, indent=2)

    print(f"\nSummary saved to {summary_txt}")
    print(f"JSON summary saved to {summary_json}")
    print(f"Example images saved to {examples_dir}")


if __name__ == "__main__":
    main()
