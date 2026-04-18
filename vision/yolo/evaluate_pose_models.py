#!/usr/bin/env python
"""
Slice-wise evaluation for YOLO pose models on the baseline test set.

Reports:
- Standard Ultralytics val() metrics
- Visibility-label-wise pose metrics
- Lighting-condition-wise detection/pose metrics derived from filename rules
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent))
    from train_PRIM import patch_yolo_with_prim, patch_yolo_with_primv2
    from train_clahe_baseline import apply_clahe
    from train_retinex_baseline import apply_msrcr
    from summarize_pose_slices import summarize_results
else:
    from .train_PRIM import patch_yolo_with_prim, patch_yolo_with_primv2
    from .train_clahe_baseline import apply_clahe
    from .train_retinex_baseline import apply_msrcr
    from .summarize_pose_slices import summarize_results


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}


def resolve_prim_modules_path(model_name: str, weights_path: Path) -> Optional[Path]:
    model_key = str(model_name).strip().lower()
    stem_key = weights_path.stem.lower()
    if "prim" not in model_key and "prim" not in stem_key:
        return None

    is_v2 = "primv2" in model_key or "primv2" in stem_key
    candidates = (
        [
            weights_path.with_name("primv2_modules.pt"),
            weights_path.parent / "primv2_modules_best.pt",
            Path("vision/result/weight/primv2_modules.pt"),
            Path("vision/result/weight/primv2_modules_best.pt"),
        ]
        if is_v2
        else [
            weights_path.with_name("prim_modules.pt"),
            weights_path.parent / "prim_modules_best.pt",
            Path("vision/result/weight/prim_modules.pt"),
            Path("vision/result/weight/prim_modules_best.pt"),
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"PRIM model '{model_name}' requires prim module weights, but none were found near {weights_path}."
    )


def load_pose_model(model_name: str, weights_path: Path) -> Tuple[YOLO, Optional[Path]]:
    model = YOLO(str(weights_path))
    prim_modules_path = resolve_prim_modules_path(model_name, weights_path)
    if prim_modules_path is not None:
        model_key = str(model_name).strip().lower()
        stem_key = weights_path.stem.lower()
        if "primv2" in model_key or "primv2" in stem_key:
            patch_yolo_with_primv2(
                model,
                module_ckpt=str(prim_modules_path),
                anchor_weight=0.0,
                film_tv_weight=0.0,
                restore_tv_weight=0.0,
                gate_sparsity_weight=0.0,
                expert_entropy_weight=0.0,
            )
        else:
            patch_yolo_with_prim(
                model,
                module_ckpt=str(prim_modules_path),
                anchor_weight=0.0,
                aux_tv_weight=0.0,
                gate_sparsity_weight=0.0,
                expert_entropy_weight=0.0,
            )
    return model, prim_modules_path


def _resolve_split_dirs(dataset_root: Path, split: str) -> Tuple[Path, Path]:
    candidates = [
        (dataset_root / split / "image", dataset_root / split / "label"),
        (dataset_root / split / "images", dataset_root / split / "labels"),
        (dataset_root / "image" / split, dataset_root / "label" / split),
        (dataset_root / "images" / split, dataset_root / "labels" / split),
    ]
    for image_dir, label_dir in candidates:
        if image_dir.exists() and label_dir.exists():
            return image_dir, label_dir
    raise FileNotFoundError(f"Could not resolve split dirs for '{split}' under {dataset_root}")


def _resolve_dataset_layout(dataset_root: Path) -> Dict[str, str]:
    layout = {}
    for split in ["train", "val", "test"]:
        image_dir, _ = _resolve_split_dirs(dataset_root, split)
        layout[split] = str(image_dir.relative_to(dataset_root)).replace("\\", "/")
    return layout


def write_eval_data_yaml(dataset_root: Path, out_path: Path) -> Path:
    src_yaml = dataset_root / "data.yaml"
    data = {}
    if src_yaml.exists():
        with open(src_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

    layout = _resolve_dataset_layout(dataset_root)
    data["path"] = str(dataset_root.resolve())
    # Evaluation uses split=test only, so point every split field to the test images.
    data["train"] = layout["test"]
    data["val"] = layout["test"]
    data["test"] = layout["test"]
    data.setdefault("kpt_shape", [9, 3])
    data.setdefault("names", {0: "CenterHole", 1: "CenterHole_B", 2: "Hole", 3: "Hole_B"})

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path


def _build_canonical_test_dataset(src_root: Path, dst_root: Path) -> Path:
    if dst_root.exists():
        return dst_root
    dst_root.mkdir(parents=True, exist_ok=True)
    src_img_dir, src_lbl_dir = _resolve_split_dirs(src_root, "test")
    dst_img_dir = dst_root / "test" / "images"
    dst_lbl_dir = dst_root / "test" / "labels"
    dst_img_dir.mkdir(parents=True, exist_ok=True)
    dst_lbl_dir.mkdir(parents=True, exist_ok=True)
    for img_path in src_img_dir.iterdir():
        if img_path.is_file():
            shutil.copy2(img_path, dst_img_dir / img_path.name)
    for lbl_path in src_lbl_dir.iterdir():
        if lbl_path.is_file():
            shutil.copy2(lbl_path, dst_lbl_dir / lbl_path.name)
    src_yaml = src_root / "data.yaml"
    if src_yaml.exists():
        shutil.copy2(src_yaml, dst_root / "data.yaml")
    return dst_root


def _prepare_clahe_test_dataset(src_root: Path, dst_root: Path, clip: float = 2.0, grid: int = 8) -> Path:
    if dst_root.exists():
        return dst_root
    src_img_dir, src_lbl_dir = _resolve_split_dirs(src_root, "test")
    dst_img_dir = dst_root / "test" / "images"
    dst_lbl_dir = dst_root / "test" / "labels"
    dst_img_dir.mkdir(parents=True, exist_ok=True)
    dst_lbl_dir.mkdir(parents=True, exist_ok=True)
    for img_path in sorted(src_img_dir.iterdir()):
        if not img_path.is_file():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        clahe_img = apply_clahe(img, clip=clip, grid=grid)
        cv2.imwrite(str(dst_img_dir / img_path.name), clahe_img)
    for lbl_path in sorted(src_lbl_dir.iterdir()):
        if lbl_path.is_file():
            shutil.copy2(lbl_path, dst_lbl_dir / lbl_path.name)
    return dst_root


def _prepare_retinex_test_dataset(
    src_root: Path,
    dst_root: Path,
    sigmas: List[int] | None = None,
    gain: float = 128.0,
    offset: float = 128.0,
) -> Path:
    if dst_root.exists():
        return dst_root
    src_img_dir, src_lbl_dir = _resolve_split_dirs(src_root, "test")
    dst_img_dir = dst_root / "test" / "images"
    dst_lbl_dir = dst_root / "test" / "labels"
    dst_img_dir.mkdir(parents=True, exist_ok=True)
    dst_lbl_dir.mkdir(parents=True, exist_ok=True)
    for img_path in sorted(src_img_dir.iterdir()):
        if not img_path.is_file():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        retinex_img = apply_msrcr(img, sigmas=sigmas or [15, 80, 250], gain=gain, offset=offset)
        cv2.imwrite(str(dst_img_dir / img_path.name), retinex_img)
    for lbl_path in sorted(src_lbl_dir.iterdir()):
        if lbl_path.is_file():
            shutil.copy2(lbl_path, dst_lbl_dir / lbl_path.name)
    return dst_root


def prepare_eval_root(model_name: str, data_root: Path, cache_root: Path) -> Path:
    model_key = str(model_name).strip().lower()
    preprocess_root = data_root
    _resolve_split_dirs(data_root, "test")
    if not (data_root / "test" / "images").exists():
        preprocess_root = _build_canonical_test_dataset(data_root, cache_root / f"{data_root.name}_canonical")
    if "clahe" in model_key:
        return _prepare_clahe_test_dataset(preprocess_root, cache_root / f"{model_key}_dataset", clip=2.0, grid=8)
    if "retinex" in model_key:
        return _prepare_retinex_test_dataset(
            preprocess_root,
            cache_root / f"{model_key}_dataset",
            sigmas=[15, 80, 250],
            gain=128.0,
            offset=128.0,
        )
    return preprocess_root


def _to_builtin(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): _to_builtin(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_builtin(v) for v in value]
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    return value


def parse_weight_specs(weight_specs: Iterable[str]) -> Dict[str, Path]:
    parsed: Dict[str, Path] = {}
    for spec in weight_specs:
        if "=" in spec:
            name, raw_path = spec.split("=", 1)
            name = name.strip()
            path = Path(raw_path.strip())
        else:
            path = Path(spec.strip())
            name = path.stem
        if not name:
            raise ValueError(f"Missing model name in weight spec '{spec}'.")
        if not path.exists():
            raise FileNotFoundError(f"Weights not found for '{name}': {path}")
        parsed[name] = path
    return parsed


def classify_condition_block(stem: str, block_size: int = 300) -> Dict[str, object]:
    """Group samples into fixed-size index blocks using only trailing numeric ids."""
    index = None
    if stem.isdigit():
        index = int(stem)
    else:
        for part in reversed(stem.split("_")):
            if part.isdigit():
                index = int(part)
                break
    if index is None:
        return {
            "block_size": block_size,
            "block_id": None,
            "block_range": "unknown",
        }
    block_id = int(index) // int(block_size)
    start = block_id * int(block_size)
    end = start + int(block_size) - 1
    block_range = f"{start:04d}-{end:04d}"
    return {
        "block_size": block_size,
        "block_id": block_id,
        "block_range": block_range,
    }


def read_label_file(label_path: Path, image_w: int, image_h: int) -> List[dict]:
    objects: List[dict] = []
    if not label_path.exists():
        return objects

    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            values = [float(v) for v in parts]
            cls_id = int(values[0])
            cx, cy, bw, bh = values[1:5]
            x1 = (cx - bw / 2.0) * image_w
            y1 = (cy - bh / 2.0) * image_h
            x2 = (cx + bw / 2.0) * image_w
            y2 = (cy + bh / 2.0) * image_h

            kpts = []
            for i in range(5, len(values), 3):
                if i + 2 >= len(values):
                    break
                kx = values[i] * image_w
                ky = values[i + 1] * image_h
                vis = int(round(values[i + 2]))
                kpts.append([kx, ky, vis])

            objects.append(
                {
                    "cls": cls_id,
                    "box": np.array([x1, y1, x2, y2], dtype=np.float32),
                    "keypoints": np.array(kpts, dtype=np.float32),
                }
            )
    return objects


def extract_predictions(result) -> List[dict]:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []

    xyxy = boxes.xyxy.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(np.int32)
    conf = boxes.conf.cpu().numpy()

    keypoints = None
    if result.keypoints is not None and result.keypoints.data is not None:
        keypoints = result.keypoints.data.cpu().numpy()

    preds = []
    for i in range(len(xyxy)):
        pred = {
            "cls": int(cls[i]),
            "conf": float(conf[i]),
            "box": xyxy[i].astype(np.float32),
        }
        if keypoints is not None and i < len(keypoints):
            pred["keypoints"] = keypoints[i].astype(np.float32)
        else:
            pred["keypoints"] = np.zeros((0, 3), dtype=np.float32)
        preds.append(pred)
    return preds


def box_iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    x1 = max(float(box_a[0]), float(box_b[0]))
    y1 = max(float(box_a[1]), float(box_b[1]))
    x2 = min(float(box_a[2]), float(box_b[2]))
    y2 = min(float(box_a[3]), float(box_b[3]))
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    if inter <= 0:
        return 0.0
    area_a = max(0.0, float(box_a[2] - box_a[0])) * max(0.0, float(box_a[3] - box_a[1]))
    area_b = max(0.0, float(box_b[2] - box_b[0])) * max(0.0, float(box_b[3] - box_b[1]))
    denom = area_a + area_b - inter
    return inter / denom if denom > 0 else 0.0


def match_predictions(gt_objects: List[dict], pred_objects: List[dict], iou_thresh: float) -> List[Tuple[int, int, float]]:
    candidates: List[Tuple[float, int, int]] = []
    for gt_idx, gt in enumerate(gt_objects):
        for pred_idx, pred in enumerate(pred_objects):
            if gt["cls"] != pred["cls"]:
                continue
            iou = box_iou(gt["box"], pred["box"])
            if iou >= iou_thresh:
                candidates.append((iou, gt_idx, pred_idx))

    candidates.sort(reverse=True)
    matches: List[Tuple[int, int, float]] = []
    used_gt = set()
    used_pred = set()
    for iou, gt_idx, pred_idx in candidates:
        if gt_idx in used_gt or pred_idx in used_pred:
            continue
        used_gt.add(gt_idx)
        used_pred.add(pred_idx)
        matches.append((gt_idx, pred_idx, iou))
    return matches


@dataclass
class SliceAccumulator:
    image_count: int = 0
    gt_objects: int = 0
    pred_objects: int = 0
    matched_objects: int = 0
    gt_keypoints: int = 0
    matched_keypoints: int = 0
    pck05_hits: int = 0
    pck10_hits: int = 0
    error_sum: float = 0.0

    def add_image(self, gt_count: int, pred_count: int) -> None:
        self.image_count += 1
        self.gt_objects += gt_count
        self.pred_objects += pred_count

    def add_match(self) -> None:
        self.matched_objects += 1

    def add_gt_keypoint(self) -> None:
        self.gt_keypoints += 1

    def add_pose(self, error_norm: float) -> None:
        self.matched_keypoints += 1
        self.error_sum += error_norm
        if error_norm <= 0.05:
            self.pck05_hits += 1
        if error_norm <= 0.10:
            self.pck10_hits += 1

    def to_dict(self) -> Dict[str, object]:
        precision = self.matched_objects / self.pred_objects if self.pred_objects else None
        recall = self.matched_objects / self.gt_objects if self.gt_objects else None
        object_coverage = self.matched_objects / self.gt_objects if self.gt_objects else None
        keypoint_coverage = self.matched_keypoints / self.gt_keypoints if self.gt_keypoints else None
        pck05 = self.pck05_hits / self.gt_keypoints if self.gt_keypoints else None
        pck10 = self.pck10_hits / self.gt_keypoints if self.gt_keypoints else None
        mean_error = self.error_sum / self.matched_keypoints if self.matched_keypoints else None
        return {
            "image_count": self.image_count,
            "gt_objects": self.gt_objects,
            "pred_objects": self.pred_objects,
            "matched_objects": self.matched_objects,
            "precision": precision,
            "recall": recall,
            "object_coverage": object_coverage,
            "gt_keypoints": self.gt_keypoints,
            "matched_keypoints": self.matched_keypoints,
            "keypoint_coverage": keypoint_coverage,
            "pck@0.05": pck05,
            "pck@0.10": pck10,
            "mean_norm_error_matched": mean_error,
        }


def ensure_accumulator(store: Dict[str, SliceAccumulator], key: str) -> SliceAccumulator:
    if key not in store:
        store[key] = SliceAccumulator()
    return store[key]


def evaluate_predictions(
    model: YOLO,
    image_paths: List[Path],
    labels_dir: Path,
    imgsz: int,
    device: str,
    conf: float,
    match_iou: float,
    block_size: int,
) -> Dict[str, object]:
    overall = SliceAccumulator()
    by_visibility: Dict[str, SliceAccumulator] = {}
    by_block: Dict[str, SliceAccumulator] = {}

    for image_path in image_paths:
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        image_h, image_w = img.shape[:2]
        label_path = labels_dir / f"{image_path.stem}.txt"
        gt_objects = read_label_file(label_path, image_w, image_h)
        block_info = classify_condition_block(image_path.stem, block_size=block_size)

        results = model.predict(
            source=str(image_path),
            imgsz=imgsz,
            conf=conf,
            device=device,
            verbose=False,
        )
        pred_objects = extract_predictions(results[0])
        matches = match_predictions(gt_objects, pred_objects, iou_thresh=match_iou)
        matched_gt = {gt_idx for gt_idx, _, _ in matches}

        accumulators = [
            overall,
            ensure_accumulator(by_block, str(block_info["block_range"])),
        ]
        for acc in accumulators:
            acc.add_image(len(gt_objects), len(pred_objects))

        vis_labels_in_image = {
            int(round(float(gt_kp[2])))
            for gt in gt_objects
            for gt_kp in gt["keypoints"]
        }
        for vis_label in vis_labels_in_image:
            ensure_accumulator(by_visibility, f"vis_{vis_label}").add_image(len(gt_objects), len(pred_objects))

        for gt_idx, _, _ in matches:
            for acc in accumulators:
                acc.add_match()

            gt = gt_objects[gt_idx]
            pred = pred_objects[[pair[1] for pair in matches if pair[0] == gt_idx][0]]
            gt_kpts = gt["keypoints"]
            pred_kpts = pred["keypoints"]
            norm = max(float(gt["box"][2] - gt["box"][0]), float(gt["box"][3] - gt["box"][1]), 1.0)

            for kp_idx, gt_kp in enumerate(gt_kpts):
                vis_label = int(round(float(gt_kp[2])))
                vis_acc = ensure_accumulator(by_visibility, f"vis_{vis_label}")
                for acc in (*accumulators, vis_acc):
                    acc.add_gt_keypoint()

                if kp_idx >= len(pred_kpts):
                    continue

                pred_xy = pred_kpts[kp_idx, :2]
                error_norm = float(np.linalg.norm(pred_xy - gt_kp[:2]) / norm)
                for acc in (*accumulators, vis_acc):
                    acc.add_pose(error_norm)

        for gt_idx, gt in enumerate(gt_objects):
            if gt_idx in matched_gt:
                continue
            for gt_kp in gt["keypoints"]:
                vis_label = int(round(float(gt_kp[2])))
                vis_acc = ensure_accumulator(by_visibility, f"vis_{vis_label}")
                for acc in (*accumulators, vis_acc):
                    acc.add_gt_keypoint()

    return {
        "overall": overall.to_dict(),
        "visibility_metrics": {
            k: {
                **v.to_dict(),
                "precision": None,
                "recall": None,
                "object_coverage": None,
                "matched_objects": None,
            }
            for k, v in sorted(by_visibility.items())
        },
        "lighting_block_metrics": {k: v.to_dict() for k, v in sorted(by_block.items())},
    }


def evaluate_model(
    model_name: str,
    weights_path: Path,
    data_root: Path,
    out_dir: Path,
    imgsz: int,
    batch: int,
    device: str,
    workers: int,
    conf: float,
    match_iou: float,
    block_size: int,
) -> Dict[str, object]:
    model, prim_modules_path = load_pose_model(model_name, weights_path)
    eval_root = prepare_eval_root(model_name, data_root, out_dir / "cache")
    yaml_path = write_eval_data_yaml(eval_root, out_dir / f"{model_name}_{eval_root.name}.yaml")
    val_results = model.val(
        data=str(yaml_path),
        split="test",
        imgsz=imgsz,
        batch=batch,
        device=device,
        workers=workers,
        project=str(out_dir.resolve()),
        name=f"{model_name}_val",
        exist_ok=True,
        plots=False,
        save_json=False,
        rect=True,
    )

    test_images = sorted(
        [
            path
            for path in _resolve_split_dirs(eval_root, "test")[0].iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        ]
    )
    _, test_labels_dir = _resolve_split_dirs(eval_root, "test")
    slices = evaluate_predictions(
        model=model,
        image_paths=test_images,
        labels_dir=test_labels_dir,
        imgsz=imgsz,
        device=device,
        conf=conf,
        match_iou=match_iou,
        block_size=block_size,
    )
    return {
        "model_name": model_name,
        "weights_path": str(weights_path.resolve()),
        "prim_modules_path": str(prim_modules_path.resolve()) if prim_modules_path is not None else None,
        "dataset_root": str(eval_root.resolve()),
        "results_dict": _to_builtin(getattr(val_results, "results_dict", {})),
        "speed_ms": _to_builtin(getattr(val_results, "speed", {})),
        "slice_metrics": slices,
    }


def print_summary(result: Dict[str, object]) -> None:
    name = result["model_name"]
    metrics = result["results_dict"]
    slices = result["slice_metrics"]
    print(f"\n[{name}]")
    print(
        "  overall val:"
        f" box50={metrics.get('metrics/mAP50(B)', None)}"
        f" box50-95={metrics.get('metrics/mAP50-95(B)', None)}"
        f" pose50={metrics.get('metrics/mAP50(P)', None)}"
        f" pose50-95={metrics.get('metrics/mAP50-95(P)', None)}"
    )
    overall = slices["overall"]
    print(
        "  slice overall:"
        f" recall={overall.get('recall', None)}"
        f" pck05={overall.get('pck@0.05', None)}"
        f" pck10={overall.get('pck@0.10', None)}"
        f" kp_cov={overall.get('keypoint_coverage', None)}"
    )
    for vis_name, vis_metrics in slices["visibility_metrics"].items():
        print(
            f"  {vis_name}:"
            f" pck05={vis_metrics.get('pck@0.05', None)}"
            f" pck10={vis_metrics.get('pck@0.10', None)}"
            f" kp_cov={vis_metrics.get('keypoint_coverage', None)}"
        )
    for block_name, block_metrics in slices.get("lighting_block_metrics", {}).items():
        print(
            f"  block {block_name}:"
            f" recall={block_metrics.get('recall', None)}"
            f" pck05={block_metrics.get('pck@0.05', None)}"
            f" pck10={block_metrics.get('pck@0.10', None)}"
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="Slice-wise baseline pose evaluation")
    ap.add_argument(
        "--data-root",
        type=Path,
        default=Path("vision/dataset/yolo_pose_baseline_dataset"),
        help="Dataset root containing split/image|images and split/label|labels",
    )
    ap.add_argument(
        "--weights",
        nargs="+",
        default=[
            "yolo_n=vision/result/weight/yolo_baseline_n.pt",
            "prim=vision/result/weight/prim_yolo.pt",
            "primv2=vision/result/weight/primv2_yolo.pt",
            "clahe=vision/result/weight/clahe_baseline.pt",
            "retinex=vision/result/weight/retinex_baseline.pt",
        ],
        help="Model weights as name=path entries, or plain paths for auto-naming",
    )
    ap.add_argument("--out-dir", type=Path, default=Path("vision/result/baseline_slice_eval"))
    ap.add_argument("--summary-out-dir", type=Path, default=None, help="Optional summary output directory; defaults to <out-dir>/summary")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--conf", type=float, default=0.25, help="Predict confidence threshold for slice metrics")
    ap.add_argument("--match-iou", type=float, default=0.5, help="GT/pred IoU threshold for matching")
    ap.add_argument("--block-size", type=int, default=300, help="Index block size for per-lighting slice tables")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    weight_specs = parse_weight_specs(args.weights)

    all_results = []
    for model_name, weight_path in weight_specs.items():
        result = evaluate_model(
            model_name=model_name,
            weights_path=weight_path,
            data_root=args.data_root,
            out_dir=args.out_dir,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            conf=args.conf,
            match_iou=args.match_iou,
            block_size=args.block_size,
        )
        all_results.append(result)
        print_summary(result)

    out_path = args.out_dir / "slice_evaluation.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(_to_builtin(all_results), f, indent=2)
    print(f"\nSaved slice evaluation to {out_path.resolve()}")

    summary_out_dir = args.summary_out_dir or (args.out_dir / "summary")
    summarize_results(out_path, summary_out_dir)


if __name__ == "__main__":
    main()
