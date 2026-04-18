#!/usr/bin/env python
"""
Run all pose models on the real dataset and count how many detections they emit.

The real dataset has no labels, so this script only reports prediction counts
per condition folder and overall totals.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List

import cv2
import numpy as np
from ultralytics import YOLO

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent))
    from evaluate_pose_models import load_pose_model, parse_weight_specs
    from train_clahe_baseline import apply_clahe
    from train_retinex_baseline import apply_msrcr
else:
    from .evaluate_pose_models import load_pose_model, parse_weight_specs
    from .train_clahe_baseline import apply_clahe
    from .train_retinex_baseline import apply_msrcr


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}


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


def _condition_dirs(real_root: Path) -> Dict[str, Path]:
    candidates = {
        "bright": real_root / "saved_images" / "bright",
        "medium": real_root / "saved_images" / "medium",
        "dark": real_root / "saved_images" / "dark",
    }
    resolved = {}
    for name, path in candidates.items():
        if path.exists():
            resolved[name] = path
    if not resolved:
        raise FileNotFoundError(f"No condition folders found under {real_root / 'saved_images'}")
    return resolved


def _resolve_condition_image_dir(condition_root: Path) -> Path:
    candidates = [
        condition_root / "images",
        condition_root / "image",
        condition_root,
    ]
    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate
    raise FileNotFoundError(f"Could not resolve image dir under {condition_root}")


def _image_files(condition_root: Path) -> List[Path]:
    image_dir = _resolve_condition_image_dir(condition_root)
    return sorted([path for path in image_dir.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES])


def _prepare_input(model_name: str, image_path: Path):
    model_key = model_name.lower()
    if "clahe" in model_key:
        img = cv2.imread(str(image_path))
        if img is None:
            return None
        return apply_clahe(img, clip=2.0, grid=8)
    if "retinex" in model_key:
        img = cv2.imread(str(image_path))
        if img is None:
            return None
        return apply_msrcr(img, sigmas=[15, 80, 250], gain=128.0, offset=128.0)
    return str(image_path)


def _render_prediction_image(result, fallback_image_path: Path) -> object:
    plotted = result.plot()
    if plotted is not None:
        return plotted
    img = cv2.imread(str(fallback_image_path))
    return img


def _extract_detections(result) -> List[Dict[str, object]]:
    if result.boxes is None or len(result.boxes) == 0:
        return []

    names = getattr(result, "names", {}) or {}
    xyxy = result.boxes.xyxy.detach().cpu().numpy()
    xywhn = result.boxes.xywhn.detach().cpu().numpy() if hasattr(result.boxes, "xywhn") else None
    confs = result.boxes.conf.detach().cpu().numpy()
    classes = result.boxes.cls.detach().cpu().numpy().astype(int)

    kp_xy = None
    kp_xyn = None
    kp_conf = None
    if getattr(result, "keypoints", None) is not None:
        if getattr(result.keypoints, "xy", None) is not None:
            kp_xy = result.keypoints.xy.detach().cpu().numpy()
        if getattr(result.keypoints, "xyn", None) is not None:
            kp_xyn = result.keypoints.xyn.detach().cpu().numpy()
        if getattr(result.keypoints, "conf", None) is not None:
            kp_conf = result.keypoints.conf.detach().cpu().numpy()

    detections = []
    for idx in range(len(result.boxes)):
        class_id = int(classes[idx])
        det = {
            "index": idx,
            "class_id": class_id,
            "class_name": str(names.get(class_id, class_id)),
            "confidence": float(confs[idx]),
            "bbox_xyxy": _to_builtin(xyxy[idx]),
            "bbox_xywhn": _to_builtin(xywhn[idx]) if xywhn is not None else None,
        }
        if kp_xy is not None:
            det["keypoints_xy"] = _to_builtin(kp_xy[idx])
        if kp_xyn is not None:
            det["keypoints_xyn"] = _to_builtin(kp_xyn[idx])
        if kp_conf is not None:
            det["keypoint_conf"] = _to_builtin(kp_conf[idx])
        detections.append(det)
    return detections


def write_detailed_predictions_json(results: List[Dict[str, object]], out_dir: Path) -> Path:
    payload = []
    for result in results:
        payload.append(
            {
                "model_name": result["model_name"],
                "weights_path": result["weights_path"],
                "prim_modules_path": result.get("prim_modules_path"),
                "real_root": result["real_root"],
                "conditions": {
                    condition: {
                        "image_count": metrics["image_count"],
                        "images": metrics["per_image"],
                    }
                    for condition, metrics in result["conditions"].items()
                },
            }
        )

    out_path = out_dir / "real_predictions.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(_to_builtin(payload), f, indent=2)
    return out_path


def evaluate_model_counts(
    model_name: str,
    weights_path: Path,
    real_root: Path,
    imgsz: int,
    conf: float,
    device: str,
    qualitative_root: Path | None = None,
    save_per_condition: int = 10,
) -> Dict[str, object]:
    model, prim_modules_path = load_pose_model(model_name, weights_path)
    per_condition = {}
    total_images = 0
    total_detections = 0
    total_images_with_det = 0
    total_conf_sum = 0.0
    total_infer_ms = 0.0

    for condition, image_dir in _condition_dirs(real_root).items():
        image_files = _image_files(image_dir)
        cond_images = 0
        cond_detections = 0
        cond_images_with_det = 0
        cond_conf_sum = 0.0
        cond_infer_ms = 0.0
        per_image_rows = []
        saved_count = 0
        save_dir = None
        if qualitative_root is not None:
            save_dir = qualitative_root / model_name / condition
            if save_dir.exists():
                shutil.rmtree(save_dir)
            save_dir.mkdir(parents=True, exist_ok=True)

        for image_path in image_files:
            source = _prepare_input(model_name, image_path)
            if source is None:
                continue
            start = time.perf_counter()
            results = model.predict(
                source=source,
                imgsz=imgsz,
                conf=conf,
                device=device,
                verbose=False,
            )
            infer_ms = (time.perf_counter() - start) * 1000.0
            result = results[0]
            num_det = int(len(result.boxes)) if result.boxes is not None else 0
            conf_sum = float(result.boxes.conf.sum().item()) if result.boxes is not None and len(result.boxes) > 0 else 0.0
            detections = _extract_detections(result)

            cond_images += 1
            cond_detections += num_det
            cond_conf_sum += conf_sum
            cond_infer_ms += infer_ms
            if num_det > 0:
                cond_images_with_det += 1
            per_image_rows.append(
                {
                    "condition": condition,
                    "image_name": image_path.name,
                    "image_path": str(image_path.resolve()),
                    "detections": num_det,
                    "infer_ms": infer_ms,
                    "prediction_details": detections,
                }
            )

            if save_dir is not None and saved_count < save_per_condition:
                plotted = _render_prediction_image(result, image_path)
                if plotted is not None:
                    cv2.imwrite(str(save_dir / image_path.name), plotted)
                    saved_count += 1

        total_images += cond_images
        total_detections += cond_detections
        total_images_with_det += cond_images_with_det
        total_conf_sum += cond_conf_sum
        total_infer_ms += cond_infer_ms
        per_condition[condition] = {
            "image_count": cond_images,
            "images_with_detections": cond_images_with_det,
            "total_detections": cond_detections,
            "avg_detections_per_image": (cond_detections / cond_images) if cond_images else None,
            "avg_confidence": (cond_conf_sum / cond_detections) if cond_detections else None,
            "avg_inference_ms": (cond_infer_ms / cond_images) if cond_images else None,
            "saved_visualizations": min(saved_count, save_per_condition),
            "per_image": per_image_rows,
        }

    return {
        "model_name": model_name,
        "weights_path": str(weights_path.resolve()),
        "prim_modules_path": str(prim_modules_path.resolve()) if prim_modules_path is not None else None,
        "real_root": str(real_root.resolve()),
        "conditions": per_condition,
        "overall": {
            "image_count": total_images,
            "images_with_detections": total_images_with_det,
            "total_detections": total_detections,
            "avg_detections_per_image": (total_detections / total_images) if total_images else None,
            "avg_confidence": (total_conf_sum / total_detections) if total_detections else None,
            "avg_inference_ms": (total_infer_ms / total_images) if total_images else None,
        },
    }


def write_summary_csv(results: List[Dict[str, object]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    overall_headers = [
        "model",
        "image_count",
        "images_with_detections",
        "total_detections",
        "avg_detections_per_image",
        "avg_confidence",
        "avg_inference_ms",
    ]
    condition_headers = [
        "model",
        "condition",
        "image_count",
        "images_with_detections",
        "total_detections",
        "avg_detections_per_image",
        "avg_confidence",
        "avg_inference_ms",
    ]

    with open(out_dir / "overall.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=overall_headers)
        writer.writeheader()
        for result in results:
            overall = result["overall"]
            writer.writerow({"model": result["model_name"], **overall})

    with open(out_dir / "per_condition.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=condition_headers)
        writer.writeheader()
        for result in results:
            for condition, metrics in result["conditions"].items():
                writer.writerow({"model": result["model_name"], "condition": condition, **{k: metrics[k] for k in condition_headers if k not in {"model", "condition"}}})


def print_summary(result: Dict[str, object]) -> None:
    overall = result["overall"]
    overall_avg_det = overall["avg_detections_per_image"]
    overall_avg_conf = overall["avg_confidence"]
    print(f"\n[{result['model_name']}]")
    print(
        f"  overall: images={overall['image_count']} det={overall['total_detections']}"
        f" avg_det={(f'{overall_avg_det:.4f}' if overall_avg_det is not None else 'None')}"
        f" avg_conf={(f'{overall_avg_conf:.4f}' if overall_avg_conf is not None else 'None')}"
    )
    for condition, metrics in result["conditions"].items():
        avg_det = metrics["avg_detections_per_image"]
        avg_conf = metrics["avg_confidence"]
        print(
            f"  {condition}: images={metrics['image_count']} det={metrics['total_detections']}"
            f" avg_det={(f'{avg_det:.4f}' if avg_det is not None else 'None')}"
            f" avg_conf={(f'{avg_conf:.4f}' if avg_conf is not None else 'None')}"
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="Count model detections on the real dataset")
    ap.add_argument("--real-root", type=Path, default=Path("vision/dataset/real_dataset"))
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
    ap.add_argument("--out-dir", type=Path, default=Path("vision/result/real_counts"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--save-per-condition", type=int, default=10, help="Save this many plotted predictions per model/condition")
    args = ap.parse_args()

    if not args.real_root.exists():
        raise FileNotFoundError(f"Real dataset root not found: {args.real_root}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    qualitative_root = args.out_dir / "qualitative"
    weight_specs = parse_weight_specs(args.weights)

    all_results = []
    for model_name, weight_path in weight_specs.items():
        result = evaluate_model_counts(
            model_name=model_name,
            weights_path=weight_path,
            real_root=args.real_root,
            imgsz=args.imgsz,
            conf=args.conf,
            device=args.device,
            qualitative_root=qualitative_root,
            save_per_condition=args.save_per_condition,
        )
        all_results.append(result)
        print_summary(result)

    json_path = args.out_dir / "real_counts.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(_to_builtin(all_results), f, indent=2)
    detailed_json_path = write_detailed_predictions_json(all_results, args.out_dir)
    write_summary_csv(all_results, args.out_dir)
    print(f"\nSaved real-dataset counts to {json_path.resolve()}")
    print(f"Saved detailed real-dataset predictions to {detailed_json_path.resolve()}")


if __name__ == "__main__":
    main()
