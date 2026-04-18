#!/usr/bin/env python
"""
MSRCR (Multi-Scale Retinex with Color Restoration) + YOLO-Pose baseline training.

Applies MSRCR preprocessing to all training images, then trains a plain
YOLO-Pose model. Inference also requires MSRCR preprocessing for consistency.

MSRCR separates illumination from reflectance via Retinex theory, then
restores color balance — stronger illumination normalization than CLAHE.
"""

import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

from online_lighting_aug import LightingAugPoseTrainer


DEFAULT_DATA_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_BASE_WEIGHTS = Path("vision/yolo/weights/yolo26n-pose.pt")
DEFAULT_RUNS_DIR = Path("vision/yolo/runs/retinex_baseline")
DEFAULT_RESULT_DIR = Path("vision/result/retinex_baseline")
DEFAULT_EPOCHS = 80
DEFAULT_EVAL_DATA_ROOTS = [
    Path("vision/dataset/yolo_pose_baseline_dataset"),
    Path("vision/dataset/yolo_pose_distill_dataset"),
]

# MSRCR default parameters
RETINEX_SIGMAS = [15, 80, 250]  # multi-scale Gaussian sigmas
RETINEX_GAIN = 128.0
RETINEX_OFFSET = 128.0
COLOR_RESTORE_ALPHA = 125.0
COLOR_RESTORE_BETA = 46.0


def _resolve_split_dirs(dataset_root: Path, split: str) -> tuple[Path, Path]:
    candidates = [
        (dataset_root / "images" / split, dataset_root / "labels" / split),
        (dataset_root / split / "images", dataset_root / split / "labels"),
    ]
    for img_dir, lbl_dir in candidates:
        if img_dir.exists():
            return img_dir, lbl_dir
    raise FileNotFoundError(f"Could not resolve split={split} under {dataset_root}")


def apply_msrcr(
    img: np.ndarray,
    sigmas: list = RETINEX_SIGMAS,
    gain: float = RETINEX_GAIN,
    offset: float = RETINEX_OFFSET,
    alpha: float = COLOR_RESTORE_ALPHA,
    beta: float = COLOR_RESTORE_BETA,
) -> np.ndarray:
    """Apply Multi-Scale Retinex with Color Restoration (MSRCR).

    1. Multi-scale Retinex: average SSR across multiple sigma values
       (large sigmas use downsample-blur-upsample for speed)
    2. Color restoration: preserve original chromaticity
    3. Gain/offset normalization to [0, 255]
    """
    img_f = img.astype(np.float32)
    h, w = img_f.shape[:2]
    log_img = np.log1p(img_f)

    # Multi-scale Retinex
    msr = np.zeros_like(img_f)
    for sigma in sigmas:
        scale = max(1, int(sigma / 20))
        if scale > 1:
            small = cv2.resize(img_f, (w // scale, h // scale), interpolation=cv2.INTER_AREA)
            blur_small = cv2.GaussianBlur(small, (0, 0), sigma / scale)
            blur = cv2.resize(blur_small, (w, h), interpolation=cv2.INTER_LINEAR)
        else:
            blur = cv2.GaussianBlur(img_f, (0, 0), sigma)
        np.maximum(blur, 1.0, out=blur)
        msr += log_img - np.log(blur)
    msr *= 1.0 / len(sigmas)

    # Color restoration: C_i = beta * log(alpha * I_i / sum(I))
    img_sum = img_f.sum(axis=2, keepdims=True) + 1.0
    color_restore = beta * np.log(alpha * img_f / img_sum + 1.0)

    # Apply and normalize
    out = gain * cv2.multiply(color_restore, msr) + offset
    np.clip(out, 0, 255, out=out)
    return out.astype(np.uint8)


def prepare_retinex_dataset(src_root: Path, dst_root: Path, **msrcr_kwargs) -> Path:
    """Copy dataset with MSRCR-processed images. Labels are copied as-is."""
    if dst_root.exists():
        print(f"[retinex] Reusing existing MSRCR dataset at {dst_root}")
        return dst_root

    print(f"[retinex] Building MSRCR dataset: {src_root} -> {dst_root}")
    dst_root.mkdir(parents=True, exist_ok=True)

    for split in ["train", "val", "test"]:
        src_img_dir, src_lbl_dir = _resolve_split_dirs(src_root, split)
        dst_img_dir = dst_root / "images" / split
        dst_lbl_dir = dst_root / "labels" / split

        dst_img_dir.mkdir(parents=True, exist_ok=True)
        dst_lbl_dir.mkdir(parents=True, exist_ok=True)

        img_files = sorted(src_img_dir.glob("*.*"))
        for i, img_path in enumerate(img_files):
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            retinex_img = apply_msrcr(img, **msrcr_kwargs)
            cv2.imwrite(str(dst_img_dir / img_path.name), retinex_img)
            if (i + 1) % 500 == 0:
                print(f"  [{split}] {i + 1}/{len(img_files)} images processed")

        if src_lbl_dir.exists():
            for lbl_path in src_lbl_dir.glob("*.*"):
                shutil.copy2(lbl_path, dst_lbl_dir / lbl_path.name)

        print(f"  [{split}] done: {len(img_files)} images")

    src_yaml = src_root / "data.yaml"
    if src_yaml.exists():
        shutil.copy2(src_yaml, dst_root / "data.yaml")

    return dst_root


def write_corrected_data_yaml(dataset_root: Path, out_path: Path) -> Path:
    src_yaml = dataset_root / "data.yaml"
    data = {}

    if src_yaml.exists():
        with open(src_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

    data["path"] = str(dataset_root.resolve())
    data.setdefault("train", "images/train")
    data.setdefault("val", "images/val")
    data.setdefault("test", "images/test")
    data.setdefault("kpt_shape", [9, 3])
    data.setdefault(
        "names",
        {
            0: "CenterHole",
            1: "CenterHole_B",
            2: "Hole",
            3: "Hole_B",
        },
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path


def _to_builtin(value):
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


def evaluate_yolo_on_datasets(
    weights_path: Path,
    eval_roots: list[Path],
    out_dir: Path,
    runs_dir: Path,
    imgsz: int,
    batch: int,
    device: str,
    workers: int,
    **msrcr_kwargs,
) -> list[dict]:
    """Evaluate on MSRCR-processed copies of eval datasets."""
    out_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights_path))
    evaluations = []

    for dataset_root in eval_roots:
        retinex_eval_root = runs_dir / "eval_retinex_data" / dataset_root.name
        prepare_retinex_dataset(dataset_root, retinex_eval_root, **msrcr_kwargs)

        yaml_path = write_corrected_data_yaml(
            retinex_eval_root, out_dir / f"{dataset_root.name}.yaml"
        )
        results = model.val(
            data=str(yaml_path),
            split="test",
            imgsz=imgsz,
            batch=batch,
            device=device,
            workers=workers,
            project=str(out_dir.resolve()),
            name=dataset_root.name,
            exist_ok=True,
            plots=False,
            save_json=False,
            rect=True,
        )
        evaluations.append(
            {
                "dataset_name": dataset_root.name,
                "dataset_root": str(dataset_root.resolve()),
                "results_dict": _to_builtin(getattr(results, "results_dict", {})),
                "speed_ms": _to_builtin(getattr(results, "speed", {})),
                "save_dir": str(Path(results.save_dir).resolve()),
            }
        )
    return evaluations


def main() -> None:
    ap = argparse.ArgumentParser(description="MSRCR + YOLO-Pose baseline training")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument("--base-weights", type=Path, default=DEFAULT_BASE_WEIGHTS)
    ap.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR,
                    help="Temp dir for training artifacts (MSRCR cache, Ultralytics logs)")
    ap.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR,
                    help="Final output dir (weights, eval, meta)")
    ap.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--retinex-sigmas", type=float, nargs="+", default=RETINEX_SIGMAS)
    ap.add_argument("--retinex-gain", type=float, default=RETINEX_GAIN)
    ap.add_argument("--retinex-offset", type=float, default=RETINEX_OFFSET)
    ap.add_argument("--eval-data-roots", type=Path, nargs="*", default=DEFAULT_EVAL_DATA_ROOTS)
    ap.add_argument("--online-aug", action="store_true", default=True)
    ap.add_argument("--no-online-aug", action="store_false", dest="online_aug")
    ap.add_argument("--lowlight-p", type=float, default=0.45)
    ap.add_argument("--backlight-p", type=float, default=0.28)
    ap.add_argument("--backlight-p-b", type=float, default=0.08)
    args = ap.parse_args()

    if not args.data_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {args.data_root}")
    if not args.base_weights.exists():
        raise FileNotFoundError(f"Base weights not found: {args.base_weights}")
    for eval_root in args.eval_data_roots:
        if not eval_root.exists():
            raise FileNotFoundError(f"Eval dataset root not found: {eval_root}")

    msrcr_kwargs = dict(
        sigmas=args.retinex_sigmas,
        gain=args.retinex_gain,
        offset=args.retinex_offset,
    )

    runs_dir = args.runs_dir
    result_dir = args.result_dir
    runs_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)

    # Step 1: Prepare MSRCR-processed training dataset (temp in runs)
    retinex_data_root = runs_dir / "retinex_dataset"
    prepare_retinex_dataset(args.data_root, retinex_data_root, **msrcr_kwargs)

    data_yaml = write_corrected_data_yaml(
        retinex_data_root,
        runs_dir / "data.yaml",
    )

    # Step 2: Train
    model = YOLO(str(args.base_weights))
    run_meta = {
        "method": "retinex_baseline_finetune",
        "preprocessing": f"MSRCR(sigmas={args.retinex_sigmas}, gain={args.retinex_gain}, offset={args.retinex_offset})",
        "train_data_root": str(args.data_root.resolve()),
        "retinex_data_root": str(retinex_data_root.resolve()),
        "eval_data_roots": [str(p.resolve()) for p in args.eval_data_roots],
        "base_weights": str(args.base_weights.resolve()),
        "epochs": args.epochs,
        "batch": args.batch,
        "imgsz": args.imgsz,
        "device": args.device,
        "workers": args.workers,
        "runs_dir": str(runs_dir.resolve()),
        "result_dir": str(result_dir.resolve()),
        "n_params": int(sum(p.numel() for p in model.model.parameters())),
    }
    with open(result_dir / "retinex_run_meta.json", "w", encoding="utf-8") as f:
        json.dump(run_meta, f, indent=2)

    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        workers=args.workers,
        project=str(runs_dir.resolve()),
        name="train",
        exist_ok=True,
        trainer=LightingAugPoseTrainer,
        online_aug=args.online_aug,
        lowlight_p=args.lowlight_p,
        backlight_p=args.backlight_p,
        backlight_p_b=args.backlight_p_b,
    )

    best_ckpt = Path(results.save_dir) / "weights" / "best.pt"
    last_ckpt = Path(results.save_dir) / "weights" / "last.pt"
    src_ckpt = best_ckpt if best_ckpt.exists() else last_ckpt
    if not src_ckpt.exists():
        raise FileNotFoundError(f"No checkpoint found in {results.save_dir}")

    # Copy training artifacts to result dir
    train_out = result_dir / "train"
    shutil.copytree(results.save_dir, str(train_out), dirs_exist_ok=True)
    print(f"[retinex] training artifacts copied to {train_out.resolve()}")

    # Save to central weight directory
    central_weights = Path("vision/result/weight")
    central_weights.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_ckpt, central_weights / "retinex_baseline.pt")
    print(f"[retinex] best weight -> {central_weights / 'retinex_baseline.pt'}")

    # Step 3: Evaluate
    eval_dir = result_dir / "eval"
    evaluations = evaluate_yolo_on_datasets(
        weights_path=src_ckpt,
        eval_roots=args.eval_data_roots,
        out_dir=eval_dir,
        runs_dir=runs_dir,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        **msrcr_kwargs,
    )
    with open(result_dir / "retinex_evaluation.json", "w", encoding="utf-8") as f:
        json.dump(evaluations, f, indent=2)
    print(f"[retinex] evaluation saved to {(result_dir / 'retinex_evaluation.json').resolve()}")


if __name__ == "__main__":
    main()
