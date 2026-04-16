#!/usr/bin/env python
"""
CLAHE + YOLO-Pose baseline training.

Copies the training dataset, applies CLAHE preprocessing to all images,
then trains a plain YOLO-Pose model on the CLAHE-processed data.
Inference also requires CLAHE preprocessing for consistency.
"""

import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO


DEFAULT_DATA_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_BASE_WEIGHTS = Path("vision/yolo/weights/yolo26n-pose.pt")
DEFAULT_RUNS_DIR = Path("vision/yolo/runs/clahe_baseline")
DEFAULT_RESULT_DIR = Path("vision/result/clahe_baseline")
DEFAULT_EVAL_DATA_ROOTS = [
    Path("vision/dataset/yolo_pose_baseline_dataset"),
    Path("vision/dataset/yolo_pose_distill_dataset"),
]

CLAHE_CLIP = 2.0
CLAHE_GRID = 8


def apply_clahe(img: np.ndarray, clip: float = CLAHE_CLIP, grid: int = CLAHE_GRID) -> np.ndarray:
    """Apply CLAHE on L-channel of LAB color space."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def prepare_clahe_dataset(src_root: Path, dst_root: Path, clip: float, grid: int) -> Path:
    """Copy dataset with CLAHE-processed images. Labels are copied as-is."""
    if dst_root.exists():
        print(f"[clahe] Reusing existing CLAHE dataset at {dst_root}")
        return dst_root

    print(f"[clahe] Building CLAHE dataset: {src_root} -> {dst_root}")
    dst_root.mkdir(parents=True, exist_ok=True)

    for split in ["train", "val", "test"]:
        src_img_dir = src_root / split / "images"
        src_lbl_dir = src_root / split / "labels"
        dst_img_dir = dst_root / split / "images"
        dst_lbl_dir = dst_root / split / "labels"

        if not src_img_dir.exists():
            continue

        dst_img_dir.mkdir(parents=True, exist_ok=True)
        dst_lbl_dir.mkdir(parents=True, exist_ok=True)

        # Process images with CLAHE
        img_files = sorted(src_img_dir.glob("*.*"))
        for i, img_path in enumerate(img_files):
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            clahe_img = apply_clahe(img, clip, grid)
            cv2.imwrite(str(dst_img_dir / img_path.name), clahe_img)
            if (i + 1) % 500 == 0:
                print(f"  [{split}] {i + 1}/{len(img_files)} images processed")

        # Copy labels unchanged
        if src_lbl_dir.exists():
            for lbl_path in src_lbl_dir.glob("*.*"):
                shutil.copy2(lbl_path, dst_lbl_dir / lbl_path.name)

        print(f"  [{split}] done: {len(img_files)} images")

    # Copy data.yaml if exists
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
    data.setdefault("train", "train/images")
    data.setdefault("val", "val/images")
    data.setdefault("test", "test/images")
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
    clahe_clip: float,
    clahe_grid: int,
) -> list[dict]:
    """Evaluate on CLAHE-processed copies of eval datasets."""
    out_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights_path))
    evaluations = []

    for dataset_root in eval_roots:
        # Build CLAHE version of eval dataset in runs (temp)
        clahe_eval_root = runs_dir / "eval_clahe_data" / dataset_root.name
        prepare_clahe_dataset(dataset_root, clahe_eval_root, clahe_clip, clahe_grid)

        yaml_path = write_corrected_data_yaml(
            clahe_eval_root, out_dir / f"{dataset_root.name}.yaml"
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
    ap = argparse.ArgumentParser(description="CLAHE + YOLO-Pose baseline training")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument("--base-weights", type=Path, default=DEFAULT_BASE_WEIGHTS)
    ap.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR,
                    help="Temp dir for training artifacts (CLAHE cache, Ultralytics logs)")
    ap.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR,
                    help="Final output dir (weights, eval, meta)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--clahe-clip", type=float, default=CLAHE_CLIP)
    ap.add_argument("--clahe-grid", type=int, default=CLAHE_GRID)
    ap.add_argument("--eval-data-roots", type=Path, nargs="*", default=DEFAULT_EVAL_DATA_ROOTS)
    args = ap.parse_args()

    if not args.data_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {args.data_root}")
    if not args.base_weights.exists():
        raise FileNotFoundError(f"Base weights not found: {args.base_weights}")
    for eval_root in args.eval_data_roots:
        if not eval_root.exists():
            raise FileNotFoundError(f"Eval dataset root not found: {eval_root}")

    runs_dir = args.runs_dir
    result_dir = args.result_dir
    runs_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)

    # Step 1: Prepare CLAHE-processed training dataset (temp in runs)
    clahe_data_root = runs_dir / "clahe_dataset"
    prepare_clahe_dataset(args.data_root, clahe_data_root, args.clahe_clip, args.clahe_grid)

    data_yaml = write_corrected_data_yaml(
        clahe_data_root,
        runs_dir / "data.yaml",
    )

    # Step 2: Train (Ultralytics artifacts go to runs)
    model = YOLO(str(args.base_weights))
    run_meta = {
        "method": "clahe_baseline_finetune",
        "preprocessing": f"CLAHE(clip={args.clahe_clip}, grid={args.clahe_grid})",
        "train_data_root": str(args.data_root.resolve()),
        "clahe_data_root": str(clahe_data_root.resolve()),
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
    with open(result_dir / "clahe_run_meta.json", "w", encoding="utf-8") as f:
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
    )

    best_ckpt = Path(results.save_dir) / "weights" / "best.pt"
    last_ckpt = Path(results.save_dir) / "weights" / "last.pt"
    src_ckpt = best_ckpt if best_ckpt.exists() else last_ckpt
    if not src_ckpt.exists():
        raise FileNotFoundError(f"No checkpoint found in {results.save_dir}")

    # Copy training artifacts to result dir
    train_out = result_dir / "train"
    shutil.copytree(results.save_dir, str(train_out), dirs_exist_ok=True)
    print(f"[clahe] training artifacts copied to {train_out.resolve()}")

    # Save to central weight directory
    central_weights = Path("vision/result/weight")
    central_weights.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_ckpt, central_weights / "clahe_baseline.pt")
    print(f"[clahe] best weight -> {central_weights / 'clahe_baseline.pt'}")

    # Step 3: Evaluate (CLAHE cache in runs, results to result dir)
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
        clahe_clip=args.clahe_clip,
        clahe_grid=args.clahe_grid,
    )
    with open(result_dir / "clahe_evaluation.json", "w", encoding="utf-8") as f:
        json.dump(evaluations, f, indent=2)
    print(f"[clahe] evaluation saved to {(result_dir / 'clahe_evaluation.json').resolve()}")


if __name__ == "__main__":
    main()
