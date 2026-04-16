#!/usr/bin/env python
"""
Plain YOLO-Pose baseline training on yolo_pose_baseline_dataset.

This script intentionally uses the default Ultralytics training loop only.
It does not add custom callbacks, losses, or augmentation overrides.
After training, it evaluates the model on both baseline/distill test sets and
copies the best checkpoint to vision/yolo/weights/yolo_baseline.pt.
"""

import argparse
import json
import shutil
from pathlib import Path

import yaml
from ultralytics import YOLO


DEFAULT_DATA_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_BASE_WEIGHTS = Path("vision/yolo/weights/yolo26n-pose.pt")
DEFAULT_RUNS_DIR = Path("vision/yolo/runs/yolo_baseline")
DEFAULT_RESULT_DIR = Path("vision/result/yolo_baseline")
DEFAULT_EVAL_DATA_ROOTS = [
    Path("vision/dataset/yolo_pose_baseline_dataset"),
    Path("vision/dataset/yolo_pose_distill_dataset"),
]


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
    imgsz: int,
    batch: int,
    device: str,
    workers: int,
) -> list[dict]:
    out_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights_path))
    evaluations = []

    for dataset_root in eval_roots:
        yaml_path = write_corrected_data_yaml(dataset_root, out_dir / f"{dataset_root.name}.yaml")
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
    ap = argparse.ArgumentParser(description="Plain YOLO baseline training")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument("--base-weights", type=Path, default=DEFAULT_BASE_WEIGHTS)
    ap.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR,
                    help="Temp dir for training artifacts (Ultralytics logs, cache)")
    ap.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR,
                    help="Final output dir (weights, eval, inference)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
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

    data_yaml = write_corrected_data_yaml(
        args.data_root,
        runs_dir / "data.yaml",
    )

    model = YOLO(str(args.base_weights))
    run_meta = {
        "method": "baseline_finetune",
        "train_data_root": str(args.data_root.resolve()),
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
    with open(result_dir / "baseline_run_meta.json", "w", encoding="utf-8") as f:
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
    print(f"[baseline] training artifacts copied to {train_out.resolve()}")

    # Save to central weight directory
    central_weights = Path("vision/result/weight")
    central_weights.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_ckpt, central_weights / "yolo_baseline.pt")
    print(f"[baseline] best weight -> {central_weights / 'yolo_baseline.pt'}")

    eval_dir = result_dir / "eval"
    evaluations = evaluate_yolo_on_datasets(
        weights_path=src_ckpt,
        eval_roots=args.eval_data_roots,
        out_dir=eval_dir,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
    )
    with open(result_dir / "baseline_evaluation.json", "w", encoding="utf-8") as f:
        json.dump(evaluations, f, indent=2)
    print(f"[baseline] evaluation saved to {(result_dir / 'baseline_evaluation.json').resolve()}")


if __name__ == "__main__":
    main()
