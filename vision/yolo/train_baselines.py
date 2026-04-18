#!/usr/bin/env python
"""
Run baseline training scripts sequentially.

Order:
1. Plain YOLO baseline (n)
2. Retinex baseline (n)
3. CLAHE baseline (n)
"""

import argparse
import subprocess
import sys
from pathlib import Path


DEFAULT_DATA_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_EVAL_DATA_ROOTS = [
    Path("vision/dataset/yolo_pose_baseline_dataset"),
]
DEFAULT_EPOCHS = 80


def run_step(command: list[str], workdir: Path) -> None:
    print("[train_baselines] running:", " ".join(command))
    subprocess.run(command, cwd=str(workdir), check=True)


def main() -> None:
    ap = argparse.ArgumentParser(description="Sequential baseline training runner")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument("--eval-data-roots", type=Path, nargs="*", default=DEFAULT_EVAL_DATA_ROOTS)
    ap.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--scales", type=str, nargs="+", default=["n"], choices=["n", "s", "m"])
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    if not args.data_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {args.data_root}")
    for eval_root in args.eval_data_roots:
        if not eval_root.exists():
            raise FileNotFoundError(f"Eval dataset root not found: {eval_root}")

    common = [
        "--data-root", str(args.data_root),
        "--epochs", str(args.epochs),
        "--batch", str(args.batch),
        "--imgsz", str(args.imgsz),
        "--device", str(args.device),
        "--workers", str(args.workers),
    ]
    if args.eval_data_roots:
        common += ["--eval-data-roots", *[str(p) for p in args.eval_data_roots]]

    run_step(
        [
            sys.executable,
            "vision/yolo/train_yolo_baseline.py",
            *common,
            "--scales",
            *args.scales,
        ],
        repo_root,
    )
    run_step(
        [
            sys.executable,
            "vision/yolo/train_retinex_baseline.py",
            *common,
        ],
        repo_root,
    )
    run_step(
        [
            sys.executable,
            "vision/yolo/train_clahe_baseline.py",
            *common,
        ],
        repo_root,
    )


if __name__ == "__main__":
    main()
