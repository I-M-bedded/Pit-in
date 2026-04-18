#!/usr/bin/env python
"""
Export YOLO .pt weights to TensorRT .engine for AGX Orin.

Run this ONCE on the AGX Orin:
    python vision/scripts/export_engine.py

The .engine file is saved next to the .pt file.
Subsequent runs skip export if .engine already exists (use --force to re-export).
"""

import argparse
from pathlib import Path


def export(pt_path: Path, imgsz: int, half: bool, force: bool) -> Path:
    engine_path = pt_path.with_suffix(".engine")
    if engine_path.exists() and not force:
        print(f"[skip] {engine_path} already exists. Use --force to re-export.")
        return engine_path

    from ultralytics import YOLO
    print(f"[export] {pt_path.name}  imgsz={imgsz}  half={half}")
    model = YOLO(str(pt_path))
    out = model.export(
        format="engine",
        half=half,
        imgsz=imgsz,
        device=0,          # CUDA:0 (AGX Orin GPU)
        workspace=4,       # TRT workspace GB (4 is safe for AGX Orin)
        verbose=True,
    )
    engine_path = Path(str(out))
    print(f"[done] saved → {engine_path}")
    return engine_path


def main():
    ap = argparse.ArgumentParser(description="Export YOLO .pt → TensorRT .engine")
    ap.add_argument(
        "--weights",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "result/weight/yolo_baseline_n.pt",
    )
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--half", action="store_true", default=True,
                    help="FP16 (default on: AGX Orin Tensor Cores)")
    ap.add_argument("--no-half", dest="half", action="store_false")
    ap.add_argument("--force", action="store_true", help="Re-export even if .engine exists")
    args = ap.parse_args()

    if not args.weights.exists():
        raise FileNotFoundError(f"Weights not found: {args.weights}")

    export(args.weights, imgsz=args.imgsz, half=args.half, force=args.force)


if __name__ == "__main__":
    main()
