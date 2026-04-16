#!/usr/bin/env python
"""
Compare all 4 vision models:
  1) YOLO baseline
  2) CLAHE + YOLO baseline
  3) MSRCR + YOLO baseline
  4) COIN-Pose

Evaluates on yolo_pose_baseline_dataset test set (mAP metrics),
then runs real-dataset inference (bright/medium/dark, 10 images each)
and saves annotated results + detection count summary.
"""

import argparse
import json
import random
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from ultralytics import YOLO

# Add parent for coin_pose imports
sys.path.insert(0, str(Path(__file__).resolve().parent))
from coin_pose import (
    SpatiallyVariantIlluminationEncoder,
    SpatiallyAwareSoftRouter,
    MultiScaleRSVFiLM,
    COINContextPyramid,
    OcclusionContextRecovery,
)

# ── COIN constants (must match train_coin_pose.py) ──
TARGET_LAYERS = [4, 6, 10]
CHANNELS = [128, 128, 256]
NUM_EXPERTS = 3
ILL_CH = 64

# ── CLAHE defaults ──
CLAHE_CLIP = 2.0
CLAHE_GRID = 8

# ── MSRCR defaults ──
RETINEX_SIGMAS = [15, 80, 250]
RETINEX_GAIN = 128.0
RETINEX_OFFSET = 128.0
COLOR_RESTORE_ALPHA = 125.0
COLOR_RESTORE_BETA = 46.0


# ══════════════════════════════════════════════════════════════════
# Preprocessing helpers
# ══════════════════════════════════════════════════════════════════

def apply_clahe(img, clip=CLAHE_CLIP, grid=CLAHE_GRID):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def apply_msrcr(img, sigmas=RETINEX_SIGMAS, gain=RETINEX_GAIN,
                offset=RETINEX_OFFSET, alpha=COLOR_RESTORE_ALPHA,
                beta=COLOR_RESTORE_BETA):
    img_f = img.astype(np.float32)
    h, w = img_f.shape[:2]
    log_img = np.log1p(img_f)
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
    img_sum = img_f.sum(axis=2, keepdims=True) + 1.0
    color_restore = beta * np.log(alpha * img_f / img_sum + 1.0)
    out = gain * cv2.multiply(color_restore, msr) + offset
    np.clip(out, 0, 255, out=out)
    return out.astype(np.uint8)


# ══════════════════════════════════════════════════════════════════
# COIN model loader (patches YOLO with COIN modules for inference)
# ══════════════════════════════════════════════════════════════════

def load_coin_model(yolo_weights, coin_weights, device="0"):
    """Load YOLO + COIN modules for inference."""
    yolo_model = YOLO(str(yolo_weights))
    det = yolo_model.model

    ie = SpatiallyVariantIlluminationEncoder(3, ILL_CH)
    sr = SpatiallyAwareSoftRouter(ILL_CH, NUM_EXPERTS)
    rsv = MultiScaleRSVFiLM(CHANNELS, ILL_CH, NUM_EXPERTS)
    ctx = COINContextPyramid(CHANNELS, context_channels=128)
    occlu = OcclusionContextRecovery(CHANNELS)

    state = torch.load(str(coin_weights), map_location="cpu")
    ie.load_state_dict(state["illumination_encoder"])
    sr.load_state_dict(state["soft_router"])
    rsv.load_state_dict(state["multi_scale_rsv"])
    if "context_pyramid" in state:
        ctx.load_state_dict(state["context_pyramid"])
    if "occlusion_recovery" in state:
        occlu.load_state_dict(state["occlusion_recovery"])

    det.add_module("coin_ie", ie)
    det.add_module("coin_sr", sr)
    det.add_module("coin_rsv", rsv)
    det.add_module("coin_ctx", ctx)
    det.add_module("coin_or", occlu)

    def _modulated_predict_once(self, x, profile=False, visualize=False, embed=None):
        x_input = x
        Z = self.coin_ie(x_input)
        P = self.coin_sr(Z)
        y, dt = [], []
        target_cache = {}
        for m in self.model:
            if m.f != -1:
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]
            x = m(x)
            if m.i in TARGET_LAYERS:
                target_cache[m.i] = x
                if m.i == TARGET_LAYERS[-1]:
                    feats = [target_cache[idx] for idx in TARGET_LAYERS]
                    modulated = []
                    for idx, feat in enumerate(feats):
                        feat_m, _, _, _ = self.coin_rsv.forward_single(feat, idx, Z, P)
                        modulated.append(feat_m)
                    shared = self.coin_ctx(modulated)
                    refined = []
                    for idx, (feat_m, feat_ctx) in enumerate(zip(modulated, shared)):
                        feat_out, _, _ = self.coin_or.forward_single(
                            feat=feat_m, context_feat=feat_ctx,
                            scale_idx=idx, reconstruct=True,
                        )
                        refined.append(feat_out)
                    for layer_idx, feat_new in zip(TARGET_LAYERS, refined):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new
                    x = target_cache[m.i]
            y.append(x if m.i in self.save else None)
        return x

    det._predict_once = types.MethodType(_modulated_predict_once, det)
    print(f"[coin] loaded COIN model: {yolo_weights} + {coin_weights}")
    return yolo_model


# ══════════════════════════════════════════════════════════════════
# Evaluation on test set
# ══════════════════════════════════════════════════════════════════

def write_data_yaml(dataset_root, out_path):
    src_yaml = Path(dataset_root) / "data.yaml"
    data = {}
    if src_yaml.exists():
        with open(src_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    data["path"] = str(Path(dataset_root).resolve())
    data.setdefault("train", "train/images")
    data.setdefault("val", "val/images")
    data.setdefault("test", "test/images")
    data.setdefault("kpt_shape", [9, 3])
    data.setdefault("names", {0: "CenterHole", 1: "CenterHole_B", 2: "Hole", 3: "Hole_B"})
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return str(out_path)


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


def evaluate_model(model, data_yaml, out_dir, name, imgsz=640, batch=16, device="0", workers=4):
    """Run model.val() and return results dict."""
    results = model.val(
        data=data_yaml,
        split="test",
        imgsz=imgsz,
        batch=batch,
        device=device,
        workers=workers,
        project=str(Path(out_dir).resolve()),
        name=name,
        exist_ok=True,
        plots=False,
        save_json=False,
        rect=True,
    )
    return {
        "results_dict": _to_builtin(getattr(results, "results_dict", {})),
        "speed_ms": _to_builtin(getattr(results, "speed", {})),
    }


# ══════════════════════════════════════════════════════════════════
# Real dataset inference
# ══════════════════════════════════════════════════════════════════

def run_real_inference(model, real_root, out_dir, model_name, preprocess_fn=None,
                       n_per_condition=10, imgsz=640, conf=0.25, seed=42):
    """Infer on bright/medium/dark, save annotated images, return counts."""
    real_root = Path(real_root)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rng = random.Random(seed)
    counts = {}

    for condition in ["bright", "medium", "dark"]:
        cond_dir = real_root / condition
        # bright has images/ subfolder, medium/dark have images directly
        img_subdir = cond_dir / "images"
        if img_subdir.exists():
            cond_dir = img_subdir
        if not cond_dir.exists():
            print(f"  [skip] {condition} not found")
            continue

        all_imgs = sorted([f for f in cond_dir.iterdir() if f.suffix.lower() in (".png", ".jpg", ".jpeg")])
        selected = rng.sample(all_imgs, min(n_per_condition, len(all_imgs)))
        cond_count = 0

        for img_path in selected:
            img = cv2.imread(str(img_path))
            if img is None:
                continue

            # Apply preprocessing if needed
            input_img = preprocess_fn(img) if preprocess_fn else img

            results = model.predict(
                input_img, imgsz=imgsz, conf=conf, verbose=False,
            )
            r = results[0]
            n_det = len(r.boxes) if r.boxes is not None else 0
            cond_count += n_det

            # Draw on ORIGINAL image (not preprocessed) for visual comparison
            annotated = img.copy()
            if r.boxes is not None and len(r.boxes) > 0:
                boxes = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                kpts = r.keypoints.xy.cpu().numpy() if r.keypoints is not None else None

                for i, (box, c) in enumerate(zip(boxes, confs)):
                    x1, y1, x2, y2 = box.astype(int)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, f"{c:.2f}", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    if kpts is not None:
                        for kx, ky in kpts[i]:
                            if kx > 0 and ky > 0:
                                cv2.circle(annotated, (int(kx), int(ky)), 3, (0, 0, 255), -1)

            out_name = f"{condition}_{img_path.stem}.png"
            cv2.imwrite(str(out_dir / out_name), annotated)

        counts[condition] = cond_count
        print(f"  [{model_name}] {condition}: {cond_count} detections in {len(selected)} images")

    return counts


# ══════════════════════════════════════════════════════════════════
# Prepare preprocessed test set (for CLAHE/retinex eval)
# ══════════════════════════════════════════════════════════════════

def prepare_preprocessed_testset(src_root, dst_root, preprocess_fn):
    """Copy test set with preprocessing applied to images. Labels copied as-is."""
    src_root, dst_root = Path(src_root), Path(dst_root)
    if dst_root.exists():
        return dst_root

    for split in ["train", "val", "test"]:
        src_img = src_root / split / "images"
        src_lbl = src_root / split / "labels"
        dst_img = dst_root / split / "images"
        dst_lbl = dst_root / split / "labels"

        if not src_img.exists():
            continue
        dst_img.mkdir(parents=True, exist_ok=True)
        dst_lbl.mkdir(parents=True, exist_ok=True)

        for img_path in sorted(src_img.glob("*.*")):
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            processed = preprocess_fn(img)
            cv2.imwrite(str(dst_img / img_path.name), processed)

        if src_lbl.exists():
            import shutil
            for lbl in src_lbl.glob("*.*"):
                shutil.copy2(lbl, dst_lbl / lbl.name)

    src_yaml = src_root / "data.yaml"
    if src_yaml.exists():
        import shutil
        shutil.copy2(src_yaml, dst_root / "data.yaml")

    return dst_root


# ══════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Compare all 4 vision models")
    ap.add_argument("--data-root", type=Path,
                    default=Path("vision/dataset/yolo_pose_baseline_dataset"))
    ap.add_argument("--real-root", type=Path,
                    default=Path("vision/dataset/real_dataset/saved_images"))
    ap.add_argument("--result-dir", type=Path,
                    default=Path("vision/result/compare"))
    ap.add_argument("--yolo-weights", type=Path,
                    default=Path("vision/result/weight/yolo_baseline.pt"))
    ap.add_argument("--clahe-weights", type=Path,
                    default=Path("vision/result/weight/clahe_baseline.pt"))
    ap.add_argument("--retinex-weights", type=Path,
                    default=Path("vision/result/weight/retinex_baseline.pt"))
    ap.add_argument("--coin-yolo-weights", type=Path,
                    default=Path("vision/result/weight/coin_yolo.pt"))
    ap.add_argument("--coin-modules", type=Path,
                    default=Path("vision/result/weight/coin_modules.pt"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--n-real", type=int, default=10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--skip-eval", action="store_true", help="Skip test-set evaluation")
    ap.add_argument("--skip-real", action="store_true", help="Skip real-dataset inference")
    args = ap.parse_args()

    result_dir = args.result_dir
    result_dir.mkdir(parents=True, exist_ok=True)

    # ── Model configs ──
    models_cfg = {
        "yolo_baseline": {
            "weights": args.yolo_weights,
            "preprocess": None,
            "loader": lambda w: YOLO(str(w)),
        },
        "clahe_baseline": {
            "weights": args.clahe_weights,
            "preprocess": apply_clahe,
            "loader": lambda w: YOLO(str(w)),
        },
        "retinex_baseline": {
            "weights": args.retinex_weights,
            "preprocess": apply_msrcr,
            "loader": lambda w: YOLO(str(w)),
        },
        "coin_pose": {
            "weights": args.coin_yolo_weights,
            "preprocess": None,  # COIN handles illumination internally
            "loader": lambda w: load_coin_model(w, args.coin_modules, args.device),
        },
    }

    # Validate weights exist
    for name, cfg in models_cfg.items():
        if not cfg["weights"].exists():
            print(f"[WARN] {name} weights not found: {cfg['weights']}")

    # ══════════════════════════════════════════════════════════════
    # Part 1: Test-set evaluation (mAP)
    # ══════════════════════════════════════════════════════════════
    eval_results = {}

    if not args.skip_eval:
        print("\n" + "=" * 60)
        print("TEST-SET EVALUATION (yolo_pose_baseline_dataset)")
        print("=" * 60)

        eval_dir = result_dir / "eval"
        eval_dir.mkdir(parents=True, exist_ok=True)
        cache_dir = result_dir / "cache"

        for name, cfg in models_cfg.items():
            if not cfg["weights"].exists():
                print(f"\n[SKIP] {name}: weights not found")
                continue

            print(f"\n── {name} ──")
            model = cfg["loader"](cfg["weights"])

            # For CLAHE/retinex, need preprocessed copy of test set
            if cfg["preprocess"] is not None:
                pp_root = cache_dir / f"{name}_dataset"
                print(f"  Preparing preprocessed dataset -> {pp_root}")
                prepare_preprocessed_testset(args.data_root, pp_root, cfg["preprocess"])
                data_yaml = write_data_yaml(pp_root, eval_dir / f"{name}_data.yaml")
            else:
                data_yaml = write_data_yaml(args.data_root, eval_dir / f"{name}_data.yaml")

            res = evaluate_model(
                model, data_yaml, str(eval_dir), name,
                imgsz=args.imgsz, batch=args.batch,
                device=args.device, workers=args.workers,
            )
            eval_results[name] = res

        # Save and print summary
        with open(result_dir / "eval_results.json", "w", encoding="utf-8") as f:
            json.dump(eval_results, f, indent=2)

        print("\n" + "=" * 60)
        print("EVALUATION SUMMARY")
        print("=" * 60)
        header = f"{'Model':<20} {'mAP50-box':>10} {'mAP50-95-box':>13} {'mAP50-pose':>11} {'mAP50-95-pose':>14}"
        print(header)
        print("-" * len(header))
        for name, res in eval_results.items():
            rd = res.get("results_dict", {})
            box50 = rd.get("metrics/mAP50(B)", "-")
            box5095 = rd.get("metrics/mAP50-95(B)", "-")
            pose50 = rd.get("metrics/mAP50(P)", "-")
            pose5095 = rd.get("metrics/mAP50-95(P)", "-")
            if isinstance(box50, float):
                print(f"{name:<20} {box50:>10.4f} {box5095:>13.4f} {pose50:>11.4f} {pose5095:>14.4f}")
            else:
                print(f"{name:<20} {box50:>10} {box5095:>13} {pose50:>11} {pose5095:>14}")

    # ══════════════════════════════════════════════════════════════
    # Part 2: Real-dataset inference
    # ══════════════════════════════════════════════════════════════
    all_counts = {}

    if not args.skip_real:
        print("\n" + "=" * 60)
        print("REAL DATASET INFERENCE")
        print("=" * 60)

        real_dir = result_dir / "real_inference"

        for name, cfg in models_cfg.items():
            if not cfg["weights"].exists():
                print(f"\n[SKIP] {name}: weights not found")
                continue

            print(f"\n── {name} ──")
            model = cfg["loader"](cfg["weights"])

            out = real_dir / name
            counts = run_real_inference(
                model, args.real_root, out, name,
                preprocess_fn=cfg["preprocess"],
                n_per_condition=args.n_real,
                imgsz=args.imgsz,
                seed=args.seed,
            )
            all_counts[name] = counts

        # Save and print summary
        with open(result_dir / "real_inference_counts.json", "w", encoding="utf-8") as f:
            json.dump(all_counts, f, indent=2)

        print("\n" + "=" * 60)
        print("REAL INFERENCE DETECTION COUNTS")
        print("=" * 60)
        header = f"{'Model':<20} {'Bright':>8} {'Medium':>8} {'Dark':>8} {'Total':>8}"
        print(header)
        print("-" * len(header))
        for name, counts in all_counts.items():
            b = counts.get("bright", 0)
            m = counts.get("medium", 0)
            d = counts.get("dark", 0)
            print(f"{name:<20} {b:>8} {m:>8} {d:>8} {b+m+d:>8}")

    print("\n[done] Results saved to", result_dir.resolve())


if __name__ == "__main__":
    main()
