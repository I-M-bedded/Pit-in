#!/usr/bin/env python
"""
Plain YOLO-Pose baseline training on yolo_pose_baseline_dataset.

By default this script trains YOLO26 baseline models for scales n/s/m
sequentially. The n-scale run keeps backward compatibility by also copying its
best checkpoint to vision/result/weight/yolo_baseline.pt.
"""

import argparse
import json
import shutil
from pathlib import Path

import yaml
from ultralytics import YOLO

from online_lighting_aug import LightingAugPoseTrainer


DEFAULT_DATA_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
DEFAULT_MODEL_ARTIFACTS = Path("vision/yolo/weights/model_artifacts.json")
DEFAULT_BASE_WEIGHTS = Path("vision/yolo/weights/yolo26n-pose.pt")
DEFAULT_RUNS_DIR = Path("vision/yolo/runs/yolo_baseline")
DEFAULT_RESULT_DIR = Path("vision/result/yolo_baseline")
DEFAULT_EVAL_DATA_ROOTS = [
    Path("vision/dataset/yolo_pose_baseline_dataset"),
    Path("vision/dataset/yolo_pose_distill_dataset"),
]
DEFAULT_SCALES = ["n", "s", "m"]
DEFAULT_EPOCHS = 80


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


def ensure_yolo26_pose_yaml(scale: str, model_artifacts_path: Path, out_dir: Path) -> Path:
    if scale not in {"n", "s", "m"}:
        raise ValueError(f"Unsupported scale: {scale}")
    if not model_artifacts_path.exists():
        raise FileNotFoundError(f"Model artifacts not found: {model_artifacts_path}")

    with open(model_artifacts_path, "r", encoding="utf-8") as f:
        artifacts = json.load(f)

    model_yaml = dict(artifacts["yaml"])
    model_yaml["scale"] = scale
    model_yaml["yaml_file"] = f"yolo26{scale}-pose.yaml"

    out_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = out_dir / f"yolo26{scale}-pose.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(model_yaml, f, sort_keys=False)
    return yaml_path


def resolve_model_source(
    scale: str,
    model_artifacts_path: Path,
    yaml_cache_dir: Path,
    explicit_base_weights: Path | None = None,
) -> Path:
    if explicit_base_weights is not None:
        return explicit_base_weights

    pretrained_pt = Path(f"vision/yolo/weights/yolo26{scale}-pose.pt")
    if pretrained_pt.exists():
        return pretrained_pt

    return ensure_yolo26_pose_yaml(scale, model_artifacts_path, yaml_cache_dir)


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


def train_single_scale(
    scale: str,
    *,
    data_root: Path,
    model_artifacts_path: Path,
    runs_root: Path,
    result_root: Path,
    base_weights: Path | None,
    epochs: int,
    batch: int,
    imgsz: int,
    device: str,
    workers: int,
    eval_data_roots: list[Path],
    online_aug: bool,
    lowlight_p: float,
    backlight_p: float,
    backlight_p_b: float,
) -> dict:
    runs_dir = runs_root / scale
    result_dir = result_root / scale
    runs_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)

    data_yaml = write_corrected_data_yaml(data_root, runs_dir / "data.yaml")
    model_source = resolve_model_source(
        scale=scale,
        model_artifacts_path=model_artifacts_path,
        yaml_cache_dir=runs_root / "_model_yamls",
        explicit_base_weights=base_weights,
    )
    warmstart_weights = None
    if base_weights is None and scale in {"s", "m"} and DEFAULT_BASE_WEIGHTS.exists():
        warmstart_weights = DEFAULT_BASE_WEIGHTS

    model = YOLO(str(model_source))
    if warmstart_weights is not None and model_source.suffix in {".yaml", ".yml"}:
        model.load(str(warmstart_weights))
    run_meta = {
        "method": "baseline_finetune",
        "scale": scale,
        "train_data_root": str(data_root.resolve()),
        "eval_data_roots": [str(p.resolve()) for p in eval_data_roots],
        "model_source": str(model_source.resolve()),
        "warmstart_weights": str(warmstart_weights.resolve()) if warmstart_weights is not None else None,
        "epochs": epochs,
        "batch": batch,
        "imgsz": imgsz,
        "device": device,
        "workers": workers,
        "runs_dir": str(runs_dir.resolve()),
        "result_dir": str(result_dir.resolve()),
        "n_params": int(sum(p.numel() for p in model.model.parameters())),
    }
    with open(result_dir / f"baseline_run_meta_{scale}.json", "w", encoding="utf-8") as f:
        json.dump(run_meta, f, indent=2)

    results = model.train(
        data=str(data_yaml),
        epochs=epochs,
        batch=batch,
        imgsz=imgsz,
        device=device,
        workers=workers,
        project=str(runs_dir.resolve()),
        name="train",
        exist_ok=True,
        trainer=LightingAugPoseTrainer,
        online_aug=online_aug,
        lowlight_p=lowlight_p,
        backlight_p=backlight_p,
        backlight_p_b=backlight_p_b,
    )

    best_ckpt = Path(results.save_dir) / "weights" / "best.pt"
    last_ckpt = Path(results.save_dir) / "weights" / "last.pt"
    src_ckpt = best_ckpt if best_ckpt.exists() else last_ckpt
    if not src_ckpt.exists():
        raise FileNotFoundError(f"No checkpoint found in {results.save_dir}")

    train_out = result_dir / "train"
    shutil.copytree(results.save_dir, str(train_out), dirs_exist_ok=True)
    print(f"[baseline:{scale}] training artifacts copied to {train_out.resolve()}")

    central_weights = Path("vision/result/weight")
    central_weights.mkdir(parents=True, exist_ok=True)
    scale_weight_path = central_weights / f"yolo_baseline_{scale}.pt"
    shutil.copy2(src_ckpt, scale_weight_path)
    print(f"[baseline:{scale}] best weight -> {scale_weight_path}")
    if scale == "n":
        shutil.copy2(src_ckpt, central_weights / "yolo_baseline.pt")
        print(f"[baseline:{scale}] best weight alias -> {central_weights / 'yolo_baseline.pt'}")

    eval_dir = result_dir / "eval"
    evaluations = evaluate_yolo_on_datasets(
        weights_path=src_ckpt,
        eval_roots=eval_data_roots,
        out_dir=eval_dir,
        imgsz=imgsz,
        batch=batch,
        device=device,
        workers=workers,
    )
    with open(result_dir / f"baseline_evaluation_{scale}.json", "w", encoding="utf-8") as f:
        json.dump(evaluations, f, indent=2)
    print(f"[baseline:{scale}] evaluation saved to {(result_dir / f'baseline_evaluation_{scale}.json').resolve()}")

    return {
        "scale": scale,
        "model_source": str(model_source.resolve()),
        "best_weight": str(scale_weight_path.resolve()),
        "result_dir": str(result_dir.resolve()),
        "runs_dir": str(runs_dir.resolve()),
        "evaluation_path": str((result_dir / f"baseline_evaluation_{scale}.json").resolve()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Plain YOLO baseline training")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument(
        "--base-weights",
        type=Path,
        default=None,
        help="Optional override for single-scale runs. If omitted, n uses yolo26n-pose.pt and s/m use generated YAML.",
    )
    ap.add_argument("--model-artifacts", type=Path, default=DEFAULT_MODEL_ARTIFACTS)
    ap.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR)
    ap.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIR)
    ap.add_argument("--scales", type=str, nargs="+", default=DEFAULT_SCALES, choices=["n", "s", "m"])
    ap.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--eval-data-roots", type=Path, nargs="*", default=DEFAULT_EVAL_DATA_ROOTS)
    ap.add_argument("--online-aug", action="store_true", default=True)
    ap.add_argument("--no-online-aug", action="store_false", dest="online_aug")
    ap.add_argument("--lowlight-p", type=float, default=0.45)
    ap.add_argument("--backlight-p", type=float, default=0.28)
    ap.add_argument("--backlight-p-b", type=float, default=0.08)
    args = ap.parse_args()

    if not args.data_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {args.data_root}")
    if not args.model_artifacts.exists():
        raise FileNotFoundError(f"Model artifacts not found: {args.model_artifacts}")
    for eval_root in args.eval_data_roots:
        if not eval_root.exists():
            raise FileNotFoundError(f"Eval dataset root not found: {eval_root}")
    if args.base_weights is not None and not args.base_weights.exists():
        raise FileNotFoundError(f"Base weights not found: {args.base_weights}")
    if args.base_weights is not None and len(args.scales) != 1:
        raise ValueError("--base-weights override is only supported for a single-scale run")

    manifests = []
    for scale in args.scales:
        manifests.append(
            train_single_scale(
                scale=scale,
                data_root=args.data_root,
                model_artifacts_path=args.model_artifacts,
                runs_root=args.runs_dir,
                result_root=args.result_dir,
                base_weights=args.base_weights,
                epochs=args.epochs,
                batch=args.batch,
                imgsz=args.imgsz,
                device=args.device,
                workers=args.workers,
                eval_data_roots=args.eval_data_roots,
                online_aug=args.online_aug,
                lowlight_p=args.lowlight_p,
                backlight_p=args.backlight_p,
                backlight_p_b=args.backlight_p_b,
            )
        )

    with open(args.result_dir / "baseline_multiscale_manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifests, f, indent=2)
    print(f"[baseline] multiscale manifest saved to {(args.result_dir / 'baseline_multiscale_manifest.json').resolve()}")


if __name__ == "__main__":
    main()
