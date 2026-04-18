"""
Unified PRIM / PRIMv2 training with online lighting augmentation.

Uses LightingAugPoseTrainer (same as baselines) for stochastic
illumination augmentation every batch — the illumination encoder
sees diverse lighting each epoch for better generalisation.

Usage examples
--------------
# PRIM v1 (FiLM + pyramid only)
python train_PRIM.py --variant prim

# PRIMv2  (restore branch + FiLM + pyramid)
python train_PRIM.py --variant primv2

# Custom data / weights
python train_PRIM.py --variant primv2 \
    --data-root vision/dataset/yolo_pose_baseline_dataset \
    --base-weights vision/result/weight/yolo_baseline_n.pt \
    --epochs 100 --batch 8

# Resume with pre-trained modules
python train_PRIM.py --variant primv2 --module-ckpt path/to/modules.pt
"""

import argparse
import copy
import json
import shutil
import sys
import types
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
import yaml
from ultralytics import YOLO
from ultralytics.nn.tasks import PoseModel
from ultralytics.utils import DEFAULT_CFG

sys.path.insert(0, str(Path(__file__).resolve().parent))
from online_lighting_aug import LightingAugPoseTrainer
from PRIM_pose import (
    SpatiallyVariantIlluminationEncoder,
    SpatiallyAwareSoftRouter,
    MultiScaleRSVFiLM,
    PRIMFeaturePyramid,
    TotalVariationLoss,
    infer_prim_feature_spec,
    infer_checkpoint_channels,
)
from PRIMv2 import (
    FrequencySpatialIlluminationEncoder,
    MultiScaleSharedRestore,
    infer_primv2_feature_spec,
)


NUM_EXPERTS = 3
ILL_CH = 64
WORKERS = 4

AUG_MILD = dict(
    hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,
    degrees=5.0, translate=0.05, scale=0.15, shear=2.0, perspective=0.0,
    flipud=0.0, fliplr=0.0,
    mosaic=0.0, mixup=0.0, copy_paste=0.0,
)

# Module names per variant — used for save/load/cleanup
_VARIANT_MODULES = {
    "prim": ["prim_ie", "prim_sr", "prim_rsv", "prim_pyramid"],
    "primv2": ["prim2_ie", "prim2_sr", "prim2_restore", "prim2_rsv", "prim2_pyramid"],
}
_VARIANT_CKPT_KEYS = {
    "prim": ["illumination_encoder", "soft_router", "multi_scale_rsv", "feature_pyramid"],
    "primv2": ["illumination_encoder", "soft_router", "restore_branch", "multi_scale_rsv", "feature_pyramid"],
}
_VARIANT_ATTR_PREFIX = {"prim": "_prim_", "primv2": "_primv2_"}


# ═══════════════════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════════════════

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


def _detect_split_layout(root: str) -> Dict[str, str]:
    root_path = Path(root)
    if (root_path / "images" / "train").exists():
        return {"train": "images/train", "val": "images/val", "test": "images/test"}
    if (root_path / "train" / "images").exists():
        return {"train": "train/images", "val": "val/images", "test": "test/images"}
    raise FileNotFoundError(f"Could not detect dataset split layout under {root_path}")


def write_pose_yaml(data_root: str, out_path: str) -> str:
    split_layout = _detect_split_layout(data_root)
    data = {
        "path": str(Path(data_root).resolve()),
        "train": split_layout["train"],
        "val": split_layout["val"],
        "test": split_layout["test"],
        "kpt_shape": [9, 3],
        "flip_idx": list(range(9)),
        "names": {0: "CenterHole", 1: "CenterHole_B", 2: "Hole", 3: "Hole_B"},
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path


def _load_state_flexible(module: nn.Module, state_dict: dict, module_name: str):
    missing, unexpected = module.load_state_dict(state_dict, strict=False)
    if missing or unexpected:
        print(f"[patch] {module_name}: missing={len(missing)} unexpected={len(unexpected)}")


def _unwrap_module(module: Optional[nn.Module]) -> Optional[nn.Module]:
    if module is None:
        return None
    return module.module if hasattr(module, "module") else module


def _resolve_detection_model(model_or_wrapper):
    if hasattr(model_or_wrapper, "model") and hasattr(model_or_wrapper.model, "model"):
        return model_or_wrapper.model
    return model_or_wrapper


# ═══════════════════════════════════════════════════════════════════════════
# Feature spec helpers (shared between PRIM / PRIMv2 patching)
# ═══════════════════════════════════════════════════════════════════════════

def _choose_feature_spec(
    det: nn.Module,
    state: dict,
    feature_source: str,
    feature_layers,
    infer_fn,
) -> Dict[str, List[int]]:
    meta = state.get("meta", {}) if isinstance(state, dict) else {}
    if feature_layers is None and meta.get("target_layers"):
        return infer_fn(
            det.model,
            feature_source=str(meta.get("feature_source", feature_source)),
            feature_layers=meta.get("target_layers"),
        )
    if feature_layers is None:
        ckpt_channels = infer_checkpoint_channels(state)
        if ckpt_channels:
            for src in ["backbone", "head"]:
                candidate = infer_fn(det.model, feature_source=src)
                if candidate["channels"] == ckpt_channels:
                    return candidate
    return infer_fn(det.model, feature_source=feature_source, feature_layers=feature_layers)


# ═══════════════════════════════════════════════════════════════════════════
# Shared aux loss helpers
# ═══════════════════════════════════════════════════════════════════════════

def _gate_sparsity_loss(gate_raws: List[torch.Tensor]) -> torch.Tensor:
    """Softplus penalty on gate logits — encourages gates to stay closed."""
    return sum(F.softplus(g).mean() for g in gate_raws) / len(gate_raws)


def _expert_entropy_loss(p: torch.Tensor) -> torch.Tensor:
    """Penalise non-uniform expert usage (max_entropy − actual_entropy)."""
    avg_usage = p.mean(dim=(0, 2, 3))
    entropy = -(avg_usage * torch.log(avg_usage + 1e-8)).sum()
    max_ent = torch.log(torch.tensor(float(p.shape[1]), device=p.device))
    return max_ent - entropy


def _bucket_indices_from_files(im_files) -> Dict[str, List[int]]:
    indices = {"all": list(range(len(im_files))), "plain": [], "bloomed": []}
    for idx, file_path in enumerate(im_files):
        stem = Path(str(file_path)).stem
        if stem.endswith("_b"):
            indices["bloomed"].append(idx)
        else:
            indices["plain"].append(idx)
    return indices


def _mean_abs_over_indices(tensor: torch.Tensor, indices: List[int]) -> float:
    if tensor is None or not indices:
        return 0.0
    subset = tensor[indices]
    return float(subset.abs().mean().item())


def _mean_sigmoid_over_indices(tensor: torch.Tensor, indices: List[int]) -> float:
    if tensor is None or not indices:
        return 0.0
    subset = tensor[indices]
    return float(torch.sigmoid(subset).mean().item())


def _expert_usage_over_indices(router_probs: torch.Tensor, indices: List[int]) -> List[float]:
    if router_probs is None or not indices:
        return []
    usage = router_probs[indices].mean(dim=(0, 2, 3))
    return [float(v.item()) for v in usage]


def _expert_entropy_from_usage(usage: List[float]) -> float:
    if not usage:
        return 0.0
    usage_tensor = torch.tensor(usage, dtype=torch.float32)
    usage_tensor = usage_tensor / usage_tensor.sum().clamp_min(1e-8)
    return float((-(usage_tensor * torch.log(usage_tensor + 1e-8)).sum()).item())


def _make_internal_bucket_state(num_scales: int, num_experts: int) -> Dict[str, object]:
    return {
        "samples": 0,
        "expert_usage_sum": [0.0] * num_experts,
        "film_gate_mean_sum": [0.0] * num_scales,
        "film_delta_gamma_abs_sum": [0.0] * num_scales,
        "film_delta_beta_abs_sum": [0.0] * num_scales,
        "restore_gate_mean_sum": [0.0] * num_scales,
        "restore_gamma_abs_sum": [0.0] * num_scales,
        "restore_beta_abs_sum": [0.0] * num_scales,
    }


def _init_internal_log_state(det: nn.Module, variant: str):
    num_scales = len(getattr(det, "_prim_target_layers", [])) if variant == "prim" else len(getattr(det, "_primv2_target_layers", []))
    setattr(
        det,
        "_internal_log_state",
        {
            "variant": variant,
            "num_scales": num_scales,
            "num_experts": NUM_EXPERTS,
            "buckets": {
                "all": _make_internal_bucket_state(num_scales, NUM_EXPERTS),
                "plain": _make_internal_bucket_state(num_scales, NUM_EXPERTS),
                "bloomed": _make_internal_bucket_state(num_scales, NUM_EXPERTS),
            },
        },
    )


def _accumulate_internal_stats(det: nn.Module, batch: dict, cache: dict):
    state = getattr(det, "_internal_log_state", None)
    if state is None:
        return

    im_files = batch.get("im_file") or []
    batch_size = len(im_files)
    if batch_size == 0:
        imgs = batch.get("img")
        batch_size = int(imgs.shape[0]) if isinstance(imgs, torch.Tensor) else 0
        im_files = [f"sample_{idx}" for idx in range(batch_size)]
    bucket_indices = _bucket_indices_from_files(im_files)

    router_probs = cache.get("router_probs")
    film_gate_raws = cache.get("film_gate_raws", [])
    delta_gammas = cache.get("delta_gammas", [])
    delta_betas = cache.get("delta_betas", [])
    restore_gate_raws = cache.get("restore_gate_raws", [])
    restore_gammas = cache.get("restore_gammas", [])
    restore_betas = cache.get("restore_betas", [])

    for bucket_name, indices in bucket_indices.items():
        if not indices:
            continue
        bucket = state["buckets"][bucket_name]
        sample_count = len(indices)
        bucket["samples"] += sample_count

        usage = _expert_usage_over_indices(router_probs, indices)
        if usage:
            for expert_idx, value in enumerate(usage):
                bucket["expert_usage_sum"][expert_idx] += value * sample_count

        for scale_idx, gate_raw in enumerate(film_gate_raws):
            bucket["film_gate_mean_sum"][scale_idx] += _mean_sigmoid_over_indices(gate_raw, indices) * sample_count
        for scale_idx, delta_gamma in enumerate(delta_gammas):
            bucket["film_delta_gamma_abs_sum"][scale_idx] += _mean_abs_over_indices(delta_gamma, indices) * sample_count
        for scale_idx, delta_beta in enumerate(delta_betas):
            bucket["film_delta_beta_abs_sum"][scale_idx] += _mean_abs_over_indices(delta_beta, indices) * sample_count

        for scale_idx, gate_raw in enumerate(restore_gate_raws):
            bucket["restore_gate_mean_sum"][scale_idx] += _mean_sigmoid_over_indices(gate_raw, indices) * sample_count
        for scale_idx, restore_gamma in enumerate(restore_gammas):
            bucket["restore_gamma_abs_sum"][scale_idx] += _mean_abs_over_indices(restore_gamma, indices) * sample_count
        for scale_idx, restore_beta in enumerate(restore_betas):
            bucket["restore_beta_abs_sum"][scale_idx] += _mean_abs_over_indices(restore_beta, indices) * sample_count


def _finalize_internal_log_state(det: nn.Module, epoch: int) -> Dict[str, object]:
    state = getattr(det, "_internal_log_state", None)
    if state is None:
        return {}

    payload = {
        "epoch": int(epoch),
        "variant": state["variant"],
        "num_scales": state["num_scales"],
        "num_experts": state["num_experts"],
        "buckets": {},
    }
    for bucket_name, bucket in state["buckets"].items():
        samples = max(int(bucket["samples"]), 1)
        expert_usage = [value / samples for value in bucket["expert_usage_sum"]]
        payload["buckets"][bucket_name] = {
            "samples": int(bucket["samples"]),
            "expert_usage": expert_usage,
            "expert_entropy": _expert_entropy_from_usage(expert_usage),
            "film_gate_mean": [value / samples for value in bucket["film_gate_mean_sum"]],
            "film_delta_gamma_abs_mean": [value / samples for value in bucket["film_delta_gamma_abs_sum"]],
            "film_delta_beta_abs_mean": [value / samples for value in bucket["film_delta_beta_abs_sum"]],
            "restore_gate_mean": [value / samples for value in bucket["restore_gate_mean_sum"]],
            "restore_gamma_abs_mean": [value / samples for value in bucket["restore_gamma_abs_sum"]],
            "restore_beta_abs_mean": [value / samples for value in bucket["restore_beta_abs_sum"]],
        }
    return payload


def _flatten_internal_metrics(payload: Dict[str, object]) -> Dict[str, object]:
    row = {
        "epoch": payload.get("epoch", -1),
        "variant": payload.get("variant", ""),
    }
    for bucket_name, bucket in payload.get("buckets", {}).items():
        row[f"{bucket_name}_samples"] = bucket.get("samples", 0)
        row[f"{bucket_name}_expert_entropy"] = bucket.get("expert_entropy", 0.0)
        for idx, value in enumerate(bucket.get("expert_usage", [])):
            row[f"{bucket_name}_expert_usage_e{idx}"] = value
        for idx, value in enumerate(bucket.get("film_gate_mean", [])):
            row[f"{bucket_name}_film_gate_s{idx}"] = value
        for idx, value in enumerate(bucket.get("film_delta_gamma_abs_mean", [])):
            row[f"{bucket_name}_film_dgamma_s{idx}"] = value
        for idx, value in enumerate(bucket.get("film_delta_beta_abs_mean", [])):
            row[f"{bucket_name}_film_dbeta_s{idx}"] = value
        for idx, value in enumerate(bucket.get("restore_gate_mean", [])):
            row[f"{bucket_name}_restore_gate_s{idx}"] = value
        for idx, value in enumerate(bucket.get("restore_gamma_abs_mean", [])):
            row[f"{bucket_name}_restore_gamma_s{idx}"] = value
        for idx, value in enumerate(bucket.get("restore_beta_abs_mean", [])):
            row[f"{bucket_name}_restore_beta_s{idx}"] = value
    return row


# ═══════════════════════════════════════════════════════════════════════════
# PRIM v1 — patch YOLO
# ═══════════════════════════════════════════════════════════════════════════

def patch_yolo_with_prim(
    model_or_wrapper,
    module_ckpt: Optional[str] = None,
    feature_source: str = "backbone",
    feature_layers=None,
    anchor_weight: float = 0.01,
    aux_tv_weight: float = 0.05,
    gate_sparsity_weight: float = 0.05,
    expert_entropy_weight: float = 0.02,
):
    det = _resolve_detection_model(model_or_wrapper)
    state = torch.load(module_ckpt, map_location="cpu") if module_ckpt else {}
    spec = _choose_feature_spec(det, state, feature_source, feature_layers, infer_prim_feature_spec)
    target_layers = list(spec["target_layers"])
    channels = list(spec["channels"])
    resolutions = list(spec.get("resolutions", []))

    ie = SpatiallyVariantIlluminationEncoder(3, ILL_CH)
    sr = SpatiallyAwareSoftRouter(ILL_CH, NUM_EXPERTS)
    rsv = MultiScaleRSVFiLM(channels, ILL_CH, NUM_EXPERTS)
    pyramid = PRIMFeaturePyramid(channels, context_channels=128)

    if module_ckpt:
        _load_state_flexible(ie, state["illumination_encoder"], "illumination_encoder")
        _load_state_flexible(sr, state["soft_router"], "soft_router")
        _load_state_flexible(rsv, state["multi_scale_rsv"], "multi_scale_rsv")
        _load_state_flexible(pyramid, state["feature_pyramid"], "feature_pyramid")

    det.add_module("prim_ie", ie)
    det.add_module("prim_sr", sr)
    det.add_module("prim_rsv", rsv)
    det.add_module("prim_pyramid", pyramid)

    det._prim_anchor_params = []
    det._prim_params_ref = []
    for name in _VARIANT_MODULES["prim"]:
        for param in getattr(det, name).parameters():
            det._prim_anchor_params.append(param.data.clone())
            det._prim_params_ref.append(param)
    det._prim_cache = None
    det._prim_feature_source = spec["feature_source"]
    det._prim_target_layers = target_layers
    det._prim_channels = channels
    det._prim_resolutions = resolutions
    _init_internal_log_state(det, "prim")

    def _modulated_predict_once(self, x, profile=False, visualize=False, embed=None):
        z = self.prim_ie(x)
        p = self.prim_sr(z)
        y = []
        target_cache = {}
        delta_gammas, delta_betas, film_gate_raws = [], [], []

        for module in self.model:
            if module.f != -1:
                x = y[module.f] if isinstance(module.f, int) else [x if j == -1 else y[j] for j in module.f]
            x = module(x)
            if module.i in self._prim_target_layers:
                target_cache[module.i] = x
                if module.i == self._prim_target_layers[-1]:
                    feats = [target_cache[idx] for idx in self._prim_target_layers]
                    modulated = []
                    for scale_idx, feat in enumerate(feats):
                        feat_m, dg, db, fg_raw = self.prim_rsv.forward_single(feat, scale_idx, z, p)
                        modulated.append(feat_m)
                        delta_gammas.append(dg)
                        delta_betas.append(db)
                        film_gate_raws.append(fg_raw)
                    fused = self.prim_pyramid(modulated)
                    for layer_idx, feat_new in zip(self._prim_target_layers, fused):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new
                    x = target_cache[module.i]
            y.append(x if module.i in self.save else None)

            self._prim_cache = {
                "delta_gammas": delta_gammas,
                "delta_betas": delta_betas,
                "film_gate_raws": film_gate_raws,
                "router_probs": p,
        }
        return x

    det._predict_once = types.MethodType(_modulated_predict_once, det)
    orig_loss = type(det).loss
    tv_loss = TotalVariationLoss()

    def _aug_loss(self, batch, preds=None):
        loss, loss_items = orig_loss(self, batch, preds)
        if not self.training:
            return loss, loss_items

        # anchor regularisation
        if getattr(self, "_prim_anchor_params", None):
            a_loss = sum(
                F.mse_loss(p, a.to(device=p.device, dtype=p.dtype))
                for p, a in zip(self._prim_params_ref, self._prim_anchor_params)
            ) / len(self._prim_anchor_params)
            loss = loss + anchor_weight * a_loss

        if self._prim_cache is not None:
            cache = self._prim_cache
            _accumulate_internal_stats(self, batch, cache)

            # FiLM TV
            dg = cache.get("delta_gammas", [])
            db = cache.get("delta_betas", [])
            if dg and db:
                tv = sum(tv_loss(g) + tv_loss(b) for g, b in zip(dg, db)) / len(dg)
                loss = loss + aux_tv_weight * tv

            # gate sparsity (FiLM gates)
            fg = cache.get("film_gate_raws", [])
            if fg and gate_sparsity_weight > 0:
                loss = loss + gate_sparsity_weight * _gate_sparsity_loss(fg)

            # expert entropy
            rp = cache.get("router_probs")
            if rp is not None and expert_entropy_weight > 0:
                loss = loss + expert_entropy_weight * _expert_entropy_loss(rp)

            self._prim_cache = None
        return loss, loss_items

    det.loss = types.MethodType(_aug_loss, det)
    return model_or_wrapper


# ═══════════════════════════════════════════════════════════════════════════
# PRIMv2 — patch YOLO
# ═══════════════════════════════════════════════════════════════════════════

def patch_yolo_with_primv2(
    model_or_wrapper,
    module_ckpt: Optional[str] = None,
    feature_source: str = "backbone",
    feature_layers=None,
    anchor_weight: float = 0.01,
    film_tv_weight: float = 0.05,
    restore_tv_weight: float = 0.05,
    gate_sparsity_weight: float = 0.05,
    expert_entropy_weight: float = 0.02,
):
    det = _resolve_detection_model(model_or_wrapper)
    state = torch.load(module_ckpt, map_location="cpu") if module_ckpt else {}
    spec = _choose_feature_spec(det, state, feature_source, feature_layers, infer_primv2_feature_spec)
    target_layers = list(spec["target_layers"])
    channels = list(spec["channels"])
    resolutions = list(spec.get("resolutions", []))

    ie = FrequencySpatialIlluminationEncoder(3, ILL_CH)
    sr = SpatiallyAwareSoftRouter(ILL_CH, NUM_EXPERTS)
    restore = MultiScaleSharedRestore(channels, illumination_channels=ILL_CH)
    rsv = MultiScaleRSVFiLM(channels, ILL_CH, NUM_EXPERTS)
    pyramid = PRIMFeaturePyramid(channels, context_channels=128)

    if module_ckpt:
        _load_state_flexible(ie, state["illumination_encoder"], "illumination_encoder")
        _load_state_flexible(sr, state["soft_router"], "soft_router")
        _load_state_flexible(restore, state["restore_branch"], "restore_branch")
        _load_state_flexible(rsv, state["multi_scale_rsv"], "multi_scale_rsv")
        _load_state_flexible(pyramid, state["feature_pyramid"], "feature_pyramid")

    det.add_module("prim2_ie", ie)
    det.add_module("prim2_sr", sr)
    det.add_module("prim2_restore", restore)
    det.add_module("prim2_rsv", rsv)
    det.add_module("prim2_pyramid", pyramid)

    det._primv2_anchor_params = []
    det._primv2_params_ref = []
    for name in _VARIANT_MODULES["primv2"]:
        for param in getattr(det, name).parameters():
            det._primv2_anchor_params.append(param.data.clone())
            det._primv2_params_ref.append(param)
    det._primv2_cache = None
    det._primv2_feature_source = spec["feature_source"]
    det._primv2_target_layers = target_layers
    det._primv2_channels = channels
    det._primv2_resolutions = resolutions
    _init_internal_log_state(det, "primv2")

    def _modulated_predict_once(self, x, profile=False, visualize=False, embed=None):
        z = self.prim2_ie(x)
        p = self.prim2_sr(z)
        y = []
        target_cache = {}
        delta_gammas, delta_betas, film_gate_raws = [], [], []
        restore_gammas, restore_betas, restore_gate_raws = [], [], []

        for module in self.model:
            if module.f != -1:
                x = y[module.f] if isinstance(module.f, int) else [x if j == -1 else y[j] for j in module.f]
            x = module(x)
            if module.i in self._primv2_target_layers:
                target_cache[module.i] = x
                if module.i == self._primv2_target_layers[-1]:
                    feats = [target_cache[idx] for idx in self._primv2_target_layers]
                    modulated = []
                    for scale_idx, feat in enumerate(feats):
                        feat_r, r_gamma, r_beta, rg_raw = self.prim2_restore.forward_single(feat, scale_idx, z)
                        feat_m, dg, db, fg_raw = self.prim2_rsv.forward_single(feat_r, scale_idx, z, p)
                        restore_gammas.append(r_gamma)
                        restore_betas.append(r_beta)
                        restore_gate_raws.append(rg_raw)
                        delta_gammas.append(dg)
                        delta_betas.append(db)
                        film_gate_raws.append(fg_raw)
                        modulated.append(feat_m)
                    fused = self.prim2_pyramid(modulated)
                    for layer_idx, feat_new in zip(self._primv2_target_layers, fused):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new
                    x = target_cache[module.i]
            y.append(x if module.i in self.save else None)

        self._primv2_cache = {
            "delta_gammas": delta_gammas,
            "delta_betas": delta_betas,
            "film_gate_raws": film_gate_raws,
            "restore_gammas": restore_gammas,
            "restore_betas": restore_betas,
            "restore_gate_raws": restore_gate_raws,
            "router_probs": p,
        }
        return x

    det._predict_once = types.MethodType(_modulated_predict_once, det)
    orig_loss = type(det).loss
    tv_loss = TotalVariationLoss()

    def _aug_loss(self, batch, preds=None):
        loss, loss_items = orig_loss(self, batch, preds)
        if not self.training:
            return loss, loss_items

        # anchor regularisation
        if getattr(self, "_primv2_anchor_params", None):
            a_loss = sum(
                F.mse_loss(p, a.to(device=p.device, dtype=p.dtype))
                for p, a in zip(self._primv2_params_ref, self._primv2_anchor_params)
            ) / len(self._primv2_anchor_params)
            loss = loss + anchor_weight * a_loss

        if self._primv2_cache is not None:
            cache = self._primv2_cache
            _accumulate_internal_stats(self, batch, cache)

            # FiLM TV
            dg = cache.get("delta_gammas", [])
            db = cache.get("delta_betas", [])
            if dg and db:
                ftv = sum(tv_loss(g) + tv_loss(b) for g, b in zip(dg, db)) / len(dg)
                loss = loss + film_tv_weight * ftv

            # Restore TV (on gamma + beta)
            rg = cache.get("restore_gammas", [])
            rb = cache.get("restore_betas", [])
            if rg and rb:
                rtv = sum(tv_loss(g) + tv_loss(b) for g, b in zip(rg, rb)) / len(rg)
                loss = loss + restore_tv_weight * rtv

            # Gate sparsity (restore + FiLM gates)
            all_gate_raws = cache.get("restore_gate_raws", []) + cache.get("film_gate_raws", [])
            if all_gate_raws and gate_sparsity_weight > 0:
                loss = loss + gate_sparsity_weight * _gate_sparsity_loss(all_gate_raws)

            # Expert entropy
            rp = cache.get("router_probs")
            if rp is not None and expert_entropy_weight > 0:
                loss = loss + expert_entropy_weight * _expert_entropy_loss(rp)

            self._primv2_cache = None
        return loss, loss_items

    det.loss = types.MethodType(_aug_loss, det)
    return model_or_wrapper


# ═══════════════════════════════════════════════════════════════════════════
# Module save / load helpers
# ═══════════════════════════════════════════════════════════════════════════

def _resolve_host(module: Optional[nn.Module], variant: str) -> Optional[nn.Module]:
    module = _unwrap_module(module)
    if module is None:
        return None
    required = tuple(_VARIANT_MODULES[variant])
    if all(hasattr(module, n) for n in required):
        return module
    inner = _unwrap_module(getattr(module, "model", None))
    if inner is not None and all(hasattr(inner, n) for n in required):
        return inner
    return None


def save_modules(detection_model: nn.Module, path: str, variant: str):
    prefix = _VARIANT_ATTR_PREFIX[variant]
    mod_names = _VARIANT_MODULES[variant]
    ckpt_keys = _VARIANT_CKPT_KEYS[variant]
    payload = {
        "meta": {
            "feature_source": getattr(detection_model, f"{prefix}feature_source", "backbone"),
            "target_layers": list(getattr(detection_model, f"{prefix}target_layers", [])),
            "channels": list(getattr(detection_model, f"{prefix}channels", [])),
            "resolutions": list(getattr(detection_model, f"{prefix}resolutions", [])),
        },
    }
    for mod_name, key in zip(mod_names, ckpt_keys):
        payload[key] = getattr(detection_model, mod_name).state_dict()
    torch.save(payload, path)


def _save_modules_from_trainer(
    trainer, path: str, variant: str, prefer_ema: bool = False,
) -> bool:
    ema_obj = getattr(getattr(trainer, "ema", None), "ema", None)
    model_obj = getattr(trainer, "model", None)
    candidates = [ema_obj, model_obj] if prefer_ema else [model_obj, ema_obj]
    for c in candidates:
        host = _resolve_host(c, variant)
        if host is not None:
            save_modules(host, path, variant)
            return True
    return False


# ═══════════════════════════════════════════════════════════════════════════
# Trainer class (unified, extends LightingAugPoseTrainer)
# ═══════════════════════════════════════════════════════════════════════════

def build_trainer_class(variant: str):
    mod_names = _VARIANT_MODULES[variant]
    attr_prefix = _VARIANT_ATTR_PREFIX[variant]

    class PRIMPoseTrainer(LightingAugPoseTrainer):
        """
        Extends LightingAugPoseTrainer with PRIM/PRIMv2 module patching.

        Inherits online lighting augmentation (lowlight + backlight with
        reduced probability for _b suffixed images) from the base class.
        """

        def __init__(self, cfg=None, overrides=None, _callbacks=None):
            overrides = dict(overrides or {})
            self._prim_variant = variant
            self._module_ckpt = overrides.pop("module_ckpt", None)
            self._feature_source = overrides.pop("prim_feature_source", "backbone")
            self._feature_layers = overrides.pop("prim_feature_layers", None)
            self._anchor_weight = float(overrides.pop("anchor_weight", 0.01))
            self._film_tv_weight = float(overrides.pop("film_tv_weight", 0.05))
            self._restore_tv_weight = float(overrides.pop("restore_tv_weight", 0.05))
            self._gate_sparsity_weight = float(overrides.pop("gate_sparsity_weight", 0.05))
            self._expert_entropy_weight = float(overrides.pop("expert_entropy_weight", 0.02))
            super().__init__(cfg or DEFAULT_CFG, overrides, _callbacks)

        def get_model(self, cfg=None, weights=None, verbose=True):
            model = PoseModel(
                cfg,
                nc=self.data["nc"],
                ch=self.data["channels"],
                data_kpt_shape=self.data["kpt_shape"],
                verbose=verbose,
            )
            if weights:
                model.load(weights)

            if self._prim_variant == "prim":
                patch_yolo_with_prim(
                    model,
                    module_ckpt=self._module_ckpt,
                    feature_source=self._feature_source,
                    feature_layers=self._feature_layers,
                    anchor_weight=self._anchor_weight,
                    aux_tv_weight=self._film_tv_weight,
                    gate_sparsity_weight=self._gate_sparsity_weight,
                    expert_entropy_weight=self._expert_entropy_weight,
                )
            else:
                patch_yolo_with_primv2(
                    model,
                    module_ckpt=self._module_ckpt,
                    feature_source=self._feature_source,
                    feature_layers=self._feature_layers,
                    anchor_weight=self._anchor_weight,
                    film_tv_weight=self._film_tv_weight,
                    restore_tv_weight=self._restore_tv_weight,
                    gate_sparsity_weight=self._gate_sparsity_weight,
                    expert_entropy_weight=self._expert_entropy_weight,
                )
            return model

        @staticmethod
        def _make_plain_export_model(model):
            plain = copy.deepcopy(model).half()
            for name in mod_names:
                if hasattr(plain, name):
                    delattr(plain, name)
            for suffix in [
                "anchor_params", "params_ref", "cache",
                "feature_source", "target_layers", "channels", "resolutions",
            ]:
                attr = f"{attr_prefix}{suffix}"
                if hasattr(plain, attr):
                    delattr(plain, attr)
            for attr in ["_predict_once", "loss"]:
                if attr in getattr(plain, "__dict__", {}):
                    del plain.__dict__[attr]
            return plain

        def save_model(self):
            export_src = getattr(getattr(self, "ema", None), "ema", None) or self.model
            export_model = self._make_plain_export_model(export_src)
            ckpt = {
                "epoch": self.epoch,
                "best_fitness": self.best_fitness,
                "model": None,
                "ema": export_model,
                "train_args": vars(self.args),
                "train_metrics": {
                    **(self.metrics or {}),
                    **({"fitness": self.fitness} if self.fitness is not None else {}),
                },
                "train_results": self.read_results_csv(),
            }
            self.wdir.mkdir(parents=True, exist_ok=True)
            torch.save(ckpt, self.last)
            if self.best_fitness == self.fitness:
                torch.save(ckpt, self.best)
            return True

        def final_eval(self):
            return

    return PRIMPoseTrainer


# ═══════════════════════════════════════════════════════════════════════════
# Callbacks
# ═══════════════════════════════════════════════════════════════════════════

class ModuleSaveCallback:
    def __init__(self, save_dir: str, variant: str):
        self.save_dir = Path(save_dir)
        self.variant = variant

    def __call__(self, trainer):
        _save_modules_from_trainer(
            trainer,
            str(self.save_dir / f"{self.variant}_modules_last.pt"),
            self.variant,
            prefer_ema=False,
        )
        if trainer.best_fitness == trainer.fitness:
            _save_modules_from_trainer(
                trainer,
                str(self.save_dir / f"{self.variant}_modules_best.pt"),
                self.variant,
                prefer_ema=True,
            )


class AnchorEMACallback:
    def __init__(self, variant: str, momentum: float = 0.999):
        self.variant = variant
        self.momentum = momentum

    def __call__(self, trainer):
        prefix = _VARIANT_ATTR_PREFIX[self.variant]
        det = _resolve_host(getattr(trainer, "model", None), self.variant)
        if det is None:
            return
        anchor_attr = f"{prefix}anchor_params"
        ref_attr = f"{prefix}params_ref"
        anchors = getattr(det, anchor_attr, None)
        refs = getattr(det, ref_attr, None)
        if anchors is None or refs is None:
            return
        for idx, (anchor, param) in enumerate(zip(anchors, refs)):
            if anchor.device != param.device or anchor.dtype != param.dtype:
                anchor = anchor.to(device=param.device, dtype=param.dtype)
                anchors[idx] = anchor
            anchor.data.mul_(self.momentum).add_(param.data, alpha=1.0 - self.momentum)


class InternalMetricsCallback:
    def __init__(self, variant: str):
        self.variant = variant

    def _resolve_output_dir(self, trainer) -> Path:
        save_dir = getattr(trainer, "save_dir", None)
        if save_dir is None:
            save_dir = getattr(trainer, "wdir", None)
        return Path(save_dir)

    def _append_csv_row(self, csv_path: Path, row: Dict[str, object]):
        import csv

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = list(row.keys())
        write_header = not csv_path.exists()
        if not write_header:
            with open(csv_path, "r", encoding="utf-8", newline="") as f:
                reader = csv.reader(f)
                existing_header = next(reader, [])
            if existing_header != fieldnames:
                write_header = True

        mode = "w" if write_header else "a"
        with open(csv_path, mode, encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    def on_epoch_start(self, trainer):
        det = _resolve_host(getattr(trainer, "model", None), self.variant)
        if det is not None:
            _init_internal_log_state(det, self.variant)

    def on_epoch_end(self, trainer):
        det = _resolve_host(getattr(trainer, "model", None), self.variant)
        if det is None:
            return
        payload = _finalize_internal_log_state(det, int(getattr(trainer, "epoch", -1)) + 1)
        if not payload:
            return

        out_dir = self._resolve_output_dir(trainer)
        jsonl_path = out_dir / f"{self.variant}_internal_metrics.jsonl"
        csv_path = out_dir / f"{self.variant}_internal_metrics.csv"

        with open(jsonl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self._append_csv_row(csv_path, _flatten_internal_metrics(payload))


# ═══════════════════════════════════════════════════════════════════════════
# Training
# ═══════════════════════════════════════════════════════════════════════════

def train(
    variant: str,
    data_root: str,
    base_weights: str,
    module_ckpt: Optional[str],
    out_dir: str,
    feature_source: str = "backbone",
    feature_layers=None,
    epochs: int = 80,
    batch_size: int = 8,
    lr: float = 1e-4,
    imgsz: int = 640,
    device_str: str = "0",
    patience: int = 15,
    anchor_weight: float = 0.01,
    film_tv_weight: float = 0.05,
    restore_tv_weight: float = 0.05,
    gate_sparsity_weight: float = 0.05,
    expert_entropy_weight: float = 0.02,
    lowlight_p: float = 0.45,
    backlight_p: float = 0.28,
    backlight_p_b: float = 0.08,
    workers: int = WORKERS,
) -> Tuple[str, str, str]:
    out = Path(out_dir) / "train"
    out.mkdir(parents=True, exist_ok=True)

    data_yaml = write_pose_yaml(data_root, str(out / "data.yaml"))
    TrainerClass = build_trainer_class(variant)

    model = YOLO(base_weights)
    model.add_callback("on_fit_epoch_end", ModuleSaveCallback(str(out), variant))
    model.add_callback("on_train_batch_end", AnchorEMACallback(variant, momentum=0.999))
    internal_metrics_cb = InternalMetricsCallback(variant)
    model.add_callback("on_train_epoch_start", internal_metrics_cb.on_epoch_start)
    model.add_callback("on_fit_epoch_end", internal_metrics_cb.on_epoch_end)

    results = model.train(
        data=data_yaml,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch_size,
        device=device_str,
        workers=workers,
        optimizer="AdamW",
        lr0=lr,
        weight_decay=5e-4,
        cos_lr=True,
        warmup_epochs=min(3, max(epochs // 10, 1)),
        close_mosaic=0,
        patience=patience,
        project=str(out.resolve()),
        name="run",
        exist_ok=True,
        rect=True,
        trainer=TrainerClass,
        module_ckpt=module_ckpt,
        prim_feature_source=feature_source,
        prim_feature_layers=feature_layers,
        anchor_weight=anchor_weight,
        film_tv_weight=film_tv_weight,
        restore_tv_weight=restore_tv_weight,
        gate_sparsity_weight=gate_sparsity_weight,
        expert_entropy_weight=expert_entropy_weight,
        online_aug=True,
        lowlight_p=lowlight_p,
        backlight_p=backlight_p,
        backlight_p_b=backlight_p_b,
        **AUG_MILD,
    )

    best_yolo = Path(results.save_dir) / "weights" / "best.pt"
    best_modules = out / f"{variant}_modules_best.pt"
    return str(best_yolo), str(best_modules), str(results.save_dir)


# ═══════════════════════════════════════════════════════════════════════════
# Evaluation
# ═══════════════════════════════════════════════════════════════════════════

def evaluate_on_datasets(
    variant: str,
    yolo_weights: str,
    module_weights: str,
    eval_roots: List[str],
    out_dir: str,
    imgsz: int,
    batch_size: int,
    device_str: str,
    workers: int = WORKERS,
) -> List[dict]:
    eval_dir = Path(out_dir) / "eval"
    eval_dir.mkdir(parents=True, exist_ok=True)
    evaluations: List[dict] = []

    for dataset_root in eval_roots:
        ds_path = Path(dataset_root)
        data_yaml = write_pose_yaml(str(ds_path), str(eval_dir / f"{ds_path.name}.yaml"))
        model = YOLO(yolo_weights)
        if variant == "prim":
            patch_yolo_with_prim(
                model, module_ckpt=module_weights,
                anchor_weight=0.0, aux_tv_weight=0.0,
                gate_sparsity_weight=0.0, expert_entropy_weight=0.0,
            )
        else:
            patch_yolo_with_primv2(
                model, module_ckpt=module_weights,
                anchor_weight=0.0, film_tv_weight=0.0, restore_tv_weight=0.0,
                gate_sparsity_weight=0.0, expert_entropy_weight=0.0,
            )
        results = model.val(
            data=data_yaml,
            split="test",
            imgsz=imgsz,
            batch=batch_size,
            device=device_str,
            workers=workers,
            project=str(eval_dir.resolve()),
            name=ds_path.name,
            exist_ok=True,
            plots=False,
            save_json=False,
            rect=True,
        )
        evaluations.append({
            "dataset_name": ds_path.name,
            "dataset_root": str(ds_path.resolve()),
            "results_dict": _to_builtin(getattr(results, "results_dict", {})),
            "speed_ms": _to_builtin(getattr(results, "speed", {})),
            "save_dir": str(Path(results.save_dir).resolve()),
        })
    return evaluations


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Unified PRIM / PRIMv2 training")
    ap.add_argument("--variant", choices=["prim", "primv2"], default="primv2")
    ap.add_argument("--data-root", type=str, default="vision/dataset/yolo_pose_baseline_dataset")
    ap.add_argument("--eval-data-roots", type=str, nargs="*",
                     default=["vision/dataset/yolo_pose_baseline_dataset"])
    ap.add_argument("--base-weights", type=str, default="vision/yolo/weights/yolo26n-pose.pt")
    ap.add_argument("--module-ckpt", type=str, default=None)
    ap.add_argument("--runs-dir", type=str, default=None,
                     help="defaults to vision/yolo/runs/{VARIANT}")
    ap.add_argument("--result-dir", type=str, default=None,
                     help="defaults to vision/result/{VARIANT}")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--workers", type=int, default=WORKERS)
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--prim-feature-source", type=str, default="backbone",
                     choices=["backbone", "head"])
    ap.add_argument("--prim-feature-layers", type=str, default=None)
    ap.add_argument("--anchor-weight", type=float, default=0.01)
    ap.add_argument("--film-tv-weight", type=float, default=0.05)
    ap.add_argument("--restore-tv-weight", type=float, default=0.05)
    ap.add_argument("--gate-sparsity-weight", type=float, default=0.05)
    ap.add_argument("--expert-entropy-weight", type=float, default=0.02)
    ap.add_argument("--lowlight-p", type=float, default=0.45)
    ap.add_argument("--backlight-p", type=float, default=0.28)
    ap.add_argument("--backlight-p-b", type=float, default=0.08)
    args = ap.parse_args()

    variant = args.variant.lower()
    variant_upper = variant.upper()

    runs_dir = args.runs_dir or f"vision/yolo/runs/{variant_upper}"
    result_dir = args.result_dir or f"vision/result/{variant_upper}"

    if not Path(args.data_root).exists():
        raise FileNotFoundError(f"Dataset root not found: {args.data_root}")
    for er in args.eval_data_roots:
        if not Path(er).exists():
            raise FileNotFoundError(f"Eval dataset root not found: {er}")

    Path(runs_dir).mkdir(parents=True, exist_ok=True)
    Path(result_dir).mkdir(parents=True, exist_ok=True)

    print(f"\n{'=' * 72}")
    print(f"  {variant_upper} training  |  epochs={args.epochs}  batch={args.batch}")
    print(f"  data={args.data_root}")
    print(f"  online aug: lowlight={args.lowlight_p}  backlight={args.backlight_p}  backlight_b={args.backlight_p_b}")
    print(f"{'=' * 72}\n")

    yolo_best, modules_best, save_dir = train(
        variant=variant,
        data_root=args.data_root,
        base_weights=args.base_weights,
        module_ckpt=args.module_ckpt,
        out_dir=runs_dir,
        feature_source=args.prim_feature_source,
        feature_layers=args.prim_feature_layers,
        epochs=args.epochs,
        batch_size=args.batch,
        lr=args.lr,
        imgsz=args.imgsz,
        device_str=args.device,
        patience=args.patience,
        anchor_weight=args.anchor_weight,
        film_tv_weight=args.film_tv_weight,
        restore_tv_weight=args.restore_tv_weight,
        gate_sparsity_weight=args.gate_sparsity_weight,
        expert_entropy_weight=args.expert_entropy_weight,
        lowlight_p=args.lowlight_p,
        backlight_p=args.backlight_p,
        backlight_p_b=args.backlight_p_b,
        workers=args.workers,
    )

    # ---- copy results ----
    train_runs = Path(save_dir)
    shutil.copytree(str(train_runs), str(Path(result_dir) / "train"), dirs_exist_ok=True)

    train_out = Path(runs_dir) / "train"
    for suffix in ["_modules_best.pt", "_modules_last.pt"]:
        src = train_out / f"{variant}{suffix}"
        if src.exists():
            shutil.copy2(str(src), str(Path(result_dir) / "train" / src.name))

    weights_out = Path(result_dir) / "weights"
    weights_out.mkdir(parents=True, exist_ok=True)
    shutil.copy2(yolo_best, weights_out / "yolo_best.pt")
    shutil.copy2(modules_best, weights_out / f"{variant}_modules_best.pt")

    central = Path("vision/result/weight")
    central.mkdir(parents=True, exist_ok=True)
    shutil.copy2(yolo_best, central / f"{variant}_yolo.pt")
    shutil.copy2(modules_best, central / f"{variant}_modules.pt")

    # ---- evaluate ----
    evaluations = evaluate_on_datasets(
        variant=variant,
        yolo_weights=yolo_best,
        module_weights=modules_best,
        eval_roots=args.eval_data_roots,
        out_dir=str(Path(result_dir)),
        imgsz=args.imgsz,
        batch_size=args.batch,
        device_str=args.device,
        workers=args.workers,
    )
    with open(Path(result_dir) / f"{variant}_evaluation.json", "w", encoding="utf-8") as f:
        json.dump(evaluations, f, indent=2)

    print(f"\n[done] Results saved to {result_dir}")


if __name__ == "__main__":
    main()
