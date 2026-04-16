"""
COIN-Pose 2-stage training.

Phase 1: SSL warm-up with YOLO frozen.
         - Warmup LR scheduler
         - Validation monitoring
         - Visibility map direct supervision
         - Single-pass loss (no double weighting)
Phase 2: Task tuning with COIN modules patched into YOLO.
         - Mild geometric augmentation
         - COIN auxiliary loss (TV + anchor regularization)
         - EMA teacher update via callback
"""

import argparse
import json
import math
import random
import shutil
import types
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import yaml
from torch.utils.data import DataLoader, Dataset

from ultralytics import YOLO

# COIN components (from same directory)
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from coin_pose import (
    SpatiallyVariantIlluminationEncoder,
    SpatiallyAwareSoftRouter,
    MultiScaleRSVFiLM,
    COINContextPyramid,
    OcclusionContextRecovery,
    COINPose,
    COINLoss,
    TotalVariationLoss,
    IlluminationAugmentor,
)

# ================================================================
# Constants
# ================================================================

TARGET_LAYERS = [4, 6, 10]
CHANNELS = [128, 128, 256]
NUM_EXPERTS = 3
ILL_CH = 64
WORKERS = 4

# Phase 1: no geometric aug (SSL consistency needs spatial alignment)
AUG_ZERO = dict(
    hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,
    degrees=0.0, translate=0.0, scale=0.0, shear=0.0, perspective=0.0,
    flipud=0.0, fliplr=0.0,
    mosaic=0.0, mixup=0.0, copy_paste=0.0,
)

# Phase 2: mild geometric aug for pose generalization (Issue #5)
AUG_MILD = dict(
    hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,  # COIN handles illumination
    degrees=5.0, translate=0.05, scale=0.15, shear=2.0, perspective=0.0,
    flipud=0.0, fliplr=0.0,
    mosaic=0.0, mixup=0.0, copy_paste=0.0,
)


# ================================================================
# Letterbox
# ================================================================

def letterbox(img: np.ndarray, new_shape: int = 640
              ) -> Tuple[np.ndarray, float, Tuple[int, int]]:
    """Resize + pad to square, preserving aspect ratio."""
    h, w = img.shape[:2]
    r = min(new_shape / h, new_shape / w)
    new_h, new_w = int(round(h * r)), int(round(w * r))
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    dh, dw = new_shape - new_h, new_shape - new_w
    top, left = dh // 2, dw // 2
    img = cv2.copyMakeBorder(img, top, dh - top, left, dw - left,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return img, r, (top, left)


# ================================================================
# Phase 1: Paired SSL Dataset
# ================================================================

class PairedSSLDataset(Dataset):
    """
    Returns (reference, degraded, pair_type) triplets for SSL.

    Two distinct learning objectives:
      'illum':  bright(ref) → dark(deg) — illumination adaptation
      'occlu':  clean(ref) → bright(deg) — occlusion reconstruction

    Ratio ~2:1 (illum oversampled)
    """

    def __init__(
        self,
        root: str,
        split: str = "train",
        imgsz: int = 640,
        max_pairs: Optional[int] = None,
        seed: int = 42,
    ):
        self.imgsz = imgsz
        img_dir = Path(root) / split / "images"

        # Group by base ID
        by_id: Dict[str, Dict[str, Path]] = {}
        for p in sorted(img_dir.glob("*.png")):
            parts = p.stem.rsplit("_", 1)
            if len(parts) == 2:
                base_id, variant = parts
                by_id.setdefault(base_id, {})[variant] = p

        # (reference, degraded, pair_type)
        self.pairs: List[Tuple[Path, Path, str]] = []
        n_illum = n_occlu = 0
        for base_id, variants in by_id.items():
            bright = variants.get("bright")
            clean = variants.get("clean")
            dark = variants.get("dark")

            # Illumination pair: bright(ref) → dark(deg) ×2 oversample
            if bright and dark:
                self.pairs.append((bright, dark, "illum"))
                self.pairs.append((bright, dark, "illum"))
                n_illum += 2
            # Occlusion pair: clean(ref) → bright(deg)
            if clean and bright:
                self.pairs.append((clean, bright, "occlu"))
                n_occlu += 1

        if max_pairs is not None and 0 < max_pairs < len(self.pairs):
            rng = random.Random(seed)
            rng.shuffle(self.pairs)
            self.pairs = self.pairs[:max_pairs]
            n_illum = sum(1 for _, _, pair_type in self.pairs if pair_type == "illum")
            n_occlu = sum(1 for _, _, pair_type in self.pairs if pair_type == "occlu")

        print(f"[PairedSSLDataset] {split}: {len(self.pairs)} pairs "
              f"(illum={n_illum} occlu={n_occlu}) from {len(by_id)} base IDs")

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int):
        ref_path, deg_path, pair_type = self.pairs[idx]
        ref = self._load(ref_path)
        deg = self._load(deg_path)
        vis_gt = self._load_vis_gt(deg_path)
        return ref, deg, pair_type, vis_gt

    def _load(self, path: Path) -> torch.Tensor:
        img = cv2.imread(str(path))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img, _, _ = letterbox(img, self.imgsz)
        img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        return torch.from_numpy(img)

    def _project_kp_to_letterbox(
        self,
        kx: float,
        ky: float,
        orig_w: int,
        orig_h: int,
    ) -> Tuple[float, float]:
        """Map normalized keypoints from the raw image to the letterboxed square image."""
        r = min(self.imgsz / orig_h, self.imgsz / orig_w)
        new_h, new_w = int(round(orig_h * r)), int(round(orig_w * r))
        dh, dw = self.imgsz - new_h, self.imgsz - new_w
        top, left = dh // 2, dw // 2

        x_px = kx * orig_w
        y_px = ky * orig_h
        x_lb = (x_px * r + left) / self.imgsz
        y_lb = (y_px * r + top) / self.imgsz
        return float(min(max(x_lb, 0.0), 1.0)), float(min(max(y_lb, 0.0), 1.0))

    def _load_vis_gt(self, img_path: Path) -> torch.Tensor:
        """Load per-keypoint visibility from YOLO label file.

        Returns (max_obj * N_kp, 3) tensor of [norm_x, norm_y, vis].
        Coordinates are projected into the same letterboxed square frame
        used by `_load`, so supervision aligns with the model input.
        vis: 0=occluded→0.0, 1=partial→0.5, 2=visible→1.0.
        Padded with -1 for missing entries.
        """
        label_path = img_path.parent.parent / "labels" / (img_path.stem + ".txt")
        vis_map = {0: 0.0, 1: 0.5, 2: 1.0}
        kps_all = []
        img = cv2.imread(str(img_path))
        if img is None:
            return torch.full((1, 3), -1.0)
        orig_h, orig_w = img.shape[:2]
        if label_path.exists():
            with open(label_path) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) < 32:  # 5 + 9*3
                        continue
                    for ki in range(9):
                        base = 5 + ki * 3
                        kx = float(parts[base])
                        ky = float(parts[base + 1])
                        kx, ky = self._project_kp_to_letterbox(kx, ky, orig_w, orig_h)
                        v = int(float(parts[base + 2]))
                        kps_all.append([kx, ky, vis_map.get(v, 1.0)])
        if not kps_all:
            return torch.full((1, 3), -1.0)
        return torch.tensor(kps_all, dtype=torch.float32)


# ================================================================
# Phase 1: SSL Training Loop (Issues #2, #3, #4, #6)
# ================================================================

def _run_validation(model, loader, device):
    """Run validation and return average losses dict."""
    model.eval()
    running = {}
    n_steps = 0
    # Keys excluded from core metric (easy-to-minimize regularizers)
    _EXCLUDE_FROM_CORE = {"vis_reg", "vis_illum", "vis_gt", "total"}
    with torch.no_grad():
        for ref_imgs, deg_imgs, pair_types, vis_gts in loader:
            ref_imgs = ref_imgs.to(device)
            deg_imgs = deg_imgs.to(device)
            vis_gt_dev = [v.to(device) for v in vis_gts]

            outputs = model(i_aug=deg_imgs, i_clean=ref_imgs)
            losses = model.criterion(
                f_aug_primes=outputs["F_primes"],
                f_cleans=outputs["F_cleans"],
                z=outputs["Z"],
                i_aug=deg_imgs,
                delta_gammas=outputs["delta_gammas"],
                delta_betas=outputs["delta_betas"],
                pose_logits_aug=outputs.get("pose_logits_aug"),
                pose_logits_clean=outputs.get("pose_logits_clean"),
                visibility_maps=outputs.get("visibility_maps"),
                gate_raws_clean=outputs.get("gate_raws_clean"),
                visibility_maps_clean=outputs.get("visibility_maps_clean"),
                feat_modulated_list=outputs.get("modulated_feats"),
                recon_list=outputs.get("reconstruction_feats"),
                pair_types=pair_types,
                vis_gt_targets=vis_gt_dev,
                router_probs=outputs["P"],
            )
            for k, v in losses.items():
                running[k] = running.get(k, 0.0) + (v.item() if isinstance(v, torch.Tensor) else v)
            n_steps += 1

    avg = {k: v / max(n_steps, 1) for k, v in running.items()}
    avg["core"] = sum(v for k, v in avg.items() if k not in _EXCLUDE_FROM_CORE)
    return avg


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


def _ssl_collate(batch):
    """Custom collate for PairedSSLDataset: variable-length vis_gt tensors."""
    refs, degs, pair_types, vis_gts = zip(*batch)
    return torch.stack(refs), torch.stack(degs), list(pair_types), list(vis_gts)


def train_phase1(
    data_root: str,
    base_weights: str,
    out_dir: str,
    epochs: int = 50,
    batch_size: int = 4,
    lr: float = 1e-3,
    imgsz: int = 640,
    device_str: str = "0",
    max_train_pairs: Optional[int] = None,
    max_val_pairs: Optional[int] = None,
) -> str:
    """Phase 1: SSL warm-up. Returns path to best checkpoint."""
    out = Path(out_dir) / "phase1"
    out.mkdir(parents=True, exist_ok=True)
    device = torch.device(f"cuda:{device_str}" if torch.cuda.is_available() else "cpu")
    print(f"\n{'='*72}\n[Phase 1] SSL warm-up  |  epochs={epochs}  device={device}\n{'='*72}")

    # Load YOLO and wrap with COIN-Pose
    yolo = YOLO(base_weights)
    model = COINPose(yolo, num_experts=NUM_EXPERTS)
    model.set_phase(1)  # freezes YOLO backbone
    model.to(device)
    model.train()

    # Dataset & Loader (train + val)  [Issue #4]
    ds_train = PairedSSLDataset(data_root, "train", imgsz, max_pairs=max_train_pairs)
    ds_val = PairedSSLDataset(data_root, "val", imgsz, max_pairs=max_val_pairs)
    loader_train = DataLoader(
        ds_train, batch_size=batch_size, shuffle=True,
        num_workers=WORKERS, pin_memory=True, drop_last=True,
        collate_fn=_ssl_collate,
    )
    loader_val = DataLoader(
        ds_val, batch_size=batch_size, shuffle=False,
        num_workers=WORKERS, pin_memory=True, drop_last=False,
        collate_fn=_ssl_collate,
    )

    # Optimizer: only COIN params (YOLO is frozen)
    trainable = [p for p in model.parameters() if p.requires_grad]
    print(f"[Phase 1] trainable params: {sum(p.numel() for p in trainable):,}")
    optimizer = torch.optim.AdamW(trainable, lr=lr, weight_decay=1e-4)

    # Warmup + Cosine schedule [Issue #6]
    warmup_epochs = min(5, max(1, epochs // 10))

    def lr_lambda(epoch):
        if epoch < warmup_epochs:
            return (epoch + 1) / warmup_epochs
        progress = (epoch - warmup_epochs) / max(epochs - warmup_epochs, 1)
        return 0.5 * (1.0 + math.cos(math.pi * progress))

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    best_val_loss = float("inf")
    best_path = str(out / "best.pt")
    metrics_rows: List[Dict[str, float]] = []

    run_meta = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "phase": 1,
        "data_root": str(Path(data_root).resolve()),
        "base_weights": base_weights,
        "device": str(device),
        "epochs": epochs,
        "batch_size": batch_size,
        "lr": lr,
        "imgsz": imgsz,
        "warmup_epochs": warmup_epochs,
        "workers": WORKERS,
        "max_train_pairs": max_train_pairs,
        "max_val_pairs": max_val_pairs,
        "n_train_pairs": len(ds_train),
        "n_val_pairs": len(ds_val),
        "trainable_params": int(sum(p.numel() for p in trainable)),
    }
    with open(out / "phase1_run_meta.json", "w", encoding="utf-8") as f:
        json.dump(run_meta, f, indent=2)

    for epoch in range(1, epochs + 1):
        model.train()
        running = {}
        for step, (ref_imgs, deg_imgs, pair_types, vis_gts) in enumerate(loader_train):
            ref_imgs = ref_imgs.to(device)
            deg_imgs = deg_imgs.to(device)
            vis_gt_dev = [v.to(device) for v in vis_gts]

            outputs = model(i_aug=deg_imgs, i_clean=ref_imgs)

            losses = model.criterion(
                f_aug_primes=outputs["F_primes"],
                f_cleans=outputs["F_cleans"],
                z=outputs["Z"],
                i_aug=deg_imgs,
                delta_gammas=outputs["delta_gammas"],
                delta_betas=outputs["delta_betas"],
                pose_logits_aug=outputs.get("pose_logits_aug"),
                pose_logits_clean=outputs.get("pose_logits_clean"),
                visibility_maps=outputs.get("visibility_maps"),
                gate_raws_clean=outputs.get("gate_raws_clean"),
                visibility_maps_clean=outputs.get("visibility_maps_clean"),
                feat_modulated_list=outputs.get("modulated_feats"),
                recon_list=outputs.get("reconstruction_feats"),
                pair_types=pair_types,
                vis_gt_targets=vis_gt_dev,
                router_probs=outputs["P"],
            )

            total = losses["total"]

            optimizer.zero_grad()
            total.backward()
            torch.nn.utils.clip_grad_norm_(trainable, max_norm=2.0)
            optimizer.step()

            for k, v in losses.items():
                running[k] = running.get(k, 0.0) + (v.item() if isinstance(v, torch.Tensor) else v)

            if (step + 1) % 50 == 0:
                n = step + 1
                msg = " | ".join(f"{k}={running[k]/n:.4f}" for k in sorted(running))
                print(f"  [E{epoch:02d} S{n:04d}] {msg}")

        scheduler.step()

        # Epoch train summary
        n_steps = len(loader_train)
        train_loss = running.get("total", 0) / max(n_steps, 1)
        msg = " | ".join(f"{k}={running[k]/n_steps:.4f}" for k in sorted(running))
        print(f"  [E{epoch:02d} train] {msg}  lr={scheduler.get_last_lr()[0]:.6f}")

        # --- Expert & Visibility Monitoring [New] ---
        # 1. Expert Entropy (Expert Collapse Monitoring)
        # 2. Visibility Stats (FiLM vs Restore branch selection)
        with torch.no_grad():
            # Get router weights from the last batch
            last_p = outputs["P"] # (B, num_experts, h, w)
            # Average across spatial and batch: (num_experts,)
            expert_usage = last_p.mean(dim=(0, 2, 3))
            expert_entropy = -(expert_usage * torch.log(expert_usage + 1e-8)).sum().item()
            expert_usage_list = expert_usage.tolist()

            # Get visibility stats scale-by-scale since resolutions differ
            vis_maps = outputs["visibility_maps"] # List of (B, 1, h, w)
            vis_means = [v.mean().item() for v in vis_maps]
            restore_ratios = [(v < 0.5).float().mean().item() for v in vis_maps]
            
            # Global averages for summary
            vis_mean = sum(vis_means) / len(vis_means)
            restore_preference = sum(restore_ratios) / len(restore_ratios)

        print(f"  [E{epoch:02d} monitor] Expert Usage: {[f'{x:.3f}' for x in expert_usage_list]} | Entropy: {expert_entropy:.4f}")
        print(f"  [E{epoch:02d} monitor] Vis Mean: {vis_mean:.4f} | Restore Preference (vis<0.5): {restore_preference*100:.1f}%")

        # Validation [Issue #4]
        val_metrics = _run_validation(model, loader_val, device)
        val_core = val_metrics.get("core", float("inf"))
        msg = " | ".join(f"{k}={v:.4f}" for k, v in sorted(val_metrics.items()))
        print(f"  [E{epoch:02d} val  ] {msg}")

        epoch_metrics = {
            "epoch": epoch, 
            "lr": float(scheduler.get_last_lr()[0]),
            "expert_entropy": expert_entropy,
            "expert_usage": expert_usage_list,
            "vis_mean": vis_mean,
            "restore_ratio": restore_preference
        }
        for k in sorted(running):
            epoch_metrics[f"train_{k}"] = float(running[k] / n_steps)
        for k, v in sorted(val_metrics.items()):
            epoch_metrics[f"val_{k}"] = float(v)
        metrics_rows.append(epoch_metrics)

        # Save checkpoint (best based on val_core — excludes vis_reg)
        ckpt = {
            "epoch": epoch,
            "train_loss": train_loss,
            "val_core": val_core,
            "illumination_encoder": model.illumination_encoder.state_dict(),
            "soft_router": model.soft_router.state_dict(),
            "multi_scale_rsv": model.multi_scale_rsv.state_dict(),
            "context_pyramid": model.context_pyramid.state_dict(),
            "occlusion_recovery": model.occlusion_recovery.state_dict(),
        }
        torch.save(ckpt, str(out / "last.pt"))
        if val_core < best_val_loss:
            best_val_loss = val_core
            torch.save(ckpt, best_path)
            print(f"  >> new best (val_core): {val_core:.4f}")

    print(f"[Phase 1] done. best={best_path}  val_core={best_val_loss:.4f}")
    with open(out / "phase1_metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics_rows, f, indent=2)
    return best_path


# ================================================================
# Monkey-patch: inject COIN modules into YOLO forward
# ================================================================

def save_coin_modules(detection_model: nn.Module, path: str):
    """Save COIN module weights from a patched PoseModel."""
    torch.save({
        "illumination_encoder": detection_model.coin_ie.state_dict(),
        "soft_router": detection_model.coin_sr.state_dict(),
        "multi_scale_rsv": detection_model.coin_rsv.state_dict(),
        "context_pyramid": detection_model.coin_ctx.state_dict(),
        "occlusion_recovery": detection_model.coin_or.state_dict(),
    }, path)


def patch_yolo_with_coin(
    yolo_model,
    coin_ckpt: Optional[str] = None,
    anchor_weight: float = 0.01,
    aux_tv_weight: float = 0.05,
):
    """
    Add COIN modules to YOLO PoseModel and override _predict_once.
    Modules become part of model.parameters() → optimizer picks them up.

    Also overrides the loss method to inject COIN auxiliary losses:
      - L2 anchor regularization toward Phase 1 weights [Issue #8]
      - TV smoothness on FiLM modulation [Issue #7]
    """
    det = yolo_model.model  # PoseModel

    # Create COIN modules
    ie = SpatiallyVariantIlluminationEncoder(3, ILL_CH)
    sr = SpatiallyAwareSoftRouter(ILL_CH, NUM_EXPERTS)
    rsv = MultiScaleRSVFiLM(CHANNELS, ILL_CH, NUM_EXPERTS)
    ctx = COINContextPyramid(CHANNELS, context_channels=128)
    occlu_recovery = OcclusionContextRecovery(CHANNELS)

    if coin_ckpt:
        state = torch.load(coin_ckpt, map_location="cpu")
        ie.load_state_dict(state["illumination_encoder"])
        sr.load_state_dict(state["soft_router"])
        rsv.load_state_dict(state["multi_scale_rsv"])
        if "context_pyramid" in state:
            ctx.load_state_dict(state["context_pyramid"])
        if "occlusion_recovery" in state:
            occlu_recovery.load_state_dict(state["occlusion_recovery"])
        print(f"[patch] loaded COIN modules from {coin_ckpt}")

    # Register as sub-modules (so parameters are discoverable)
    det.add_module("coin_ie", ie)
    det.add_module("coin_sr", sr)
    det.add_module("coin_rsv", rsv)
    det.add_module("coin_ctx", ctx)
    det.add_module("coin_or", occlu_recovery)

    # Store anchor weights for L2 regularization [Issue #8]
    # Anchors live on the same device as the params (no CPU↔GPU copies)
    anchor_params = []
    coin_params_ref = []
    for name in ["coin_ie", "coin_sr", "coin_rsv", "coin_ctx", "coin_or"]:
        for p in getattr(det, name).parameters():
            anchor_params.append(p.data.clone())  # same device as p
            coin_params_ref.append(p)
    det._coin_anchor_params = anchor_params
    det._coin_params_ref = coin_params_ref
    det._coin_cache = None

    # Override _predict_once to include COIN modulation + cache intermediates
    def _modulated_predict_once(self, x, profile=False, visualize=False, embed=None):
        x_input = x
        Z = self.coin_ie(x_input)
        P = self.coin_sr(Z)

        y, dt = [], []
        target_cache = {}
        delta_gammas, delta_betas = [], []

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
                        feat_m, dg, db, _ = self.coin_rsv.forward_single(feat, idx, Z, P)
                        modulated.append(feat_m)
                        delta_gammas.append(dg)
                        delta_betas.append(db)

                    shared = self.coin_ctx(modulated)
                    refined = []
                    for idx, (feat_m, feat_ctx) in enumerate(zip(modulated, shared)):
                        feat_out, _, _ = self.coin_or.forward_single(
                            feat=feat_m,
                            context_feat=feat_ctx,
                            scale_idx=idx,
                            reconstruct=True,
                        )
                        refined.append(feat_out)

                    for layer_idx, feat_new in zip(TARGET_LAYERS, refined):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new

                    x = target_cache[m.i]

            y.append(x if m.i in self.save else None)

        # Cache for auxiliary loss [Issue #7]
        self._coin_cache = {
            "delta_gammas": delta_gammas,
            "delta_betas": delta_betas,
        }
        return x

    det._predict_once = types.MethodType(_modulated_predict_once, det)
    print(f"[patch] YOLO _predict_once overridden with COIN modulation")

    # Override loss method to inject COIN auxiliary losses [Issues #7, #8]
    _OrigLossMethod = type(det).loss
    _tv = TotalVariationLoss()
    _anchor_w = anchor_weight
    _aux_tv_w = aux_tv_weight

    def _augmented_loss(self, batch, preds=None):
        loss, loss_items = _OrigLossMethod(self, batch, preds)

        if not self.training:
            return loss, loss_items

        # Anchor regularization: prevent COIN drift from Phase 1 [Issue #8]
        if hasattr(self, "_coin_anchor_params") and self._coin_anchor_params:
            anchor_loss = sum(
                F.mse_loss(p, a)
                for p, a in zip(self._coin_params_ref, self._coin_anchor_params)
            ) / len(self._coin_anchor_params)
            loss = loss + _anchor_w * anchor_loss

        # TV smoothness on FiLM modulation [Issue #7]
        if hasattr(self, "_coin_cache") and self._coin_cache is not None:
            cache = self._coin_cache
            dgs = cache.get("delta_gammas", [])
            dbs = cache.get("delta_betas", [])
            if dgs and dbs:
                tv_val = sum(
                    _tv(dg) + _tv(db) for dg, db in zip(dgs, dbs)
                ) / max(len(dgs), 1)
                loss = loss + _aux_tv_w * tv_val
            self._coin_cache = None

        return loss, loss_items

    det.loss = types.MethodType(_augmented_loss, det)
    print(f"[patch] loss method augmented (anchor_w={_anchor_w}, tv_w={_aux_tv_w})")

    return yolo_model


# ================================================================
# Phase 2: ultralytics fine-tuning with patched model
# ================================================================

def write_pose_yaml(data_root: str, out_path: str) -> str:
    """Write a corrected YOLO pose data.yaml for any dataset root."""
    data = {
        "path": str(Path(data_root).resolve()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "kpt_shape": [9, 3],
        "flip_idx": list(range(9)),
        "names": {
            0: "CenterHole",
            1: "CenterHole_B",
            2: "Hole",
            3: "Hole_B",
        },
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"[data.yaml] written: {out_path}")
    return out_path


def evaluate_coin_on_datasets(
    yolo_weights: str,
    coin_weights: str,
    eval_roots: List[str],
    out_dir: str,
    imgsz: int,
    batch_size: int,
    device_str: str,
) -> List[dict]:
    eval_dir = Path(out_dir) / "eval"
    eval_dir.mkdir(parents=True, exist_ok=True)
    evaluations: List[dict] = []

    for dataset_root in eval_roots:
        dataset_root_path = Path(dataset_root)
        data_yaml = write_pose_yaml(str(dataset_root_path), str(eval_dir / f"{dataset_root_path.name}.yaml"))
        model = YOLO(yolo_weights)
        patch_yolo_with_coin(model, coin_ckpt=coin_weights, anchor_weight=0.0, aux_tv_weight=0.0)
        results = model.val(
            data=data_yaml,
            split="test",
            imgsz=imgsz,
            batch=batch_size,
            device=device_str,
            workers=WORKERS,
            project=str(Path(eval_dir).resolve()),
            name=dataset_root_path.name,
            exist_ok=True,
            plots=False,
            save_json=False,
            rect=True,
        )
        evaluations.append(
            {
                "dataset_name": dataset_root_path.name,
                "dataset_root": str(dataset_root_path.resolve()),
                "results_dict": _to_builtin(getattr(results, "results_dict", {})),
                "speed_ms": _to_builtin(getattr(results, "speed", {})),
                "save_dir": str(Path(results.save_dir).resolve()),
            }
        )
    return evaluations


class COINSaveCallback:
    """Save COIN module weights alongside ultralytics checkpoints."""
    def __init__(self, detection_model: nn.Module, save_dir: str):
        self.det = detection_model
        self.save_dir = Path(save_dir)

    def __call__(self, trainer):
        save_coin_modules(self.det, str(self.save_dir / "coin_modules_last.pt"))
        if trainer.best_fitness == trainer.fitness:
            save_coin_modules(self.det, str(self.save_dir / "coin_modules_best.pt"))


class EMATeacherCallback:
    """EMA update of anchor weights for COIN modules [Issue #8]."""
    def __init__(self, detection_model: nn.Module, momentum: float = 0.999):
        self.det = detection_model
        self.momentum = momentum

    def __call__(self, trainer):
        if not hasattr(self.det, "_coin_anchor_params"):
            return
        mu = self.momentum
        for anchor, param in zip(self.det._coin_anchor_params, self.det._coin_params_ref):
            anchor.data.mul_(mu).add_(param.data, alpha=1.0 - mu)


def train_phase2(
    data_root: str,
    base_weights: str,
    p1_ckpt: str,
    out_dir: str,
    epochs: int = 80,
    warmup_epochs: int = 5,
    batch_size: int = 8,
    lr: float = 1e-4,
    imgsz: int = 640,
    device_str: str = "0",
    patience: int = 15,
    anchor_weight: float = 0.01,
    aux_tv_weight: float = 0.05,
) -> Tuple[str, str]:
    """Phase 2: ultralytics training with COIN modulation.

    Split into two stages:
      2a (warmup): backbone frozen, only COIN modules train at LR for warmup_epochs
      2b (full):   all params unfrozen, lower LR, cosine schedule
    """
    out = Path(out_dir) / "phase2"
    out.mkdir(parents=True, exist_ok=True)

    # Write corrected data.yaml
    data_yaml = write_pose_yaml(data_root, str(out / "data.yaml"))

    # Count YOLO backbone layers to determine freeze range
    probe = YOLO(base_weights)
    n_yolo_layers = len(list(probe.model.model))
    del probe

    run_meta = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "phase": 2,
        "train_data_root": str(Path(data_root).resolve()),
        "base_weights": str(Path(base_weights).resolve()),
        "phase1_checkpoint": str(Path(p1_ckpt).resolve()),
        "epochs": epochs,
        "warmup_epochs": warmup_epochs,
        "batch_size": batch_size,
        "lr": lr,
        "imgsz": imgsz,
        "device": device_str,
        "workers": WORKERS,
        "patience": patience,
        "anchor_weight": anchor_weight,
        "aux_tv_weight": aux_tv_weight,
    }
    with open(out / "phase2_run_meta.json", "w", encoding="utf-8") as f:
        json.dump(run_meta, f, indent=2)

    # ── Phase 2a: Backbone frozen warmup ──
    if warmup_epochs > 0:
        print(f"\n{'='*72}")
        print(f"[Phase 2a] Backbone-frozen warmup  |  {warmup_epochs} epochs  device={device_str}")
        print(f"           Freeze YOLO layers 0..{n_yolo_layers-1}, train COIN only")
        print(f"{'='*72}")

        model_2a = YOLO(base_weights)
        patch_yolo_with_coin(model_2a, coin_ckpt=p1_ckpt,
                             anchor_weight=anchor_weight, aux_tv_weight=aux_tv_weight)

        coin_saver_2a = COINSaveCallback(model_2a.model, str(out))
        model_2a.add_callback("on_fit_epoch_end", coin_saver_2a)

        model_2a.train(
            data=data_yaml,
            epochs=warmup_epochs,
            imgsz=imgsz,
            batch=batch_size,
            device=device_str,
            workers=WORKERS,
            optimizer="AdamW",
            lr0=lr * 3,  # higher LR for COIN-only warmup
            weight_decay=5e-4,
            cos_lr=False,
            warmup_epochs=1,
            close_mosaic=0,
            patience=0,  # no early stopping in warmup
            freeze=list(range(n_yolo_layers)),  # freeze all YOLO layers
            project=str(out.resolve()),
            name="warmup",
            exist_ok=True,
            rect=True,
            **AUG_MILD,
        )
        # Use warmup result as starting point for 2b
        warmup_yolo = out / "warmup" / "weights" / "best.pt"
        if not warmup_yolo.exists():
            warmup_yolo = out / "warmup" / "weights" / "last.pt"
        warmup_coin = out / "coin_modules_best.pt"
        if not warmup_coin.exists():
            warmup_coin = out / "coin_modules_last.pt"
        print(f"[Phase 2a] done. YOLO: {warmup_yolo}, COIN: {warmup_coin}")
    else:
        warmup_yolo = base_weights
        warmup_coin = p1_ckpt

    # ── Phase 2b: Full training (all params) ──
    remaining_epochs = epochs - warmup_epochs
    print(f"\n{'='*72}")
    print(f"[Phase 2b] Full training  |  {remaining_epochs} epochs  device={device_str}")
    print(f"           All params unfrozen, LR={lr}")
    print(f"{'='*72}")

    model = YOLO(str(warmup_yolo))
    patch_yolo_with_coin(model, coin_ckpt=str(warmup_coin),
                         anchor_weight=anchor_weight, aux_tv_weight=aux_tv_weight)

    coin_saver = COINSaveCallback(model.model, str(out))
    ema_updater = EMATeacherCallback(model.model, momentum=0.999)
    model.add_callback("on_fit_epoch_end", coin_saver)
    model.add_callback("on_train_batch_end", ema_updater)

    results = model.train(
        data=data_yaml,
        epochs=remaining_epochs,
        imgsz=imgsz,
        batch=batch_size,
        device=device_str,
        workers=WORKERS,
        optimizer="AdamW",
        lr0=lr,
        weight_decay=5e-4,
        cos_lr=True,
        warmup_epochs=min(3, remaining_epochs // 10),
        close_mosaic=0,
        patience=patience,
        project=str(out.resolve()),
        name="train",
        exist_ok=True,
        rect=True,
        **AUG_MILD,
    )

    best_yolo = Path(results.save_dir) / "weights" / "best.pt"
    best_coin = out / "coin_modules_best.pt"
    print(f"[Phase 2] done.")
    print(f"  YOLO weights: {best_yolo}")
    print(f"  COIN modules: {best_coin}")
    return str(best_yolo), str(best_coin), str(results.save_dir)


# ================================================================
# Main
# ================================================================

def main():
    ap = argparse.ArgumentParser(description="COIN-Pose 2-stage training")
    ap.add_argument("--phase", choices=["1", "2", "12"], default="12",
                    help="Which phase(s) to run")
    ap.add_argument("--data-root", type=str, default=None,
                    help="Legacy alias: use the same dataset for phase1 and phase2")
    ap.add_argument("--phase1-data-root", type=str,
                    default="vision/dataset/yolo_pose_distill_dataset",
                    help="Path to distill dataset for Phase 1")
    ap.add_argument("--phase2-data-root", type=str,
                    default="vision/dataset/yolo_pose_baseline_dataset",
                    help="Path to baseline dataset for Phase 2")
    ap.add_argument("--eval-data-roots", type=str, nargs="*",
                    default=[
                        "vision/dataset/yolo_pose_baseline_dataset",
                        "vision/dataset/yolo_pose_distill_dataset",
                    ],
                    help="Datasets evaluated on split=test after training")
    ap.add_argument("--base-weights", type=str,
                    default="vision/yolo/weights/yolo26n-pose.pt",
                    help="Base YOLO26n-pose weights")
    ap.add_argument("--p1-ckpt", type=str, default=None,
                    help="Phase 1 checkpoint (for --phase 2)")
    ap.add_argument("--runs-dir", type=str, default="vision/yolo/runs/coin",
                    help="Temp dir for training artifacts")
    ap.add_argument("--result-dir", type=str, default="vision/result/coin",
                    help="Final output dir (weights, eval, meta)")
    ap.add_argument("--epochs-p1", type=int, default=50)
    ap.add_argument("--epochs-p2", type=int, default=80)
    ap.add_argument("--warmup-p2", type=int, default=5,
                    help="Phase 2 backbone-frozen warmup epochs")
    ap.add_argument("--batch-p1", type=int, default=4)
    ap.add_argument("--batch-p2", type=int, default=8)
    ap.add_argument("--max-train-pairs-p1", type=int, default=None,
                    help="Optional cap on Phase 1 train pairs for pilot runs")
    ap.add_argument("--max-val-pairs-p1", type=int, default=None,
                    help="Optional cap on Phase 1 val pairs for pilot runs")
    ap.add_argument("--lr-p1", type=float, default=1e-3)
    ap.add_argument("--lr-p2", type=float, default=1e-4)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--anchor-weight", type=float, default=0.01,
                    help="L2 anchor regularization weight for Phase 2")
    ap.add_argument("--aux-tv-weight", type=float, default=0.05,
                    help="TV smoothness auxiliary weight for Phase 2")
    args = ap.parse_args()

    if args.data_root:
        args.phase1_data_root = args.data_root
        args.phase2_data_root = args.data_root

    for root in [args.phase1_data_root, args.phase2_data_root, *args.eval_data_roots]:
        if not Path(root).exists():
            raise FileNotFoundError(f"Dataset root not found: {root}")

    p1_ckpt = args.p1_ckpt

    runs_dir = args.runs_dir
    result_dir = args.result_dir
    Path(runs_dir).mkdir(parents=True, exist_ok=True)
    Path(result_dir).mkdir(parents=True, exist_ok=True)

    if args.phase in ("1", "12"):
        p1_ckpt = train_phase1(
            data_root=args.phase1_data_root,
            base_weights=args.base_weights,
            out_dir=runs_dir,
            epochs=args.epochs_p1,
            batch_size=args.batch_p1,
            lr=args.lr_p1,
            imgsz=args.imgsz,
            device_str=args.device,
            max_train_pairs=args.max_train_pairs_p1,
            max_val_pairs=args.max_val_pairs_p1,
        )

    if args.phase in ("2", "12"):
        if p1_ckpt is None:
            raise ValueError("Phase 2 requires --p1-ckpt (Phase 1 checkpoint)")
        yolo_best, coin_best, p2_save_dir = train_phase2(
            data_root=args.phase2_data_root,
            base_weights=args.base_weights,
            p1_ckpt=p1_ckpt,
            out_dir=runs_dir,
            epochs=args.epochs_p2,
            warmup_epochs=args.warmup_p2,
            batch_size=args.batch_p2,
            lr=args.lr_p2,
            imgsz=args.imgsz,
            device_str=args.device,
            patience=args.patience,
            anchor_weight=args.anchor_weight,
            aux_tv_weight=args.aux_tv_weight,
        )
        # Copy training artifacts to result dir
        p1_runs = Path(runs_dir) / "phase1"
        p2_runs = Path(p2_save_dir)
        if p1_runs.exists():
            shutil.copytree(str(p1_runs), str(Path(result_dir) / "phase1"), dirs_exist_ok=True)
        shutil.copytree(str(p2_runs), str(Path(result_dir) / "phase2"), dirs_exist_ok=True)
        # Also copy coin modules into result phase2
        coin_src = Path(runs_dir) / "phase2" / "coin_modules_best.pt"
        coin_last = Path(runs_dir) / "phase2" / "coin_modules_last.pt"
        for f in [coin_src, coin_last]:
            if f.exists():
                shutil.copy2(str(f), str(Path(result_dir) / "phase2" / f.name))

        weights_out = Path(result_dir) / "weights"
        weights_out.mkdir(parents=True, exist_ok=True)
        shutil.copy2(yolo_best, weights_out / "yolo_best.pt")
        shutil.copy2(coin_best, weights_out / "coin_modules_best.pt")

        # Also save to central weight directory for easy access
        central_weights = Path("vision/result/weight")
        central_weights.mkdir(parents=True, exist_ok=True)
        shutil.copy2(yolo_best, central_weights / "coin_yolo.pt")
        shutil.copy2(coin_best, central_weights / "coin_modules.pt")

        eval_out = Path(result_dir) / "eval"
        evaluations = evaluate_coin_on_datasets(
            yolo_weights=yolo_best,
            coin_weights=coin_best,
            eval_roots=args.eval_data_roots,
            out_dir=str(eval_out),
            imgsz=args.imgsz,
            batch_size=args.batch_p2,
            device_str=args.device,
        )
        with open(Path(result_dir) / "coin_evaluation.json", "w", encoding="utf-8") as f:
            json.dump(evaluations, f, indent=2)
        print(f"\n[DONE] Final model:")
        print(f"  YOLO:  {weights_out / 'yolo_best.pt'}")
        print(f"  COIN:  {weights_out / 'coin_modules_best.pt'}")
        print(f"  Central: {central_weights.resolve()}")


if __name__ == "__main__":
    main()
