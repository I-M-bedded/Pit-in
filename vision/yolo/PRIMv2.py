"""
PRIM v2 core modules.

Extends PRIM with:
1) Laplacian-pyramid frequency + spatial illumination encoder (AMP-safe),
   using full RGB channels to preserve chroma information,
2) shared-stem restoration with multiplicative + additive (gamma/beta)
   design — additive path is NOT bounded by feature magnitude, enabling
   restoration even when backbone features collapse in severe low-light,
3) extended loss with restore-branch regularization.

All reusable components (router, FiLM, pyramid, base losses, utilities)
are imported from PRIM_pose — no duplication.
"""

import copy
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

from PRIM_pose import (
    # Utilities
    _num_groups,
    _small_init_module,
    # Shared modules
    SpatiallyAwareSoftRouter,
    MultiScaleRSVFiLM,
    PRIMFeaturePyramid,
    # Feature inference (generic — works for both PRIM and v2)
    infer_backbone_layer_shapes,
    infer_prim_feature_spec,
    infer_checkpoint_channels,
    # Loss base & components
    PRIMLoss,
    TotalVariationLoss,
)

# Backward-compatible alias
infer_primv2_feature_spec = infer_prim_feature_spec


# ---------------------------------------------------------------------------
# Illumination encoder — Laplacian pyramid + spatial (AMP-safe, RGB)
# ---------------------------------------------------------------------------

class FrequencySpatialIlluminationEncoder(nn.Module):
    """
    AMP-safe illumination encoder fusing spatial and frequency-domain cues.

    Spatial branch
        3-stage conv stem (RGB → stride-8 local illumination features).

    Frequency branch
        **Per-channel** Laplacian pyramid decomposes the full RGB input
        into multi-scale frequency bands using only ``avg_pool2d`` +
        ``bilinear interpolate`` — no ``torch.fft``, fully float16-
        compatible under ``torch.cuda.amp.autocast``.

        Operating on RGB (not grayscale) preserves chroma information
        at every frequency scale, which is important for detecting
        color cast and chroma noise in dark / medium-light scenes.

        Each pyramid level captures a distinct frequency band:
          * Fine levels  — high-freq edges, shadow boundaries, chroma noise
          * Coarse level — low-freq global illumination + color temperature

    Fusion
        Concatenated features are projected to ``out_channels`` and
        re-weighted by squeeze-excitation (SE) channel attention so
        that the model adaptively emphasises spatial vs. frequency
        contributions depending on the illumination condition.

    References
        Burt & Adelson (1983)      — Laplacian pyramid
        RetinexFormer (ICCV 2023)  — illumination-aware feature modulation
        FourLLIE (CVPR 2023)       — frequency-based low-light enhancement
        Hu et al. (CVPR 2018)      — squeeze-excitation channel attention
    """

    def __init__(
        self,
        in_channels: int = 3,
        out_channels: int = 64,
        spatial_channels: int = 48,
        num_freq_levels: int = 3,
        freq_channels_per_level: int = 8,
    ):
        super().__init__()
        self.num_freq_levels = num_freq_levels
        self.pyramid_in_channels = in_channels
        freq_total = freq_channels_per_level * num_freq_levels

        # ---- spatial branch (stride-8) ----
        self.spatial_encoder = nn.Sequential(
            nn.Conv2d(in_channels, 24, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(24),
            nn.SiLU(inplace=True),
            nn.Conv2d(24, 32, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.SiLU(inplace=True),
            nn.Conv2d(32, spatial_channels, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(spatial_channels),
            nn.SiLU(inplace=True),
        )

        # ---- frequency branch: per-level conv on RGB Laplacian bands ----
        self.level_encoders = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(in_channels, freq_channels_per_level, 3, padding=1, bias=False),
                    nn.BatchNorm2d(freq_channels_per_level),
                    nn.SiLU(inplace=True),
                )
                for _ in range(num_freq_levels)
            ]
        )

        # ---- fusion: 1×1 proj + SE channel attention ----
        fused_ch = spatial_channels + freq_total
        self.fuse = nn.Sequential(
            nn.Conv2d(fused_ch, out_channels, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
        )
        se_mid = max(out_channels // 4, 8)
        self.se_pool = nn.AdaptiveAvgPool2d(1)
        self.se_fc = nn.Sequential(
            nn.Conv2d(out_channels, se_mid, 1, bias=False),
            nn.SiLU(inplace=True),
            nn.Conv2d(se_mid, out_channels, 1, bias=False),
            nn.Sigmoid(),
        )

    # ---- Per-channel Laplacian pyramid (AMP-safe, no torch.fft) ----

    def _laplacian_pyramid(
        self,
        x: torch.Tensor,
        target_size: Tuple[int, int],
    ) -> List[torch.Tensor]:
        """
        Build a per-channel Laplacian pyramid from RGB input.

        Returns ``num_freq_levels`` bands, all resized to ``target_size``:
          * bands[0..N-2]: high-freq detail at progressively coarser scales
          * bands[N-1]:    low-freq residual (global illumination + color)
        """
        bands: List[torch.Tensor] = []
        current = x  # (B, C, H, W) — full RGB
        for _ in range(self.num_freq_levels - 1):
            h, w = current.shape[-2:]
            if h < 4 or w < 4:
                bands.append(
                    torch.zeros(
                        x.shape[0], x.shape[1], *target_size,
                        device=x.device, dtype=x.dtype,
                    )
                )
                continue
            low = F.avg_pool2d(current, kernel_size=2, stride=2)
            low_up = F.interpolate(
                low, size=(h, w), mode="bilinear", align_corners=False,
            )
            high = current - low_up
            bands.append(
                F.interpolate(
                    high, size=target_size, mode="bilinear", align_corners=False,
                )
            )
            current = low
        # final band: low-frequency residual
        bands.append(
            F.interpolate(
                current, size=target_size, mode="bilinear", align_corners=False,
            )
        )
        return bands

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        spatial = self.spatial_encoder(x)
        target_size = spatial.shape[-2:]

        bands = self._laplacian_pyramid(x, target_size)
        freq_feats = [enc(band) for enc, band in zip(self.level_encoders, bands)]

        fused = self.fuse(torch.cat([spatial] + freq_feats, dim=1))
        se_weight = self.se_fc(self.se_pool(fused))
        return fused * se_weight


# ---------------------------------------------------------------------------
# Restoration branch — multiplicative + additive (gamma / beta)
# ---------------------------------------------------------------------------

class SharedRestoreResidualBlock(nn.Module):
    def __init__(self, channels: int):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(channels, channels, 3, padding=1, groups=channels, bias=False),
            nn.GroupNorm(_num_groups(channels), channels),
            nn.SiLU(inplace=True),
            nn.Conv2d(channels, channels, 1, bias=False),
            nn.GroupNorm(_num_groups(channels), channels),
        )
        self.act = nn.SiLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(x + self.block(x))


class MultiScaleSharedRestore(nn.Module):
    """
    Shared-stem restoration branch with multiplicative + additive design.

    Each scale projects into a common stem space, runs the same residual
    stem, then predicts:

      * **gamma** (multiplicative): ``tanh``-bounded, scales existing features
      * **beta** (additive): unbounded, injects new signal

    The additive path is critical: in severe low-light where backbone
    features collapse to near-zero, ``tanh(·) * feat_scale ≈ 0`` in the
    old design.  The additive beta can inject meaningful corrections
    regardless of feature magnitude.

    Both paths are gated by a small illumination-conditioned gate
    (initialised nearly closed) so that clean / easy images pass through
    with minimal perturbation.
    """

    def __init__(
        self,
        channels_list: List[int],
        illumination_channels: int = 64,
        stem_channels: int = 64,
        num_shared_blocks: int = 2,
        gate_init_bias: float = -2.5,
    ):
        super().__init__()
        self.in_proj = nn.ModuleList()
        self.cond_proj = nn.ModuleList()
        self.gamma_proj = nn.ModuleList()
        self.beta_proj = nn.ModuleList()
        self.gate_list = nn.ModuleList()
        self.num_scales = len(channels_list)

        for channels in channels_list:
            self.in_proj.append(
                nn.Sequential(
                    nn.Conv2d(channels, stem_channels, 1, bias=False),
                    nn.GroupNorm(_num_groups(stem_channels), stem_channels),
                    nn.SiLU(inplace=True),
                )
            )
            cond = nn.Conv2d(illumination_channels, stem_channels, 1)
            _small_init_module(cond, std=1e-3)
            self.cond_proj.append(cond)

            gamma_head = nn.Conv2d(stem_channels, channels, 1)
            _small_init_module(gamma_head, std=1e-3)
            self.gamma_proj.append(gamma_head)

            beta_head = nn.Conv2d(stem_channels, channels, 1)
            _small_init_module(beta_head, std=1e-3)
            self.beta_proj.append(beta_head)

            gate_conv = nn.Conv2d(illumination_channels, 1, 1)
            nn.init.zeros_(gate_conv.weight)
            nn.init.constant_(gate_conv.bias, gate_init_bias)
            self.gate_list.append(gate_conv)

        self.shared_stem = nn.Sequential(
            *[SharedRestoreResidualBlock(stem_channels) for _ in range(num_shared_blocks)]
        )

    def forward_single(
        self,
        feat: torch.Tensor,
        scale_idx: int,
        z: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Returns
        -------
        restored : torch.Tensor
            ``feat * (1 + gate * gamma) + gate * beta``
        gamma : torch.Tensor
            Multiplicative correction (tanh-bounded).
        beta : torch.Tensor
            Additive correction (not bounded by feature magnitude).
        gate_raw : torch.Tensor
            Pre-sigmoid gate logit (for sparsity regularisation).
        """
        h, w = feat.shape[-2:]
        z_up = F.interpolate(z, size=(h, w), mode="bilinear", align_corners=False)

        stem = self.in_proj[scale_idx](feat) + self.cond_proj[scale_idx](z_up)
        stem = self.shared_stem(stem)

        gamma = torch.tanh(self.gamma_proj[scale_idx](stem))
        beta = self.beta_proj[scale_idx](stem)

        gate_raw = self.gate_list[scale_idx](z_up)
        gate = torch.sigmoid(gate_raw)

        restored = feat * (1.0 + gate * gamma) + gate * beta
        return restored, gamma, beta, gate_raw


# ---------------------------------------------------------------------------
# Loss
# ---------------------------------------------------------------------------

class PRIMv2Loss(PRIMLoss):
    """
    Extended loss for PRIMv2 — inherits all PRIM loss terms and adds
    restore-branch regularization.

    Extra terms:
      * ``restore_tv``       — total variation on restore gamma and beta;
                                 encourages spatially smooth corrections
      * ``restore_sparsity`` — softplus penalty on restore gate logits;
                                 encourages gates to stay closed unless the
                                 illumination truly requires restoration
    """

    PHASE_WEIGHTS: Dict[int, Dict[str, float]] = {
        1: dict(
            kd=0.3, ssl=1.0, hist=1.0, smooth=0.3, tv=0.05, identity=0.2,
            expert_entropy=0.05, load_balance=0.02,
            restore_tv=0.05, restore_sparsity=0.1,
        ),
        2: dict(
            kd=1.0, ssl=0.5, hist=0.3, smooth=0.1, tv=0.05, identity=0.1,
            expert_entropy=0.02, load_balance=0.01,
            restore_tv=0.03, restore_sparsity=0.05,
        ),
    }

    def __init__(self, phase: int = 1):
        super().__init__(phase)
        self.restore_tv_loss = TotalVariationLoss()

    def set_phase(self, phase: int):
        super().set_phase(phase)
        weights = self.PHASE_WEIGHTS.get(phase, self.PHASE_WEIGHTS[2])
        self.lambda_restore_tv = weights.get("restore_tv", 0.05)
        self.lambda_restore_sparsity = weights.get("restore_sparsity", 0.1)

    def forward(
        self,
        *,
        restore_gammas: Optional[List[torch.Tensor]] = None,
        restore_betas: Optional[List[torch.Tensor]] = None,
        restore_gate_raws: Optional[List[torch.Tensor]] = None,
        **kwargs,
    ) -> Dict[str, torch.Tensor]:
        losses = super().forward(**kwargs)

        ref = losses["total"]
        zero = torch.zeros((), device=ref.device, dtype=ref.dtype)

        if restore_gammas and restore_betas and self.lambda_restore_tv > 0:
            losses["restore_tv"] = self.lambda_restore_tv * sum(
                self.restore_tv_loss(g) + self.restore_tv_loss(b)
                for g, b in zip(restore_gammas, restore_betas)
            ) / len(restore_gammas)
        else:
            losses["restore_tv"] = zero

        if restore_gate_raws and self.lambda_restore_sparsity > 0:
            losses["restore_sparsity"] = self.lambda_restore_sparsity * sum(
                F.softplus(g).mean() for g in restore_gate_raws
            ) / len(restore_gate_raws)
        else:
            losses["restore_sparsity"] = zero

        losses["total"] = sum(v for k, v in losses.items() if k != "total")
        return losses


# ---------------------------------------------------------------------------
# PRIMv2Pose  (YOLO wrapper)
# ---------------------------------------------------------------------------

class PRIMv2Pose(nn.Module):
    """YOLO26 pose wrapper with restore residuals, FiLM, and pyramid fusion."""

    def __init__(
        self,
        yolo_model,
        num_experts: int = 3,
        feature_source: str = "backbone",
        feature_layers=None,
    ):
        super().__init__()
        self.yolo_model = yolo_model.model.model
        object.__setattr__(self, "_yolo_detection_model", yolo_model.model)
        self.teacher_backbone = None
        self.feature_spec = infer_primv2_feature_spec(
            self.yolo_model,
            feature_source=feature_source,
            feature_layers=feature_layers,
        )
        self.target_layers = list(self.feature_spec["target_layers"])
        self.channels_list = list(self.feature_spec["channels"])
        self.feature_source = str(self.feature_spec["feature_source"])

        illumination_channels = 64
        self.illumination_encoder = FrequencySpatialIlluminationEncoder(
            3, illumination_channels,
        )
        self.soft_router = SpatiallyAwareSoftRouter(
            illumination_channels, num_experts,
        )
        self.restore_branch = MultiScaleSharedRestore(
            self.channels_list, illumination_channels=illumination_channels,
        )
        self.multi_scale_rsv = MultiScaleRSVFiLM(
            self.channels_list, illumination_channels, num_experts,
        )
        self.feature_pyramid = PRIMFeaturePyramid(
            self.channels_list, context_channels=128,
        )
        self.criterion = PRIMv2Loss(phase=1)

    def export_meta(self) -> Dict[str, object]:
        return {
            "feature_source": self.feature_source,
            "target_layers": list(self.target_layers),
            "channels": list(self.channels_list),
            "resolutions": list(self.feature_spec.get("resolutions", [])),
        }

    def _freeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = False

    def _unfreeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = True

    def _init_frozen_teacher(self):
        self.teacher_backbone = copy.deepcopy(self.yolo_model)
        for param in self.teacher_backbone.parameters():
            param.requires_grad = False
        self.teacher_backbone.eval()

    def set_phase(self, phase: int):
        self.criterion.set_phase(phase)
        if phase == 1:
            self._freeze_yolo()
        else:
            self._unfreeze_yolo()
            self._init_frozen_teacher()

    def train(self, mode: bool = True):
        super().train(mode)
        if self.teacher_backbone is not None:
            self.teacher_backbone.eval()
        return self

    @torch.no_grad()
    def update_teacher_ema(self, momentum: float = 0.999):
        if self.teacher_backbone is None:
            return
        for tp, sp in zip(
            self.teacher_backbone.parameters(), self.yolo_model.parameters()
        ):
            tp.data.mul_(momentum).add_(sp.data, alpha=1.0 - momentum)

    # ---- internal pipeline --------------------------------------------------

    def _apply_primv2_modules(
        self,
        target_feats: List[torch.Tensor],
        modulate: bool,
        z: Optional[torch.Tensor] = None,
        p: Optional[torch.Tensor] = None,
    ):
        if not modulate:
            return target_feats, [], [], [], [], [], []

        if z is None or p is None:
            raise ValueError("PRIMv2 modulation requires z and p.")

        # restore (gamma + beta)
        restored_feats: List[torch.Tensor] = []
        restore_gammas: List[torch.Tensor] = []
        restore_betas: List[torch.Tensor] = []
        restore_gate_raws: List[torch.Tensor] = []
        for idx, feat in enumerate(target_feats):
            feat_r, r_gamma, r_beta, rg_raw = self.restore_branch.forward_single(
                feat, idx, z,
            )
            restored_feats.append(feat_r)
            restore_gammas.append(r_gamma)
            restore_betas.append(r_beta)
            restore_gate_raws.append(rg_raw)

        # FiLM
        modulated_feats: List[torch.Tensor] = []
        delta_gammas: List[torch.Tensor] = []
        delta_betas: List[torch.Tensor] = []
        film_gate_raws: List[torch.Tensor] = []
        for idx, feat in enumerate(restored_feats):
            feat_m, dg, db, fg_raw = self.multi_scale_rsv.forward_single(
                feat, idx, z, p,
            )
            modulated_feats.append(feat_m)
            delta_gammas.append(dg)
            delta_betas.append(db)
            film_gate_raws.append(fg_raw)

        fused_feats = self.feature_pyramid(modulated_feats)
        return (
            fused_feats,
            restore_gammas, restore_betas, restore_gate_raws,
            delta_gammas, delta_betas, film_gate_raws,
        )

    def _run_backbone(self, x, modulate=False, z=None, p=None, backbone=None):
        if backbone is None:
            backbone = self.yolo_model

        y: List = []
        f_targets: List[torch.Tensor] = []
        restore_gammas: List[torch.Tensor] = []
        restore_betas: List[torch.Tensor] = []
        restore_gate_raws: List[torch.Tensor] = []
        delta_gammas: List[torch.Tensor] = []
        delta_betas: List[torch.Tensor] = []
        film_gate_raws: List[torch.Tensor] = []
        pose_logits = None
        target_cache: Dict[int, torch.Tensor] = {}

        for idx, layer in enumerate(backbone):
            if layer.f != -1:
                if isinstance(layer.f, int):
                    x = y[layer.f]
                else:
                    x = [x if j == -1 else y[j] for j in layer.f]

            if idx == len(backbone) - 1:
                pose_logits = (
                    [feat.clone() for feat in x]
                    if isinstance(x, list)
                    else x.clone()
                )

            x = layer(x)

            if idx in self.target_layers:
                target_cache[idx] = x
                if idx == self.target_layers[-1]:
                    feats = [target_cache[li] for li in self.target_layers]
                    (
                        fused_feats,
                        restore_gammas, restore_betas, restore_gate_raws,
                        delta_gammas, delta_betas, film_gate_raws,
                    ) = self._apply_primv2_modules(
                        feats, modulate=modulate, z=z, p=p,
                    )
                    for li, fn in zip(self.target_layers, fused_feats):
                        target_cache[li] = fn
                        if li < len(y):
                            y[li] = fn
                    x = target_cache[idx]
                    f_targets = fused_feats
            y.append(x)

        return (
            f_targets,
            restore_gammas, restore_betas, restore_gate_raws,
            delta_gammas, delta_betas, film_gate_raws,
            pose_logits, x,
        )

    # ---- public forward -----------------------------------------------------

    def forward(self, i_aug=None, i_clean=None, **legacy_kwargs):
        if i_aug is None:
            i_aug = legacy_kwargs.get("I_aug")
        if i_clean is None:
            i_clean = legacy_kwargs.get("I_clean")
        if i_aug is None:
            raise ValueError("PRIMv2Pose.forward requires i_aug.")

        z = self.illumination_encoder(i_aug)
        p = self.soft_router(z)
        outputs: Dict[str, object] = {"Z": z, "P": p}

        if i_clean is not None:
            teacher = (
                self.teacher_backbone
                if self.teacher_backbone is not None
                else self.yolo_model
            )
            with torch.no_grad():
                (
                    f_cleans, _, _, _, _, _, _,
                    pose_logits_clean, x_clean,
                ) = self._run_backbone(i_clean, modulate=False, backbone=teacher)
            outputs["F_cleans"] = f_cleans
            outputs["pose_logits_clean"] = pose_logits_clean
            outputs["x_clean"] = x_clean

            # collect gate_raws on clean image for identity loss
            z_clean = self.illumination_encoder(i_clean)
            p_clean = self.soft_router(z_clean)
            (
                _, _, _, rg_clean,
                _, _, fg_clean,
                _, _,
            ) = self._run_backbone(
                i_clean, modulate=True, z=z_clean, p=p_clean,
            )
            outputs["gate_raws_clean"] = rg_clean + fg_clean

        (
            f_primes,
            restore_gammas, restore_betas, restore_gate_raws,
            delta_gammas, delta_betas, film_gate_raws,
            pose_logits_aug, x_aug,
        ) = self._run_backbone(i_aug, modulate=True, z=z, p=p)

        outputs["F_primes"] = f_primes
        outputs["restore_gammas"] = restore_gammas
        outputs["restore_betas"] = restore_betas
        outputs["restore_gate_raws"] = restore_gate_raws
        outputs["delta_gammas"] = delta_gammas
        outputs["delta_betas"] = delta_betas
        outputs["film_gate_raws"] = film_gate_raws
        outputs["pose_logits_aug"] = pose_logits_aug
        outputs["x_aug"] = x_aug
        return outputs
