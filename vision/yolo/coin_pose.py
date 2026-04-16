"""
COIN-Pose core modules.

This module introduces:
1) multi-scale context sharing (top-down + bottom-up),
2) occlusion-aware reconstruction.
"""

import copy
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F


# ==============================================================================
# Core blocks
# ==============================================================================


def _num_groups(channels: int, max_groups: int = 8) -> int:
    for groups in range(min(max_groups, channels), 0, -1):
        if channels % groups == 0:
            return groups
    return 1


def _zero_module(module: nn.Module) -> nn.Module:
    if hasattr(module, "weight") and module.weight is not None:
        nn.init.zeros_(module.weight)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.zeros_(module.bias)
    return module


def _small_init_module(module: nn.Module, std: float = 0.01) -> nn.Module:
    """Small random init: output ≈ 0 at init, but gradient flows to all layers."""
    if hasattr(module, "weight") and module.weight is not None:
        nn.init.normal_(module.weight, std=std)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.zeros_(module.bias)
    return module


class SpatiallyVariantIlluminationEncoder(nn.Module):
    """Extracts illumination embedding from RGB image."""

    def __init__(self, in_channels: int = 3, out_channels: int = 64):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(in_channels, 32, 3, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, out_channels, 3, stride=2, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.encoder(x)


class SpatiallyAwareSoftRouter(nn.Module):
    """Predicts soft expert routing weights from illumination embedding."""

    def __init__(self, illumination_channels: int = 64, num_experts: int = 3):
        super().__init__()
        self.router = nn.Sequential(
            nn.Conv2d(illumination_channels, illumination_channels, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(illumination_channels, num_experts, 1),
        )
        # Small random init for router logits → near-uniform routing at start
        _small_init_module(self.router[-1], std=0.01)

    def forward(self, z: torch.Tensor) -> torch.Tensor:
        return F.softmax(self.router(z), dim=1)


class MultiScaleRSVFiLM(nn.Module):
    """
    Multi-scale FiLM modulation with identity-preserving gates.
    """

    def __init__(
        self,
        channels_list: List[int] = [128, 128, 256],
        illumination_channels: int = 64,
        num_experts: int = 3,
        gate_init_bias: float = -0.5,
    ):
        super().__init__()
        self.num_scales = len(channels_list)
        self.num_experts = num_experts

        self.expert_gamma_list = nn.ModuleList()
        self.expert_beta_list = nn.ModuleList()
        self.gate_list = nn.ModuleList()

        for c in channels_list:
            gammas = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            betas = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            for g, b in zip(gammas, betas):
                _small_init_module(g, std=1e-3)
                _small_init_module(b, std=1e-3)
            self.expert_gamma_list.append(gammas)
            self.expert_beta_list.append(betas)

            gate_conv = nn.Conv2d(illumination_channels, 1, 1)
            nn.init.zeros_(gate_conv.weight)
            nn.init.constant_(gate_conv.bias, gate_init_bias)
            self.gate_list.append(gate_conv)

    def forward_single(
        self,
        feat: torch.Tensor,
        scale_idx: int,
        z: torch.Tensor,
        p: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        b, c, h, w = feat.shape
        z_up = F.interpolate(z, size=(h, w), mode="bilinear", align_corners=False)
        p_up = F.interpolate(p, size=(h, w), mode="bilinear", align_corners=False)

        delta_gamma = torch.zeros(b, c, h, w, device=feat.device, dtype=feat.dtype)
        delta_beta = torch.zeros(b, c, h, w, device=feat.device, dtype=feat.dtype)
        for k in range(self.num_experts):
            gamma_k = self.expert_gamma_list[scale_idx][k](z_up)
            beta_k = self.expert_beta_list[scale_idx][k](z_up)
            w_k = p_up[:, k:k + 1, :, :]
            delta_gamma = delta_gamma + w_k * gamma_k
            delta_beta = delta_beta + w_k * beta_k

        gate_raw = self.gate_list[scale_idx](z_up)
        gate = torch.sigmoid(gate_raw)
        delta_gamma = gate * delta_gamma
        delta_beta = gate * delta_beta

        # Clamp δγ to prevent modulation explosion: effective γ ∈ [0.1, 5.0]
        delta_gamma = delta_gamma.clamp(-0.9, 4.0)
        out = feat * (1.0 + delta_gamma) + delta_beta
        return out, delta_gamma, delta_beta, gate_raw


class COINContextPyramid(nn.Module):
    """
    Cross-scale context sharing neck (FPN top-down + PAN bottom-up).
    Input order: [P3, P4, P5].
    """

    def __init__(self, channels_list: List[int] = [128, 128, 256], context_channels: int = 128):
        super().__init__()
        self.context_channels = context_channels
        self.lateral = nn.ModuleList([
            nn.Sequential(
                nn.Conv2d(c, context_channels, 1, bias=False),
                nn.GroupNorm(_num_groups(context_channels), context_channels),
                nn.ReLU(inplace=True),
            )
            for c in channels_list
        ])
        self.td_smooth = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(context_channels, context_channels, 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for _ in range(2)
            ]
        )
        self.down = nn.ModuleList([
            nn.Sequential(
                nn.Conv2d(context_channels, context_channels, 3, stride=2, padding=1, bias=False),
                nn.GroupNorm(_num_groups(context_channels), context_channels),
                nn.ReLU(inplace=True),
            )
            for _ in range(2)
        ])
        self.bu_smooth = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(context_channels, context_channels, 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for _ in range(2)
            ]
        )
        self.out_proj = nn.ModuleList([_small_init_module(nn.Conv2d(context_channels, c, 1)) for c in channels_list])

    def forward(self, feats: List[torch.Tensor]) -> List[torch.Tensor]:
        p3, p4, p5 = feats
        l3, l4, l5 = self.lateral[0](p3), self.lateral[1](p4), self.lateral[2](p5)

        # Top-down context propagation
        td5 = l5
        td4 = self.td_smooth[0](l4 + F.interpolate(td5, size=l4.shape[-2:], mode="nearest"))
        td3 = self.td_smooth[1](l3 + F.interpolate(td4, size=l3.shape[-2:], mode="nearest"))

        # Bottom-up context propagation
        bu3 = td3
        bu4 = self.bu_smooth[0](td4 + self.down[0](bu3))
        bu5 = self.bu_smooth[1](td5 + self.down[1](bu4))

        deltas = [self.out_proj[0](bu3), self.out_proj[1](bu4), self.out_proj[2](bu5)]
        return [feat + delta for feat, delta in zip(feats, deltas)]


class OcclusionContextRecovery(nn.Module):
    """
    Learns visibility map and reconstruction feature, then blends:
    F_out = V * F_visible + (1 - V) * F_recon
    """

    def __init__(self, channels_list: List[int] = [128, 128, 256]):
        super().__init__()
        self.visibility_heads = nn.ModuleList()
        self.reconstruction_heads = nn.ModuleList()

        for c in channels_list:
            in_c = c * 2
            self.visibility_heads.append(
                nn.Sequential(
                    nn.Conv2d(in_c, max(c // 2, 16), 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(max(c // 2, 16)), max(c // 2, 16)),
                    nn.ReLU(inplace=True),
                    _small_init_module(nn.Conv2d(max(c // 2, 16), 1, 1)),
                    nn.Sigmoid(),
                )
            )
            self.reconstruction_heads.append(
                nn.Sequential(
                    nn.Conv2d(in_c, c, 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(c), c),
                    nn.ReLU(inplace=True),
                    _small_init_module(nn.Conv2d(c, c, 3, padding=1)),
                )
            )

    def forward_single(
        self,
        feat: torch.Tensor,
        context_feat: torch.Tensor,
        scale_idx: int,
        reconstruct: bool = True,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        fused = torch.cat([feat, context_feat], dim=1)
        vis = self.visibility_heads[scale_idx](fused)
        recon_delta = self.reconstruction_heads[scale_idx](fused)
        # Restore on top of the FiLM-conditioned feature, using context only as evidence.
        recon = feat + recon_delta
        if reconstruct:
            out = vis * feat + (1.0 - vis) * recon
        else:
            out = feat
        return out, vis, recon


# Kept for backwards compatibility with older scripts.
class OcclusionAwareAttention(nn.Module):
    def __init__(self, channels_list: List[int] = [128, 128, 256]):
        super().__init__()
        self.heads = nn.ModuleList()
        for c in channels_list:
            self.heads.append(
                nn.Sequential(
                    nn.Conv2d(c, max(c // 4, 16), 3, padding=1),
                    nn.ReLU(inplace=True),
                    nn.Conv2d(max(c // 4, 16), 1, 1),
                    nn.Sigmoid(),
                )
            )

    def forward(self, feat: torch.Tensor, scale_idx: int) -> torch.Tensor:
        return self.heads[scale_idx](feat)


# ==============================================================================
# Losses
# ==============================================================================


class FeatureConsistencySSLLoss(nn.Module):
    def forward(self, f_aug: torch.Tensor, f_clean: torch.Tensor) -> torch.Tensor:
        f_aug_norm = F.normalize(f_aug, dim=1, eps=1e-6)
        f_clean_norm = F.normalize(f_clean.detach(), dim=1, eps=1e-6)
        return F.smooth_l1_loss(f_aug_norm, f_clean_norm)


class OcclusionAwareConsistencyLoss(nn.Module):
    """Compare feat_modulated and recon SEPARATELY against clean.

    This gives vis a direct, first-order gradient:
      dL/dvis ∝ diff_feat - diff_recon
      - Where feat matches clean better (visible region): vis increases
      - Where recon matches clean better (occluded region): vis decreases
    """

    def forward(
        self,
        feat_modulated: torch.Tensor,
        recon: torch.Tensor,
        f_clean: torch.Tensor,
        vis: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        target = F.normalize(f_clean.detach(), dim=1, eps=1e-6)
        diff_feat = F.smooth_l1_loss(
            F.normalize(feat_modulated, dim=1, eps=1e-6), target, reduction="none"
        )
        diff_recon = F.smooth_l1_loss(
            F.normalize(recon, dim=1, eps=1e-6), target, reduction="none"
        )
        l_visible = (diff_feat * vis).mean()
        # Detach V to decouple visibility gradient from recon quality (RobustNeRF)
        l_occluded = (diff_recon * (1.0 - vis.detach())).mean()
        return l_visible, l_occluded


class KDESoftHistogramLoss(nn.Module):
    def __init__(self, num_bins: int = 64, sigma: float = 0.1):
        super().__init__()
        self.sigma = sigma
        self.register_buffer("centers", torch.linspace(0, 1, num_bins))

    def _soft_hist(self, x: torch.Tensor) -> torch.Tensor:
        x_min = x.min()
        x_max = x.max()
        x_norm = (x - x_min) / (x_max - x_min + 1e-8)
        x_flat = x_norm.reshape(-1, 1)
        diff = x_flat - self.centers.unsqueeze(0)
        weights = torch.exp(-0.5 * (diff / self.sigma) ** 2)
        hist = weights.sum(dim=0)
        return hist / (hist.sum() + 1e-8)

    def forward(self, f_aug: torch.Tensor, f_clean: torch.Tensor) -> torch.Tensor:
        return F.mse_loss(self._soft_hist(f_aug), self._soft_hist(f_clean.detach()))


class IlluminationSmoothLoss(nn.Module):
    def forward(self, z: torch.Tensor, _i_aug: torch.Tensor) -> torch.Tensor:
        grad_x = torch.abs(z[:, :, :, :-1] - z[:, :, :, 1:])
        grad_y = torch.abs(z[:, :, :-1, :] - z[:, :, 1:, :])
        return (grad_x.mean() + grad_y.mean()) * 0.5


class TotalVariationLoss(nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return (
            torch.abs(x[:, :, 1:, :] - x[:, :, :-1, :]).mean()
            + torch.abs(x[:, :, :, 1:] - x[:, :, :, :-1]).mean()
        )


class ExpertOrthogonalityLoss(nn.Module):
    def forward(self, expert_weights: torch.Tensor) -> torch.Tensor:
        normed = F.normalize(expert_weights, dim=1)
        gram = torch.mm(normed, normed.t())
        eye = torch.eye(gram.size(0), device=gram.device, dtype=gram.dtype)
        return F.mse_loss(gram, eye)


class ExpertEntropyLoss(nn.Module):
    """Maximize entropy of router probability distribution to prevent expert collapse.

    Input: P tensor of shape (B, num_experts, H, W) — softmax routing weights.
    Computes average usage per expert across batch and spatial dims, then returns
    negative entropy: minimizing this loss maximizes routing diversity.
    """

    def forward(self, p: torch.Tensor) -> torch.Tensor:
        # p: (B, K, H, W), average across batch and spatial → (K,)
        avg_usage = p.mean(dim=(0, 2, 3))
        # Negative entropy (minimize → maximize diversity)
        entropy = -(avg_usage * torch.log(avg_usage + 1e-8)).sum()
        # Return negative: lower loss = higher entropy
        max_entropy = torch.log(torch.tensor(float(p.shape[1]), device=p.device))
        return max_entropy - entropy


class LoadBalancingLoss(nn.Module):
    """Penalize uneven expert load distribution (Switch Transformer style).

    L = K * Σ_i (f_i * p_i)
    where f_i = fraction of tokens where expert i has highest weight,
          p_i = mean routing probability for expert i.
    """

    def forward(self, p: torch.Tensor) -> torch.Tensor:
        # p: (B, K, H, W)
        K = p.shape[1]
        # f_i: fraction assigned to expert i (hard assignment)
        hard = p.argmax(dim=1)  # (B, H, W)
        f = torch.zeros(K, device=p.device, dtype=p.dtype)
        total = hard.numel()
        for i in range(K):
            f[i] = (hard == i).float().sum() / total
        # p_i: mean routing probability
        p_mean = p.mean(dim=(0, 2, 3))  # (K,)
        return K * (f * p_mean).sum()


class CleanIdentityRegLoss(nn.Module):
    def forward(self, gate_raws_clean: List[torch.Tensor]) -> torch.Tensor:
        penalty = torch.zeros((), device=gate_raws_clean[0].device, dtype=gate_raws_clean[0].dtype)
        for gate_raw in gate_raws_clean:
            penalty = penalty + F.softplus(gate_raw).mean()
        return penalty / len(gate_raws_clean)


class VisibilityRegLoss(nn.Module):
    """Push visibility maps toward a target value."""

    def forward(self, vis_maps: List[torch.Tensor], target_value: float = 1.0) -> torch.Tensor:
        if not vis_maps:
            return torch.zeros(())
        return sum(
            F.mse_loss(v, torch.full_like(v, target_value)) for v in vis_maps
        ) / len(vis_maps)


class VisibilitySparsityLoss(nn.Module):
    """Encourage binary visibility decisions: vis → 0 or 1.

    4*v*(1-v) peaks at 0.5 and is zero at 0 and 1.
    """

    def forward(self, vis_maps: List[torch.Tensor]) -> torch.Tensor:
        if not vis_maps:
            return torch.zeros(())
        return sum(
            (4.0 * v * (1.0 - v)).mean() for v in vis_maps
        ) / len(vis_maps)


class VisibilityBinaryEntropyLoss(nn.Module):
    """Push V toward 0 or 1 via negative binary entropy (NeRF-W style).

    L = -mean(V*log(V) + (1-V)*log(1-V))
    Maximized at V=0.5 (=log2), zero at V=0 or V=1.
    Minimizing this loss forces V to be decisive.
    """

    def forward(self, vis_maps: List[torch.Tensor]) -> torch.Tensor:
        if not vis_maps:
            return torch.zeros(())
        eps = 1e-6
        total = torch.zeros((), device=vis_maps[0].device, dtype=vis_maps[0].dtype)
        for v in vis_maps:
            vc = v.clamp(eps, 1.0 - eps)
            total = total - (vc * vc.log() + (1.0 - vc) * (1.0 - vc).log()).mean()
        return total / len(vis_maps)


class VisibilityGTLoss(nn.Module):
    """Supervise visibility maps at keypoint locations using GT visibility labels.

    vis_gt_targets: list[Tensor] of length B, each (N_kp, 3) — [norm_x, norm_y, gt_vis].
    gt_vis ∈ {0.0, 0.5, 1.0} mapped from label {0, 1, 2}.
    Entries with gt_vis < 0 are skipped (padding).
    """

    def forward(
        self,
        vis_maps: List[torch.Tensor],
        vis_gt_targets: List[torch.Tensor],
    ) -> torch.Tensor:
        if not vis_maps or not vis_gt_targets:
            return torch.zeros(())
        device = vis_maps[0].device
        dtype = vis_maps[0].dtype
        total = torch.zeros((), device=device, dtype=dtype)
        count = 0
        for v in vis_maps:  # per scale
            _, _, h, w = v.shape
            for bi, kps in enumerate(vis_gt_targets):  # per batch sample
                if bi >= v.shape[0]:
                    break
                if kps.ndim != 2 or kps.shape[1] < 3:
                    continue
                kps = kps.to(device)
                valid = kps[:, 2] >= 0
                if not valid.any():
                    continue
                kps = kps[valid]
                gx = (kps[:, 0] * w).clamp(0, w - 1).long()
                gy = (kps[:, 1] * h).clamp(0, h - 1).long()
                pred_vis = v[bi, 0, gy, gx]
                gt_vis = kps[:, 2]
                total = total + F.mse_loss(pred_vis, gt_vis)
                count += 1
        return total / max(count, 1)


class COINLoss(nn.Module):
    PHASE_WEIGHTS: Dict[int, Dict[str, float]] = {
        1: dict(
            kd=0.3,
            ssl=1.0,
            occ=1.0,
            hist=1.0,
            smooth=0.3,
            tv=0.05,
            identity=0.2,
            vis_reg=0.05,
            vis_illum=0.10,
            vis_gt=0.5,
            expert_entropy=0.05,  # prevent expert collapse
            load_balance=0.02,    # even expert utilization
        ),
        2: dict(
            kd=1.0,
            ssl=0.5,
            occ=0.8,
            hist=0.3,
            smooth=0.1,
            tv=0.05,
            identity=0.1,
            vis_reg=0.02,
            vis_illum=0.05,
            vis_gt=0.3,
            expert_entropy=0.02,
            load_balance=0.01,
        ),
    }

    def __init__(self, phase: int = 1):
        super().__init__()
        self.ssl_loss = FeatureConsistencySSLLoss()
        self.occ_loss = OcclusionAwareConsistencyLoss()
        self.hist_loss = KDESoftHistogramLoss()
        self.smooth_loss = IlluminationSmoothLoss()
        self.tv_loss = TotalVariationLoss()
        self.identity_loss = CleanIdentityRegLoss()
        self.vis_reg_loss = VisibilityRegLoss()
        self.vis_gt_loss = VisibilityGTLoss()
        self.expert_entropy_loss = ExpertEntropyLoss()
        self.load_balance_loss = LoadBalancingLoss()
        self.set_phase(phase)

    def set_phase(self, phase: int):
        self.phase = phase
        w = self.PHASE_WEIGHTS.get(phase, self.PHASE_WEIGHTS[2])
        self.lambda_kd = w["kd"]
        self.lambda_ssl = w["ssl"]
        self.lambda_occ = w["occ"]
        self.lambda_hist = w["hist"]
        self.lambda_smooth = w["smooth"]
        self.lambda_tv = w["tv"]
        self.lambda_identity = w["identity"]
        self.lambda_vis_reg = w.get("vis_reg", 0.0)
        self.lambda_vis_illum = w.get("vis_illum", 0.0)
        self.lambda_vis_gt = w.get("vis_gt", 0.0)
        self.lambda_expert_entropy = w.get("expert_entropy", 0.0)
        self.lambda_load_balance = w.get("load_balance", 0.0)

    @staticmethod
    def _select_tensor_list(tensors: Optional[List[torch.Tensor]], mask: torch.Tensor) -> List[torch.Tensor]:
        if tensors is None or len(tensors) == 0 or mask.numel() == 0 or not bool(mask.any()):
            return []
        return [t[mask] for t in tensors]

    @staticmethod
    def _normalize_pair_types(
        pair_types: Optional[List[str]],
        batch_size: int,
        device: torch.device,
    ) -> Tuple[Optional[torch.Tensor], Optional[torch.Tensor]]:
        if pair_types is None:
            return None, None
        if isinstance(pair_types, str):
            pair_list = [pair_types] * batch_size
        else:
            pair_list = list(pair_types)
        if len(pair_list) != batch_size:
            return None, None
        illum_mask = torch.tensor([p == "illum" for p in pair_list], device=device, dtype=torch.bool)
        occlu_mask = torch.tensor([p == "occlu" for p in pair_list], device=device, dtype=torch.bool)
        return illum_mask, occlu_mask

    def forward(
        self,
        f_aug_primes: Optional[List[torch.Tensor]] = None,
        f_cleans: Optional[List[torch.Tensor]] = None,
        z: Optional[torch.Tensor] = None,
        i_aug: Optional[torch.Tensor] = None,
        delta_gammas: Optional[List[torch.Tensor]] = None,
        delta_betas: Optional[List[torch.Tensor]] = None,
        expert_weights_list: Optional[List[torch.Tensor]] = None,
        pose_logits_aug: Optional[torch.Tensor] = None,
        pose_logits_clean: Optional[torch.Tensor] = None,
        visibility_maps: Optional[List[torch.Tensor]] = None,
        gate_raws_clean: Optional[List[torch.Tensor]] = None,
        visibility_maps_clean: Optional[List[torch.Tensor]] = None,
        feat_modulated_list: Optional[List[torch.Tensor]] = None,
        recon_list: Optional[List[torch.Tensor]] = None,
        pair_types: Optional[List[str]] = None,
        vis_gt_targets: Optional[List[torch.Tensor]] = None,
        router_probs: Optional[torch.Tensor] = None,
        **legacy_kwargs,
    ) -> Dict[str, torch.Tensor]:
        # Backward-compatible argument aliases (legacy signature).
        if f_aug_primes is None:
            f_aug_primes = legacy_kwargs.get("F_aug_primes", [])
        if f_cleans is None:
            f_cleans = legacy_kwargs.get("F_cleans", [])
        if z is None:
            z = legacy_kwargs.get("Z")
        if i_aug is None:
            i_aug = legacy_kwargs.get("I_aug")
        if delta_gammas is None:
            delta_gammas = legacy_kwargs.get("delta_gammas", [])
        if delta_betas is None:
            delta_betas = legacy_kwargs.get("delta_betas", [])
        if expert_weights_list is None:
            expert_weights_list = legacy_kwargs.get("expert_weights_list", [])
        if pair_types is None:
            pair_types = legacy_kwargs.get("pair_types")

        if z is None or i_aug is None:
            raise ValueError("COINLoss requires z and i_aug tensors.")

        losses: Dict[str, torch.Tensor] = {}
        n = max(len(f_aug_primes), 1)
        batch_size = z.shape[0]
        zero = torch.zeros((), device=z.device, dtype=z.dtype)
        illum_mask, occlu_mask = self._normalize_pair_types(pair_types, batch_size, z.device)

        has_occ_inputs = (
            visibility_maps is not None
            and feat_modulated_list is not None
            and recon_list is not None
            and len(visibility_maps) == len(f_cleans)
        )

        if illum_mask is not None or occlu_mask is not None:
            ssl_terms = []
            ssl_weights = []

            if illum_mask is not None and bool(illum_mask.any()) and feat_modulated_list is not None:
                feat_m_illum = self._select_tensor_list(feat_modulated_list, illum_mask)
                f_clean_illum = self._select_tensor_list(f_cleans, illum_mask)
                if feat_m_illum and f_clean_illum:
                    illum_ssl = sum(
                        self.ssl_loss(feat_m, f_clean)
                        for feat_m, f_clean in zip(feat_m_illum, f_clean_illum)
                    ) / len(feat_m_illum)
                    ssl_terms.append(illum_ssl * int(illum_mask.sum().item()))
                    ssl_weights.append(int(illum_mask.sum().item()))

            if has_occ_inputs and occlu_mask is not None and bool(occlu_mask.any()):
                feat_m_occ = self._select_tensor_list(feat_modulated_list, occlu_mask)
                recon_occ = self._select_tensor_list(recon_list, occlu_mask)
                f_clean_occ = self._select_tensor_list(f_cleans, occlu_mask)
                vis_occ = self._select_tensor_list(visibility_maps, occlu_mask)
                vis_terms, occ_terms = [], []
                for feat_m, recon, f_clean, vis in zip(feat_m_occ, recon_occ, f_clean_occ, vis_occ):
                    lv, lo = self.occ_loss(feat_m, recon, f_clean, vis)
                    vis_terms.append(lv)
                    occ_terms.append(lo)
                occlu_count = int(occlu_mask.sum().item())
                if vis_terms:
                    ssl_terms.append((sum(vis_terms) / len(vis_terms)) * occlu_count)
                    ssl_weights.append(occlu_count)
                losses["occ_rec"] = self.lambda_occ * (sum(occ_terms) / max(len(occ_terms), 1))
            else:
                losses["occ_rec"] = zero

            if ssl_terms:
                losses["ssl"] = self.lambda_ssl * (sum(ssl_terms) / max(sum(ssl_weights), 1))
            else:
                losses["ssl"] = zero
        else:
            if has_occ_inputs:
                vis_terms, occ_terms = [], []
                for feat_m, recon, f_clean, vis in zip(
                    feat_modulated_list, recon_list, f_cleans, visibility_maps
                ):
                    lv, lo = self.occ_loss(feat_m, recon, f_clean, vis)
                    vis_terms.append(lv)
                    occ_terms.append(lo)
                losses["ssl"] = self.lambda_ssl * sum(vis_terms) / n
                losses["occ_rec"] = self.lambda_occ * sum(occ_terms) / n
            else:
                losses["ssl"] = self.lambda_ssl * sum(self.ssl_loss(fa, fc) for fa, fc in zip(f_aug_primes, f_cleans)) / n
                losses["occ_rec"] = zero

        losses["hist"] = self.lambda_hist * sum(self.hist_loss(fa, fc) for fa, fc in zip(f_aug_primes, f_cleans)) / n
        losses["smooth"] = self.lambda_smooth * self.smooth_loss(z, i_aug)
        losses["tv"] = self.lambda_tv * sum(self.tv_loss(dg) + self.tv_loss(db) for dg, db in zip(delta_gammas, delta_betas)) / max(len(delta_gammas), 1)

        if gate_raws_clean is not None and self.lambda_identity > 0 and len(gate_raws_clean) > 0:
            losses["identity"] = self.lambda_identity * self.identity_loss(gate_raws_clean)
        else:
            losses["identity"] = zero

        if visibility_maps_clean is not None and len(visibility_maps_clean) > 0 and self.lambda_vis_reg > 0:
            losses["vis_reg"] = self.lambda_vis_reg * self.vis_reg_loss(visibility_maps_clean)
        else:
            losses["vis_reg"] = zero

        if (
            visibility_maps is not None
            and self.lambda_vis_illum > 0
            and illum_mask is not None
            and bool(illum_mask.any())
        ):
            vis_illum = self._select_tensor_list(visibility_maps, illum_mask)
            losses["vis_illum"] = self.lambda_vis_illum * self.vis_reg_loss(vis_illum)
        else:
            losses["vis_illum"] = zero

        # GT visibility supervision at keypoint locations
        if (
            vis_gt_targets is not None
            and visibility_maps is not None
            and len(visibility_maps) > 0
            and self.lambda_vis_gt > 0
        ):
            losses["vis_gt"] = self.lambda_vis_gt * self.vis_gt_loss(visibility_maps, vis_gt_targets)
        else:
            losses["vis_gt"] = zero

        # Expert routing regularization (prevent collapse)
        if router_probs is None:
            router_probs = legacy_kwargs.get("router_probs")
        if router_probs is not None and self.lambda_expert_entropy > 0:
            losses["expert_entropy"] = self.lambda_expert_entropy * self.expert_entropy_loss(router_probs)
        else:
            losses["expert_entropy"] = zero
        if router_probs is not None and self.lambda_load_balance > 0:
            losses["load_balance"] = self.lambda_load_balance * self.load_balance_loss(router_probs)
        else:
            losses["load_balance"] = zero

        if self.lambda_kd > 0 and pose_logits_aug is not None and pose_logits_clean is not None:
            def _l1_recursive(a, c):
                if isinstance(a, torch.Tensor) and isinstance(c, torch.Tensor):
                    return F.l1_loss(a, c.detach())
                if isinstance(a, (list, tuple)) and isinstance(c, (list, tuple)) and len(a) > 0:
                    return sum(_l1_recursive(ia, ic) for ia, ic in zip(a, c)) / len(a)
                if isinstance(a, dict) and isinstance(c, dict) and len(a) > 0:
                    keys = [k for k in a if k in c]
                    if not keys:
                        return torch.zeros((), device=z.device, dtype=z.dtype)
                    return sum(_l1_recursive(a[k], c[k]) for k in keys) / len(keys)
                return torch.zeros((), device=z.device, dtype=z.dtype)

            losses["kd"] = self.lambda_kd * _l1_recursive(pose_logits_aug, pose_logits_clean)
        else:
            losses["kd"] = zero

        losses["total"] = sum(losses.values())
        return losses


# ==============================================================================
# Augmentation helper
# ==============================================================================


class IlluminationAugmentor:
    """Simple illumination-only augmentation helper."""

    @staticmethod
    def augment(i_clean: torch.Tensor) -> Tuple[torch.Tensor, str]:
        b = i_clean.shape[0]
        device = i_clean.device
        aug_type = torch.randint(0, 4, (1,)).item()

        if aug_type == 0:
            factor = 0.1 + 0.4 * torch.rand(b, 1, 1, 1, device=device)
            i_aug = i_clean * factor
            name = "dark"
        elif aug_type == 1:
            _, _, h, w = i_clean.shape
            grad = torch.linspace(0.2, 1.0, w, device=device).view(1, 1, 1, w).expand(b, 1, h, w)
            i_aug = i_clean * grad
            name = "gradient"
        elif aug_type == 2:
            gray = i_clean.mean(dim=1, keepdim=True)
            alpha = 0.1 + 0.5 * torch.rand(b, 1, 1, 1, device=device)
            i_aug = alpha * i_clean + (1.0 - alpha) * gray
            name = "desaturate"
        else:
            gamma = 1.5 + 1.5 * torch.rand(b, 1, 1, 1, device=device)
            i_aug = torch.clamp(i_clean, 0, 1).pow(gamma)
            name = "gamma"

        return torch.clamp(i_aug, 0, 1), name


# ==============================================================================
# End-to-end wrapper
# ==============================================================================


class COINPose(nn.Module):
    """
    YOLO26n-pose + COIN-Pose modules:
    - illumination-aware FiLM
    - cross-scale context sharing
    - occlusion-aware reconstruction
    """

    TARGET_LAYERS = [4, 6, 10]
    CHANNELS = [128, 128, 256]

    def __init__(self, yolo_model, num_experts: int = 3):
        super().__init__()
        self.yolo_model = yolo_model.model.model
        self.save_list = yolo_model.model.save
        object.__setattr__(self, "_yolo_detection_model", yolo_model.model)

        self.teacher_backbone = None

        illumination_channels = 64
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(3, illumination_channels)
        self.soft_router = SpatiallyAwareSoftRouter(illumination_channels, num_experts)
        self.multi_scale_rsv = MultiScaleRSVFiLM(self.CHANNELS, illumination_channels, num_experts)
        self.context_pyramid = COINContextPyramid(self.CHANNELS, context_channels=128)
        self.occlusion_recovery = OcclusionContextRecovery(self.CHANNELS)
        self.criterion = COINLoss(phase=1)

    def _freeze_yolo(self):
        for p in self.yolo_model.parameters():
            p.requires_grad = False

    def _unfreeze_yolo(self):
        for p in self.yolo_model.parameters():
            p.requires_grad = True

    def _init_frozen_teacher(self):
        self.teacher_backbone = copy.deepcopy(self.yolo_model)
        for p in self.teacher_backbone.parameters():
            p.requires_grad = False
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
        """EMA update of teacher backbone from current backbone."""
        if self.teacher_backbone is None:
            return
        for t_param, s_param in zip(
            self.teacher_backbone.parameters(), self.yolo_model.parameters()
        ):
            t_param.data.mul_(momentum).add_(s_param.data, alpha=1.0 - momentum)

    def get_expert_weights(self) -> List[torch.Tensor]:
        weights_list = []
        for i in range(self.multi_scale_rsv.num_scales):
            weights = [e.weight.reshape(-1) for e in self.multi_scale_rsv.expert_gamma_list[i]]
            weights_list.append(torch.stack(weights, dim=0))
        return weights_list

    def _apply_coin_modules(
        self,
        target_feats: List[torch.Tensor],
        modulate: bool,
        z: Optional[torch.Tensor] = None,
        p: Optional[torch.Tensor] = None,
    ):
        modulated_feats = []
        delta_gammas, delta_betas, gate_raws = [], [], []

        if modulate:
            assert z is not None and p is not None
            for idx, feat in enumerate(target_feats):
                feat_m, dg, db, gate_raw = self.multi_scale_rsv.forward_single(feat, idx, z, p)
                modulated_feats.append(feat_m)
                delta_gammas.append(dg)
                delta_betas.append(db)
                gate_raws.append(gate_raw)
        else:
            modulated_feats = target_feats

        context_feats = self.context_pyramid(modulated_feats)
        out_feats, visibility_maps, recon_feats = [], [], []
        for idx, (feat_m, feat_ctx) in enumerate(zip(modulated_feats, context_feats)):
            out, vis, recon = self.occlusion_recovery.forward_single(
                feat=feat_m,
                context_feat=feat_ctx,
                scale_idx=idx,
                reconstruct=modulate,
            )
            out_feats.append(out)
            visibility_maps.append(vis)
            recon_feats.append(recon)

        return out_feats, delta_gammas, delta_betas, gate_raws, visibility_maps, recon_feats, modulated_feats

    def _run_backbone(self, x, modulate: bool = False, z=None, p=None, backbone=None):
        if backbone is None:
            backbone = self.yolo_model

        y = []
        f_targets: List[torch.Tensor] = []
        delta_gammas: List[torch.Tensor] = []
        delta_betas: List[torch.Tensor] = []
        gate_raws: List[torch.Tensor] = []
        visibility_maps: List[torch.Tensor] = []
        recon_feats: List[torch.Tensor] = []
        modulated_feats: List[torch.Tensor] = []
        pose_logits = None

        target_cache: Dict[int, torch.Tensor] = {}

        for i, m in enumerate(backbone):
            if m.f != -1:
                if isinstance(m.f, int):
                    x = y[m.f]
                else:
                    x = [x if j == -1 else y[j] for j in m.f]

            if i == len(backbone) - 1:
                pose_logits = [f.clone() for f in x] if isinstance(x, list) else x.clone()

            x = m(x)

            if i in self.TARGET_LAYERS:
                target_cache[i] = x

                if modulate and i == self.TARGET_LAYERS[-1]:
                    feats = [target_cache[layer_idx] for layer_idx in self.TARGET_LAYERS]
                    (
                        fused_feats,
                        delta_gammas,
                        delta_betas,
                        gate_raws,
                        visibility_maps,
                        recon_feats,
                        modulated_feats,
                    ) = self._apply_coin_modules(feats, modulate=modulate, z=z, p=p)

                    for layer_idx, feat_new in zip(self.TARGET_LAYERS, fused_feats):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new

                    x = target_cache[i]
                    f_targets = fused_feats
                elif (not modulate) and i == self.TARGET_LAYERS[-1]:
                    f_targets = [target_cache[layer_idx] for layer_idx in self.TARGET_LAYERS]

            y.append(x)

        return f_targets, delta_gammas, delta_betas, gate_raws, visibility_maps, recon_feats, modulated_feats, pose_logits, x

    def forward(self, i_aug=None, i_clean=None, **legacy_kwargs):
        if i_aug is None:
            i_aug = legacy_kwargs.get("I_aug")
        if i_clean is None:
            i_clean = legacy_kwargs.get("I_clean")
        if i_aug is None:
            raise ValueError("COINPose.forward requires i_aug (or legacy I_aug).")

        z = self.illumination_encoder(i_aug)
        p = self.soft_router(z)
        outputs = {"Z": z, "P": p}

        if i_clean is not None:
            teacher = self.teacher_backbone if self.teacher_backbone is not None else self.yolo_model
            with torch.no_grad():
                (
                    f_cleans,
                    _, _, _, _, _, _,
                    pose_logits_clean,
                    x_clean,
                ) = self._run_backbone(i_clean, modulate=False, backbone=teacher)
            outputs["F_cleans"] = f_cleans
            outputs["pose_logits_clean"] = pose_logits_clean
            outputs["x_clean"] = x_clean

            z_clean = self.illumination_encoder(i_clean)
            p_clean = self.soft_router(z_clean)
            _, _, _, gate_raws_clean, vis_maps_clean, _, _, _, _ = self._run_backbone(i_clean, modulate=True, z=z_clean, p=p_clean)
            outputs["gate_raws_clean"] = gate_raws_clean
            outputs["visibility_maps_clean"] = vis_maps_clean

        (
            f_primes,
            delta_gammas,
            delta_betas,
            gate_raws,
            vis_maps,
            recon_feats,
            mod_feats,
            pose_logits_aug,
            x_aug,
        ) = self._run_backbone(i_aug, modulate=True, z=z, p=p)
        outputs["F_primes"] = f_primes
        outputs["delta_gammas"] = delta_gammas
        outputs["delta_betas"] = delta_betas
        outputs["gate_raws"] = gate_raws
        outputs["visibility_maps"] = vis_maps
        outputs["reconstruction_feats"] = recon_feats
        outputs["modulated_feats"] = mod_feats
        outputs["pose_logits_aug"] = pose_logits_aug
        outputs["x_aug"] = x_aug

        return outputs
