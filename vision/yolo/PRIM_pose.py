"""
PRIM's core modules.

PRIM keeps only:
1) illumination-conditioned expert routing,
2) FiLM modulation on multi-scale features,
3) pyramid context fusion.
"""

import copy
from typing import Dict, List, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F


def _num_groups(channels: int, max_groups: int = 8) -> int:
    for groups in range(min(max_groups, channels), 0, -1):
        if channels % groups == 0:
            return groups
    return 1


def _small_init_module(module: nn.Module, std: float = 0.01) -> nn.Module:
    if hasattr(module, "weight") and module.weight is not None:
        nn.init.normal_(module.weight, std=std)
    if hasattr(module, "bias") and module.bias is not None:
        nn.init.zeros_(module.bias)
    return module


def _parse_feature_layers(feature_layers) -> Optional[List[int]]:
    if feature_layers is None:
        return None
    if isinstance(feature_layers, str):
        parts = [p.strip() for p in feature_layers.split(",") if p.strip()]
        return [int(p) for p in parts] if parts else None
    return [int(v) for v in feature_layers]


def _first_floating_param(module: nn.Module) -> Optional[torch.nn.Parameter]:
    for param in module.parameters():
        if param.is_floating_point():
            return param
    return None


@torch.no_grad()
def infer_backbone_layer_shapes(
    backbone: nn.Module,
    input_shape: Tuple[int, int, int, int] = (1, 3, 640, 640),
) -> Dict[int, Tuple[int, int, int, int]]:
    param = _first_floating_param(backbone)
    device = param.device if param is not None else torch.device("cpu")
    dtype = param.dtype if param is not None else torch.float32
    was_training = backbone.training
    backbone.eval()

    x = torch.zeros(input_shape, device=device, dtype=dtype)
    y = []
    shapes: Dict[int, Tuple[int, int, int, int]] = {}

    for i, layer in enumerate(backbone):
        if layer.f != -1:
            if isinstance(layer.f, int):
                x = y[layer.f]
            else:
                x = [x if j == -1 else y[j] for j in layer.f]
        x = layer(x)
        if isinstance(x, torch.Tensor) and x.ndim == 4:
            shapes[i] = tuple(int(v) for v in x.shape)
        y.append(x)

    backbone.train(was_training)
    return shapes


def infer_checkpoint_channels(state: dict) -> List[int]:
    channels = {}
    for key, value in state.get("multi_scale_rsv", {}).items():
        if key.startswith("expert_gamma_list.") and key.endswith(".weight"):
            parts = key.split(".")
            if len(parts) >= 4:
                scale_idx = int(parts[1])
                channels.setdefault(scale_idx, int(value.shape[0]))
    return [channels[idx] for idx in sorted(channels)]


def infer_prim_feature_spec(
    backbone: nn.Module,
    feature_source: str = "backbone",
    feature_layers=None,
    input_shape: Tuple[int, int, int, int] = (1, 3, 640, 640),
) -> Dict[str, List[int]]:
    feature_source = str(feature_source).strip().lower()
    if feature_source not in {"backbone", "head"}:
        raise ValueError(f"Unsupported feature_source: {feature_source}")

    shapes = infer_backbone_layer_shapes(backbone, input_shape=input_shape)
    parsed_layers = _parse_feature_layers(feature_layers)
    detect_sources = list(getattr(backbone[-1], "f", [])) if len(backbone) > 0 else []
    num_scales = len(detect_sources) if detect_sources else 3

    if parsed_layers is None:
        if feature_source == "head":
            parsed_layers = [int(layer_idx) for layer_idx in detect_sources if int(layer_idx) in shapes]
        else:
            first_neck_idx = next(
                (i for i, layer in enumerate(backbone) if type(layer).__name__ == "Upsample"),
                len(backbone),
            )
            resolution_to_layer = {}
            for idx in sorted(shapes):
                if idx >= first_neck_idx:
                    continue
                _, _, h, w = shapes[idx]
                resolution_to_layer[(h, w)] = idx
            ordered_resolutions = sorted(resolution_to_layer, key=lambda hw: hw[0] * hw[1], reverse=True)
            selected_resolutions = ordered_resolutions[-num_scales:] if len(ordered_resolutions) > num_scales else ordered_resolutions
            parsed_layers = [resolution_to_layer[res] for res in selected_resolutions]

    missing = [idx for idx in parsed_layers if idx not in shapes]
    if missing:
        raise ValueError(f"Could not infer PRIM feature shapes for layers: {missing}")

    parsed_layers = sorted(
        [int(idx) for idx in parsed_layers],
        key=lambda idx: (-(shapes[idx][2] * shapes[idx][3]), idx),
    )
    resolutions = [(shapes[idx][2], shapes[idx][3]) for idx in parsed_layers]
    if len(set(resolutions)) != len(resolutions):
        raise ValueError(f"PRIM feature layers must have distinct spatial resolutions, got {parsed_layers} with {resolutions}")

    return {
        "target_layers": parsed_layers,
        "channels": [int(shapes[idx][1]) for idx in parsed_layers],
        "resolutions": [[int(shapes[idx][2]), int(shapes[idx][3])] for idx in parsed_layers],
        "feature_source": feature_source,
    }


class SpatiallyVariantIlluminationEncoder(nn.Module):
    """Extract an illumination embedding from RGB input."""

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
    """Predict soft expert routing weights from the illumination embedding."""

    def __init__(self, illumination_channels: int = 64, num_experts: int = 3):
        super().__init__()
        self.router = nn.Sequential(
            nn.Conv2d(illumination_channels, illumination_channels, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(illumination_channels, num_experts, 1),
        )
        _small_init_module(self.router[-1], std=0.01)

    def forward(self, z: torch.Tensor) -> torch.Tensor:
        return F.softmax(self.router(z), dim=1)


class MultiScaleRSVFiLM(nn.Module):
    """Multi-scale FiLM modulation with expert routing and identity-preserving gates."""

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

        for channels in channels_list:
            gammas = nn.ModuleList([nn.Conv2d(illumination_channels, channels, 1) for _ in range(num_experts)])
            betas = nn.ModuleList([nn.Conv2d(illumination_channels, channels, 1) for _ in range(num_experts)])
            for gamma_head, beta_head in zip(gammas, betas):
                _small_init_module(gamma_head, std=1e-3)
                _small_init_module(beta_head, std=1e-3)
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
        batch, channels, height, width = feat.shape
        z_up = F.interpolate(z, size=(height, width), mode="bilinear", align_corners=False)
        p_up = F.interpolate(p, size=(height, width), mode="bilinear", align_corners=False)

        delta_gamma = torch.zeros(batch, channels, height, width, device=feat.device, dtype=feat.dtype)
        delta_beta = torch.zeros(batch, channels, height, width, device=feat.device, dtype=feat.dtype)
        for expert_idx in range(self.num_experts):
            gamma_k = self.expert_gamma_list[scale_idx][expert_idx](z_up)
            beta_k = self.expert_beta_list[scale_idx][expert_idx](z_up)
            w_k = p_up[:, expert_idx:expert_idx + 1, :, :]
            delta_gamma = delta_gamma + w_k * gamma_k
            delta_beta = delta_beta + w_k * beta_k

        gate_raw = self.gate_list[scale_idx](z_up)
        gate = torch.sigmoid(gate_raw)
        delta_gamma = (gate * delta_gamma).clamp(-0.9, 4.0)
        delta_beta = gate * delta_beta
        out = feat * (1.0 + delta_gamma) + delta_beta
        return out, delta_gamma, delta_beta, gate_raw


class PRIMFeaturePyramid(nn.Module):
    """Pyramid context fusion built from top-down and bottom-up propagation."""

    def __init__(self, channels_list: List[int] = [128, 128, 256], context_channels: int = 128):
        super().__init__()
        self.lateral = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(channels, context_channels, 1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for channels in channels_list
            ]
        )
        self.td_smooth = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(context_channels, context_channels, 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for _ in range(max(len(channels_list) - 1, 0))
            ]
        )
        self.down = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(context_channels, context_channels, 3, stride=2, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for _ in range(max(len(channels_list) - 1, 0))
            ]
        )
        self.bu_smooth = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(context_channels, context_channels, 3, padding=1, bias=False),
                    nn.GroupNorm(_num_groups(context_channels), context_channels),
                    nn.ReLU(inplace=True),
                )
                for _ in range(max(len(channels_list) - 1, 0))
            ]
        )
        self.out_proj = nn.ModuleList([_small_init_module(nn.Conv2d(context_channels, channels, 1)) for channels in channels_list])
        self.fuse_gate = nn.ModuleList()
        for channels in channels_list:
            gate_conv = nn.Conv2d(channels, 1, 1)
            nn.init.zeros_(gate_conv.weight)
            nn.init.constant_(gate_conv.bias, -2.0)
            self.fuse_gate.append(gate_conv)

    def forward(self, feats: List[torch.Tensor]) -> List[torch.Tensor]:
        if len(feats) != len(self.lateral):
            raise ValueError(f"Expected {len(self.lateral)} features, got {len(feats)}")
        if not feats:
            return []

        lateral_feats = [proj(feat) for proj, feat in zip(self.lateral, feats)]

        td_feats = list(lateral_feats)
        for idx in range(len(td_feats) - 2, -1, -1):
            td_feats[idx] = self.td_smooth[idx](
                lateral_feats[idx] + F.interpolate(td_feats[idx + 1], size=lateral_feats[idx].shape[-2:], mode="nearest")
            )

        bu_feats = list(td_feats)
        for idx in range(1, len(bu_feats)):
            bu_feats[idx] = self.bu_smooth[idx - 1](td_feats[idx] + self.down[idx - 1](bu_feats[idx - 1]))

        fused = []
        for feat, ctx_feat, out_proj, gate_conv in zip(feats, bu_feats, self.out_proj, self.fuse_gate):
            context_delta = out_proj(ctx_feat)
            gate = torch.sigmoid(gate_conv(feat))
            fused.append(feat + gate * context_delta)
        return fused


class FeatureConsistencySSLLoss(nn.Module):
    def forward(self, f_aug: torch.Tensor, f_clean: torch.Tensor) -> torch.Tensor:
        f_aug_norm = F.normalize(f_aug, dim=1, eps=1e-6)
        f_clean_norm = F.normalize(f_clean.detach(), dim=1, eps=1e-6)
        return F.smooth_l1_loss(f_aug_norm, f_clean_norm)


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
        return 0.5 * (grad_x.mean() + grad_y.mean())


class TotalVariationLoss(nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return (
            torch.abs(x[:, :, 1:, :] - x[:, :, :-1, :]).mean()
            + torch.abs(x[:, :, :, 1:] - x[:, :, :, :-1]).mean()
        )


class ExpertEntropyLoss(nn.Module):
    def forward(self, p: torch.Tensor) -> torch.Tensor:
        avg_usage = p.mean(dim=(0, 2, 3))
        entropy = -(avg_usage * torch.log(avg_usage + 1e-8)).sum()
        max_entropy = torch.log(torch.tensor(float(p.shape[1]), device=p.device))
        return max_entropy - entropy


class LoadBalancingLoss(nn.Module):
    def forward(self, p: torch.Tensor) -> torch.Tensor:
        num_experts = p.shape[1]
        hard = p.argmax(dim=1)
        hard_frac = torch.zeros(num_experts, device=p.device, dtype=p.dtype)
        total = hard.numel()
        for expert_idx in range(num_experts):
            hard_frac[expert_idx] = (hard == expert_idx).float().sum() / total
        p_mean = p.mean(dim=(0, 2, 3))
        return num_experts * (hard_frac * p_mean).sum()


class CleanIdentityRegLoss(nn.Module):
    def forward(self, gate_raws_clean: List[torch.Tensor]) -> torch.Tensor:
        penalty = torch.zeros((), device=gate_raws_clean[0].device, dtype=gate_raws_clean[0].dtype)
        for gate_raw in gate_raws_clean:
            penalty = penalty + F.softplus(gate_raw).mean()
        return penalty / len(gate_raws_clean)


class PRIMLoss(nn.Module):
    PHASE_WEIGHTS: Dict[int, Dict[str, float]] = {
        1: dict(kd=0.3, ssl=1.0, hist=1.0, smooth=0.3, tv=0.05, identity=0.2, expert_entropy=0.05, load_balance=0.02),
        2: dict(kd=1.0, ssl=0.5, hist=0.3, smooth=0.1, tv=0.05, identity=0.1, expert_entropy=0.02, load_balance=0.01),
    }

    def __init__(self, phase: int = 1):
        super().__init__()
        self.ssl_loss = FeatureConsistencySSLLoss()
        self.hist_loss = KDESoftHistogramLoss()
        self.smooth_loss = IlluminationSmoothLoss()
        self.tv_loss = TotalVariationLoss()
        self.identity_loss = CleanIdentityRegLoss()
        self.expert_entropy_loss = ExpertEntropyLoss()
        self.load_balance_loss = LoadBalancingLoss()
        self.set_phase(phase)

    def set_phase(self, phase: int):
        weights = self.PHASE_WEIGHTS.get(phase, self.PHASE_WEIGHTS[2])
        self.lambda_kd = weights["kd"]
        self.lambda_ssl = weights["ssl"]
        self.lambda_hist = weights["hist"]
        self.lambda_smooth = weights["smooth"]
        self.lambda_tv = weights["tv"]
        self.lambda_identity = weights["identity"]
        self.lambda_expert_entropy = weights["expert_entropy"]
        self.lambda_load_balance = weights["load_balance"]

    def forward(
        self,
        f_aug_primes: Optional[List[torch.Tensor]] = None,
        f_cleans: Optional[List[torch.Tensor]] = None,
        z: Optional[torch.Tensor] = None,
        i_aug: Optional[torch.Tensor] = None,
        delta_gammas: Optional[List[torch.Tensor]] = None,
        delta_betas: Optional[List[torch.Tensor]] = None,
        pose_logits_aug: Optional[torch.Tensor] = None,
        pose_logits_clean: Optional[torch.Tensor] = None,
        gate_raws_clean: Optional[List[torch.Tensor]] = None,
        router_probs: Optional[torch.Tensor] = None,
        **legacy_kwargs,
    ) -> Dict[str, torch.Tensor]:
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
        if router_probs is None:
            router_probs = legacy_kwargs.get("router_probs")
        if z is None or i_aug is None:
            raise ValueError("PRIMLoss requires z and i_aug tensors.")

        zero = torch.zeros((), device=z.device, dtype=z.dtype)
        num_scales = max(len(f_aug_primes), 1)
        losses: Dict[str, torch.Tensor] = {}

        if f_aug_primes and f_cleans:
            losses["ssl"] = self.lambda_ssl * sum(self.ssl_loss(f_aug, f_clean) for f_aug, f_clean in zip(f_aug_primes, f_cleans)) / num_scales
            losses["hist"] = self.lambda_hist * sum(self.hist_loss(f_aug, f_clean) for f_aug, f_clean in zip(f_aug_primes, f_cleans)) / num_scales
        else:
            losses["ssl"] = zero
            losses["hist"] = zero

        losses["smooth"] = self.lambda_smooth * self.smooth_loss(z, i_aug)
        if delta_gammas and delta_betas:
            losses["tv"] = self.lambda_tv * sum(self.tv_loss(dg) + self.tv_loss(db) for dg, db in zip(delta_gammas, delta_betas)) / len(delta_gammas)
        else:
            losses["tv"] = zero

        if gate_raws_clean and self.lambda_identity > 0:
            losses["identity"] = self.lambda_identity * self.identity_loss(gate_raws_clean)
        else:
            losses["identity"] = zero

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
                        return zero
                    return sum(_l1_recursive(a[k], c[k]) for k in keys) / len(keys)
                return zero

            losses["kd"] = self.lambda_kd * _l1_recursive(pose_logits_aug, pose_logits_clean)
        else:
            losses["kd"] = zero

        losses["total"] = sum(losses.values())
        return losses


class PRIMPose(nn.Module):
    """YOLO26 pose wrapper with routed FiLM and pyramid fusion."""

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
        self.feature_spec = infer_prim_feature_spec(self.yolo_model, feature_source=feature_source, feature_layers=feature_layers)
        self.target_layers = list(self.feature_spec["target_layers"])
        self.channels_list = list(self.feature_spec["channels"])
        self.feature_source = str(self.feature_spec["feature_source"])

        illumination_channels = 64
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(3, illumination_channels)
        self.soft_router = SpatiallyAwareSoftRouter(illumination_channels, num_experts)
        self.multi_scale_rsv = MultiScaleRSVFiLM(self.channels_list, illumination_channels, num_experts)
        self.feature_pyramid = PRIMFeaturePyramid(self.channels_list, context_channels=128)
        self.criterion = PRIMLoss(phase=1)

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
        for teacher_param, student_param in zip(self.teacher_backbone.parameters(), self.yolo_model.parameters()):
            teacher_param.data.mul_(momentum).add_(student_param.data, alpha=1.0 - momentum)

    def _apply_prim_modules(
        self,
        target_feats: List[torch.Tensor],
        modulate: bool,
        z: Optional[torch.Tensor] = None,
        p: Optional[torch.Tensor] = None,
    ):
        if not modulate:
            # Clean/reference features must remain on the plain YOLO path.
            return target_feats, [], [], []

        if z is None or p is None:
            raise ValueError("PRIM modulation requires illumination features and router weights.")

        modulated_feats = []
        delta_gammas, delta_betas, gate_raws = [], [], []
        for idx, feat in enumerate(target_feats):
            feat_m, delta_gamma, delta_beta, gate_raw = self.multi_scale_rsv.forward_single(feat, idx, z, p)
            modulated_feats.append(feat_m)
            delta_gammas.append(delta_gamma)
            delta_betas.append(delta_beta)
            gate_raws.append(gate_raw)

        fused_feats = self.feature_pyramid(modulated_feats)
        return fused_feats, delta_gammas, delta_betas, gate_raws

    def _run_backbone(self, x, modulate: bool = False, z=None, p=None, backbone=None):
        if backbone is None:
            backbone = self.yolo_model

        y = []
        f_targets, delta_gammas, delta_betas, gate_raws = [], [], [], []
        pose_logits = None
        target_cache: Dict[int, torch.Tensor] = {}

        for idx, layer in enumerate(backbone):
            if layer.f != -1:
                if isinstance(layer.f, int):
                    x = y[layer.f]
                else:
                    x = [x if j == -1 else y[j] for j in layer.f]

            if idx == len(backbone) - 1:
                pose_logits = [feat.clone() for feat in x] if isinstance(x, list) else x.clone()

            x = layer(x)
            if idx in self.target_layers:
                target_cache[idx] = x
                if idx == self.target_layers[-1]:
                    feats = [target_cache[layer_idx] for layer_idx in self.target_layers]
                    fused_feats, delta_gammas, delta_betas, gate_raws = self._apply_prim_modules(feats, modulate=modulate, z=z, p=p)
                    for layer_idx, feat_new in zip(self.target_layers, fused_feats):
                        target_cache[layer_idx] = feat_new
                        if layer_idx < len(y):
                            y[layer_idx] = feat_new
                    x = target_cache[idx]
                    f_targets = fused_feats
            y.append(x)

        return f_targets, delta_gammas, delta_betas, gate_raws, pose_logits, x

    def forward(self, i_aug=None, i_clean=None, **legacy_kwargs):
        if i_aug is None:
            i_aug = legacy_kwargs.get("I_aug")
        if i_clean is None:
            i_clean = legacy_kwargs.get("I_clean")
        if i_aug is None:
            raise ValueError("PRIMPose.forward requires i_aug (or legacy I_aug).")

        z = self.illumination_encoder(i_aug)
        p = self.soft_router(z)
        outputs = {"Z": z, "P": p}

        if i_clean is not None:
            teacher = self.teacher_backbone if self.teacher_backbone is not None else self.yolo_model
            with torch.no_grad():
                f_cleans, _, _, _, pose_logits_clean, x_clean = self._run_backbone(i_clean, modulate=False, backbone=teacher)
            outputs["F_cleans"] = f_cleans
            outputs["pose_logits_clean"] = pose_logits_clean
            outputs["x_clean"] = x_clean

            z_clean = self.illumination_encoder(i_clean)
            p_clean = self.soft_router(z_clean)
            _, _, _, gate_raws_clean, _, _ = self._run_backbone(i_clean, modulate=True, z=z_clean, p=p_clean)
            outputs["gate_raws_clean"] = gate_raws_clean

        f_primes, delta_gammas, delta_betas, gate_raws, pose_logits_aug, x_aug = self._run_backbone(
            i_aug,
            modulate=True,
            z=z,
            p=p,
        )
        outputs["F_primes"] = f_primes
        outputs["delta_gammas"] = delta_gammas
        outputs["delta_betas"] = delta_betas
        outputs["gate_raws"] = gate_raws
        outputs["pose_logits_aug"] = pose_logits_aug
        outputs["x_aug"] = x_aug
        return outputs
