"""
TD-DRP (Task-Driven Dynamic Routing with Feature-Consistency SSL)
통합 모듈: 기본 블록 + Multi-scale (P3, P4, P5) 확장 + End-to-End Pose Wrapper

Backbone: YOLO26n-pose
- P3(layer 4, 128ch), P4(layer 6, 128ch), P5(layer 10, 256ch)
"""
import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional, Tuple, List


# ==============================================================================
# Core Building Blocks
# ==============================================================================

class SpatiallyVariantIlluminationEncoder(nn.Module):
    """RGB 입력으로부터 공간적으로 변하는 조명 임베딩 Z를 추출"""
    def __init__(self, in_channels=3, out_channels=64):
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
    """조명 임베딩 Z로부터 전문가 라우팅 가중치 P를 생성 (softmax)"""
    def __init__(self, illumination_channels=64, num_experts=3):
        super().__init__()
        self.router = nn.Sequential(
            nn.Conv2d(illumination_channels, illumination_channels, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(illumination_channels, num_experts, 1),
        )

    def forward(self, Z: torch.Tensor) -> torch.Tensor:
        return F.softmax(self.router(Z), dim=1)


class MultiScaleRSVFiLM(nn.Module):
    """
    다중 스케일(P3, P4, P5) RSV-FiLM 모듈
    각 스케일별 expert gamma/beta를 라우팅 가중치 P로 혼합하여 feature modulation
    """
    def __init__(self, channels_list=[128, 128, 256], illumination_channels=64, num_experts=3):
        super().__init__()
        self.num_scales = len(channels_list)
        self.num_experts = num_experts

        self.expert_gamma_list = nn.ModuleList()
        self.expert_beta_list = nn.ModuleList()

        for c in channels_list:
            gammas = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            betas  = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            for g, b in zip(gammas, betas):
                nn.init.zeros_(g.weight); nn.init.zeros_(g.bias)
                nn.init.zeros_(b.weight); nn.init.zeros_(b.bias)
            self.expert_gamma_list.append(gammas)
            self.expert_beta_list.append(betas)

    def forward_single(self, feat: torch.Tensor, scale_idx: int,
                        Z: torch.Tensor, P: torch.Tensor
                        ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        B, C, H, W = feat.shape
        Z_up = F.interpolate(Z, size=(H, W), mode="bilinear", align_corners=False)
        P_up = F.interpolate(P, size=(H, W), mode="bilinear", align_corners=False)

        delta_gamma = torch.zeros(B, C, H, W, device=feat.device, dtype=feat.dtype)
        delta_beta  = torch.zeros(B, C, H, W, device=feat.device, dtype=feat.dtype)

        for k in range(self.num_experts):
            g_k = self.expert_gamma_list[scale_idx][k](Z_up)
            b_k = self.expert_beta_list[scale_idx][k](Z_up)
            w_k = P_up[:, k:k+1, :, :]
            delta_gamma = delta_gamma + w_k * g_k
            delta_beta  = delta_beta  + w_k * b_k

        F_prime = feat * (1.0 + delta_gamma) + delta_beta
        return F_prime, delta_gamma, delta_beta


# ==============================================================================
# Loss Functions
# ==============================================================================

class FeatureConsistencySSLLoss(nn.Module):
    """Clean feature와 Augmented-modulated feature 간 일관성 (MSE)"""
    def forward(self, F_aug_prime: torch.Tensor, F_clean: torch.Tensor) -> torch.Tensor:
        return F.mse_loss(F_aug_prime, F_clean.detach())


class KDESoftHistogramLoss(nn.Module):
    """KDE 기반 소프트 히스토그램 분포 정합 손실"""
    def __init__(self, num_bins=64, sigma=0.1):
        super().__init__()
        self.num_bins = num_bins
        self.sigma = sigma
        self.register_buffer("centers", torch.linspace(0, 1, num_bins))

    def _soft_hist(self, x: torch.Tensor) -> torch.Tensor:
        x_flat = x.reshape(-1, 1)
        diff = x_flat - self.centers.unsqueeze(0)
        weights = torch.exp(-0.5 * (diff / self.sigma) ** 2)
        hist = weights.sum(dim=0)
        return hist / (hist.sum() + 1e-8)

    def forward(self, F_aug_prime: torch.Tensor, F_clean: torch.Tensor) -> torch.Tensor:
        return F.mse_loss(self._soft_hist(F_aug_prime), self._soft_hist(F_clean.detach()))


class IlluminationSmoothLoss(nn.Module):
    """조명 맵 Z의 공간적 부드러움"""
    def forward(self, Z: torch.Tensor, I_aug: torch.Tensor) -> torch.Tensor:
        grad_x = torch.abs(Z[:, :, :, :-1] - Z[:, :, :, 1:])
        grad_y = torch.abs(Z[:, :, :-1, :] - Z[:, :, 1:, :])
        return (grad_x.mean() + grad_y.mean()) * 0.5


class TotalVariationLoss(nn.Module):
    """Total Variation 정규화"""
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return torch.abs(x[:, :, 1:, :] - x[:, :, :-1, :]).mean() + \
               torch.abs(x[:, :, :, 1:] - x[:, :, :, :-1]).mean()


class ExpertOrthogonalityLoss(nn.Module):
    """전문가 가중치 간 직교성 촉진 (다양성 보장)"""
    def forward(self, expert_weights: torch.Tensor) -> torch.Tensor:
        normed = F.normalize(expert_weights, dim=1)
        gram = torch.mm(normed, normed.t())
        return F.mse_loss(gram, torch.eye(gram.size(0), device=gram.device))


class TDDRPLoss(nn.Module):
    """
    Multi-scale TD-DRP Loss with KD (Knowledge Distillation)
    Phase 1: SSL only (backbone frozen, 조명 어댑터만 학습)
    Phase 2: SSL + KD (backbone unfrozen, logit distillation)
    """
    PHASE_WEIGHTS = {
        1: dict(kd=0.0, ssl=1.0, hist=1.0, smooth=1.0, tv=0.1, orth=0.1),
        2: dict(kd=1.0, ssl=0.5, hist=0.3, smooth=0.3, tv=0.1, orth=0.1),
    }

    def __init__(self, phase: int = 1):
        super().__init__()
        self.ssl_loss = FeatureConsistencySSLLoss()
        self.hist_loss = KDESoftHistogramLoss()
        self.smooth_loss = IlluminationSmoothLoss()
        self.tv_loss = TotalVariationLoss()
        self.orth_loss = ExpertOrthogonalityLoss()
        self.set_phase(phase)

    def set_phase(self, phase: int):
        self.phase = phase
        w = self.PHASE_WEIGHTS.get(phase, self.PHASE_WEIGHTS[2])
        self.lambda_kd = w["kd"]
        self.lambda_ssl = w["ssl"]
        self.lambda_hist = w["hist"]
        self.lambda_smooth = w["smooth"]
        self.lambda_tv = w["tv"]
        self.lambda_orth = w["orth"]

    def forward(
        self,
        F_aug_primes: List[torch.Tensor],
        F_cleans: List[torch.Tensor],
        Z: torch.Tensor,
        I_aug: torch.Tensor,
        delta_gammas: List[torch.Tensor],
        delta_betas: List[torch.Tensor],
        expert_weights_list: List[torch.Tensor],
        pose_logits_aug: Optional[torch.Tensor] = None,
        pose_logits_clean: Optional[torch.Tensor] = None,
    ):
        losses = {}
        n = len(F_aug_primes)

        # SSL, Hist: multi-scale 평균
        losses["ssl"] = self.lambda_ssl * sum(
            self.ssl_loss(fa, fc) for fa, fc in zip(F_aug_primes, F_cleans)) / n
        losses["hist"] = self.lambda_hist * sum(
            self.hist_loss(fa, fc) for fa, fc in zip(F_aug_primes, F_cleans)) / n
        losses["smooth"] = self.lambda_smooth * self.smooth_loss(Z, I_aug)
        losses["tv"] = self.lambda_tv * sum(
            self.tv_loss(dg) + self.tv_loss(db) for dg, db in zip(delta_gammas, delta_betas))
        losses["orth"] = self.lambda_orth * sum(self.orth_loss(ew) for ew in expert_weights_list)

        # KD Loss (Phase 2)
        if self.lambda_kd > 0 and pose_logits_aug is not None and pose_logits_clean is not None:
            def _mse_recursive(a, c):
                if isinstance(a, torch.Tensor) and isinstance(c, torch.Tensor):
                    return F.mse_loss(a, c.detach())
                elif isinstance(a, (list, tuple)) and isinstance(c, (list, tuple)):
                    return sum(_mse_recursive(ia, ic) for ia, ic in zip(a, c))
                elif isinstance(a, dict) and isinstance(c, dict):
                    return sum(_mse_recursive(a[k], c[k]) for k in a if k in c)
                return torch.tensor(0.0, device=Z.device)
            losses["kd"] = self.lambda_kd * _mse_recursive(pose_logits_aug, pose_logits_clean)
        else:
            losses["kd"] = torch.tensor(0.0, device=Z.device)

        losses["total"] = sum(losses.values())
        return losses


# ==============================================================================
# Illumination Augmentation
# ==============================================================================

class IlluminationAugmentor:
    """조명 증강: 저조도/역광/채도 변화/감마 시뮬레이션"""

    @staticmethod
    def augment(I_clean: torch.Tensor) -> Tuple[torch.Tensor, str]:
        B = I_clean.shape[0]
        device = I_clean.device
        aug_type = torch.randint(0, 4, (1,)).item()

        if aug_type == 0:
            factor = 0.1 + 0.4 * torch.rand(B, 1, 1, 1, device=device)
            I_aug = I_clean * factor
            name = "dark"
        elif aug_type == 1:
            _, _, H, W = I_clean.shape
            grad = torch.linspace(0.2, 1.0, W, device=device).view(1, 1, 1, W).expand(B, 1, H, W)
            I_aug = I_clean * grad
            name = "gradient"
        elif aug_type == 2:
            gray = I_clean.mean(dim=1, keepdim=True)
            alpha = 0.1 + 0.5 * torch.rand(B, 1, 1, 1, device=device)
            I_aug = alpha * I_clean + (1 - alpha) * gray
            name = "desaturate"
        else:
            gamma = 1.5 + 1.5 * torch.rand(B, 1, 1, 1, device=device)
            I_aug = torch.clamp(I_clean, 0, 1).pow(gamma)
            name = "gamma"

        return torch.clamp(I_aug, 0, 1), name


# ==============================================================================
# End-to-End Pose Wrapper (Multi-scale P3, P4, P5)
# ==============================================================================

class TDDRPEndToEndPose(nn.Module):
    """
    YOLO26n-pose + Multi-scale TD-DRP
    P3(layer 4), P4(layer 6), P5(layer 10) 에서 조명 적응적 feature modulation 수행
    """
    TARGET_LAYERS = [4, 6, 10]
    CHANNELS = [128, 128, 256]

    def __init__(self, yolo_model, num_experts=3):
        super().__init__()
        self.yolo_model = yolo_model.model.model
        self.save_list = yolo_model.model.save

        illumination_channels = 64
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(3, illumination_channels)
        self.soft_router = SpatiallyAwareSoftRouter(illumination_channels, num_experts)
        self.multi_scale_rsv = MultiScaleRSVFiLM(self.CHANNELS, illumination_channels, num_experts)
        self.criterion = TDDRPLoss(phase=1)

    def _freeze_yolo(self):
        for p in self.yolo_model.parameters():
            p.requires_grad = False

    def _unfreeze_yolo(self):
        for p in self.yolo_model.parameters():
            p.requires_grad = True

    def set_phase(self, phase: int):
        self.criterion.set_phase(phase)
        self._freeze_yolo() if phase == 1 else self._unfreeze_yolo()

    def get_expert_weights(self) -> List[torch.Tensor]:
        weights_list = []
        for i in range(self.multi_scale_rsv.num_scales):
            weights = [e.weight.reshape(-1) for e in self.multi_scale_rsv.expert_gamma_list[i]]
            weights_list.append(torch.stack(weights, dim=0))
        return weights_list

    def _run_backbone(self, x, modulate=False, Z=None, P=None):
        """YOLO 백본 순전파. modulate=True면 target layer에서 RSV-FiLM 적용"""
        y = []
        F_targets = []
        delta_gammas = []
        delta_betas = []
        pose_logits = None

        for i, m in enumerate(self.yolo_model):
            if m.f != -1:
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]

            # 마지막 레이어(Head) 입력 캡처 (KD용)
            if i == len(self.yolo_model) - 1:
                pose_logits = [f.clone() for f in x] if isinstance(x, list) else x.clone()

            x = m(x)

            if modulate and i in self.TARGET_LAYERS:
                idx = self.TARGET_LAYERS.index(i)
                x, dg, db = self.multi_scale_rsv.forward_single(x, idx, Z, P)
                F_targets.append(x)
                delta_gammas.append(dg)
                delta_betas.append(db)
            elif i in self.TARGET_LAYERS:
                F_targets.append(x)

            y.append(x if m.i in self.save_list else None)

        return F_targets, delta_gammas, delta_betas, pose_logits

    def forward(self, I_aug, I_clean=None):
        Z = self.illumination_encoder(I_aug)
        P = self.soft_router(Z)

        outputs = {"Z": Z, "P": P}

        # Teacher (clean)
        if I_clean is not None:
            with torch.no_grad():
                F_cleans, _, _, pose_logits_clean = self._run_backbone(I_clean, modulate=False)
            outputs["F_cleans"] = F_cleans
            outputs["pose_logits_clean"] = pose_logits_clean

        # Student (aug + modulation)
        F_primes, delta_gammas, delta_betas, pose_logits_aug = self._run_backbone(
            I_aug, modulate=True, Z=Z, P=P)
        outputs["F_primes"] = F_primes
        outputs["delta_gammas"] = delta_gammas
        outputs["delta_betas"] = delta_betas
        outputs["pose_logits_aug"] = pose_logits_aug

        return outputs
