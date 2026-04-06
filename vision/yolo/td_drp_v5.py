import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional, Tuple, Dict

from vision.yolo.td_drp_v4 import (
    SpatiallyVariantIlluminationEncoder,
    SpatiallyAwareSoftRouter,
    FeatureConsistencySSLLoss,
    KDESoftHistogramLoss,
    IlluminationSmoothLoss,
    TotalVariationLoss,
    ExpertOrthogonalityLoss
)

class MultiScaleRSVFiLM(nn.Module):
    """
    다중 스케일(P3, P4, P5) 특징 맵을 위한 RSV-FiLM 모듈의 집합
    """
    def __init__(self, channels_list=[128, 128, 256], illumination_channels=64, num_experts=3):
        super().__init__()
        self.num_scales = len(channels_list)
        
        self.expert_gamma_list = nn.ModuleList()
        self.expert_beta_list = nn.ModuleList()
        
        for c in channels_list:
            gamma_experts = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            beta_experts = nn.ModuleList([nn.Conv2d(illumination_channels, c, 1) for _ in range(num_experts)])
            
            for expert_g, expert_b in zip(gamma_experts, beta_experts):
                nn.init.zeros_(expert_g.weight)
                nn.init.zeros_(expert_g.bias)
                nn.init.zeros_(expert_b.weight)
                nn.init.zeros_(expert_b.bias)
                
            self.expert_gamma_list.append(gamma_experts)
            self.expert_beta_list.append(beta_experts)
            
        self.num_experts = num_experts

    def forward_single(self, feat: torch.Tensor, scale_idx: int, Z: torch.Tensor, P: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        B, C_feat, H_f, W_f = feat.shape
        
        Z_up = F.interpolate(Z, size=(H_f, W_f), mode="bilinear", align_corners=False)
        P_up = F.interpolate(P, size=(H_f, W_f), mode="bilinear", align_corners=False)
        
        delta_gamma = torch.zeros(B, C_feat, H_f, W_f, device=feat.device, dtype=feat.dtype)
        delta_beta  = torch.zeros(B, C_feat, H_f, W_f, device=feat.device, dtype=feat.dtype)
        
        for k in range(self.num_experts):
            g_k = self.expert_gamma_list[scale_idx][k](Z_up)
            b_k = self.expert_beta_list[scale_idx][k](Z_up)
            w_k = P_up[:, k:k+1, :, :]
            
            delta_gamma = delta_gamma + w_k * g_k
            delta_beta  = delta_beta  + w_k * b_k
            
        F_prime = feat * (1.0 + delta_gamma) + delta_beta
        return F_prime, delta_gamma, delta_beta

    def forward(self, feats: list, Z: torch.Tensor, P: torch.Tensor) -> Tuple[list, list, list]:
        F_primes = []
        delta_gammas = []
        delta_betas = []
        
        for i, feat in enumerate(feats):
            B, C_feat, H_f, W_f = feat.shape
            
            Z_up = F.interpolate(Z, size=(H_f, W_f), mode="bilinear", align_corners=False)
            P_up = F.interpolate(P, size=(H_f, W_f), mode="bilinear", align_corners=False)
            
            delta_gamma = torch.zeros(B, C_feat, H_f, W_f, device=feat.device, dtype=feat.dtype)
            delta_beta  = torch.zeros(B, C_feat, H_f, W_f, device=feat.device, dtype=feat.dtype)
            
            for k in range(self.num_experts):
                g_k = self.expert_gamma_list[i][k](Z_up)
                b_k = self.expert_beta_list[i][k](Z_up)
                w_k = P_up[:, k:k+1, :, :]
                
                delta_gamma = delta_gamma + w_k * g_k
                delta_beta  = delta_beta  + w_k * b_k
                
            F_prime = feat * (1.0 + delta_gamma) + delta_beta
            
            F_primes.append(F_prime)
            delta_gammas.append(delta_gamma)
            delta_betas.append(delta_beta)
            
        return F_primes, delta_gammas, delta_betas

class TDDRPv5Loss(nn.Module):
    """
    Multi-scale 특징 맵 및 KD(Knowledge Distillation) Loss를 포함한 v5 손실 함수
    """
    PHASE_WEIGHTS = {
        1: dict(kd=0.0, ssl=1.0, hist=1.0, smooth=1.0, tv=0.1, orth=0.1),
        2: dict(kd=1.0, ssl=0.5, hist=0.3, smooth=0.3, tv=0.1, orth=0.1), # Phase 2부터 KD Loss 개입
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
        F_aug_primes: list,
        F_cleans: list,
        Z: torch.Tensor,
        I_aug: torch.Tensor,
        delta_gammas: list,
        delta_betas: list,
        expert_weights_list: list,
        pose_logits_aug: Optional[torch.Tensor] = None,
        pose_logits_clean: Optional[torch.Tensor] = None,
    ):
        losses = {}
        
        # SSL, Hist는 Multi-scale 평균으로 계산
        ssl_val = sum(self.ssl_loss(f_aug, f_cln) for f_aug, f_cln in zip(F_aug_primes, F_cleans)) / len(F_aug_primes)
        hist_val = sum(self.hist_loss(f_aug, f_cln) for f_aug, f_cln in zip(F_aug_primes, F_cleans)) / len(F_aug_primes)
        
        losses["ssl"] = self.lambda_ssl * ssl_val
        losses["hist"] = self.lambda_hist * hist_val
        losses["smooth"] = self.lambda_smooth * self.smooth_loss(Z, I_aug)
        
        tv_val = sum(self.tv_loss(dg) + self.tv_loss(db) for dg, db in zip(delta_gammas, delta_betas))
        losses["tv"] = self.lambda_tv * tv_val
        
        orth_val = sum(self.orth_loss(ew) for ew in expert_weights_list)
        losses["orth"] = self.lambda_orth * orth_val
        
        # KD Loss (Logit-level distillation: Teacher의 출력을 Student가 모방)
        if self.lambda_kd > 0 and pose_logits_aug is not None and pose_logits_clean is not None:
            def _calc_mse(a, c):
                if isinstance(a, torch.Tensor) and isinstance(c, torch.Tensor):
                    return F.mse_loss(a, c.detach())
                elif isinstance(a, (list, tuple)) and isinstance(c, (list, tuple)):
                    return sum(_calc_mse(ia, ic) for ia, ic in zip(a, c))
                elif isinstance(a, dict) and isinstance(c, dict):
                    return sum(_calc_mse(a[k], c[k]) for k in a.keys() if k in c)
                return torch.tensor(0.0, device=Z.device)
                
            kd_val = _calc_mse(pose_logits_aug, pose_logits_clean)
            losses["kd"] = self.lambda_kd * kd_val
        else:
            losses["kd"] = torch.tensor(0.0, device=Z.device)

        losses["total"] = sum(losses.values())
        return losses

class TDDRPEndToEndPoseV5(nn.Module):
    """
    YOLO26n-pose P3, P4, P5 Multi-scale TD-DRP v5 모델
    """
    def __init__(self, yolo_model, num_experts=3):
        super().__init__()
        self.yolo_model = yolo_model.model.model
        self.save_list = yolo_model.model.save
        
        # P3(4), P4(6), P5(10) 채널
        self.target_layers = [4, 6, 10]
        channels_list = [128, 128, 256] 
        illumination_channels = 64
        
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(3, illumination_channels)
        self.soft_router = SpatiallyAwareSoftRouter(illumination_channels, num_experts)
        self.multi_scale_rsv = MultiScaleRSVFiLM(channels_list, illumination_channels, num_experts)
        
        self.criterion = TDDRPv5Loss(phase=1)

    def _freeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = False

    def _unfreeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = True

    def set_phase(self, phase: int):
        self.criterion.set_phase(phase)
        if phase == 1:
            self._freeze_yolo()
        else:
            self._unfreeze_yolo()

    def get_expert_weights(self):
        weights_list = []
        for i in range(self.multi_scale_rsv.num_scales):
            weights = []
            for expert in self.multi_scale_rsv.expert_gamma_list[i]:
                weights.append(expert.weight.reshape(-1))
            weights_list.append(torch.stack(weights, dim=0))
        return weights_list

    def forward(self, I_aug, I_clean=None):
        outputs = {}
        
        Z = self.illumination_encoder(I_aug)
        P = self.soft_router(Z)
        outputs["Z"] = Z
        outputs["P"] = P
        
        # Teacher (I_clean)
        if I_clean is not None:
            with torch.no_grad():
                x_c = I_clean
                y_c = []
                F_cleans = []
                for i, m in enumerate(self.yolo_model):
                    if m.f != -1:
                        x_c = y_c[m.f] if isinstance(m.f, int) else [x_c if j == -1 else y_c[j] for j in m.f]
                    
                    if i == len(self.yolo_model) - 1:
                        # Capture input to the last layer (head) for distillation
                        outputs["pose_logits_clean"] = [f.clone() for f in x_c] if isinstance(x_c, list) else x_c.clone()
                        
                    x_c = m(x_c)
                    y_c.append(x_c if m.i in self.save_list else None)
                    if i in self.target_layers:
                        F_cleans.append(x_c)
                outputs["F_cleans"] = F_cleans
        
        # Student (I_aug)
        x_a = I_aug
        y_a = []
        F_augs = []
        
        target_idx = 0
        delta_gammas = []
        delta_betas = []
        F_primes = []
        
        for i, m in enumerate(self.yolo_model):
            if m.f != -1:
                x_a = y_a[m.f] if isinstance(m.f, int) else [x_a if j == -1 else y_a[j] for j in m.f]
            
            if i == len(self.yolo_model) - 1:
                # Capture input to the last layer (head) for distillation
                outputs["pose_logits_aug"] = x_a 
                
            x_a = m(x_a)
            
            if i in self.target_layers:
                # Modulate this specific layer
                idx = self.target_layers.index(i)
                f_p, dg, db = self.multi_scale_rsv.forward_single(x_a, idx, Z, P)
                x_a = f_p # replace with modulated
                
                F_primes.append(x_a)
                delta_gammas.append(dg)
                delta_betas.append(db)
                
            y_a.append(x_a if m.i in self.save_list else None)
            
        outputs["F_primes"] = F_primes
        outputs["delta_gammas"] = delta_gammas
        outputs["delta_betas"] = delta_betas
        
        return outputs
