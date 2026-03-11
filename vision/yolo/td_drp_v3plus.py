"""
TD-DRP v3+ (Task-Driven Dynamic Routing with Feature-Consistency SSL)
=====================================================================
조명 변화 강건 비전 모델 - 산업용 이중 구멍 인식 특화

아키텍처 구성:
  1. SpatiallyVariantIlluminationEncoder  - 공간별 조명 임베딩 맵 Z 생성
  2. SpatiallyAwareSoftRouter             - K=3 Expert 가중치 맵 P 생성
  3. RSVFiLM                              - 잔차 기반 특징 맵 변조
  4. TDDRPv3Plus                          - 전체 파이프라인 통합

손실 함수:
  - L_YOLO    : Task-Driven Segmentation Loss
  - L_SSL     : Feature-Consistency (Teacher-Student MSE)
  - L_Hist    : DHURE 기반 히스토그램 정규화
  - L_Smooth  : 조명 맵 공간 스무스 정규화
  - L_TV      : Total Variation 정규화
  - L_Orth    : Expert Orthogonality 정규화

학습 전략:
  Phase 1 - SSL Warm-up    : L_SSL + L_Hist + L_Smooth
  Phase 2 - Task Tuning    : L_YOLO + L_SSL + all regularizers
  Phase 3 - Few-shot       : L_total (전체, 낮은 LR)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional, Tuple, Dict


# ─────────────────────────────────────────────────────────────────────────────
# 1. Spatially-Variant Illumination Encoder
# ─────────────────────────────────────────────────────────────────────────────

class SpatiallyVariantIlluminationEncoder(nn.Module):
    """
    입력 이미지에서 공간별 조명 특징을 추출합니다.

    GAP(Global Average Pooling)를 사용하지 않아 "어디가 밝고 어두운지"에 대한
    공간 정보를 그대로 보존합니다.

    Args:
        in_channels  : 입력 이미지 채널 수 (기본값: 3, RGB)
        out_channels : 출력 임베딩 채널 수 (기본값: 64)

    Input:  I  ∈ R^{B × 3 × H × W}
    Output: Z  ∈ R^{B × 64 × H' × W'}
    """

    def __init__(self, in_channels: int = 3, out_channels: int = 64):
        super().__init__()
        mid = out_channels // 2  # 32

        self.encoder = nn.Sequential(
            # Layer 1: 저수준 조명 엣지 감지
            nn.Conv2d(in_channels, mid, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(mid),
            nn.ReLU(inplace=True),

            # Layer 2: 중간 조명 패턴 통합
            nn.Conv2d(mid, mid, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(mid),
            nn.ReLU(inplace=True),

            # Layer 3: 고수준 조명 임베딩 생성
            # NOTE: GAP 없음 - 공간 구조 보존이 핵심
            nn.Conv2d(mid, out_channels, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.encoder(x)  # Z ∈ R^{B × 64 × H/4 × W/4}


# ─────────────────────────────────────────────────────────────────────────────
# 2. Spatially-Aware Soft Router
# ─────────────────────────────────────────────────────────────────────────────

class SpatiallyAwareSoftRouter(nn.Module):
    """
    조명 임베딩 맵 Z로부터 K개 Expert의 공간별 가중치 맵 P를 생성합니다.

    K=3 Expert:
      - Expert 0: 저조도 (Underexposure) 전문
      - Expert 1: 과노출 (Overexposure) 전문
      - Expert 2: 정상 조명 (Normal) 전문

    Hard routing(argmax) 대신 Soft routing(weighted sum)을 사용하여
    조명 경계(Transition zone)에서의 제어 안정성을 확보합니다.

    Args:
        in_channels : 조명 임베딩 채널 수 (기본값: 64)
        num_experts : Expert 수 K (기본값: 3)

    Input:  Z ∈ R^{B × 64 × H' × W'}
    Output: P ∈ R^{B × K × H' × W'}  (각 위치별 Expert 확률 분포)
    """

    def __init__(self, in_channels: int = 64, num_experts: int = 3):
        super().__init__()
        self.num_experts = num_experts

        # 1×1 Conv: 채널 축소 후 Expert 확률 맵 생성 (공간 구조 유지)
        self.router = nn.Sequential(
            nn.Conv2d(in_channels, in_channels // 2, kernel_size=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(in_channels // 2, num_experts, kernel_size=1),
        )

    def forward(self, z: torch.Tensor) -> torch.Tensor:
        logits = self.router(z)           # (B, K, H', W')
        p = F.softmax(logits, dim=1)      # Spatial Softmax: 각 위치별 확률 합=1
        return p                          # P ∈ R^{B × K × H' × W'}


# ─────────────────────────────────────────────────────────────────────────────
# 3. RSV-FiLM (Residual Spatially-Variant FiLM)
# ─────────────────────────────────────────────────────────────────────────────

class RSVFiLM(nn.Module):
    """
    YOLO 특징 맵 F를 조명 조건에 맞게 공간별 변조합니다.

    수식:  F' = F ⊙ (1 + Δγ_final) + Δβ_final

    핵심 설계 원칙:
      - Δγ, Δβ를 0으로 초기화 (Zero-init) → 학습 초기에 항등 변환
      - 잔차(Residual) 구조 → 고정된 YOLO 백본의 사전학습 지식을 보호
      - K개 Expert 각각이 독립적인 Δγ, Δβ를 학습
      - Router P로 Expert 출력을 가중합하여 최종 변조 파라미터 결정

    Args:
        feature_channels : YOLO 특징 맵 채널 수 (기본값: 256)
        illumination_channels : 조명 임베딩 채널 수 (기본값: 64)
        num_experts      : Expert 수 K (기본값: 3)

    Input:
        F : YOLO 특징 맵    ∈ R^{B × C_feat × H_f × W_f}
        Z : 조명 임베딩 맵  ∈ R^{B × 64 × H' × W'}
        P : Router 확률 맵  ∈ R^{B × K × H' × W'}
    Output:
        F' : 변조된 특징 맵 ∈ R^{B × C_feat × H_f × W_f}
    """

    def __init__(
        self,
        feature_channels: int = 256,
        illumination_channels: int = 64,
        num_experts: int = 3,
    ):
        super().__init__()
        self.num_experts = num_experts
        self.feature_channels = feature_channels

        # K개 Expert - 각각 독립적인 Δγ, Δβ 예측기
        self.expert_gamma = nn.ModuleList([
            nn.Conv2d(illumination_channels, feature_channels, kernel_size=1)
            for _ in range(num_experts)
        ])
        self.expert_beta = nn.ModuleList([
            nn.Conv2d(illumination_channels, feature_channels, kernel_size=1)
            for _ in range(num_experts)
        ])

        # Zero-initialization: 학습 초기 항등 변환 보장
        for expert_g, expert_b in zip(self.expert_gamma, self.expert_beta):
            nn.init.zeros_(expert_g.weight)
            nn.init.zeros_(expert_g.bias)
            nn.init.zeros_(expert_b.weight)
            nn.init.zeros_(expert_b.bias)

    def forward(
        self,
        F: torch.Tensor,  # YOLO 특징 맵
        Z: torch.Tensor,  # 조명 임베딩 맵
        P: torch.Tensor,  # Router 확률 맵
    ) -> torch.Tensor:

        B, C_feat, H_f, W_f = F.shape

        # Z, P를 특징 맵 해상도에 맞게 업샘플링
        Z_up = F.interpolate(Z, size=(H_f, W_f), mode="bilinear", align_corners=False)
        P_up = F.interpolate(P, size=(H_f, W_f), mode="bilinear", align_corners=False)

        # K개 Expert의 Δγ, Δβ를 P로 가중합
        delta_gamma = torch.zeros_like(F)  # (B, C_feat, H_f, W_f)
        delta_beta  = torch.zeros_like(F)

        for k in range(self.num_experts):
            g_k = self.expert_gamma[k](Z_up)          # (B, C_feat, H_f, W_f)
            b_k = self.expert_beta[k](Z_up)            # (B, C_feat, H_f, W_f)
            w_k = P_up[:, k:k+1, :, :]                 # (B, 1, H_f, W_f) - 브로드캐스트용

            delta_gamma += w_k * g_k
            delta_beta  += w_k * b_k

        # 잔차 변조: F' = F ⊙ (1 + Δγ) + Δβ
        F_prime = F * (1.0 + delta_gamma) + delta_beta
        return F_prime


# ─────────────────────────────────────────────────────────────────────────────
# 4. 손실 함수 모음
# ─────────────────────────────────────────────────────────────────────────────

class FeatureConsistencySSLLoss(nn.Module):
    """
    Teacher-Student 특징 일관성 자가지도 손실

    수식: L_SSL = MSE(F'_aug, F_clean) = ‖F'_aug - F_clean‖²₂

    동작 원리:
      - Teacher: I_clean → Frozen YOLO → F_clean (정답)
      - Student: I_aug → Encoder → Router → RSV-FiLM → F'_aug (학습 대상)
      - 목표: 조명이 망가진 이미지도 정상 이미지의 특징 맵을 재현하도록 강제

    장점:
      - 라벨(Bounding Box) 없이 학습 가능
      - 수만 시간의 Unlabeled 영상 데이터 활용 가능
      - "예쁜 이미지"가 아닌 "좋은 YOLO 특징"을 복원 목표로 명확히 정의
    """

    def __init__(self):
        super().__init__()

    def forward(
        self,
        F_aug_prime: torch.Tensor,  # Student: 변조된 열화 이미지 특징 맵
        F_clean: torch.Tensor,      # Teacher: 정상 이미지 특징 맵 (Frozen YOLO)
    ) -> torch.Tensor:
        return F.mse_loss(F_aug_prime, F_clean.detach())  # Teacher는 역전파 차단


class DHUREHistogramLoss(nn.Module):
    """
    DHURE 기반 미분 가능 히스토그램 정규화 손실

    보정된 특징의 밝기(채널 평균) 분포가 정상 범위를 벗어나지 않도록 규제합니다.
    구체적으로, 변조 후 특징 맵 F'의 채널 평균이 F_clean의 통계와 일치하도록 유도합니다.

    L_Hist = ‖μ(F'_aug) - μ(F_clean)‖² + ‖σ(F'_aug) - σ(F_clean)‖²
    """

    def __init__(self):
        super().__init__()

    def forward(
        self,
        F_aug_prime: torch.Tensor,  # 변조된 특징 맵 (B, C, H, W)
        F_clean: torch.Tensor,      # 정상 특징 맵    (B, C, H, W)
    ) -> torch.Tensor:
        # 채널별 평균 & 표준편차 계산
        mu_aug   = F_aug_prime.mean(dim=[2, 3])   # (B, C)
        mu_clean = F_clean.mean(dim=[2, 3])

        std_aug   = F_aug_prime.std(dim=[2, 3]) + 1e-8
        std_clean = F_clean.std(dim=[2, 3])     + 1e-8

        loss_mean = F.mse_loss(mu_aug, mu_clean.detach())
        loss_std  = F.mse_loss(std_aug, std_clean.detach())

        return loss_mean + loss_std


class IlluminationSmoothLoss(nn.Module):
    """
    조명 임베딩 맵 Z의 공간적 스무스 정규화 손실

    물리적 근거:
      조명 맵은 객체의 질감(Texture)을 무시하고 부드럽게 이어져야 한다.

    수식:
      L_Smooth = Σ_{x,y} ‖∇Z(x,y)‖² · exp(-λ ‖∇I(x,y)‖²)

    해석:
      - 원본 이미지 I에서 엣지(∇I가 큰 곳)에서는 Z의 불연속 허용
      - 평탄한 영역(∇I가 작은 곳)에서는 Z도 부드럽게 이어지도록 강제
      - Encoder가 객체 무늬에 속지 않고 순수 조명 분포만 인코딩하도록 유도

    Args:
        lambda_edge : 엣지 억제 강도 (기본값: 10.0)
    """

    def __init__(self, lambda_edge: float = 10.0):
        super().__init__()
        self.lambda_edge = lambda_edge

    def _gradient(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """수평/수직 방향 기울기 계산 (유한 차분)"""
        grad_x = x[:, :, :, 1:] - x[:, :, :, :-1]  # 수평
        grad_y = x[:, :, 1:, :] - x[:, :, :-1, :]  # 수직
        return grad_x, grad_y

    def forward(self, Z: torch.Tensor, I: torch.Tensor) -> torch.Tensor:
        # 입력 이미지 I를 Z 해상도로 다운샘플링
        I_down = F.interpolate(I, size=Z.shape[2:], mode="bilinear", align_corners=False)

        # 이미지 그레이스케일 변환 (단순 채널 평균)
        I_gray = I_down.mean(dim=1, keepdim=True)

        # Z 기울기
        gz_x, gz_y = self._gradient(Z)
        gz_norm_x  = (gz_x ** 2).mean(dim=1, keepdim=True)  # 채널 평균
        gz_norm_y  = (gz_y ** 2).mean(dim=1, keepdim=True)

        # I 기울기 (엣지 위치 감지용)
        gi_x, gi_y = self._gradient(I_gray)
        gi_norm_x  = gi_x ** 2
        gi_norm_y  = gi_y ** 2

        # 엣지 억제 가중치: 이미지 엣지 없는 곳에서 Z 변화 억제
        weight_x = torch.exp(-self.lambda_edge * gi_norm_x)
        weight_y = torch.exp(-self.lambda_edge * gi_norm_y)

        loss_x = (gz_norm_x * weight_x).mean()
        loss_y = (gz_norm_y * weight_y).mean()

        return loss_x + loss_y


class TotalVariationLoss(nn.Module):
    """
    변조 파라미터 맵의 공간적 부드러움을 강제합니다.

    L_TV = Σ_{x,y} |Δγ(x+1,y) - Δγ(x,y)| + |Δγ(x,y+1) - Δγ(x,y)|
           + 동일한 항 for Δβ

    조명은 공간적으로 급격히 변하지 않는다는 물리적 사전 지식 적용.
    """

    def __init__(self):
        super().__init__()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        tv_h = (x[:, :, 1:, :] - x[:, :, :-1, :]).abs().mean()
        tv_w = (x[:, :, :, 1:] - x[:, :, :, :-1]).abs().mean()
        return tv_h + tv_w


class ExpertOrthogonalityLoss(nn.Module):
    """
    K개 Expert 간의 역할 분담을 명확히 하기 위한 Orthogonality 손실

    Expert의 가중치 벡터들이 서로 직교(Orthogonal)에 가까울수록
    각 Expert가 독립적인 특성을 학습하도록 유도합니다.

    L_Orth = ‖W_k^T W_k - I_K‖²_F   (Frobenius norm)
    """

    def __init__(self):
        super().__init__()

    def forward(self, expert_weights: torch.Tensor) -> torch.Tensor:
        """
        Args:
            expert_weights : Expert 가중치 행렬 (K, D)
                             K=Expert 수, D=가중치 차원
        """
        K = expert_weights.shape[0]
        # 행 단위 정규화
        w_norm = F.normalize(expert_weights, dim=1)          # (K, D)
        gram   = torch.mm(w_norm, w_norm.t())                # (K, K)
        identity = torch.eye(K, device=expert_weights.device)
        return ((gram - identity) ** 2).sum()


# ─────────────────────────────────────────────────────────────────────────────
# 5. 전체 손실 함수 통합
# ─────────────────────────────────────────────────────────────────────────────

class TDDRPLoss(nn.Module):
    """
    TD-DRP v3+ 전체 손실 함수

    L_total = λ1·L_YOLO + λ2·L_SSL + λ3·L_Hist + λ4·L_Smooth
              + λ5·L_TV + λ6·L_Orth

    Phase별 활성화:
      Phase 1 (SSL Warm-up): λ1=0,  λ2=1, λ3=1, λ4=1, λ5=0.1, λ6=0.1
      Phase 2 (Task Tuning): λ1=1,  λ2=0.5, λ3=0.3, λ4=0.3, λ5=0.1, λ6=0.1
      Phase 3 (Few-shot):    λ1=1,  λ2=0.3, λ3=0.1, λ4=0.1, λ5=0.05, λ6=0.05
    """

    PHASE_WEIGHTS = {
        1: dict(yolo=0.0, ssl=1.0, hist=1.0, smooth=1.0, tv=0.1,  orth=0.1),
        2: dict(yolo=1.0, ssl=0.5, hist=0.3, smooth=0.3, tv=0.1,  orth=0.1),
        3: dict(yolo=1.0, ssl=0.3, hist=0.1, smooth=0.1, tv=0.05, orth=0.05),
    }

    def __init__(self, phase: int = 1):
        super().__init__()
        self.set_phase(phase)

        self.ssl_loss    = FeatureConsistencySSLLoss()
        self.hist_loss   = DHUREHistogramLoss()
        self.smooth_loss = IlluminationSmoothLoss()
        self.tv_loss     = TotalVariationLoss()
        self.orth_loss   = ExpertOrthogonalityLoss()

    def set_phase(self, phase: int):
        assert phase in [1, 2, 3], "phase는 1, 2, 3 중 하나여야 합니다."
        self.phase = phase
        w = self.PHASE_WEIGHTS[phase]
        self.lambda_yolo   = w["yolo"]
        self.lambda_ssl    = w["ssl"]
        self.lambda_hist   = w["hist"]
        self.lambda_smooth = w["smooth"]
        self.lambda_tv     = w["tv"]
        self.lambda_orth   = w["orth"]
        print(f"[TDDRPLoss] Phase {phase} 가중치 설정 완료: {w}")

    def forward(
        self,
        # SSL 관련
        F_aug_prime : torch.Tensor,              # Student 특징 맵
        F_clean     : torch.Tensor,              # Teacher 특징 맵
        # 조명 맵 스무스
        Z           : torch.Tensor,              # 조명 임베딩 맵
        I_aug       : torch.Tensor,              # 열화 이미지 (원본 해상도)
        # 변조 파라미터 (TV 손실용)
        delta_gamma : Optional[torch.Tensor] = None,
        delta_beta  : Optional[torch.Tensor] = None,
        # Expert 가중치 (Orthogonality 손실용)
        expert_weights: Optional[torch.Tensor] = None,
        # YOLO 손실 (Phase 2, 3에서 외부에서 전달)
        yolo_loss   : Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:

        losses = {}

        # L_SSL: Feature-Consistency
        losses["ssl"] = self.lambda_ssl * self.ssl_loss(F_aug_prime, F_clean)

        # L_Hist: DHURE 히스토그램
        losses["hist"] = self.lambda_hist * self.hist_loss(F_aug_prime, F_clean)

        # L_Smooth: 조명 맵 스무스
        losses["smooth"] = self.lambda_smooth * self.smooth_loss(Z, I_aug)

        # L_TV: 변조 파라미터 부드러움
        if delta_gamma is not None and delta_beta is not None:
            tv = self.tv_loss(delta_gamma) + self.tv_loss(delta_beta)
            losses["tv"] = self.lambda_tv * tv
        else:
            losses["tv"] = torch.tensor(0.0)

        # L_Orth: Expert 직교성
        if expert_weights is not None:
            losses["orth"] = self.lambda_orth * self.orth_loss(expert_weights)
        else:
            losses["orth"] = torch.tensor(0.0)

        # L_YOLO: Task-Driven (Phase 2, 3)
        if yolo_loss is not None and self.lambda_yolo > 0:
            losses["yolo"] = self.lambda_yolo * yolo_loss
        else:
            losses["yolo"] = torch.tensor(0.0)

        losses["total"] = sum(losses.values())
        return losses


# ─────────────────────────────────────────────────────────────────────────────
# 6. TDDRPv3Plus 통합 모델
# ─────────────────────────────────────────────────────────────────────────────

class TDDRPv3Plus(nn.Module):
    """
    TD-DRP v3+ 전체 파이프라인

    파이프라인:
      I (입력)
      ├── Illumination Encoder ──→ Z (조명 임베딩 맵)
      │         └──────────────→ Soft Router ──→ P (Expert 가중치 맵)
      │
      ├── Frozen YOLO Backbone ──→ F (원본 특징 맵)
      │
      └── RSV-FiLM(F, Z, P) ──→ F' (조명 보정된 특징 맵)
                └──→ YOLO Head ──→ 구멍 Segmentation 결과

    학습 모드:
      - phase=1: Encoder, Router, RSV-FiLM만 학습 (YOLO 완전 고정)
      - phase=2: YOLO 헤드까지 포함하여 학습
      - phase=3: 전체 파이프라인 낮은 LR로 미세 조정

    Args:
        feature_channels    : YOLO 특징 맵 채널 수 (기본값: 256)
        illumination_channels: 조명 임베딩 채널 수 (기본값: 64)
        num_experts         : Router Expert 수 (기본값: 3)
        yolo_backbone       : 사용할 YOLO 백본 (None이면 더미 백본 사용)
    """

    def __init__(
        self,
        feature_channels: int = 256,
        illumination_channels: int = 64,
        num_experts: int = 3,
        yolo_backbone: Optional[nn.Module] = None,
    ):
        super().__init__()

        # ── 조명 어댑터 모듈 (학습 가능)
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(
            in_channels=3,
            out_channels=illumination_channels,
        )
        self.soft_router = SpatiallyAwareSoftRouter(
            in_channels=illumination_channels,
            num_experts=num_experts,
        )
        self.rsv_film = RSVFiLM(
            feature_channels=feature_channels,
            illumination_channels=illumination_channels,
            num_experts=num_experts,
        )

        # ── YOLO 백본 (기본: 더미 백본, 실제 사용 시 교체)
        if yolo_backbone is not None:
            self.yolo_backbone = yolo_backbone
        else:
            # 프로토타입용 더미 백본 (Conv stride=4로 H/4×W/4 출력)
            self.yolo_backbone = nn.Sequential(
                nn.Conv2d(3, feature_channels // 2, 7, stride=2, padding=3),
                nn.BatchNorm2d(feature_channels // 2),
                nn.ReLU(inplace=True),
                nn.Conv2d(feature_channels // 2, feature_channels, 3, stride=2, padding=1),
                nn.BatchNorm2d(feature_channels),
                nn.ReLU(inplace=True),
            )

        # Phase 1에서 YOLO 백본 고정
        self._freeze_yolo()

        # ── YOLO 헤드 (세그멘테이션 출력)
        # 실제 구현 시 YOLOv8-seg 헤드로 교체
        self.yolo_head = nn.Sequential(
            nn.Conv2d(feature_channels, feature_channels // 2, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(feature_channels // 2, 2, 1),  # 2 클래스: 구멍 내부 / 외부 경계
        )

        # ── 손실 함수
        self.criterion = TDDRPLoss(phase=1)
        self.num_experts = num_experts

    def _freeze_yolo(self):
        """YOLO 백본 파라미터 고정"""
        for param in self.yolo_backbone.parameters():
            param.requires_grad = False
        print("[TDDRPv3Plus] YOLO 백본 고정(Freeze) 완료")

    def _unfreeze_yolo(self):
        """YOLO 백본 파라미터 해제"""
        for param in self.yolo_backbone.parameters():
            param.requires_grad = True
        print("[TDDRPv3Plus] YOLO 백본 해제(Unfreeze) 완료")

    def set_phase(self, phase: int):
        """학습 Phase 전환"""
        self.criterion.set_phase(phase)
        if phase == 1:
            self._freeze_yolo()
        else:
            self._unfreeze_yolo()

    def get_expert_weights(self) -> torch.Tensor:
        """
        Orthogonality 손실 계산용 Expert 가중치 추출
        각 Expert의 첫 번째 Conv 가중치를 (K, D) 형태로 반환
        """
        weights = []
        for expert in self.rsv_film.expert_gamma:
            w = expert.weight.view(expert.weight.shape[0], -1)  # (C_feat, ...)
            weights.append(w.mean(dim=1, keepdim=True))         # (C_feat, 1)
        return torch.cat(weights, dim=1).t()                    # (K, C_feat)

    def forward(
        self,
        I: torch.Tensor,                         # 입력 이미지 (정상 or 열화)
        I_clean: Optional[torch.Tensor] = None,  # SSL용 정상 이미지
    ) -> Dict[str, torch.Tensor]:
        """
        Forward pass

        Args:
            I       : 입력 이미지 (B, 3, H, W) - 정상 or 열화 이미지
            I_clean : 정상 이미지 (B, 3, H, W) - SSL 학습 시에만 필요

        Returns:
            dict:
              "seg_logits" : 세그멘테이션 로짓 (B, 2, H', W')
              "Z"          : 조명 임베딩 맵
              "P"          : Expert 확률 맵
              "F_prime"    : 변조된 특징 맵
              "F_clean"    : 정상 특징 맵 (I_clean 제공 시)
        """
        outputs = {}

        # ── Step 1: 조명 임베딩 추출
        Z = self.illumination_encoder(I)    # (B, 64, H/4, W/4)
        P = self.soft_router(Z)             # (B, K, H/4, W/4)

        outputs["Z"] = Z
        outputs["P"] = P

        # ── Step 2: YOLO 특징 맵 추출 (Frozen)
        with torch.no_grad():
            F_aug = self.yolo_backbone(I)   # (B, C_feat, H/4, W/4)

        # ── Step 3: RSV-FiLM으로 특징 맵 변조
        F_prime = self.rsv_film(F_aug, Z, P)  # (B, C_feat, H/4, W/4)
        outputs["F_prime"] = F_prime

        # ── Step 4: YOLO 헤드로 세그멘테이션
        seg_logits = self.yolo_head(F_prime)  # (B, 2, H/4, W/4)
        outputs["seg_logits"] = seg_logits

        # ── Step 5: SSL용 정상 이미지 특징 맵 추출
        if I_clean is not None:
            with torch.no_grad():
                F_clean = self.yolo_backbone(I_clean)  # Teacher (Frozen)
            outputs["F_clean"] = F_clean

        return outputs

    def compute_loss(
        self,
        outputs: Dict[str, torch.Tensor],
        I_aug: torch.Tensor,
        yolo_loss: Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:
        """
        전체 손실 계산

        Args:
            outputs   : forward()의 반환값
            I_aug     : 열화 이미지 원본 (L_Smooth 계산용)
            yolo_loss : 외부 YOLO 손실 (Phase 2, 3에서 전달)
        """
        assert "F_clean" in outputs, "SSL 손실 계산을 위해 I_clean을 forward에 전달하세요."

        # TV 손실용 변조 파라미터 재계산 (간략화: F_prime - F_aug 사용)
        # 실제 구현에서는 RSV-FiLM에서 delta_gamma, delta_beta를 직접 반환
        delta_approx = outputs["F_prime"] - self.yolo_backbone(I_aug).detach()

        return self.criterion(
            F_aug_prime    = outputs["F_prime"],
            F_clean        = outputs["F_clean"],
            Z              = outputs["Z"],
            I_aug          = I_aug,
            delta_gamma    = delta_approx,
            delta_beta     = delta_approx,
            expert_weights = self.get_expert_weights(),
            yolo_loss      = yolo_loss,
        )


# ─────────────────────────────────────────────────────────────────────────────
# 7. 데이터 증강 (조명 섭동 합성)
# ─────────────────────────────────────────────────────────────────────────────

class IlluminationAugmentor:
    """
    정상 이미지에 인위적인 조명 열화를 가하는 증강 모듈

    증강 종류:
      - 저조도 (Underexposure) : Gamma 증가
      - 과노출 (Overexposure)  : 픽셀 클리핑 + 역광 패턴 추가
      - 혼합 (Mixed)           : 랜덤 조합

    학습 Phase 1에서 I_clean → I_aug 쌍을 동적으로 생성합니다.
    """

    @staticmethod
    def underexpose(
        I: torch.Tensor,
        gamma: float = 2.5,
        noise_std: float = 0.02,
    ) -> torch.Tensor:
        """저조도 시뮬레이션: Gamma 변환 + Poisson 노이즈"""
        I_dark = I.pow(gamma)
        noise  = torch.randn_like(I_dark) * noise_std
        return (I_dark + noise).clamp(0.0, 1.0)

    @staticmethod
    def overexpose(
        I: torch.Tensor,
        scale: float = 2.0,
        glare_prob: float = 0.5,
    ) -> torch.Tensor:
        """과노출 시뮬레이션: 픽셀 클리핑 + 선택적 글레어 패턴"""
        I_bright = (I * scale).clamp(0.0, 1.0)
        if torch.rand(1).item() < glare_prob:
            # 임의 위치에 사각형 글레어 패턴 추가
            B, C, H, W = I_bright.shape
            x1 = torch.randint(0, W // 2, (1,)).item()
            y1 = torch.randint(0, H // 2, (1,)).item()
            x2 = x1 + torch.randint(W // 4, W // 2, (1,)).item()
            y2 = y1 + torch.randint(H // 4, H // 2, (1,)).item()
            I_bright[:, :, y1:y2, x1:x2] = 1.0  # 완전 포화
        return I_bright

    @classmethod
    def augment(cls, I: torch.Tensor) -> Tuple[torch.Tensor, str]:
        """랜덤으로 조명 열화 종류 선택"""
        r = torch.rand(1).item()
        if r < 0.5:
            return cls.underexpose(I), "underexpose"
        else:
            return cls.overexpose(I), "overexpose"


# ─────────────────────────────────────────────────────────────────────────────
# 8. 학습 루프 (3-Phase Training)
# ─────────────────────────────────────────────────────────────────────────────

def get_optimizer(model: TDDRPv3Plus, phase: int) -> torch.optim.Optimizer:
    """Phase별 학습률 및 최적화 대상 파라미터 반환"""
    if phase == 1:
        # Phase 1: 조명 어댑터만 학습
        params = (
            list(model.illumination_encoder.parameters()) +
            list(model.soft_router.parameters()) +
            list(model.rsv_film.parameters())
        )
        lr = 1e-3
    elif phase == 2:
        # Phase 2: 전체 (YOLO 포함)
        params = model.parameters()
        lr = 5e-4
    else:
        # Phase 3: 전체, 매우 낮은 LR
        params = model.parameters()
        lr = 1e-5

    return torch.optim.AdamW(params, lr=lr, weight_decay=1e-4)


def train_one_epoch(
    model: TDDRPv3Plus,
    dataloader: torch.utils.data.DataLoader,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    phase: int,
    augmentor: IlluminationAugmentor = IlluminationAugmentor(),
) -> Dict[str, float]:
    """
    한 Epoch 학습 루프

    Phase 1: (I_clean, I_aug) 쌍으로 SSL + 정규화 손실만 학습
    Phase 2: 위 + YOLO Segmentation 손실 추가
    Phase 3: 위 + 실제 데이터 미세 조정
    """
    model.train()
    epoch_losses = {k: 0.0 for k in ["total", "ssl", "hist", "smooth", "tv", "orth", "yolo"]}
    n_batches = 0

    for batch in dataloader:
        I_clean = batch["image"].to(device)           # 정상 이미지 (B, 3, H, W)
        I_aug, _ = augmentor.augment(I_clean)         # 열화 이미지 동적 생성

        # Forward
        outputs = model(I_aug, I_clean=I_clean)

        # YOLO 손실 (Phase 2, 3에서 라벨이 있을 경우)
        yolo_loss = None
        if phase >= 2 and "masks" in batch:
            masks = batch["masks"].to(device)         # 구멍 세그멘테이션 마스크
            yolo_loss = F.cross_entropy(
                outputs["seg_logits"],
                F.interpolate(masks.float(), size=outputs["seg_logits"].shape[2:]).long().squeeze(1),
            )

        # 전체 손실 계산
        losses = model.compute_loss(outputs, I_aug, yolo_loss=yolo_loss)

        # 역전파
        optimizer.zero_grad()
        losses["total"].backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        for k, v in losses.items():
            epoch_losses[k] += v.item()
        n_batches += 1

    return {k: v / n_batches for k, v in epoch_losses.items()}


# ─────────────────────────────────────────────────────────────────────────────
# 9. 사용 예시 (Quick Start)
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("TD-DRP v3+ Quick Start Demo")
    print("=" * 60)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}\n")

    # ── 모델 초기화
    model = TDDRPv3Plus(
        feature_channels=256,
        illumination_channels=64,
        num_experts=3,
    ).to(device)

    total_params = sum(p.numel() for p in model.parameters())
    trainable    = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"전체 파라미터: {total_params:,}")
    print(f"학습 가능 파라미터 (Phase 1): {trainable:,}\n")

    # ── 더미 입력 생성
    B, H, W = 2, 256, 256
    I_clean = torch.rand(B, 3, H, W).to(device)
    I_aug   = IlluminationAugmentor.underexpose(I_clean)

    # ── Phase 1: SSL Warm-up
    print("─── Phase 1: SSL Warm-up ───")
    model.set_phase(1)
    optimizer = get_optimizer(model, phase=1)

    outputs = model(I_aug, I_clean=I_clean)
    losses  = model.compute_loss(outputs, I_aug)

    losses["total"].backward()
    optimizer.step()

    for k, v in losses.items():
        print(f"  L_{k:8s} = {v.item():.6f}")

    # ── Phase 2: Task Tuning
    print("\n─── Phase 2: Task Tuning ───")
    model.set_phase(2)
    optimizer = get_optimizer(model, phase=2)

    dummy_yolo_loss = torch.tensor(0.5, requires_grad=True).to(device)
    outputs = model(I_aug, I_clean=I_clean)
    losses  = model.compute_loss(outputs, I_aug, yolo_loss=dummy_yolo_loss)

    losses["total"].backward()
    optimizer.step()

    for k, v in losses.items():
        print(f"  L_{k:8s} = {v.item():.6f}")

    # ── 추론 모드
    print("\n─── 추론 (Inference) ───")
    model.eval()
    with torch.no_grad():
        test_img = torch.rand(1, 3, 640, 640).to(device)
        result   = model(test_img)
        print(f"  입력 해상도:   {test_img.shape}")
        print(f"  세그멘테이션 출력: {result['seg_logits'].shape}")
        print(f"  조명 임베딩 맵 Z: {result['Z'].shape}")
        print(f"  Expert 확률 맵 P: {result['P'].shape}")

    print("\n✅ TD-DRP v3+ 구현 완료!")
    print("   실제 사용 시 yolo_backbone 인자에 YOLOv8 백본을 전달하세요.")
