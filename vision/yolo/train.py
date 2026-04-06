"""
TD-DRP v4 학습 파이프라인 (YOLO26n-pose backbone)
===================================================
Phase 1 (SSL Warm-up) + Phase 2 (Task Tuning) 학습.

Backbone: YOLO26n-pose (P4, 128ch, stride=16)
Seg classes: 4 (center_outer, center_inner, guide_outer, guide_inner)

사용법:
  # Phase 1: SSL Warm-up (Encoder + Router + RSV-FiLM만 학습)
  python train.py --data_root ./data/LIS --phase 1 --epochs 50 --batch_size 4

  # Phase 2: Task Tuning (전체, Phase 1 체크포인트에서 이어서)
  python train.py --data_root ./data/LIS --phase 2 --epochs 30 --batch_size 4 --resume ./checkpoints/tddrp_phase1.pt

  # Phase 1 → Phase 2 연속 학습
  python train.py --data_root ./data/LIS --phase 12 --epochs_p1 50 --epochs_p2 30
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import torch
import torch.nn as nn
import torch.nn.functional as F

# 같은 디렉토리의 모듈 임포트
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from td_drp_v4 import (
    TDDRPv4,
    load_yolo26_pose_backbone,
    IlluminationAugmentor,
)
from dataloader import get_dataloader, get_lis_class_names


# ─────────────────────────────────────────────────────────────────────────────
# 1. 학습 유틸리티
# ─────────────────────────────────────────────────────────────────────────────

def get_optimizer(
    model: TDDRPv4,
    phase: int,
    lr: Optional[float] = None,
) -> torch.optim.Optimizer:
    """Phase별 최적화기 생성"""
    if phase == 1:
        # Phase 1: 조명 어댑터만 학습 (YOLO backbone frozen)
        params = (
            list(model.illumination_encoder.parameters()) +
            list(model.soft_router.parameters()) +
            list(model.rsv_film.parameters())
        )
        default_lr = 1e-3
    elif phase == 2:
        # Phase 2: 전체 파라미터 학습 (YOLO 포함)
        params = model.parameters()
        default_lr = 5e-4
    else:
        raise ValueError(f"지원하지 않는 Phase: {phase}")

    return torch.optim.AdamW(
        params,
        lr=lr or default_lr,
        weight_decay=1e-4,
    )


def get_scheduler(
    optimizer: torch.optim.Optimizer,
    epochs: int,
) -> torch.optim.lr_scheduler._LRScheduler:
    """코사인 어니링 스케줄러"""
    return torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=epochs, eta_min=1e-6,
    )


def compute_seg_loss(
    seg_logits: torch.Tensor,
    masks_list: list,
    labels_list: list,
    num_classes: int = 4,
) -> torch.Tensor:
    """
    세그멘테이션 손실 계산 (Phase 2)

    seg_logits에서 GT 마스크와 비교하여 Pixel-wise Cross Entropy 계산.
    모든 인스턴스 마스크를 하나의 시맨틱 마스크로 병합합니다.

    Args:
        seg_logits  : (B, num_classes, H', W') 모델 출력
        masks_list  : [Tensor(N_i, H, W)] GT 인스턴스 마스크 리스트
        labels_list : [Tensor(N_i,)] GT 클래스 라벨 리스트
        num_classes : 세그멘테이션 클래스 수 (기본: 2, 전경/배경)

    Returns:
        loss : 스칼라 텐서
    """
    B = seg_logits.shape[0]
    H_out, W_out = seg_logits.shape[2:]
    device = seg_logits.device
    total_loss = torch.tensor(0.0, device=device, requires_grad=True)

    for b in range(B):
        masks = masks_list[b]   # (N, H, W)
        # labels = labels_list[b]  # (N,) - 미래 사용을 위해 보존

        # 모든 인스턴스 마스크를 바이너리 전경 마스크로 병합
        if masks.shape[0] > 0:
            # (N, H, W) → (H, W) max pool
            merged_mask = masks.max(dim=0).values   # (H, W)
        else:
            # 마스크가 없으면 전체 배경
            merged_mask = torch.zeros(
                seg_logits.shape[2] * (640 // seg_logits.shape[2]),
                seg_logits.shape[3] * (640 // seg_logits.shape[3]),
                device=device
            )

        # GT 마스크를 seg_logits 해상도로 리사이즈
        gt = F.interpolate(
            merged_mask.unsqueeze(0).unsqueeze(0).float().to(device),
            size=(H_out, W_out),
            mode="nearest",
        ).long().squeeze()   # (H', W')

        # Cross-Entropy Loss
        logits_b = seg_logits[b]  # (num_classes, H', W')
        loss_b = F.cross_entropy(
            logits_b.unsqueeze(0), gt.unsqueeze(0),
        )
        total_loss = total_loss + loss_b

    return total_loss / B


# ─────────────────────────────────────────────────────────────────────────────
# 2. 학습 에폭
# ─────────────────────────────────────────────────────────────────────────────

def train_one_epoch(
    model: TDDRPv4,
    dataloader: torch.utils.data.DataLoader,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    phase: int,
    epoch: int,
    log_interval: int = 10,
) -> Dict[str, float]:
    """
    한 에폭 학습

    Phase 1: SSL (정상 이미지 → 합성 열화 → 특징 일관성)
    Phase 2: SSL + 세그멘테이션 Task Loss
    """
    model.train()
    epoch_losses = {
        k: 0.0 for k in
        ["total", "ssl", "hist", "smooth", "tv", "orth", "yolo"]
    }
    n_batches = 0
    start_time = time.time()

    for batch_idx, batch in enumerate(dataloader):
        if phase == 1:
            # Phase 1: 정상/열화 이미지 쌍
            I_clean = batch["I_clean"].to(device)
            I_aug = batch["I_aug"].to(device)

            # Forward: 열화 이미지 + 정상 이미지 (Teacher)
            outputs = model(I_aug, I_clean=I_clean)

            # SSL + 정규화 손실만
            losses = model.compute_loss(outputs, I_aug)

        elif phase == 2:
            # Phase 2: 이미지 + 세그멘테이션 라벨
            image = batch["image"].to(device)

            # I_clean이 있으면 사용 (SSL 손실 계산용)
            I_clean = batch.get("I_clean")
            if I_clean is not None:
                I_clean = I_clean.to(device)

            # Forward
            outputs = model(image, I_clean=I_clean)

            # 세그멘테이션 Task Loss 계산
            yolo_loss = None
            if "masks" in batch:
                yolo_loss = compute_seg_loss(
                    outputs["seg_logits"],
                    batch["masks"],
                    batch["labels"],
                )

            if I_clean is not None:
                losses = model.compute_loss(
                    outputs, image, yolo_loss=yolo_loss,
                )
            else:
                # I_clean이 없으면 YOLO loss만 사용
                losses = {"total": yolo_loss or torch.tensor(0.0, device=device)}
                for k in epoch_losses:
                    if k not in losses:
                        losses[k] = torch.tensor(0.0, device=device)

        # 역전파
        optimizer.zero_grad()
        losses["total"].backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        # 손실 기록
        for k in epoch_losses:
            if k in losses:
                val = losses[k]
                epoch_losses[k] += val.item() if torch.is_tensor(val) else val
        n_batches += 1

        # 로그 출력
        if (batch_idx + 1) % log_interval == 0:
            elapsed = time.time() - start_time
            avg_loss = epoch_losses["total"] / n_batches
            print(f"  [{epoch}][{batch_idx+1}/{len(dataloader)}] "
                  f"loss={avg_loss:.4f}  "
                  f"time={elapsed:.1f}s")

    return {k: v / max(n_batches, 1) for k, v in epoch_losses.items()}


# ─────────────────────────────────────────────────────────────────────────────
# 3. 검증 에폭
# ─────────────────────────────────────────────────────────────────────────────

@torch.no_grad()
def validate(
    model: TDDRPv4,
    dataloader: torch.utils.data.DataLoader,
    device: torch.device,
    phase: int,
) -> Dict[str, float]:
    """검증 루프"""
    model.eval()
    val_losses = {
        k: 0.0 for k in
        ["total", "ssl", "hist", "smooth", "tv", "orth", "yolo"]
    }
    n_batches = 0

    for batch in dataloader:
        if phase == 1:
            I_clean = batch["I_clean"].to(device)
            I_aug = batch["I_aug"].to(device)
            outputs = model(I_aug, I_clean=I_clean)
            losses = model.compute_loss(outputs, I_aug)
        elif phase == 2:
            image = batch["image"].to(device)
            I_clean = batch.get("I_clean")
            if I_clean is not None:
                I_clean = I_clean.to(device)
            outputs = model(image, I_clean=I_clean)

            yolo_loss = None
            if "masks" in batch:
                yolo_loss = compute_seg_loss(
                    outputs["seg_logits"],
                    batch["masks"],
                    batch["labels"],
                )

            if I_clean is not None:
                losses = model.compute_loss(
                    outputs, image, yolo_loss=yolo_loss,
                )
            else:
                losses = {"total": yolo_loss or torch.tensor(0.0, device=device)}
                for k in val_losses:
                    if k not in losses:
                        losses[k] = torch.tensor(0.0, device=device)

        for k in val_losses:
            if k in losses:
                val = losses[k]
                val_losses[k] += val.item() if torch.is_tensor(val) else val
        n_batches += 1

    return {k: v / max(n_batches, 1) for k, v in val_losses.items()}


# ─────────────────────────────────────────────────────────────────────────────
# 4. 체크포인트 관리
# ─────────────────────────────────────────────────────────────────────────────

def save_checkpoint(
    model: TDDRPv4,
    optimizer: torch.optim.Optimizer,
    epoch: int,
    phase: int,
    losses: Dict[str, float],
    save_path: str,
) -> None:
    """체크포인트 저장"""
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    torch.save({
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "epoch": epoch,
        "phase": phase,
        "losses": losses,
    }, save_path)
    print(f"  체크포인트 저장: {save_path}")


def load_checkpoint(
    model: TDDRPv4,
    checkpoint_path: str,
    optimizer: Optional[torch.optim.Optimizer] = None,
    device: torch.device = torch.device("cpu"),
) -> Dict:
    """체크포인트 로드"""
    ckpt = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(ckpt["model_state_dict"], strict=False)
    if optimizer and "optimizer_state_dict" in ckpt:
        try:
            optimizer.load_state_dict(ckpt["optimizer_state_dict"])
        except ValueError:
            print("  [경고] Optimizer 상태가 호환되지 않아 건너뜀")
    print(f"  체크포인트 로드: {checkpoint_path}")
    print(f"    Phase={ckpt.get('phase')}, Epoch={ckpt.get('epoch')}")
    return ckpt


# ─────────────────────────────────────────────────────────────────────────────
# 5. 메인 학습 함수
# ─────────────────────────────────────────────────────────────────────────────

def train(args: argparse.Namespace) -> None:
    """메인 학습 루프"""
    print("=" * 60)
    print("TD-DRP v4 Training Pipeline (YOLO26n-pose)")
    print("=" * 60)

    device = torch.device(args.device)
    print(f"Device: {device}")

    # ── 모델 초기화
    print("\n── 모델 초기화 ──")
    backbone = load_yolo26_pose_backbone(
        model_name=args.model,
        feature_channels=args.feature_channels,
        extract_layer=args.extract_layer,
    )

    model = TDDRPv4(
        feature_channels=args.feature_channels,
        illumination_channels=args.illumination_channels,
        num_experts=args.num_experts,
        yolo_backbone=backbone,
    ).to(device)

    total_params = sum(p.numel() for p in model.parameters())
    print(f"전체 파라미터: {total_params:,}")

    # ── 체크포인트 로드
    if args.resume:
        load_checkpoint(model, args.resume, device=device)

    # ── Phase 학습
    phases = []
    if args.phase == 1:
        phases = [(1, args.epochs)]
    elif args.phase == 2:
        phases = [(2, args.epochs)]
    elif args.phase == 12:
        # Phase 1 → 2 연속
        phases = [(1, args.epochs_p1), (2, args.epochs_p2)]

    for phase, n_epochs in phases:
        print(f"\n{'═' * 60}")
        print(f"Phase {phase}: {'SSL Warm-up' if phase == 1 else 'Task Tuning'}")
        print(f"  Epochs: {n_epochs}")
        print(f"{'═' * 60}")

        model.set_phase(phase)

        # DataLoader
        train_loader = get_dataloader(
            data_root=args.data_root,
            split="train",
            phase=phase,
            batch_size=args.batch_size,
            img_size=args.img_size,
            num_workers=args.num_workers,
            use_dark=(phase == 2),
        )

        val_loader = get_dataloader(
            data_root=args.data_root,
            split="test",
            phase=phase,
            batch_size=args.batch_size,
            img_size=args.img_size,
            num_workers=args.num_workers,
            use_dark=(phase == 2),
        )

        trainable = sum(
            p.numel() for p in model.parameters() if p.requires_grad
        )
        print(f"학습 파라미터: {trainable:,}")
        print(f"학습 데이터: {len(train_loader.dataset)}개")
        print(f"검증 데이터: {len(val_loader.dataset)}개")

        # Optimizer & Scheduler
        optimizer = get_optimizer(model, phase, lr=args.lr)
        scheduler = get_scheduler(optimizer, n_epochs)

        best_loss = float("inf")

        for epoch in range(1, n_epochs + 1):
            print(f"\n── Epoch {epoch}/{n_epochs} ──")

            # 학습
            train_losses = train_one_epoch(
                model, train_loader, optimizer, device,
                phase=phase, epoch=epoch,
                log_interval=args.log_interval,
            )

            # LR 스케줄링
            scheduler.step()
            current_lr = optimizer.param_groups[0]["lr"]

            # 검증
            val_losses = validate(model, val_loader, device, phase)

            # 에폭 요약
            print(f"\n  [Train] ", end="")
            for k, v in train_losses.items():
                print(f"{k}={v:.4f}  ", end="")
            print(f"\n  [Val]   ", end="")
            for k, v in val_losses.items():
                print(f"{k}={v:.4f}  ", end="")
            print(f"\n  LR={current_lr:.2e}")

            # 체크포인트 저장
            is_best = val_losses["total"] < best_loss
            if is_best:
                best_loss = val_losses["total"]

            if epoch % args.save_every == 0 or is_best or epoch == n_epochs:
                ckpt_dir = args.checkpoint_dir
                save_checkpoint(
                    model, optimizer, epoch, phase,
                    val_losses,
                    os.path.join(ckpt_dir, f"tddrp_phase{phase}_ep{epoch}.pt"),
                )
                if is_best:
                    save_checkpoint(
                        model, optimizer, epoch, phase,
                        val_losses,
                        os.path.join(ckpt_dir, f"tddrp_phase{phase}_best.pt"),
                    )

        # Phase 완료 후 최종 체크포인트
        save_checkpoint(
            model, optimizer, n_epochs, phase,
            val_losses,
            os.path.join(args.checkpoint_dir, f"tddrp_phase{phase}.pt"),
        )
        print(f"\n✅ Phase {phase} 학습 완료! Best val loss: {best_loss:.4f}")

    print("\n" + "=" * 60)
    print("전체 학습 완료!")
    print("=" * 60)
    print(f"\n체크포인트 저장 위치: {args.checkpoint_dir}")
    print("\nYOLO26n-pose 단독 모델과 비교하려면:")
    print(f"  python yolo_backbone.py --mode eval "
          f"--data_root {args.data_root}")


# ─────────────────────────────────────────────────────────────────────────────
# 6. CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="TD-DRP v3+ Training Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # Phase 1 (SSL Warm-up)
  python train.py --data_root ./data/LIS --phase 1 --epochs 50

  # Phase 2 (Task Tuning, Phase 1 결과에서 이어서)
  python train.py --data_root ./data/LIS --phase 2 --epochs 30 \\
    --resume ./checkpoints/tddrp_phase1.pt

  # Phase 1 → 2 연속
  python train.py --data_root ./data/LIS --phase 12 \\
    --epochs_p1 50 --epochs_p2 30
""",
    )

    # 데이터
    parser.add_argument("--data_root", type=str, required=True,
                        help="LIS 데이터셋 루트 경로")
    parser.add_argument("--img_size", type=int, default=640,
                        help="이미지 리사이즈 크기")

    # 학습 설정
    parser.add_argument("--phase", type=int, default=1,
                        choices=[1, 2, 12],
                        help="학습 Phase (1, 2, 또는 12=연속)")
    parser.add_argument("--epochs", type=int, default=50,
                        help="학습 에폭 수 (phase 1 또는 2 단독)")
    parser.add_argument("--epochs_p1", type=int, default=50,
                        help="Phase 1 에폭 (phase=12에서)")
    parser.add_argument("--epochs_p2", type=int, default=30,
                        help="Phase 2 에폭 (phase=12에서)")
    parser.add_argument("--batch_size", type=int, default=4,
                        help="배치 크기")
    parser.add_argument("--lr", type=float, default=None,
                        help="학습률 (None이면 Phase별 기본값)")
    parser.add_argument("--num_workers", type=int, default=2,
                        help="데이터 로딩 워커 수")

    # 모델
    parser.add_argument("--model", type=str, default="yolo26n-pose.pt",
                        help="YOLO26n-pose 모델 이름")
    parser.add_argument("--feature_channels", type=int, default=128)
    parser.add_argument("--illumination_channels", type=int, default=64)
    parser.add_argument("--num_experts", type=int, default=3)
    parser.add_argument("--extract_layer", type=int, default=6,
                        help="백본 추출 레이어 (4=P3, 6=P4, 9=P5)")

    # 체크포인트
    parser.add_argument("--resume", type=str, default=None,
                        help="이어서 학습할 체크포인트 경로")
    parser.add_argument("--checkpoint_dir", type=str,
                        default="./checkpoints",
                        help="체크포인트 저장 디렉토리")
    parser.add_argument("--save_every", type=int, default=10,
                        help="N 에폭마다 체크포인트 저장")

    # 기타
    parser.add_argument("--device", type=str, default="auto",
                        help="학습 장치 (cuda/cpu/auto)")
    parser.add_argument("--log_interval", type=int, default=10,
                        help="N 배치마다 로그 출력")

    args = parser.parse_args()

    # 자동 디바이스 선택
    if args.device == "auto":
        args.device = "cuda" if torch.cuda.is_available() else "cpu"

    return args


if __name__ == "__main__":
    args = parse_args()
    train(args)
