# Refactored from the user-provided training pipeline. :contentReference[oaicite:0]{index=0}
"""
TD-DRP 학습 파이프라인 v2 (Frozen Teacher + Task Loss)

Refactor points:
1) aspect ratio 유지 + center padding(letterbox) 적용
2) GT bbox/keypoint도 동일한 scale/pad로 재매핑
3) GPU 사용 여부를 학습 시작 시 강하게 검증
4) train/eval 모두 device 명시 및 로그 강화
"""

import os
import sys
import glob
import shutil
from typing import Tuple, List, Dict, Any

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

import torch
from torch.utils.data import DataLoader, Dataset
from torchvision.transforms import ToTensor
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from vision.yolo.td_drp import TDDRPEndToEndPose
from vision.yolo.inference_utils import plot_loss_curve


# ==========================================
# [설정]
# ==========================================
FOUNDATION_YOLO_WEIGHTS = os.path.abspath("vision/yolo/weights/yolo26n-pose_best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_v2")

BATCH_SIZE = 4
NUM_WORKERS = 4
IMG_SIZE = 640

PHASE1_EPOCHS = 10
PHASE1_LR = 1e-3

EXLPOSE_ROOT = os.path.abspath("vision/dataset/ExLPose/ExLPose")
EXLPOSE_PAIR_MAP = os.path.abspath(
    "vision/dataset/ExLPose/Annotations/Annotations/ExLPose/brightPath2darkPath.txt"
)

DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
TRAIN_CLEAN_DIR = os.path.join(DATASET_ROOT, "train/images_clean")
TRAIN_AUG_DIR = os.path.join(DATASET_ROOT, "train/images_aug")
TRAIN_LABEL_DIR = os.path.join(DATASET_ROOT, "train/labels_clean")

PHASE2_EPOCHS = 100
PHASE2_LR = 1e-4
PHASE2_TASK_WEIGHT = 1.0

TEST_DIR = os.path.join(DATASET_ROOT, "test")

TO_TENSOR = ToTensor()


# ==========================================
# Utility
# ==========================================

def ensure_gpu_or_raise() -> torch.device:
    print("=" * 80)
    print("[Device Check]")
    print(f"torch.__version__      : {torch.__version__}")
    print(f"torch.version.cuda     : {torch.version.cuda}")
    print(f"torch.cuda.is_available: {torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is not available. Check your PyTorch CUDA build / NVIDIA driver / uv environment."
        )

    device = torch.device("cuda:0")
    print(f"GPU count              : {torch.cuda.device_count()}")
    print(f"Current device idx     : {torch.cuda.current_device()}")
    print(f"GPU name               : {torch.cuda.get_device_name(device)}")

    x = torch.randn(8, 8, device=device)
    y = x @ x
    torch.cuda.synchronize(device)
    print(f"CUDA smoke test        : PASS ({tuple(y.shape)})")
    print("=" * 80)
    return device


def assert_model_on_device(model: torch.nn.Module, device: torch.device, name: str = "model") -> None:
    params = list(model.parameters())
    if not params:
        print(f"[Warning] {name} has no parameters to inspect.")
        return
    dev = params[0].device
    print(f"[Device] {name} first param device: {dev}")
    if dev != device:
        raise RuntimeError(f"{name} is on {dev}, expected {device}.")


def letterbox_pil(
    img: Image.Image,
    target_size: int,
    pad_color: Tuple[int, int, int] = (0, 0, 0),
) -> Tuple[Image.Image, float, int, int, int, int]:
    """
    Returns:
        padded_img, scale, pad_x, pad_y, orig_w, orig_h
    """
    img = img.convert("RGB")
    orig_w, orig_h = img.size

    scale = min(target_size / orig_w, target_size / orig_h)
    new_w = max(1, int(round(orig_w * scale)))
    new_h = max(1, int(round(orig_h * scale)))

    resized = img.resize((new_w, new_h), Image.BILINEAR)
    canvas = Image.new("RGB", (target_size, target_size), pad_color)

    pad_x = (target_size - new_w) // 2
    pad_y = (target_size - new_h) // 2
    canvas.paste(resized, (pad_x, pad_y))

    return canvas, scale, pad_x, pad_y, orig_w, orig_h


def remap_yolo_targets_for_letterbox(
    targets: List[Tuple[float, List[float], np.ndarray]],
    orig_w: int,
    orig_h: int,
    scale: float,
    pad_x: int,
    pad_y: int,
    target_size: int,
    num_kpts: int = 9,
) -> torch.Tensor:
    """
    Input target format:
        [(cls, [cx, cy, w, h], kpts[N,3]), ...]
        bbox/kpts are normalized to original image size.
    Output:
        torch.Tensor [N, 5 + num_kpts*3]
        normalized to letterboxed target_size x target_size
    """
    rows = []
    for cls, bbox, kpts in targets:
        cx, cy, bw, bh = bbox

        cx_px = cx * orig_w
        cy_px = cy * orig_h
        bw_px = bw * orig_w
        bh_px = bh * orig_h

        cx_new = (cx_px * scale + pad_x) / target_size
        cy_new = (cy_px * scale + pad_y) / target_size
        bw_new = (bw_px * scale) / target_size
        bh_new = (bh_px * scale) / target_size

        kpts_new = []
        for j in range(num_kpts):
            kx, ky, kv = kpts[j]
            if kv > 0:
                kx_px = kx * orig_w
                ky_px = ky * orig_h
                kx_new = (kx_px * scale + pad_x) / target_size
                ky_new = (ky_px * scale + pad_y) / target_size
            else:
                kx_new, ky_new = 0.0, 0.0
            kpts_new.extend([kx_new, ky_new, kv])

        row = [cls, cx_new, cy_new, bw_new, bh_new] + kpts_new
        rows.append(row)

    if rows:
        return torch.tensor(rows, dtype=torch.float32)
    return torch.zeros(0, 5 + num_kpts * 3, dtype=torch.float32)


# ==========================================
# Datasets
# ==========================================

class ExLPosePairedDataset(Dataset):
    """ExLPose bright→dark 이미지 쌍 (Phase 1-A: 조명 매핑 학습)"""
    def __init__(self, exlpose_root: str, pair_map_path: str):
        self.root = exlpose_root
        self.pairs = []
        if not os.path.isfile(pair_map_path):
            print(f"[Warning] Pair map not found: {pair_map_path}")
            return

        with open(pair_map_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 2:
                    continue
                bright_rel, dark_rel = parts
                bright_path = os.path.join(self.root, bright_rel)
                dark_path = os.path.join(self.root, dark_rel)
                if os.path.isfile(bright_path) and os.path.isfile(dark_path):
                    self.pairs.append((bright_path, dark_path))

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int):
        bright_path, dark_path = self.pairs[idx]

        bright_img = Image.open(bright_path).convert("RGB")
        dark_img = Image.open(dark_path).convert("RGB")

        bright_lb, *_ = letterbox_pil(bright_img, IMG_SIZE)
        dark_lb, *_ = letterbox_pil(dark_img, IMG_SIZE)

        return TO_TENSOR(bright_lb), TO_TENSOR(dark_lb)


class PairedImageDataset(Dataset):
    """Clean-Aug 이미지 쌍 (Phase 1-B: 도메인 특화 warm-up, labels 불필요)"""
    def __init__(self, clean_dir: str, aug_dir: str):
        self.clean_dir = clean_dir
        self.aug_dir = aug_dir
        clean_files = set(f for f in os.listdir(clean_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))) if os.path.isdir(clean_dir) else set()
        aug_files = set(f for f in os.listdir(aug_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))) if os.path.isdir(aug_dir) else set()
        self.files = sorted(clean_files & aug_files)

    def __len__(self) -> int:
        return len(self.files)

    def __getitem__(self, idx: int):
        fname = self.files[idx]
        clean_img = Image.open(os.path.join(self.clean_dir, fname)).convert("RGB")
        aug_img = Image.open(os.path.join(self.aug_dir, fname)).convert("RGB")

        clean_lb, *_ = letterbox_pil(clean_img, IMG_SIZE)
        aug_lb, *_ = letterbox_pil(aug_img, IMG_SIZE)

        return TO_TENSOR(clean_lb), TO_TENSOR(aug_lb)


class PairedImageWithLabelsDataset(Dataset):
    """
    Clean-Aug 이미지 쌍 + GT labels (Phase 2: Task Loss 학습용)
    Label format:
        cls cx cy w h kp1_x kp1_y kp1_v ... kp9_x kp9_y kp9_v
    """
    def __init__(self, clean_dir: str, aug_dir: str, label_dir: str, num_kpts: int = 9):
        self.clean_dir = clean_dir
        self.aug_dir = aug_dir
        self.label_dir = label_dir
        self.num_kpts = num_kpts

        clean_files = set(f for f in os.listdir(clean_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))) if os.path.isdir(clean_dir) else set()
        aug_files = set(f for f in os.listdir(aug_dir) if f.lower().endswith((".png", ".jpg", ".jpeg"))) if os.path.isdir(aug_dir) else set()
        self.files = sorted(clean_files & aug_files)

    def __len__(self) -> int:
        return len(self.files)

    def _parse_label(self, label_path: str) -> List[Tuple[float, List[float], np.ndarray]]:
        targets = []
        if not os.path.exists(label_path):
            return targets

        with open(label_path, "r", encoding="utf-8") as f:
            for line in f:
                vals = list(map(float, line.strip().split()))
                if len(vals) < 5 + self.num_kpts * 3:
                    continue
                cls = vals[0]
                bbox = vals[1:5]
                kpts = np.array(vals[5:5 + self.num_kpts * 3], dtype=np.float32).reshape(self.num_kpts, 3)
                targets.append((cls, bbox, kpts))
        return targets

    def __getitem__(self, idx: int):
        fname = self.files[idx]
        stem = os.path.splitext(fname)[0]

        clean_path = os.path.join(self.clean_dir, fname)
        aug_path = os.path.join(self.aug_dir, fname)
        label_path = os.path.join(self.label_dir, stem + ".txt")

        clean_img = Image.open(clean_path).convert("RGB")
        aug_img = Image.open(aug_path).convert("RGB")

        clean_lb, clean_scale, clean_pad_x, clean_pad_y, clean_w, clean_h = letterbox_pil(clean_img, IMG_SIZE)
        aug_lb, _, _, _, _, _ = letterbox_pil(aug_img, IMG_SIZE)

        targets = self._parse_label(label_path)
        labels = remap_yolo_targets_for_letterbox(
            targets=targets,
            orig_w=clean_w,
            orig_h=clean_h,
            scale=clean_scale,
            pad_x=clean_pad_x,
            pad_y=clean_pad_y,
            target_size=IMG_SIZE,
            num_kpts=self.num_kpts,
        )

        return TO_TENSOR(clean_lb), TO_TENSOR(aug_lb), labels


def collate_with_labels(batch):
    cleans, augs, labels_list = zip(*batch)
    cleans = torch.stack(cleans)
    augs = torch.stack(augs)

    batch_labels = []
    for i, labels in enumerate(labels_list):
        if labels.shape[0] > 0:
            batch_idx = torch.full((labels.shape[0], 1), i, dtype=torch.float32)
            batch_labels.append(torch.cat([batch_idx, labels], dim=1))

    if batch_labels:
        batch_labels = torch.cat(batch_labels, dim=0)
    else:
        batch_labels = torch.zeros(0, 1 + 5 + 9 * 3, dtype=torch.float32)

    return cleans, augs, batch_labels


def labels_to_batch_dict(batch_labels: torch.Tensor, device: torch.device, num_kpts: int = 9) -> Dict[str, torch.Tensor]:
    if batch_labels.shape[0] == 0:
        return {
            "batch_idx": torch.zeros(0, device=device),
            "cls": torch.zeros(0, device=device),
            "bboxes": torch.zeros(0, 4, device=device),
            "keypoints": torch.zeros(0, num_kpts, 3, device=device),
        }

    batch_labels = batch_labels.to(device)
    return {
        "batch_idx": batch_labels[:, 0],
        "cls": batch_labels[:, 1],
        "bboxes": batch_labels[:, 2:6],
        "keypoints": batch_labels[:, 6:].reshape(-1, num_kpts, 3),
    }


# ==========================================
# Task Loss
# ==========================================

def init_task_loss(yolo_detection_model):
    from ultralytics.utils.loss import PoseLoss26
    from ultralytics.cfg import get_cfg
    # YOLO 로드 후 model.args가 plain dict → PoseLoss26이 .box attribute 접근 실패
    # IterableSimpleNamespace로 교체하여 default hyp 제공
    if not hasattr(yolo_detection_model.args, 'box'):
        yolo_detection_model.args = get_cfg()
    criterion = PoseLoss26(yolo_detection_model)
    return criterion


def extract_preds_for_loss(head_output):
    """YOLO26 Pose head 출력에서 PoseLoss26이 기대하는 형식으로 추출.

    YOLO26 training mode 출력 형식:
      - dict {"one2many": (feats, kpts), ...}  → one2many 브랜치 반환
      - tuple (feats, kpts)                    → 그대로 반환
      - tensor                                 → 그대로 반환
    """
    if isinstance(head_output, dict):
        if "one2many" in head_output:
            return head_output["one2many"]
        return head_output
    # tuple (feats, kpts) 또는 raw tensor → 그대로 전달
    return head_output


# ==========================================
# Training Loops
# ==========================================

def run_phase1(model, loader, optimizer, num_epochs, phase_name, grad_clip=None):
    device = next(model.parameters()).device
    epoch_losses = []

    for epoch in range(num_epochs):
        model.train()
        total_loss = 0.0

        for i, batch in enumerate(loader):
            I_clean = batch[0].to(device, non_blocking=True)
            I_aug = batch[1].to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)

            outputs = model(I_aug, I_clean=I_clean)
            losses = model.criterion(
                F_aug_primes=outputs["F_primes"],
                F_cleans=outputs["F_cleans"],
                Z=outputs["Z"],
                I_aug=I_aug,
                delta_gammas=outputs["delta_gammas"],
                delta_betas=outputs["delta_betas"],
                expert_weights_list=model.get_expert_weights(),
                pose_logits_aug=outputs.get("pose_logits_aug"),
                pose_logits_clean=outputs.get("pose_logits_clean"),
            )

            losses["total"].backward()

            if grad_clip:
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=grad_clip)

            optimizer.step()
            total_loss += losses["total"].item()

            if (i + 1) % BATCH_SIZE == 0:
                gpu_mem = torch.cuda.memory_allocated(device) / (1024 ** 2)
                print(f"  Batch {i+1}/{len(loader)} | Loss: {losses['total'].item():.4f} | GPU Mem: {gpu_mem:.1f} MB")

        avg = total_loss / max(len(loader), 1)
        epoch_losses.append(avg)
        print(f"[{phase_name}] Epoch {epoch+1}/{num_epochs} | Loss: {avg:.4f}")

    return epoch_losses


def run_phase2(model, loader, optimizer, num_epochs, phase_name, task_criterion, task_weight=1.0, grad_clip=1.0):
    device = next(model.parameters()).device
    epoch_losses = []
    epoch_task_losses = []

    for epoch in range(num_epochs):
        model.train()
        total_loss = 0.0
        total_task = 0.0

        for i, (I_clean, I_aug, batch_labels) in enumerate(loader):
            I_clean = I_clean.to(device, non_blocking=True)
            I_aug = I_aug.to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)

            outputs = model(I_aug, I_clean=I_clean)

            ssl_kd_losses = model.criterion(
                F_aug_primes=outputs["F_primes"],
                F_cleans=outputs["F_cleans"],
                Z=outputs["Z"],
                I_aug=I_aug,
                delta_gammas=outputs["delta_gammas"],
                delta_betas=outputs["delta_betas"],
                expert_weights_list=model.get_expert_weights(),
                pose_logits_aug=outputs.get("pose_logits_aug"),
                pose_logits_clean=outputs.get("pose_logits_clean"),
            )

            task_loss = torch.tensor(0.0, device=device)
            if batch_labels.shape[0] > 0:
                preds = extract_preds_for_loss(outputs["x_aug"])
                batch_dict = labels_to_batch_dict(batch_labels, device)
                try:
                    raw_task_loss, _ = task_criterion(preds, batch_dict)
                    task_loss = raw_task_loss.sum() / I_aug.shape[0]
                except Exception as e:
                    if (i + 1) % 50 == 0:
                        print(f"  [Warning] Task loss failed: {e}")

            loss = ssl_kd_losses["total"] + task_weight * task_loss
            loss.backward()

            if grad_clip:
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=grad_clip)

            optimizer.step()

            total_loss += loss.item()
            total_task += task_loss.item()

            if (i + 1) % BATCH_SIZE == 0:
                gpu_mem = torch.cuda.memory_allocated(device) / (1024 ** 2)
                print(
                    f"  Batch {i+1}/{len(loader)} | "
                    f"Total: {loss.item():.4f} | "
                    f"SSL+KD: {ssl_kd_losses['total'].item():.4f} | "
                    f"Task: {task_loss.item():.4f} | "
                    f"GPU Mem: {gpu_mem:.1f} MB"
                )

        n = max(len(loader), 1)
        avg_loss = total_loss / n
        avg_task = total_task / n
        epoch_losses.append(avg_loss)
        epoch_task_losses.append(avg_task)

        print(f"[{phase_name}] Epoch {epoch+1}/{num_epochs} | Total: {avg_loss:.4f} | Task: {avg_task:.4f}")

    return epoch_losses, epoch_task_losses


# ==========================================
# Main Training
# ==========================================

def train():
    if not os.path.exists(FOUNDATION_YOLO_WEIGHTS):
        raise FileNotFoundError(f"Foundation weights not found: {FOUNDATION_YOLO_WEIGHTS}")

    device = ensure_gpu_or_raise()
    os.makedirs(RESULT_DIR, exist_ok=True)

    # ============================
    # Phase 1-A
    # ============================
    existing_p1a = os.path.abspath("result/td_drp_yolo26n_pose/td_drp_phase1.pt")
    p1a_save_path = os.path.join(RESULT_DIR, "td_drp_phase1_A.pt")

    if os.path.exists(existing_p1a):
        print(f"\n[Phase 1-A] 기존 가중치 발견: {existing_p1a}")
        shutil.copy2(existing_p1a, p1a_save_path)
        print(f"  → {p1a_save_path} 로 복사 완료")
    else:
        print("\n[Phase 1-A] ExLPose SSL Warm-up")
        yolo_a = YOLO(FOUNDATION_YOLO_WEIGHTS)
        model_a = TDDRPEndToEndPose(yolo_a).to(device)
        model_a.set_phase(1)
        assert_model_on_device(model_a, device, "model_a")

        ds1a = ExLPosePairedDataset(EXLPOSE_ROOT, EXLPOSE_PAIR_MAP)
        loader1a = DataLoader(
            ds1a,
            batch_size=BATCH_SIZE,
            shuffle=True,
            num_workers=NUM_WORKERS,
            pin_memory=True,
            persistent_workers=NUM_WORKERS > 0,
        )
        opt1a = torch.optim.Adam([p for p in model_a.parameters() if p.requires_grad], lr=PHASE1_LR)
        p1a_losses = run_phase1(model_a, loader1a, opt1a, PHASE1_EPOCHS, "Phase 1-A (ExLPose)")
        torch.save(model_a.state_dict(), p1a_save_path)
        plot_loss_curve(p1a_losses, os.path.join(RESULT_DIR, "p1a_loss.png"))

        del model_a, yolo_a, opt1a, loader1a, ds1a
        torch.cuda.empty_cache()

    # ============================
    # Phase 1-B
    # ============================
    p1b_save_path = os.path.join(RESULT_DIR, "td_drp_phase1_B.pt")

    if os.path.exists(p1b_save_path):
        print(f"\n[Phase 1-B] 기존 가중치 발견: {p1b_save_path} → 건너뜀")
    else:
        print("\n[Phase 1-B] Custom Synthetic SSL Warm-up")
        yolo_b = YOLO(FOUNDATION_YOLO_WEIGHTS)
        model_b = TDDRPEndToEndPose(yolo_b).to(device)
        model_b.set_phase(1)
        assert_model_on_device(model_b, device, "model_b")

        ds1b = PairedImageDataset(TRAIN_CLEAN_DIR, TRAIN_AUG_DIR)
        loader1b = DataLoader(
            ds1b,
            batch_size=BATCH_SIZE,
            shuffle=True,
            num_workers=NUM_WORKERS,
            pin_memory=True,
            persistent_workers=NUM_WORKERS > 0,
        )
        opt1b = torch.optim.Adam([p for p in model_b.parameters() if p.requires_grad], lr=PHASE1_LR)
        p1b_losses = run_phase1(model_b, loader1b, opt1b, PHASE1_EPOCHS, "Phase 1-B (Custom)")
        torch.save(model_b.state_dict(), p1b_save_path)
        plot_loss_curve(p1b_losses, os.path.join(RESULT_DIR, "p1b_loss.png"))

        del model_b, yolo_b, opt1b, loader1b, ds1b
        torch.cuda.empty_cache()

    # ============================
    # Phase 2
    # ============================
    ds2 = PairedImageWithLabelsDataset(TRAIN_CLEAN_DIR, TRAIN_AUG_DIR, TRAIN_LABEL_DIR)
    loader2 = DataLoader(
        ds2,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=NUM_WORKERS,
        pin_memory=True,
        persistent_workers=NUM_WORKERS > 0,
        collate_fn=collate_with_labels,
    )

    phase1_variants = [
        ("A_ExLPose", p1a_save_path),
        ("B_Custom", p1b_save_path),
    ]

    for variant_name, p1_weights in phase1_variants:
        print(f"\n{'=' * 60}")
        print(f"[Phase 2] {variant_name} → Task Tuning (Frozen Teacher + Task Loss)")
        print(f"{'=' * 60}")

        if not os.path.exists(p1_weights):
            print(f"  [Skip] Phase 1 weights not found: {p1_weights}")
            continue

        yolo = YOLO(FOUNDATION_YOLO_WEIGHTS)
        model = TDDRPEndToEndPose(yolo).to(device)

        # Phase 1 가중치 로드: adapter(TD-DRP) 키만 선택적 로드
        # Phase 1-A(ExLPose)는 COCO 17-kpt로 학습되어 YOLO head 차원 불일치 →
        # adapter 가중치(illumination_encoder, soft_router, multi_scale_rsv, criterion)만 로드
        ADAPTER_PREFIXES = ("illumination_encoder.", "soft_router.", "multi_scale_rsv.", "criterion.")
        p1_state = torch.load(p1_weights, map_location=device)
        model_state = model.state_dict()
        adapter_state = {k: v for k, v in p1_state.items() if k.startswith(ADAPTER_PREFIXES)}
        model_state.update(adapter_state)
        model.load_state_dict(model_state)
        model.set_phase(2)
        assert_model_on_device(model, device, f"model_phase2_{variant_name}")

        task_criterion = init_task_loss(model._yolo_detection_model)

        backbone_params = list(model.yolo_model.parameters())
        backbone_ids = set(id(p) for p in backbone_params)
        adapter_params = [p for p in model.parameters() if id(p) not in backbone_ids and p.requires_grad]

        opt2 = torch.optim.Adam([
            {"params": [p for p in backbone_params if p.requires_grad], "lr": PHASE2_LR},
            {"params": adapter_params, "lr": PHASE2_LR * 10},
        ])

        p2_losses, p2_task_losses = run_phase2(
            model=model,
            loader=loader2,
            optimizer=opt2,
            num_epochs=PHASE2_EPOCHS,
            phase_name=f"Phase 2 ({variant_name})",
            task_criterion=task_criterion,
            task_weight=PHASE2_TASK_WEIGHT,
            grad_clip=1.0,
        )

        save_path = os.path.join(RESULT_DIR, f"td_drp_phase2_{variant_name}.pt")
        torch.save(model.state_dict(), save_path)
        print(f"  Saved: {save_path}")

        _, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        ax1.plot(range(1, len(p2_losses) + 1), p2_losses, "b-o", label="Total")
        ax1.set_title(f"Phase 2 Total Loss ({variant_name})")
        ax1.set_xlabel("Epoch")
        ax1.set_ylabel("Loss")
        ax1.grid(True)
        ax1.legend()

        ax2.plot(range(1, len(p2_task_losses) + 1), p2_task_losses, "r-o", label="Task Loss")
        ax2.set_title(f"Phase 2 Task Loss ({variant_name})")
        ax2.set_xlabel("Epoch")
        ax2.set_ylabel("Loss")
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(RESULT_DIR, f"p2_loss_{variant_name}.png"))
        plt.close()

        del model, yolo, task_criterion, opt2
        torch.cuda.empty_cache()

        print(f"\nTraining Complete. Results saved to {RESULT_DIR}/")



if __name__ == "__main__":
    train()