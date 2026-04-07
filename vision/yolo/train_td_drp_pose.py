"""
TD-DRP 학습 파이프라인 (Multi-scale P3/P4/P5, YOLO26n-pose backbone)
Phase 1: SSL Warm-up on ExLPose dark images (backbone frozen)
Phase 2: Task Tuning on yolo_pose_dataset (backbone unfrozen, KD enabled)
"""
import os
import sys
import torch
from torch.utils.data import DataLoader, Dataset
from torchvision.transforms import ToTensor
from PIL import Image
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.td_drp import TDDRPEndToEndPose, IlluminationAugmentor

# ==========================================
# [설정 영역]
# ==========================================
BEST_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_yolo26n_pose")

BATCH_SIZE = 32
NUM_WORKERS = 8

# Phase 1: ExLPose dark
PHASE1_EPOCHS = 5
PHASE1_LR = 1e-3
EXLPOSE_IMG_DIR = os.path.abspath("vision/dataset/ExLPose/ExLPose/dark")

# Phase 2: yolo_pose_dataset (paired: images_clean + images_aug)
PHASE2_EPOCHS = 2
PHASE2_LR = 1e-5
CUSTOM_CLEAN_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/val/images_clean")
CUSTOM_AUG_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/val/images_aug")
# ==========================================


class ImageOnlyDataset(Dataset):
    """SSL 학습용 — 이미지만 로드 (라벨 불필요). Phase 1용."""
    def __init__(self, img_dir):
        self.img_dir = img_dir
        self.img_files = []
        if os.path.isdir(img_dir):
            self.img_files = sorted(
                f for f in os.listdir(img_dir) if f.lower().endswith(('.png', '.jpg'))
            )
        self.transform = ToTensor()

    def __len__(self):
        return len(self.img_files)

    def __getitem__(self, idx):
        path = os.path.join(self.img_dir, self.img_files[idx])
        img = Image.open(path).convert("RGB").resize((640, 640))
        return self.transform(img)


class PairedImageDataset(Dataset):
    """Paired clean/aug 이미지 로드. Phase 2용.
    같은 파일명으로 매칭된 (clean, aug) 쌍을 반환."""
    def __init__(self, clean_dir, aug_dir):
        self.clean_dir = clean_dir
        self.aug_dir = aug_dir
        self.transform = ToTensor()

        clean_files = set(
            f for f in os.listdir(clean_dir) if f.lower().endswith(('.png', '.jpg'))
        ) if os.path.isdir(clean_dir) else set()
        aug_files = set(
            f for f in os.listdir(aug_dir) if f.lower().endswith(('.png', '.jpg'))
        ) if os.path.isdir(aug_dir) else set()

        self.files = sorted(clean_files & aug_files)

    def __len__(self):
        return len(self.files)

    def __getitem__(self, idx):
        fname = self.files[idx]
        clean = Image.open(os.path.join(self.clean_dir, fname)).convert("RGB").resize((640, 640))
        aug = Image.open(os.path.join(self.aug_dir, fname)).convert("RGB").resize((640, 640))
        return self.transform(clean), self.transform(aug)


def run_phase(model, loader, optimizer, num_epochs, phase_name,
              grad_clip=None, paired=False):
    """
    paired=False: loader가 I_clean 하나만 반환 → IlluminationAugmentor로 I_aug 생성
    paired=True:  loader가 (I_clean, I_aug) 쌍을 반환 → 실제 paired 데이터 사용
    """
    device = next(model.parameters()).device
    for epoch in range(num_epochs):
        total_loss = 0.0
        for batch in loader:
            if paired:
                I_clean, I_aug = batch
                I_clean = I_clean.to(device, non_blocking=True)
                I_aug = I_aug.to(device, non_blocking=True)
            else:
                I_clean = batch.to(device, non_blocking=True)
                I_aug, _ = IlluminationAugmentor.augment(I_clean)

            optimizer.zero_grad()
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

        avg = total_loss / len(loader)
        print(f"[{phase_name}] Epoch {epoch+1}/{num_epochs} | Loss: {avg:.4f}")


def train():
    if not os.path.exists(BEST_YOLO_WEIGHTS):
        print(f"YOLO weights not found: {BEST_YOLO_WEIGHTS}")
        return

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    if torch.cuda.is_available():
        print(f"  GPU: {torch.cuda.get_device_name(0)}")

    yolo_model = YOLO(BEST_YOLO_WEIGHTS)
    model = TDDRPEndToEndPose(yolo_model).to(device)
    os.makedirs(RESULT_DIR, exist_ok=True)

    # === Phase 1: SSL Warm-up (ExLPose) ===
    print("\n" + "=" * 50)
    print(" [Phase 1] SSL Warm-up — ExLPose dark, backbone frozen")
    print("=" * 50)

    ds1 = ImageOnlyDataset(EXLPOSE_IMG_DIR)
    if len(ds1) == 0:
        print(f"No images in {EXLPOSE_IMG_DIR}")
        return
    print(f"ExLPose: {len(ds1)} images")

    loader1 = DataLoader(ds1, batch_size=BATCH_SIZE, shuffle=True,
                         num_workers=NUM_WORKERS, pin_memory=True)
    model.set_phase(1)
    opt1 = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=PHASE1_LR)
    run_phase(model, loader1, opt1, PHASE1_EPOCHS, "Phase 1")

    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_phase1.pt"))
    print("Phase 1 saved.")

    # === Phase 2: Task Tuning (yolo_pose_dataset) ===
    print("\n" + "=" * 50)
    print(" [Phase 2] Task Tuning — yolo_pose_dataset, backbone unfrozen, KD on")
    print("=" * 50)

    ds2 = PairedImageDataset(CUSTOM_CLEAN_DIR, CUSTOM_AUG_DIR)
    if len(ds2) == 0:
        print(f"No paired images in {CUSTOM_CLEAN_DIR} / {CUSTOM_AUG_DIR}")
        return
    print(f"Paired dataset: {len(ds2)} pairs (clean + aug)")

    loader2 = DataLoader(ds2, batch_size=BATCH_SIZE, shuffle=True,
                         num_workers=NUM_WORKERS, pin_memory=True)
    model.set_phase(2)
    opt2 = torch.optim.Adam(model.parameters(), lr=PHASE2_LR)
    run_phase(model, loader2, opt2, PHASE2_EPOCHS, "Phase 2",
              grad_clip=1.0, paired=True)

    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_phase2_best.pt"))
    print(f"Phase 2 saved. Results in {RESULT_DIR}")


if __name__ == "__main__":
    train()
