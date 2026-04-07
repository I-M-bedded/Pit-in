import os
import sys
import torch
import torch.nn as nn
from ultralytics import YOLO
from torch.utils.data import DataLoader, Dataset
from torchvision.transforms import ToTensor
from PIL import Image

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.td_drp_v4 import IlluminationAugmentor
from vision.yolo.td_drp_v5 import TDDRPEndToEndPoseV5

# ==========================================
# [설정 영역]
# ==========================================
BEST_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_yolo26n_pose_v5")

# GPU 활용 극대화 설정 (RTX 4070 12GB 기준)
BATCH_SIZE = 32
NUM_WORKERS = 8

# Phase 1: SSL Warm-up (ExLPose 데이터셋 사용)
PHASE1_EPOCHS = 5
PHASE1_LR = 1e-3
EXLPOSE_IMG_DIR = os.path.abspath("vision/dataset/ExLPose/ExLPose/dark") # 폴백 경로

# Phase 2: Task Tuning (Custom yolo_pose_dataset 사용)
PHASE2_EPOCHS = 2
PHASE2_LR = 1e-5
CUSTOM_IMG_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/train/images")
# ==========================================

class PoseImageDataset(Dataset):
    """디렉토리에서 이미지를 로드하는 범용 데이터셋"""
    def __init__(self, img_dir, max_size=None):
        self.img_dir = img_dir
        self.img_files = []
        if os.path.exists(img_dir):
            # 하위 폴더까지 검색할 경우 고려
            if os.path.isdir(os.path.join(img_dir, 'imgs')):
                self.img_dir = os.path.join(img_dir, 'imgs')
                
            self.img_files = [f for f in os.listdir(self.img_dir) if f.lower().endswith(('.png', '.jpg'))]
            
        if max_size and len(self.img_files) > max_size:
            self.img_files = self.img_files[:max_size]
            
        self.transform = ToTensor()

    def __len__(self):
        return len(self.img_files)

    def __getitem__(self, idx):
        img_name = self.img_files[idx]
        img_path = os.path.join(self.img_dir, img_name)
        img = Image.open(img_path).convert("RGB").resize((640, 640))
        return self.transform(img)

def train():
    if not os.path.exists(BEST_YOLO_WEIGHTS):
        print(f"❌ 에러: YOLO 사전학습 가중치를 찾을 수 없습니다. ({BEST_YOLO_WEIGHTS})")
        return
        
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n🚀 학습 디바이스: {device}")
    if torch.cuda.is_available():
        print(f"   GPU명: {torch.cuda.get_device_name(0)}")
        print(f"   Batch Size: {BATCH_SIZE}, Workers: {NUM_WORKERS} (GPU 풀가동 모드)")
    
    yolo_model = YOLO(BEST_YOLO_WEIGHTS)
    model = TDDRPEndToEndPoseV5(yolo_model).to(device)
    
    os.makedirs(RESULT_DIR, exist_ok=True)
    
    # ---------------------------------------------------------
    # Phase 1: SSL Warm-up (ExLPose 사용)
    # ---------------------------------------------------------
    print("\n" + "="*50)
    print(" 🟢 [Phase 1] SSL Warm-up 시작 (ExLPose 데이터, YOLO Backbone Frozen)")
    print("="*50)
    
    dataset_p1 = PoseImageDataset(EXLPOSE_IMG_DIR)
    if len(dataset_p1) == 0:
        print(f"❌ 에러: ExLPose 이미지를 찾을 수 없습니다. ({EXLPOSE_IMG_DIR})")
        return
    print(f"📥 ExLPose 데이터 {len(dataset_p1)}장 로드 완료.")
    
    # pin_memory=True를 주어 CPU->GPU 데이터 전송 속도 극대화
    loader_p1 = DataLoader(dataset_p1, batch_size=BATCH_SIZE, shuffle=True, num_workers=NUM_WORKERS, pin_memory=True)
    
    model.set_phase(1)
    optimizer1 = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=PHASE1_LR)
    
    for epoch in range(PHASE1_EPOCHS):
        total_loss = 0.0
        for i, I_clean in enumerate(loader_p1):
            I_clean = I_clean.to(device, non_blocking=True)
            # SSL을 위해 원본을 clean으로 두고, augment 된 것을 student에 넣음
            I_aug, _ = IlluminationAugmentor.augment(I_clean)
            
            optimizer1.zero_grad()
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
                pose_logits_clean=outputs.get("pose_logits_clean")
            )
            
            loss = losses["total"]
            loss.backward()
            optimizer1.step()
            total_loss += loss.item()
            
        print(f"[Phase 1] Epoch {epoch+1}/{PHASE1_EPOCHS} | Loss: {total_loss/len(loader_p1):.4f}")
        
    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_v5_phase1.pt"))
    print("✅ Phase 1 완료 및 가중치 저장됨.")

    # ---------------------------------------------------------
    # Phase 2: Task Tuning (Custom Dataset 사용)
    # ---------------------------------------------------------
    print("\n" + "="*50)
    print(" 🟢 [Phase 2] Task Tuning 시작 (Custom 데이터, YOLO Unfrozen, KD Loss Enabled)")
    print("="*50)
    
    dataset_p2 = PoseImageDataset(CUSTOM_IMG_DIR)
    if len(dataset_p2) == 0:
        print(f"❌ 에러: Custom 데이터셋 이미지를 찾을 수 없습니다. ({CUSTOM_IMG_DIR})")
        return
    print(f"📥 Custom 데이터 {len(dataset_p2)}장 로드 완료.")
    
    loader_p2 = DataLoader(dataset_p2, batch_size=BATCH_SIZE, shuffle=True, num_workers=NUM_WORKERS, pin_memory=True)
    
    model.set_phase(2)
    optimizer2 = torch.optim.Adam(model.parameters(), lr=PHASE2_LR)
    
    for epoch in range(PHASE2_EPOCHS):
        total_loss = 0.0
        for i, I_clean in enumerate(loader_p2):
            I_clean = I_clean.to(device, non_blocking=True)
            I_aug, _ = IlluminationAugmentor.augment(I_clean)
            
            optimizer2.zero_grad()
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
                pose_logits_clean=outputs.get("pose_logits_clean")
            )
            
            loss = losses["total"]
            loss.backward()
            
            # Gradient clipping to prevent exploding gradients
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            
            optimizer2.step()
            total_loss += loss.item()
            
        print(f"[Phase 2] Epoch {epoch+1}/{PHASE2_EPOCHS} | Loss: {total_loss/len(loader_p2):.4f}")
        
    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_v5_phase2_best.pt"))
    print("✅ Phase 2 완료 및 최종 가중치 저장됨.")

if __name__ == "__main__":
    train()
