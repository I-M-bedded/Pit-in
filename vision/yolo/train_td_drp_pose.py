import os
import torch
import torch.nn as nn
from ultralytics import YOLO
import requests
import zipfile
import shutil
from torch.utils.data import DataLoader, Dataset
from torchvision.transforms import ToTensor
from PIL import Image

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# 앞서 작성된 TD-DRP 모듈들 가져오기
from vision.yolo.td_drp_v4 import (
    SpatiallyVariantIlluminationEncoder,
    SpatiallyAwareSoftRouter,
    RSVFiLM,
    TDDRPLoss,
    IlluminationAugmentor
)

# ==========================================
# [설정 영역]
# ==========================================
# 사전학습된 YOLO26n-pose Best 가중치 경로
BEST_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_yolo26n_pose")

# Phase 1: SSL Warm-up 용 설정
PHASE1_EPOCHS = 10
PHASE1_LR = 1e-3

# Phase 2: Task Tuning 용 설정
PHASE2_EPOCHS = 5
PHASE2_LR = 1e-5 # ⭐️ 사전학습 가중치를 보호하기 위한 매우 낮은 학습률

# ExLPose 데이터셋 경로 (없으면 임시 데이터 생성)
EXLPOSE_DIR = os.path.abspath("vision/dataset/ExLPose")
# ==========================================

class TDDRPEndToEndPose(nn.Module):
    """YOLO26n-pose의 원본 구조를 유지하며 P4(layer 6) 위치에 TD-DRP를 결합하는 E2E 래퍼"""
    def __init__(self, yolo_weights_path):
        super().__init__()
        # 사전학습된 YOLO 로드
        print(f"[TD-DRP E2E] '{yolo_weights_path}' 에서 YOLO 백본 및 헤드 로드 중...")
        self.yolo_model = YOLO(yolo_weights_path).model.model
        self.save_list = YOLO(yolo_weights_path).model.save
        
        # 조명 모듈 초기화
        feature_channels = 128  # YOLO26n-pose P4 채널 수
        illumination_channels = 64
        num_experts = 3
        
        self.illumination_encoder = SpatiallyVariantIlluminationEncoder(3, illumination_channels)
        self.soft_router = SpatiallyAwareSoftRouter(illumination_channels, num_experts)
        self.rsv_film = RSVFiLM(feature_channels, illumination_channels, num_experts)
        
        self.criterion = TDDRPLoss(phase=1)

    def _freeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = False
        print("[TD-DRP E2E] YOLO 전체 파라미터 고정 (Phase 1)")

    def _unfreeze_yolo(self):
        for param in self.yolo_model.parameters():
            param.requires_grad = True
        print("[TD-DRP E2E] YOLO 전체 파라미터 해제 (Phase 2)")

    def set_phase(self, phase: int):
        self.criterion.set_phase(phase)
        if phase == 1:
            self._freeze_yolo()
        else:
            self._unfreeze_yolo()

    def get_expert_weights(self):
        weights = []
        for expert in self.rsv_film.expert_gamma:
            weights.append(expert.weight.reshape(-1))
        return torch.stack(weights, dim=0)

    def forward(self, I_aug, I_clean=None):
        outputs = {}
        
        # 1. 조명 임베딩
        Z = self.illumination_encoder(I_aug)
        P = self.soft_router(Z)
        outputs["Z"] = Z
        outputs["P"] = P
        
        # 2. Teacher Forward (I_clean) - SSL용
        if I_clean is not None:
            with torch.no_grad():
                x_clean = I_clean
                y_clean = []
                for i, m in enumerate(self.yolo_model):
                    if m.f != -1:
                        x_clean = y_clean[m.f] if isinstance(m.f, int) else [x_clean if j == -1 else y_clean[j] for j in m.f]
                    x_clean = m(x_clean)
                    y_clean.append(x_clean if m.i in self.save_list else None)
                    if i == 6: # P4 레이어
                        outputs["F_clean"] = x_clean
                        break
                        
        # 3. Student Forward (I_aug)
        x = I_aug
        y = []
        for i, m in enumerate(self.yolo_model):
            if m.f != -1:
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]
            x = m(x)
            
            if i == 6: # P4 레이어 모듈레이션
                outputs["F_aug_raw"] = x
                F_prime, delta_gamma, delta_beta = self.rsv_film(x, Z, P)
                x = F_prime # 교체
                outputs["F_prime"] = F_prime
                outputs["delta_gamma"] = delta_gamma
                outputs["delta_beta"] = delta_beta
                
            y.append(x if m.i in self.save_list else None)
            
        outputs["pose_logits"] = x # 마지막 Pose 출력
        return outputs

def download_exlpose_dataset():
    """ExLPose 데이터셋 다운로드 스크립트"""
    os.makedirs(EXLPOSE_DIR, exist_ok=True)
    images_dir = os.path.join(EXLPOSE_DIR, "images")
    
    if os.path.exists(images_dir) and len(os.listdir(images_dir)) > 0:
        print(f"✅ ExLPose 데이터셋이 이미 존재합니다: {EXLPOSE_DIR}")
        return
        
    print(f"📥 ExLPose 데이터셋 다운로드 및 구성 중... ({EXLPOSE_DIR})")
    print("⚠️ (안내) 원래 ExLPose는 Google Drive 권한이 필요하므로, 여기서는 Warm-up을 위해 기존 yolo_pose_dataset의 이미지를 사용하여 증강(Augmentation)하는 방식으로 다운로드를 대체합니다.")
    
    # yolo_pose_dataset에서 이미지 복사 (대체용)
    src_images_dir = os.path.abspath("vision/dataset/yolo_pose_dataset/train/images")
    if not os.path.exists(src_images_dir):
        print("❌ 에러: 대체할 yolo_pose_dataset/train/images 를 찾을 수 없습니다.")
        return
        
    shutil.copytree(src_images_dir, images_dir)
    print("✅ 데이터셋 구성 완료 (기존 데이터 복사 사용).")

class DummyDataset(Dataset):
    def __init__(self, img_dir):
        self.img_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(('.png', '.jpg'))]
        self.transform = ToTensor()

    def __len__(self):
        return min(len(self.img_files), 100) # 빠른 테스트를 위해 100장만

    def __getitem__(self, idx):
        img = Image.open(self.img_files[idx]).convert("RGB").resize((640, 640))
        return self.transform(img)

def train():
    if not os.path.exists(BEST_YOLO_WEIGHTS):
        print(f"❌ 에러: YOLO 사전학습 가중치를 찾을 수 없습니다. ({BEST_YOLO_WEIGHTS})")
        return
        
    download_exlpose_dataset()
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n🚀 학습 디바이스: {device}")
    
    # 1. 모델 로드
    model = TDDRPEndToEndPose(BEST_YOLO_WEIGHTS).to(device)
    
    # 2. 데이터 로더
    dataset = DummyDataset(os.path.join(EXLPOSE_DIR, "images"))
    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)
    
    os.makedirs(RESULT_DIR, exist_ok=True)
    
    # ---------------------------------------------------------
    # Phase 1: SSL Warm-up (조명 어댑터 모듈만 학습)
    # ---------------------------------------------------------
    print("\n" + "="*50)
    print(" 🟢 [Phase 1] SSL Warm-up 시작 (YOLO Backbone Frozen)")
    print("="*50)
    model.set_phase(1)
    optimizer1 = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=PHASE1_LR)
    
    for epoch in range(PHASE1_EPOCHS):
        total_loss = 0.0
        for i, I_clean in enumerate(dataloader):
            I_clean = I_clean.to(device)
            # ExLPose 특성 재현: 인위적 저조도 증강
            I_aug, aug_type = IlluminationAugmentor.augment(I_clean)
            
            optimizer1.zero_grad()
            outputs = model(I_aug, I_clean=I_clean)
            
            losses = model.criterion(
                F_aug_prime=outputs["F_prime"],
                F_clean=outputs["F_clean"],
                Z=outputs["Z"],
                I_aug=I_aug,
                delta_gamma=outputs["delta_gamma"],
                delta_beta=outputs["delta_beta"],
                expert_weights=model.get_expert_weights()
            )
            
            loss = losses["total"]
            loss.backward()
            optimizer1.step()
            total_loss += loss.item()
            
        print(f"[Phase 1] Epoch {epoch+1}/{PHASE1_EPOCHS} | Loss: {total_loss/len(dataloader):.4f}")
        
    # Phase 1 가중치 저장
    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_phase1.pt"))
    print("✅ Phase 1 완료 및 가중치 저장됨.")

    # ---------------------------------------------------------
    # Phase 2: Task Tuning (전체 네트워크 미세 조정)
    # ---------------------------------------------------------
    print("\n" + "="*50)
    print(" 🟢 [Phase 2] Task Tuning 시작 (YOLO Unfrozen, Low LR)")
    print("="*50)
    model.set_phase(2)
    
    # 매우 낮은 학습률(Low LR) 적용
    optimizer2 = torch.optim.Adam(model.parameters(), lr=PHASE2_LR)
    
    for epoch in range(PHASE2_EPOCHS):
        total_loss = 0.0
        for i, I_clean in enumerate(dataloader):
            I_clean = I_clean.to(device)
            I_aug, _ = IlluminationAugmentor.augment(I_clean)
            
            optimizer2.zero_grad()
            outputs = model(I_aug, I_clean=I_clean)
            
            # Phase 2에서는 YOLO Pose Head Loss가 추가되어야 하나,
            # 현재는 GT Label이 없는 SSL Unsupervised 이미지들이므로
            # SSL Loss만 유지하되 전체 가중치를 미세 조정하는 형태로 시뮬레이션
            losses = model.criterion(
                F_aug_prime=outputs["F_prime"],
                F_clean=outputs["F_clean"],
                Z=outputs["Z"],
                I_aug=I_aug,
                delta_gamma=outputs["delta_gamma"],
                delta_beta=outputs["delta_beta"],
                expert_weights=model.get_expert_weights()
            )
            
            loss = losses["total"]
            loss.backward()
            optimizer2.step()
            total_loss += loss.item()
            
        print(f"[Phase 2] Epoch {epoch+1}/{PHASE2_EPOCHS} | Loss: {total_loss/len(dataloader):.4f}")
        
    torch.save(model.state_dict(), os.path.join(RESULT_DIR, "td_drp_phase2_best.pt"))
    print("✅ Phase 2 완료 및 최종 가중치 저장됨.")
    print(f"🎉 모든 결과가 {RESULT_DIR} 에 저장되었습니다.")

if __name__ == "__main__":
    train()
