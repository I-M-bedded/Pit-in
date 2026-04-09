import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
import os
import cv2
from PIL import Image

def visualize_inference(model, image_path, save_path, device, title="Inference"):
    """
    TD-DRP 모델을 사용하여 조명 맵과 실제 Pose 결과를 모두 시각화
    """
    model.eval()
    img = Image.open(image_path).convert("RGB")
    img_np_orig = np.array(img.resize((640, 640)))
    img_tensor = torch.from_numpy(img_np_orig).permute(2, 0, 1).float().unsqueeze(0).to(device) / 255.0

    with torch.no_grad():
        outputs = model(img_tensor)
        # outputs["x_aug"][0] -> [1, 300, 57] (후처리된 결과)
        preds = outputs["x_aug"][0]
        Z = outputs["Z"][0].cpu().numpy()

    # 1. 조명 맵 생성
    Z_mean = np.mean(Z, axis=0)
    Z_mean = (Z_mean - Z_mean.min()) / (Z_mean.max() - Z_mean.min() + 1e-8)
    Z_heatmap = cv2.applyColorMap((Z_mean * 255).astype(np.uint8), cv2.COLORMAP_JET)
    Z_heatmap = cv2.resize(Z_heatmap, (640, 640))
    illum_res = cv2.addWeighted(img_np_orig, 0.6, Z_heatmap, 0.4, 0)

    # 2. Pose 결과 그리기 (Top-5 점수 박스만)
    pose_res = cv2.cvtColor(img_np_orig, cv2.COLOR_RGB2BGR)
    
    # 점수 기준 내림차순 정렬되어 있다고 가정 (보통 300개는 점수순)
    # 점수가 너무 낮으면(파괴된 상태) 박스가 안 보일 수 있음
    for i in range(5):
        det = preds[i]
        conf = det[4].item()
        if conf < 0.05: continue # 아주 낮은 점수는 무시
        
        box = det[:4].cpu().numpy()
        kpts = det[5:56].cpu().numpy().reshape(-1, 3) # 17개 점
        
        # 좌표 복원 (0~1 scale인 경우 640 곱함)
        if box[2] <= 1.0:
            box = box * 640
            kpts[:, :2] *= 640
            
        x1, y1, x2, y2 = box
        cv2.rectangle(pose_res, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(pose_res, f"{conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        for kp in kpts:
            kx, ky, kv = kp
            if kv > 0.1:
                cv2.circle(pose_res, (int(kx), int(ky)), 3, (0, 0, 255), -1)

    pose_res = cv2.cvtColor(pose_res, cv2.COLOR_BGR2RGB)

    # 3. 플롯 생성
    plt.figure(figsize=(15, 5))
    plt.subplot(1, 3, 1)
    plt.imshow(img_np_orig)
    plt.title("Input")
    plt.axis("off")
    
    plt.subplot(1, 3, 2)
    plt.imshow(illum_res)
    plt.title("Illumination Map (Z)")
    plt.axis("off")
    
    plt.subplot(1, 3, 3)
    plt.imshow(pose_res)
    plt.title("Pose Result (Box/Kpts)")
    plt.axis("off")
    
    plt.suptitle(title)
    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()

def plot_loss_curve(losses, save_path):
    plt.figure(figsize=(10, 6))
    plt.plot(range(1, len(losses) + 1), losses, marker='o')
    plt.title("Training Loss Curve")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.grid(True)
    plt.savefig(save_path)
    plt.close()
