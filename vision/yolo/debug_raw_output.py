"""
TD-DRP 직접 추론 및 시각화 검증 스크립트
API 레이어를 통하지 않고 모델의 Raw 출력을 직접 분석
"""
import torch
import numpy as np
import cv2
import os
import sys
from PIL import Image
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from vision.yolo.td_drp import TDDRPEndToEndPose

def debug_inference(img_path, base_pt, tddrp_weights, device):
    print(f"\nAnalyzing: {os.path.basename(img_path)}")
    
    # 1. 모델 준비
    yolo = YOLO(base_pt)
    model = TDDRPEndToEndPose(yolo).to(device)
    model.load_state_dict(torch.load(tddrp_weights, map_location=device))
    model.eval()
    
    # 2. 이미지 전처리
    img_pil = Image.open(img_path).convert("RGB")
    img_resized = img_pil.resize((640, 640))
    img_t = torch.from_numpy(np.array(img_resized)).permute(2, 0, 1).float().unsqueeze(0).to(device) / 255.0
    
    # 3. 추론
    with torch.no_grad():
        outputs = model(img_t)
        # x_aug는 (inference_output, train_output) 튜플
        raw_preds = outputs["x_aug"][0] # (1, 4+1+K*3, 8400) 형태 예상
        
    print(f"  Raw output shape: {raw_preds.shape}")
    
    # 4. 데이터 해석 (Shape: [1, 300, 57])
    # 300은 박스 개수, 57은 4(box) + 1(score) + 52(kpts?)
    # 52 kpts는 (17 points * 3) + alpha 일 가능성이 높음 (COCO 기준 17*3=51)
    
    # 첫 번째 박스(가장 점수 높은 것) 추출
    # 이미 정렬되어 있을 가능성이 큼
    best_idx = 0
    best_pred = raw_preds[0, best_idx, :]
    
    score = best_pred[4].item()
    print(f"  Top-1 Score: {score:.4f}")
    
    box = best_pred[:4].cpu().numpy()
    # Keypoints 파싱 (나머지 52개 중 51개 사용)
    kpts_raw = best_pred[5:56].cpu().numpy() # 17 * 3 = 51
    kpts = kpts_raw.reshape(-1, 3)
    
    print(f"  Top-1 Box (norm): {box}")
    
    # 결과 그리기
    img_cv = cv2.cvtColor(np.array(img_resized), cv2.COLOR_RGB2BGR)
    
    # Box (x1, y1, x2, y2 인지 cx, cy, w, h 인지 확인 필요)
    # 보통 후처리된 300개는 x1, y1, x2, y2 일 확률이 높음
    x1, y1, x2, y2 = box
    # 만약 좌표가 0~1 사이라면 640을 곱함
    if x2 <= 1.0:
        x1, y1, x2, y2 = x1*640, y1*640, x2*640, y2*640
    
    cv2.rectangle(img_cv, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    
    # Keypoints
    for k in kpts:
        kx, ky, kv = k
        if kx <= 1.0: kx, ky = kx*640, ky*640
        if kv > 0.1 or kv < 0: # 점수가 이상하므로 일단 다 그림
            cv2.circle(img_cv, (int(kx), int(ky)), 3, (0, 0, 255), -1)
            
    save_path = f"debug_res_{os.path.basename(img_path)}"
    cv2.imwrite(save_path, img_cv)
    print(f"  Saved visual debug to: {save_path}")

if __name__ == "__main__":
    try:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        BASE_PT = "yolo26n-pose.pt"
        TDDRP_W = "result/td_drp_yolo26n_pose/td_drp_phase2_best.pt"
        
        # 테스트할 이미지 (정상 1개, 증강 1개)
        test_imgs = [
            "vision/dataset/yolo_pose_dataset/test/images_clean/000003.png",
            "vision/dataset/yolo_pose_dataset/test/images_aug/000003.png"
        ]
        
        for img in test_imgs:
            if os.path.exists(img):
                debug_inference(img, BASE_PT, TDDRP_W, device)
            else:
                print(f"File not found: {img}")
    except Exception as e:
        import traceback
        print(f"Error occurred: {e}")
        traceback.print_exc()
