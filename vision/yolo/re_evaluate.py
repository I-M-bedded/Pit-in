"""
TD-DRP vs Foundation Model (best.pt) 정밀 비교 스크립트
- Head 출력 형식을 YOLO Predictor와 완벽히 일치시킴
- NMS 및 Confidence 임계값을 동일하게 설정하여 객관적 비교
"""
import os
import sys
import torch
import numpy as np
import pandas as pd
import glob
from PIL import Image
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.td_drp import TDDRPEndToEndPose

# ==========================================
# [설정 영역]
# ==========================================
BASE_YOLO_PT = os.path.abspath("yolo26n-pose.pt")
FOUNDATION_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
TD_DRP_WEIGHTS = os.path.abspath("result/td_drp_yolo26n_pose/td_drp_phase2_best.pt")
VAL_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test")
RESULT_DIR = os.path.abspath("result/td_drp_yolo26n_pose")
# ==========================================

def calculate_iou(box1, box2):
    b1_x1, b1_y1 = box1[0] - box1[2]/2, box1[1] - box1[3]/2
    b1_x2, b1_y2 = box1[0] + box1[2]/2, box1[1] + box1[3]/2
    b2_x1, b2_y1 = box2[0] - box2[2]/2, box2[1] - box2[3]/2
    b2_x2, b2_y2 = box2[0] + box2[2]/2, box2[1] + box2[3]/2
    inter_x1, inter_y1 = max(b1_x1, b2_x1), max(b1_y1, b2_y1)
    inter_x2, inter_y2 = min(b1_x2, b2_x2), min(b1_y2, b2_y2)
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    union_area = (b1_x2 - b1_x1) * (b1_y2 - b1_y1) + (b2_x2 - b2_x1) * (b2_y2 - b2_y1) - inter_area
    return inter_area / union_area if union_area > 0 else 0

def parse_label(label_path):
    gts = []
    if not os.path.exists(label_path): return gts
    with open(label_path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            gts.append({'class': int(parts[0]), 'box': parts[1:5], 'keypoints': np.array(parts[5:]).reshape(-1, 3)})
    return gts

def build_tddrp_predictor(base_pt, tddrp_weights, device):
    """TD-DRP 래핑된 YOLO 모델 생성 (Head 출력 유지)"""
    # 1. 원본 구조 로드
    yolo = YOLO(base_pt)
    # 2. TD-DRP 래퍼 생성 및 가중치 로드
    tddrp_wrapper = TDDRPEndToEndPose(yolo).to(device)
    tddrp_wrapper.load_state_dict(torch.load(tddrp_weights, map_location=device))
    tddrp_wrapper.eval()
    
    # 3. Predict용 YOLO 객체 생성 (동일한 가중치 공유)
    # 이 객체의 forward를 tddrp_wrapper의 로직으로 대체
    predictor_model = YOLO(base_pt)
    
    def custom_forward(x, *args, **kwargs):
        with torch.no_grad():
            # tddrp_wrapper.forward() -> outputs["x_aug"]는 Head의 원본 출력을 포함함
            # 이 x_aug는 (inference_output, train_output) 튜플임
            out = tddrp_wrapper(x)
            return out["x_aug"]

    predictor_model.model.forward = custom_forward
    return predictor_model

def run_eval(model, img_dir, lbl_dir, condition_name):
    img_paths = sorted(glob.glob(os.path.join(img_dir, "*.png")) + glob.glob(os.path.join(img_dir, "*.jpg")))
    matches, total_gt, total_rmse = 0, 0, 0.0
    
    print(f"  [{condition_name}] Evaluating {len(img_paths)} images...")
    
    for i, img_path in enumerate(img_paths):
        fname = os.path.basename(img_path)
        gts = parse_label(os.path.join(lbl_dir, os.path.splitext(fname)[0] + ".txt"))
        if not gts: continue
        total_gt += len(gts)
        
        # conf=0.1로 낮추어 모델이 아주 약하게라도 잡는지 확인
        results = model.predict(img_path, imgsz=640, conf=0.1, verbose=False)[0]
        
        preds = []
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes.xywhn.cpu().numpy()
            kpts = results.keypoints.xyn.cpu().numpy()
            for b, k in zip(boxes, kpts):
                preds.append({'box': b, 'keypoints': k})
        
        matched_indices = set()
        for pred in preds:
            best_iou, best_idx = 0.45, -1 # 타이트한 IOU 기준
            for j, gt in enumerate(gts):
                if j in matched_indices: continue
                iou = calculate_iou(pred['box'], gt['box'])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = j
            
            if best_idx != -1:
                matched_indices.add(best_idx)
                matches += 1
                pk, gk = np.array(pred['keypoints']), gts[best_idx]['keypoints']
                rmse, valid = 0.0, 0
                for p, g in zip(pk, gk):
                    if g[2] > 0:
                        rmse += np.sqrt((p[0]-g[0])**2 + (p[1]-g[1])**2)
                        valid += 1
                if valid > 0:
                    total_rmse += rmse / valid
        
        if (i+1) % 100 == 0:
            print(f"    Processed {i+1}/{len(img_paths)}...")

    recall = (matches / total_gt * 100) if total_gt > 0 else 0
    avg_rmse = (total_rmse / max(matches, 1) * 100)
    return recall, avg_rmse

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    
    if not os.path.exists(TD_DRP_WEIGHTS):
        print(f"Weights not found: {TD_DRP_WEIGHTS}")
        return

    print("Loading models...")
    model_foundation = YOLO(FOUNDATION_YOLO_WEIGHTS)
    model_tddrp = build_tddrp_predictor(BASE_YOLO_PT, TD_DRP_WEIGHTS, device)
    
    conds = [
        ("Clean", "images_clean", "labels_clean"),
        ("Augmented", "images_aug", "labels_aug")
    ]
    
    final_stats = []
    
    for name, img_sub, lbl_sub in conds:
        img_dir = os.path.join(VAL_DIR, img_sub)
        lbl_dir = os.path.join(VAL_DIR, lbl_sub)
        
        if not os.path.isdir(img_dir): continue
        
        print(f"\nEvaluating Foundation Model on {name}...")
        f_rec, f_err = run_eval(model_foundation, img_dir, lbl_dir, name)
        
        print(f"Evaluating TD-DRP Model on {name}...")
        t_rec, t_err = run_eval(model_tddrp, img_dir, lbl_dir, name)
        
        final_stats.append({
            "Condition": name,
            "Foundation Recall": f"{f_rec:.2f}%",
            "TD-DRP Recall": f"{t_rec:.2f}%",
            "Foundation Error": f"{f_err:.4f}",
            "TD-DRP Error": f"{t_err:.4f}",
            "Improvement": f"{t_rec - f_rec:+.2f}%"
        })
    
    df = pd.DataFrame(final_stats)
    print("\n" + "="*80)
    print("FINAL PERFORMANCE COMPARISON")
    print("="*80)
    print(df.to_markdown(index=False))
    
    df.to_csv(os.path.join(RESULT_DIR, "re_evaluation_report.csv"), index=False)
    print(f"\nReport saved to {RESULT_DIR}/re_evaluation_report.csv")

if __name__ == "__main__":
    main()
