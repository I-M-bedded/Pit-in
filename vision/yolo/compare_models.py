import os
import glob
import numpy as np
import torch
from ultralytics import YOLO
import sys
import pandas as pd
import matplotlib.pyplot as plt

# 프로젝트 루트 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.train_td_drp_pose import TDDRPEndToEndPose

# ==========================================
# [설정 영역]
# ==========================================
BASE_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
TD_DRP_WEIGHTS = os.path.abspath("result/td_drp_yolo26n_pose/td_drp_phase2_best.pt")

TEST_IMG_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test/images")
TEST_LBL_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test/labels")

RESULT_DIR = os.path.abspath("result/compare")
# ==========================================

def calculate_iou(box1, box2):
    """Calculate IoU between two bounding boxes (cx, cy, w, h format -> normalized)"""
    # box: [cx, cy, w, h]
    b1_x1, b1_y1 = box1[0] - box1[2]/2, box1[1] - box1[3]/2
    b1_x2, b1_y2 = box1[0] + box1[2]/2, box1[1] + box1[3]/2
    
    b2_x1, b2_y1 = box2[0] - box2[2]/2, box2[1] - box2[3]/2
    b2_x2, b2_y2 = box2[0] + box2[2]/2, box2[1] + box2[3]/2
    
    inter_x1 = max(b1_x1, b2_x1)
    inter_y1 = max(b1_y1, b2_y1)
    inter_x2 = min(b1_x2, b2_x2)
    inter_y2 = min(b1_y2, b2_y2)
    
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    b1_area = (b1_x2 - b1_x1) * (b1_y2 - b1_y1)
    b2_area = (b2_x2 - b2_x1) * (b2_y2 - b2_y1)
    
    union_area = b1_area + b2_area - inter_area
    if union_area == 0: return 0
    return inter_area / union_area

def parse_label(label_path):
    """Parse YOLO pose label file"""
    gts = []
    if not os.path.exists(label_path):
        return gts
    with open(label_path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            class_id = int(parts[0])
            box = parts[1:5] # cx, cy, w, h (normalized)
            # keypoints: x, y, vis
            kpts = []
            for i in range(5, len(parts), 3):
                if i + 2 < len(parts):
                    kpts.append([parts[i], parts[i+1], parts[i+2]])
            gts.append({'class': class_id, 'box': box, 'keypoints': np.array(kpts)})
    return gts

def evaluate_predictions(preds, gts, iou_thresh=0.5):
    """
    preds: list of dict {'box': [cx, cy, w, h], 'keypoints': [[x,y], ...]} (normalized)
    gts: list of dict parsed from label
    Returns: matched count, total gt count, sum of RMSE for matched keypoints
    """
    matched_gts = set()
    total_rmse = 0.0
    matches = 0
    
    for pred in preds:
        best_iou = 0
        best_gt_idx = -1
        for i, gt in enumerate(gts):
            if i in matched_gts: continue
            iou = calculate_iou(pred['box'], gt['box'])
            if iou > best_iou:
                best_iou = iou
                best_gt_idx = i
                
        if best_iou > iou_thresh and best_gt_idx != -1:
            matched_gts.add(best_gt_idx)
            matches += 1
            
            # Calculate RMSE for visible keypoints
            pred_kpts = np.array(pred['keypoints']) # (N, 2)
            gt_kpts = gts[best_gt_idx]['keypoints'] # (N, 3)
            
            # Normalize RMSE by distance (Euclidean distance in normalized 0-1 coordinates)
            rmse = 0.0
            valid_pts = 0
            for pk, gk in zip(pred_kpts, gt_kpts):
                vis = gk[2]
                if vis > 0: # Calculate error even if occluded (vis=1) or visible (vis=2)
                    dist = np.sqrt((pk[0] - gk[0])**2 + (pk[1] - gk[1])**2)
                    rmse += dist
                    valid_pts += 1
            
            if valid_pts > 0:
                total_rmse += (rmse / valid_pts)
                
    return matches, len(gts), total_rmse

def get_difficulty(filename):
    """Determine difficulty based on filename prefix (output_masks_X)"""
    # Extract the number X from output_masks_X
    parts = filename.split('output_masks_')
    if len(parts) < 2: return "Unknown"
    
    num_str = parts[1].split('_')[0]
    try:
        num = int(num_str)
        if num == 0: return "Easy (Normal)"
        elif 1 <= num <= 4: return "Mild (Low-light)"
        elif 5 <= num <= 9: return "Severe (Low-light)"
    except:
        pass
    return "Unknown"

def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    print("🚀 성능 비교 평가 시작...")
    
    # 1. Load Baseline YOLO
    print(f"📦 로딩: Baseline YOLO26n-pose")
    model_base = YOLO(BASE_YOLO_WEIGHTS)
    
    # 2. Load TD-DRP YOLO
    print(f"📦 로딩: TD-DRP YOLO26n-pose")
    td_drp_wrapper = TDDRPEndToEndPose(BASE_YOLO_WEIGHTS)
    td_drp_wrapper.load_state_dict(torch.load(TD_DRP_WEIGHTS, map_location=device))
    td_drp_wrapper.to(device)
    td_drp_wrapper.eval()
    
    model_tddrp = YOLO(BASE_YOLO_WEIGHTS)
    def custom_forward(x, *args, **kwargs):
        with torch.no_grad():
            outputs = td_drp_wrapper(x)
        return outputs["pose_logits"]
    model_tddrp.model.forward = custom_forward
    
    img_paths = glob.glob(os.path.join(TEST_IMG_DIR, "*.png")) + glob.glob(os.path.join(TEST_IMG_DIR, "*.jpg"))
    
    results = {
        "Easy (Normal)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0},
        "Mild (Low-light)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0},
        "Severe (Low-light)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0}
    }
    
    print(f"\n🔍 총 {len(img_paths)}장의 테스트 이미지 평가 진행 중...")
    
    # Helper to format predictions
    def get_preds(result, img_w, img_h):
        preds = []
        if result.boxes is not None and result.keypoints is not None:
            boxes = result.boxes.xywhn.cpu().numpy() # cx, cy, w, h (normalized)
            kpts = result.keypoints.xyn.cpu().numpy() # x, y (normalized)
            for b, k in zip(boxes, kpts):
                preds.append({'box': b, 'keypoints': k})
        return preds

    for i, img_path in enumerate(img_paths):
        filename = os.path.basename(img_path)
        diff = get_difficulty(filename)
        if diff == "Unknown": continue
        
        label_path = os.path.join(TEST_LBL_DIR, filename.replace('.png', '.txt').replace('.jpg', '.txt'))
        gts = parse_label(label_path)
        if not gts: continue # GT 없는 이미지는 건너뜀
        
        results[diff]["total_gt"] += len(gts)
        results[diff]["count"] += 1
        
        # Inference Baseline
        res_base = model_base.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        preds_base = get_preds(res_base, res_base.orig_shape[1], res_base.orig_shape[0])
        
        b_match, _, b_rmse = evaluate_predictions(preds_base, gts)
        results[diff]["base_matches"] += b_match
        results[diff]["base_rmse"] += b_rmse
        
        # Inference TD-DRP
        res_td = model_tddrp.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        preds_td = get_preds(res_td, res_td.orig_shape[1], res_td.orig_shape[0])
        
        t_match, _, t_rmse = evaluate_predictions(preds_td, gts)
        results[diff]["td_matches"] += t_match
        results[diff]["td_rmse"] += t_rmse
        
        if (i+1) % 50 == 0:
            print(f"  진행률: {i+1}/{len(img_paths)}")
            
    # Calculate Final Metrics
    print("\n📊 === 평가 결과 집계 ===")
    records = []
    for diff, data in results.items():
        if data["total_gt"] == 0: continue
        
        # Recall (Detection Rate)
        base_recall = (data["base_matches"] / data["total_gt"]) * 100
        td_recall = (data["td_matches"] / data["total_gt"]) * 100
        
        # Average RMSE Error (Normalized scale 0~1 -> converted to percentage 0~100 for readability)
        # Lower is better!
        base_avg_rmse = (data["base_rmse"] / max(data["base_matches"], 1)) * 100
        td_avg_rmse = (data["td_rmse"] / max(data["td_matches"], 1)) * 100
        
        records.append({
            "Difficulty": diff,
            "Total Objects": data["total_gt"],
            "Baseline Recall (%)": round(base_recall, 2),
            "TD-DRP Recall (%)": round(td_recall, 2),
            "Baseline Kpt Error (↓)": round(base_avg_rmse, 4),
            "TD-DRP Kpt Error (↓)": round(td_avg_rmse, 4)
        })
        
    df = pd.DataFrame(records)
    print("\n", df.to_markdown(index=False))
    
    # Save to CSV
    csv_path = os.path.join(RESULT_DIR, "performance_comparison.csv")
    df.to_csv(csv_path, index=False)
    
    # Plotting
    labels = df["Difficulty"]
    x = np.arange(len(labels))
    width = 0.35
    
    # 1. Recall Plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(x - width/2, df["Baseline Recall (%)"], width, label='Baseline YOLO26', color='lightcoral')
    ax.bar(x + width/2, df["TD-DRP Recall (%)"], width, label='TD-DRP YOLO26', color='dodgerblue')
    ax.set_ylabel('Recall (%) - Higher is better')
    ax.set_title('Detection Recall by Illumination Difficulty')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    plt.savefig(os.path.join(RESULT_DIR, "recall_comparison.png"))
    
    # 2. Keypoint Error Plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(x - width/2, df["Baseline Kpt Error (↓)"], width, label='Baseline YOLO26', color='lightcoral')
    ax.bar(x + width/2, df["TD-DRP Kpt Error (↓)"], width, label='TD-DRP YOLO26', color='dodgerblue')
    ax.set_ylabel('Keypoint RMSE Error (%) - Lower is better')
    ax.set_title('Keypoint Estimation Error by Illumination Difficulty')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    plt.savefig(os.path.join(RESULT_DIR, "keypoint_error_comparison.png"))
    
    print(f"\n✅ 평가 완료! CSV 및 그래프 이미지가 {RESULT_DIR} 에 저장되었습니다.")

if __name__ == "__main__":
    main()
