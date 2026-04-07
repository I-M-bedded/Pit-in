import os
import glob
import numpy as np
import torch
from ultralytics import YOLO
import sys
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.td_drp_v5 import TDDRPEndToEndPoseV5

# ==========================================
# [설정 영역]
# ==========================================
BASE_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
TD_DRP_WEIGHTS = os.path.abspath("result/td_drp_yolo26n_pose_v5/td_drp_v5_phase2_best.pt")

TEST_IMG_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test/images")
TEST_LBL_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test/labels")

RESULT_DIR = os.path.abspath("result/compare_v5")
# ==========================================

def calculate_iou(box1, box2):
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
    gts = []
    if not os.path.exists(label_path):
        return gts
    with open(label_path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            class_id = int(parts[0])
            box = parts[1:5]
            kpts = []
            for i in range(5, len(parts), 3):
                if i + 2 < len(parts):
                    kpts.append([parts[i], parts[i+1], parts[i+2]])
            gts.append({'class': class_id, 'box': box, 'keypoints': np.array(kpts)})
    return gts

def evaluate_predictions(preds, gts, iou_thresh=0.5):
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
            
            pred_kpts = np.array(pred['keypoints'])
            gt_kpts = gts[best_gt_idx]['keypoints']
            
            rmse = 0.0
            valid_pts = 0
            for pk, gk in zip(pred_kpts, gt_kpts):
                vis = gk[2]
                if vis > 0:
                    dist = np.sqrt((pk[0] - gk[0])**2 + (pk[1] - gk[1])**2)
                    rmse += dist
                    valid_pts += 1
            
            if valid_pts > 0:
                total_rmse += (rmse / valid_pts)
                
    return matches, len(gts), total_rmse

def get_difficulty(filename):
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
    
    print("🚀 성능 비교 평가 시작 (V5 Multi-scale TD-DRP)...")
    
    print(f"📦 로딩: Baseline YOLO26n-pose")
    model_base = YOLO(BASE_YOLO_WEIGHTS)
    
    print(f"📦 로딩: TD-DRP v5 YOLO26n-pose")
    yolo_model = YOLO(BASE_YOLO_WEIGHTS)
    td_drp_wrapper = TDDRPEndToEndPoseV5(yolo_model)
    td_drp_wrapper.load_state_dict(torch.load(TD_DRP_WEIGHTS, map_location=device))
    td_drp_wrapper.to(device)
    td_drp_wrapper.eval()
    
    model_tddrp = YOLO(BASE_YOLO_WEIGHTS)
    def custom_forward(x, *args, **kwargs):
        with torch.no_grad():
            outputs = td_drp_wrapper(x)
            
            # 몽키패칭: V5 wrapper는 pose_logits_aug로 결과를 반환함 (layer 23 입력 이전 logit)
            # 그러나 YOLO 엔진은 Decode된 최종 결과(Tuple 등)를 기대하므로
            # 사실상 가장 좋은 방법은 wrapper 안의 변조가 끝난 상태에서 
            # 마지막 레이어(Pose)를 직접 한 번 더 실행해 주는 것!
            # 하지만 이미 forward에서 x_a가 마지막 레이어 통과 전 형태이므로,
            # 원본 헤드 self.yolo_model[-1] 을 호출합니다.
            
            pose_input = outputs["pose_logits_aug"]
            final_output = yolo_model.model.model[-1](pose_input)
            
        return final_output

    model_tddrp.model.forward = custom_forward
    
    img_paths = glob.glob(os.path.join(TEST_IMG_DIR, "*.png")) + glob.glob(os.path.join(TEST_IMG_DIR, "*.jpg"))
    
    results = {
        "Easy (Normal)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0},
        "Mild (Low-light)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0},
        "Severe (Low-light)": {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0, "count": 0}
    }
    
    print(f"\n🔍 총 {len(img_paths)}장의 테스트 이미지 평가 진행 중...")
    
    def get_preds(result, img_w, img_h):
        preds = []
        if result.boxes is not None and result.keypoints is not None:
            boxes = result.boxes.xywhn.cpu().numpy()
            kpts = result.keypoints.xyn.cpu().numpy()
            for b, k in zip(boxes, kpts):
                preds.append({'box': b, 'keypoints': k})
        return preds

    for i, img_path in enumerate(img_paths):
        filename = os.path.basename(img_path)
        diff = get_difficulty(filename)
        if diff == "Unknown": continue
        
        label_path = os.path.join(TEST_LBL_DIR, filename.replace('.png', '.txt').replace('.jpg', '.txt'))
        gts = parse_label(label_path)
        if not gts: continue
        
        results[diff]["total_gt"] += len(gts)
        results[diff]["count"] += 1
        
        # Inference Baseline
        res_base = model_base.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        preds_base = get_preds(res_base, res_base.orig_shape[1], res_base.orig_shape[0])
        
        b_match, _, b_rmse = evaluate_predictions(preds_base, gts)
        results[diff]["base_matches"] += b_match
        results[diff]["base_rmse"] += b_rmse
        
        # Inference TD-DRP V5
        res_td = model_tddrp.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        preds_td = get_preds(res_td, res_td.orig_shape[1], res_td.orig_shape[0])
        
        t_match, _, t_rmse = evaluate_predictions(preds_td, gts)
        results[diff]["td_matches"] += t_match
        results[diff]["td_rmse"] += t_rmse
        
        if (i+1) % 50 == 0:
            print(f"  진행률: {i+1}/{len(img_paths)}")
            
    print("\n📊 === TD-DRP V5 평가 결과 집계 ===")
    records = []
    for diff, data in results.items():
        if data["total_gt"] == 0: continue
        
        base_recall = (data["base_matches"] / data["total_gt"]) * 100
        td_recall = (data["td_matches"] / data["total_gt"]) * 100
        
        base_avg_rmse = (data["base_rmse"] / max(data["base_matches"], 1)) * 100
        td_avg_rmse = (data["td_rmse"] / max(data["td_matches"], 1)) * 100
        
        records.append({
            "Difficulty": diff,
            "Total Objects": data["total_gt"],
            "Baseline Recall (%)": round(base_recall, 2),
            "TD-DRP V5 Recall (%)": round(td_recall, 2),
            "Baseline Kpt Error (↓)": round(base_avg_rmse, 4),
            "TD-DRP V5 Kpt Error (↓)": round(td_avg_rmse, 4)
        })
        
    df = pd.DataFrame(records)
    print("\n", df.to_markdown(index=False))
    
    csv_path = os.path.join(RESULT_DIR, "performance_comparison_v5.csv")
    df.to_csv(csv_path, index=False)
    print(f"\n✅ V5 평가 완료! CSV가 {RESULT_DIR} 에 저장되었습니다.")

if __name__ == "__main__":
    main()
