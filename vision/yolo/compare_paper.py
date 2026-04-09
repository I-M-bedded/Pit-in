"""
TD-DRP vs Foundation Model Comparison for Research Paper
=========================================================
Analyzes models in result/td_drp_v2/ and generates paper-quality tables and plots.
Metrics: Box Precision/Recall/F1, Keypoint RMSE, PCK@0.1, Latency.
"""

import os
import sys
import glob
import time
import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
from tqdm import tqdm
from ultralytics import YOLO

# Project root setup
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from vision.yolo.td_drp import TDDRPEndToEndPose

# ==========================================
# [Configuration]
# ==========================================
FOUNDATION_W = os.path.abspath("vision/yolo/weights/yolo26n-pose_best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_v2")
DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
TEST_DIR = os.path.join(DATASET_ROOT, "test")

# Comparison Target
VARIANTS = [
    ("TD-DRP (ExLPose)", os.path.join(RESULT_DIR, "td_drp_phase2_A_ExLPose.pt")),
    ("TD-DRP (Custom)", os.path.join(RESULT_DIR, "td_drp_phase2_B_Custom.pt")),
]

OUTPUT_REPORT_DIR = os.path.join(RESULT_DIR, "paper_report")
IMG_SIZE = 640
CONF_THRES = 0.25
IOU_THRES = 0.5
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# ==========================================
# [Utility Functions]
# ==========================================

def calculate_iou(box1, box2):
    """box: [x_center, y_center, w, h] (normalized)"""
    b1_x1, b1_y1 = box1[0] - box1[2] / 2, box1[1] - box1[3] / 2
    b1_x2, b1_y2 = box1[0] + box1[2] / 2, box1[1] + box1[3] / 2
    b2_x1, b2_y1 = box2[0] - box2[2] / 2, box2[1] - box2[3] / 2
    b2_x2, b2_y2 = box2[0] + box2[2] / 2, box2[1] + box2[3] / 2

    inter_x1, inter_y1 = max(b1_x1, b2_x1), max(b1_y1, b2_y1)
    inter_x2, inter_y2 = min(b1_x2, b2_x2), min(b1_y2, b2_y2)
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)

    union_area = (b1_x2 - b1_x1) * (b1_y2 - b1_y1) + (b2_x2 - b2_x1) * (b2_y2 - b2_y1) - inter_area
    return inter_area / union_area if union_area > 0 else 0.0

def parse_label(label_path):
    gts = []
    if not os.path.exists(label_path):
        return gts
    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            if len(parts) < 5: continue
            # Box: [class, x, y, w, h]
            # Keypoints: [x, y, v] * 17 (or 9 depending on dataset)
            gt = {
                "class": int(parts[0]),
                "box": np.array(parts[1:5]),
            }
            if len(parts) > 5:
                gt["keypoints"] = np.array(parts[5:]).reshape(-1, 3)
            gts.append(gt)
    return gts

def build_tddrp_predictor(base_weights, tddrp_weights, device):
    print(f"  Loading TD-DRP: {os.path.basename(tddrp_weights)}")
    model = YOLO(base_weights)
    tddrp_wrapper = TDDRPEndToEndPose(YOLO(base_weights)).to(device)
    # Filter state_dict to load only matching keys (avoiding head mismatch if any)
    p1_state = torch.load(tddrp_weights, map_location=device)
    model_state = tddrp_wrapper.state_dict()
    # Check if weights contain tddrp adapter keys
    adapter_keys = {k: v for k, v in p1_state.items() if k in model_state}
    model_state.update(adapter_keys)
    tddrp_wrapper.load_state_dict(model_state, strict=False)
    tddrp_wrapper.eval()

    def custom_forward(x, *args, **kwargs):
        with torch.no_grad():
            out = tddrp_wrapper(x)
            preds = out["x_aug"]
            return preds[0] if isinstance(preds, (list, tuple)) else preds

    model.model.forward = custom_forward
    return model

# ==========================================
# [Evaluation Core]
# ==========================================

def evaluate_model(model, img_dir, lbl_dir, condition_name):
    img_paths = sorted(glob.glob(os.path.join(img_dir, "*.png")) + 
                       glob.glob(os.path.join(img_dir, "*.jpg")))
    
    metrics = {
        "tp": 0, "fp": 0, "fn": 0,
        "total_gt": 0,
        "rmse_sum": 0.0,
        "valid_kpt_count": 0,
        "pck_sum": 0.0, # PCK@0.1 (normalized by box size)
        "latency_ms": []
    }

    PCK_THRESHOLD = 0.1 # 10% of box max dimension

    for img_path in tqdm(img_paths, desc=f"    {condition_name}", leave=False):
        fname = os.path.basename(img_path)
        gts = parse_label(os.path.join(lbl_dir, os.path.splitext(fname)[0] + ".txt"))
        if not gts: continue
        metrics["total_gt"] += len(gts)

        t0 = time.time()
        results = model.predict(img_path, imgsz=IMG_SIZE, conf=CONF_THRES, verbose=False, device=DEVICE)[0]
        metrics["latency_ms"].append((time.time() - t0) * 1000)

        preds = []
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes.xywhn.cpu().numpy()
            kpts = results.keypoints.xyn.cpu().numpy() if results.keypoints is not None else [None]*len(boxes)
            confs = results.boxes.conf.cpu().numpy()
            for b, k, c in zip(boxes, kpts, confs):
                preds.append({"box": b, "keypoints": k, "conf": c})

        matched_gt_indices = set()
        for pred in preds:
            best_iou = 0
            best_idx = -1
            for j, gt in enumerate(gts):
                if j in matched_gt_indices: continue
                iou = calculate_iou(pred["box"], gt["box"])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = j
            
            if best_idx != -1 and best_iou >= IOU_THRES:
                metrics["tp"] += 1
                matched_gt_indices.add(best_idx)
                
                # Keypoint Evaluation
                if pred["keypoints"] is not None and "keypoints" in gts[best_idx]:
                    pk = pred["keypoints"]
                    gk = gts[best_idx]["keypoints"]
                    # Box max dimension for normalization (normalized coords)
                    box_w, box_h = gts[best_idx]["box"][2], gts[best_idx]["box"][3]
                    ref_size = max(box_w, box_h)
                    
                    local_rmse = 0.0
                    local_valid = 0
                    local_pck = 0
                    
                    for p, g in zip(pk, gk):
                        if g[2] > 0: # visibility > 0
                            dist = np.sqrt((p[0] - g[0])**2 + (p[1] - g[1])**2)
                            local_rmse += dist
                            if dist < PCK_THRESHOLD * ref_size:
                                local_pck += 1
                            local_valid += 1
                    
                    if local_valid > 0:
                        metrics["rmse_sum"] += (local_rmse / local_valid)
                        metrics["pck_sum"] += (local_pck / local_valid)
                        metrics["valid_kpt_count"] += 1
            else:
                metrics["fp"] += 1
        
        metrics["fn"] += (len(gts) - len(matched_gt_indices))

    # Summary
    tp, fp, fn = metrics["tp"], metrics["fp"], metrics["fn"]
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    
    avg_rmse = (metrics["rmse_sum"] / metrics["valid_kpt_count"] * 100) if metrics["valid_kpt_count"] > 0 else 0
    avg_pck = (metrics["pck_sum"] / metrics["valid_kpt_count"] * 100) if metrics["valid_kpt_count"] > 0 else 0
    avg_lat = np.mean(metrics["latency_ms"]) if metrics["latency_ms"] else 0

    return {
        "Precision": precision * 100,
        "Recall": recall * 100,
        "F1-Score": f1 * 100,
        "RMSE(%)": avg_rmse,
        "PCK@0.1(%)": avg_pck,
        "Latency(ms)": avg_lat
    }

# ==========================================
# [Main Runner]
# ==========================================

def run_comparison():
    os.makedirs(OUTPUT_REPORT_DIR, exist_ok=True)
    print(f"Comparison started. Results will be in: {OUTPUT_REPORT_DIR}")

    conditions = [
        ("Clean", os.path.join(TEST_DIR, "images_clean"), os.path.join(TEST_DIR, "labels_clean")),
        ("Augmented", os.path.join(TEST_DIR, "images_aug"), os.path.join(TEST_DIR, "labels_aug")),
    ]

    # Models to test
    model_list = [("Foundation", FOUNDATION_W, False)]
    for name, path in VARIANTS:
        if os.path.exists(path):
            model_list.append((name, path, True))
        else:
            print(f"[Warning] Weights not found: {path}")

    all_results = []

    for model_name, weights, is_tddrp in model_list:
        print(f"Evaluating model: {model_name}")
        if is_tddrp:
            model = build_tddrp_predictor(FOUNDATION_W, weights, DEVICE)
        else:
            model = YOLO(weights)
        
        for cond_name, img_dir, lbl_dir in conditions:
            if not os.path.exists(img_dir): continue
            
            res = evaluate_model(model, img_dir, lbl_dir, cond_name)
            res["Model"] = model_name
            res["Condition"] = cond_name
            all_results.append(res)
            
            # Simple Visualization for qualitative check
            if cond_name == "Augmented":
                # Save one or two sample inferences
                sample_imgs = glob.glob(os.path.join(img_dir, "*.png"))[:3]
                for i, s_img in enumerate(sample_imgs):
                    results = model.predict(s_img, imgsz=IMG_SIZE, conf=CONF_THRES, verbose=False, device=DEVICE)[0]
                    res_plot = results.plot()
                    cv2.imwrite(os.path.join(OUTPUT_REPORT_DIR, f"vis_{model_name.replace(' ', '_')}_{i}.jpg"), res_plot)

    df = pd.DataFrame(all_results)
    # Reorder columns
    cols = ["Model", "Condition", "Precision", "Recall", "F1-Score", "RMSE(%)", "PCK@0.1(%)", "Latency(ms)"]
    df = df[cols]
    
    # Save CSV and Markdown
    df.to_csv(os.path.join(OUTPUT_REPORT_DIR, "comparison_full.csv"), index=False)
    with open(os.path.join(OUTPUT_REPORT_DIR, "comparison_full.md"), "w") as f:
        f.write("# Model Comparison Results\n\n")
        f.write(df.to_markdown(index=False))
    
    # Pivot for Paper (Condition as sub-columns)
    pivot_df = df.pivot(index="Model", columns="Condition")
    # Flatten columns: ('Precision', 'Clean') -> 'Clean Precision'
    pivot_df.columns = [f"{c[1]} {c[0]}" for c in pivot_df.columns]
    pivot_df.to_csv(os.path.join(OUTPUT_REPORT_DIR, "comparison_paper.csv"))
    
    print("\n" + df.to_markdown(index=False))
    print(f"\nReport generated in {OUTPUT_REPORT_DIR}")

    # Latex Table snippet
    latex_str = df.to_latex(index=False, float_format="%.2f")
    with open(os.path.join(OUTPUT_REPORT_DIR, "comparison_latex.txt"), "w") as f:
        f.write(latex_str)

if __name__ == "__main__":
    run_comparison()
