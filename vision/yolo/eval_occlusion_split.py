"""
Occlusion Impact Analysis for Research Paper
==============================================
Aug 테스트셋을 visibility 기준으로 분리 평가:
- all_visible: 모든 keypoint vis==2
- has_occlusion: vis<2 keypoint 1개 이상
Metrics: Box Precision/Recall/F1, Keypoint RMSE, PCK@0.1, Latency.
"""
import os
import sys
import glob
import time
import torch
import numpy as np
import pandas as pd
from tqdm import tqdm
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from vision.yolo.td_drp import TDDRPEndToEndPose

# ==========================================
# [Configuration]
# ==========================================
FOUNDATION_W = os.path.abspath("vision/yolo/weights/yolo26n-pose_best.pt")
RESULT_DIR = os.path.abspath("result/td_drp_v2")
DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
TEST_DIR = os.path.join(DATASET_ROOT, "test")

VARIANTS = [
    ("TD-DRP (ExLPose)", os.path.join(RESULT_DIR, "td_drp_phase2_A_ExLPose.pt")),
    ("TD-DRP (Custom)", os.path.join(RESULT_DIR, "td_drp_phase2_B_Custom.pt")),
]

OUTPUT_REPORT_DIR = os.path.join(RESULT_DIR, "occlusion_analysis")
IMG_SIZE = 640
CONF_THRES = 0.25
IOU_THRES = 0.5
PCK_THRESHOLD = 0.1
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# ==========================================
# [Utility Functions]
# ==========================================

def calculate_iou(box1, box2):
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
            if len(parts) < 5:
                continue
            gt = {
                "class": int(parts[0]),
                "box": np.array(parts[1:5]),
            }
            if len(parts) > 5:
                gt["keypoints"] = np.array(parts[5:]).reshape(-1, 3)
                gt["has_occlusion"] = bool(np.any(gt["keypoints"][:, 2] < 2))
            else:
                gt["has_occlusion"] = False
            gts.append(gt)
    return gts


def classify_image(gts):
    if any(g["has_occlusion"] for g in gts):
        return "has_occlusion"
    return "all_visible"


def build_tddrp_predictor(base_weights, tddrp_weights, device):
    print(f"  Loading TD-DRP: {os.path.basename(tddrp_weights)}")
    model = YOLO(base_weights)
    tddrp_wrapper = TDDRPEndToEndPose(YOLO(base_weights)).to(device)
    p1_state = torch.load(tddrp_weights, map_location=device)
    model_state = tddrp_wrapper.state_dict()
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

def evaluate_model_split(model, img_dir, lbl_dir, condition_name):
    """Evaluate and split results by occlusion status."""
    img_paths = sorted(
        glob.glob(os.path.join(img_dir, "*.png")) +
        glob.glob(os.path.join(img_dir, "*.jpg"))
    )

    categories = ["all_visible", "has_occlusion"]
    metrics = {}
    for cat in categories:
        metrics[cat] = {
            "tp": 0, "fp": 0, "fn": 0, "total_gt": 0,
            "rmse_sum": 0.0, "valid_kpt_count": 0,
            "pck_sum": 0.0, "latency_ms": [], "n_images": 0,
        }

    for img_path in tqdm(img_paths, desc=f"    {condition_name}", leave=False):
        fname = os.path.basename(img_path)
        gts = parse_label(os.path.join(lbl_dir, os.path.splitext(fname)[0] + ".txt"))
        if not gts:
            continue

        cat = classify_image(gts)
        m = metrics[cat]
        m["n_images"] += 1
        m["total_gt"] += len(gts)

        t0 = time.time()
        results = model.predict(img_path, imgsz=IMG_SIZE, conf=CONF_THRES, verbose=False, device=DEVICE)[0]
        m["latency_ms"].append((time.time() - t0) * 1000)

        preds = []
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes.xywhn.cpu().numpy()
            kpts = results.keypoints.xyn.cpu().numpy() if results.keypoints is not None else [None] * len(boxes)
            confs = results.boxes.conf.cpu().numpy()
            for b, k, c in zip(boxes, kpts, confs):
                preds.append({"box": b, "keypoints": k, "conf": c})

        matched_gt_indices = set()
        for pred in preds:
            best_iou = 0
            best_idx = -1
            for j, gt in enumerate(gts):
                if j in matched_gt_indices:
                    continue
                iou = calculate_iou(pred["box"], gt["box"])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = j

            if best_idx != -1 and best_iou >= IOU_THRES:
                m["tp"] += 1
                matched_gt_indices.add(best_idx)

                if pred["keypoints"] is not None and "keypoints" in gts[best_idx]:
                    pk = pred["keypoints"]
                    gk = gts[best_idx]["keypoints"]
                    box_w, box_h = gts[best_idx]["box"][2], gts[best_idx]["box"][3]
                    ref_size = max(box_w, box_h)

                    local_rmse, local_valid, local_pck = 0.0, 0, 0
                    for p, g in zip(pk, gk):
                        if g[2] > 0:
                            dist = np.sqrt((p[0] - g[0]) ** 2 + (p[1] - g[1]) ** 2)
                            local_rmse += dist
                            if dist < PCK_THRESHOLD * ref_size:
                                local_pck += 1
                            local_valid += 1

                    if local_valid > 0:
                        m["rmse_sum"] += local_rmse / local_valid
                        m["pck_sum"] += local_pck / local_valid
                        m["valid_kpt_count"] += 1
            else:
                m["fp"] += 1

        m["fn"] += len(gts) - len(matched_gt_indices)

    # Summarize per category
    summary = {}
    for cat in categories:
        m = metrics[cat]
        tp, fp, fn = m["tp"], m["fp"], m["fn"]
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
        avg_rmse = (m["rmse_sum"] / m["valid_kpt_count"] * 100) if m["valid_kpt_count"] > 0 else 0
        avg_pck = (m["pck_sum"] / m["valid_kpt_count"] * 100) if m["valid_kpt_count"] > 0 else 0
        avg_lat = np.mean(m["latency_ms"]) if m["latency_ms"] else 0

        summary[cat] = {
            "Images": m["n_images"],
            "GT Objects": m["total_gt"],
            "Precision": precision * 100,
            "Recall": recall * 100,
            "F1-Score": f1 * 100,
            "RMSE(%)": avg_rmse,
            "PCK@0.1(%)": avg_pck,
            "Latency(ms)": avg_lat,
        }

    return summary


# ==========================================
# [Main Runner]
# ==========================================

def run_occlusion_analysis():
    os.makedirs(OUTPUT_REPORT_DIR, exist_ok=True)
    print(f"Occlusion analysis started. Results → {OUTPUT_REPORT_DIR}")

    # Aug 조건에서만 평가 (clean에는 occlusion 없음)
    img_dir = os.path.join(TEST_DIR, "images_aug")
    lbl_dir = os.path.join(TEST_DIR, "labels_aug")
    if not os.path.exists(img_dir):
        print(f"Aug images not found: {img_dir}")
        return

    # Models
    model_list = [("Foundation", FOUNDATION_W, False)]
    for name, path in VARIANTS:
        if os.path.exists(path):
            model_list.append((name, path, True))
        else:
            print(f"[Warning] Weights not found: {path}")

    all_results = []

    for model_name, weights, is_tddrp in model_list:
        print(f"\nEvaluating: {model_name}")
        if is_tddrp:
            model = build_tddrp_predictor(FOUNDATION_W, weights, DEVICE)
        else:
            model = YOLO(weights)

        summary = evaluate_model_split(model, img_dir, lbl_dir, f"Aug ({model_name})")

        for cat, stats in summary.items():
            row = {"Model": model_name, "Category": cat}
            row.update(stats)
            all_results.append(row)

    df = pd.DataFrame(all_results)
    cols = ["Model", "Category", "Images", "GT Objects",
            "Precision", "Recall", "F1-Score", "RMSE(%)", "PCK@0.1(%)", "Latency(ms)"]
    df = df[cols]

    # CSV + Markdown
    df.to_csv(os.path.join(OUTPUT_REPORT_DIR, "occlusion_split.csv"), index=False)
    with open(os.path.join(OUTPUT_REPORT_DIR, "occlusion_split.md"), "w", encoding="utf-8") as f:
        f.write("# Occlusion Impact Analysis (Aug condition)\n\n")
        f.write(df.to_markdown(index=False))
        f.write("\n\n## Recall Gap (all_visible − has_occlusion)\n\n")
        for model_name, _, _ in model_list:
            rows = df[df["Model"] == model_name]
            vis = rows[rows["Category"] == "all_visible"]
            occ = rows[rows["Category"] == "has_occlusion"]
            if not vis.empty and not occ.empty:
                gap = vis.iloc[0]["Recall"] - occ.iloc[0]["Recall"]
                f.write(f"- **{model_name}**: {gap:+.2f}%p\n")

    # LaTeX
    latex_str = df.to_latex(index=False, float_format="%.2f")
    with open(os.path.join(OUTPUT_REPORT_DIR, "occlusion_split_latex.txt"), "w", encoding="utf-8") as f:
        f.write(latex_str)

    # Console output
    print("\n" + "=" * 90)
    print("OCCLUSION IMPACT ANALYSIS (Aug condition)")
    print("=" * 90)
    print(df.to_markdown(index=False))

    print("\n--- Recall Gap (all_visible − has_occlusion) ---")
    for model_name, _, _ in model_list:
        rows = df[df["Model"] == model_name]
        vis = rows[rows["Category"] == "all_visible"]
        occ = rows[rows["Category"] == "has_occlusion"]
        if not vis.empty and not occ.empty:
            gap = vis.iloc[0]["Recall"] - occ.iloc[0]["Recall"]
            print(f"  {model_name}: {gap:+.2f}%p")

    print(f"\nReport saved to {OUTPUT_REPORT_DIR}/")


if __name__ == "__main__":
    run_occlusion_analysis()
