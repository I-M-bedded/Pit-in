"""
TD-DRP v5 vs Baseline YOLO26n-pose 성능 비교
- images_clean: 정상 조건
- images_aug: 저조도/역광 증강 조건
두 조건에서 Recall과 Keypoint RMSE를 비교
"""
import os
import glob
import numpy as np
import torch
from ultralytics import YOLO
import sys
import pandas as pd

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.td_drp import TDDRPEndToEndPose

# ==========================================
# [설정 영역]
# ==========================================
BASE_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")
# train_td_drp_pose.py 또는 train_td_drp_v5.py 결과물 중 존재하는 것 사용
_CANDIDATES = [
    "result/td_drp_yolo26n_pose/td_drp_phase2_best.pt",
    "result/td_drp_yolo26n_pose_v5/td_drp_v5_phase2_best.pt",
]
TD_DRP_WEIGHTS = next(
    (os.path.abspath(p) for p in _CANDIDATES if os.path.exists(p)),
    os.path.abspath(_CANDIDATES[0])
)

VAL_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test")
RESULT_DIR = os.path.abspath("result/compare_v5")
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
    if not os.path.exists(label_path):
        return gts
    with open(label_path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            class_id = int(parts[0])
            box = parts[1:5]
            kpts = []
            for i in range(5, len(parts), 3):
                if i + 2 <= len(parts):
                    kpts.append([parts[i], parts[i+1], parts[i+2]])
            gts.append({'class': class_id, 'box': box, 'keypoints': np.array(kpts)})
    return gts


def evaluate_predictions(preds, gts, iou_thresh=0.5):
    matched_gts = set()
    total_rmse = 0.0
    matches = 0

    for pred in preds:
        best_iou, best_idx = 0, -1
        for i, gt in enumerate(gts):
            if i in matched_gts:
                continue
            iou = calculate_iou(pred['box'], gt['box'])
            if iou > best_iou:
                best_iou = iou
                best_idx = i

        if best_iou > iou_thresh and best_idx != -1:
            matched_gts.add(best_idx)
            matches += 1
            pred_kpts = np.array(pred['keypoints'])
            gt_kpts = gts[best_idx]['keypoints']
            rmse, valid = 0.0, 0
            for pk, gk in zip(pred_kpts, gt_kpts):
                if gk[2] > 0:
                    rmse += np.sqrt((pk[0] - gk[0])**2 + (pk[1] - gk[1])**2)
                    valid += 1
            if valid > 0:
                total_rmse += rmse / valid

    return matches, len(gts), total_rmse


def get_preds(result):
    preds = []
    if result.boxes is not None and result.keypoints is not None:
        boxes = result.boxes.xywhn.cpu().numpy()
        kpts = result.keypoints.xyn.cpu().numpy()
        for b, k in zip(boxes, kpts):
            preds.append({'box': b, 'keypoints': k})
    return preds


def build_tddrp_model(device):
    """TD-DRP v5 래핑된 YOLO 모델 생성"""
    yolo_model = YOLO(BASE_YOLO_WEIGHTS)
    td_drp_wrapper = TDDRPEndToEndPose(yolo_model)
    td_drp_wrapper.load_state_dict(torch.load(TD_DRP_WEIGHTS, map_location=device))
    td_drp_wrapper.to(device)
    td_drp_wrapper.eval()

    model_tddrp = YOLO(BASE_YOLO_WEIGHTS)

    def custom_forward(x, *args, **kwargs):
        with torch.no_grad():
            outputs = td_drp_wrapper(x)
            pose_input = outputs["pose_logits_aug"]
            return yolo_model.model.model[-1](pose_input)

    model_tddrp.model.forward = custom_forward
    return model_tddrp


def run_eval(model_base, model_tddrp, img_dir, lbl_dir, condition_name):
    """한 조건(clean/aug)에 대해 두 모델 평가"""
    img_paths = sorted(
        glob.glob(os.path.join(img_dir, "*.png")) +
        glob.glob(os.path.join(img_dir, "*.jpg"))
    )

    stats = {"base_matches": 0, "base_rmse": 0, "td_matches": 0, "td_rmse": 0, "total_gt": 0}

    for i, img_path in enumerate(img_paths):
        fname = os.path.basename(img_path)
        lbl_path = os.path.join(lbl_dir, os.path.splitext(fname)[0] + ".txt")
        gts = parse_label(lbl_path)
        if not gts:
            continue

        stats["total_gt"] += len(gts)

        # Baseline
        res_b = model_base.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        bm, _, br = evaluate_predictions(get_preds(res_b), gts)
        stats["base_matches"] += bm
        stats["base_rmse"] += br

        # TD-DRP v5
        res_t = model_tddrp.predict(img_path, imgsz=640, conf=0.25, verbose=False)[0]
        tm, _, tr = evaluate_predictions(get_preds(res_t), gts)
        stats["td_matches"] += tm
        stats["td_rmse"] += tr

        if (i + 1) % 100 == 0:
            print(f"  [{condition_name}] {i+1}/{len(img_paths)}")

    return stats


def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    if not os.path.exists(TD_DRP_WEIGHTS):
        print(f"TD-DRP v5 weights not found: {TD_DRP_WEIGHTS}")
        print("Run train_td_drp_v5.py first.")
        return

    print("Loading Baseline YOLO26n-pose...")
    model_base = YOLO(BASE_YOLO_WEIGHTS)

    print("Loading TD-DRP v5...")
    model_tddrp = build_tddrp_model(device)

    # 두 조건에서 평가
    conditions = [
        ("Clean (Normal)", "images_clean", "labels_clean"),
        ("Augmented (Low-light)", "images_aug", "labels_aug"),
    ]

    records = []
    for cond_name, img_sub, lbl_sub in conditions:
        img_dir = os.path.join(VAL_DIR, img_sub)
        lbl_dir = os.path.join(VAL_DIR, lbl_sub)

        if not os.path.isdir(img_dir):
            print(f"  Skip {cond_name}: {img_dir} not found")
            continue

        print(f"\nEvaluating: {cond_name}...")
        stats = run_eval(model_base, model_tddrp, img_dir, lbl_dir, cond_name)

        if stats["total_gt"] == 0:
            continue

        base_recall = stats["base_matches"] / stats["total_gt"] * 100
        td_recall = stats["td_matches"] / stats["total_gt"] * 100
        base_rmse = stats["base_rmse"] / max(stats["base_matches"], 1) * 100
        td_rmse = stats["td_rmse"] / max(stats["td_matches"], 1) * 100

        records.append({
            "Condition": cond_name,
            "Total Objects": stats["total_gt"],
            "Baseline Recall (%)": round(base_recall, 2),
            "TD-DRP V5 Recall (%)": round(td_recall, 2),
            "Baseline Kpt Error": round(base_rmse, 4),
            "TD-DRP V5 Kpt Error": round(td_rmse, 4),
        })

    df = pd.DataFrame(records)
    print("\n" + df.to_markdown(index=False))

    csv_path = os.path.join(RESULT_DIR, "performance_comparison_v5.csv")
    df.to_csv(csv_path, index=False)
    print(f"\nCSV saved to {csv_path}")


if __name__ == "__main__":
    main()
