"""
고전 파이프라인 (Phase Congruency + AAMED) vs GT keypoint 기반 ellipse 비교
- 100장 aug 이미지에 대해 IoU, center error, axis error 통계
"""
import os
import sys
import glob
import numpy as np
import cv2
from collections import defaultdict

# phasepack
from phasepack import phasecong

# ============================================================
# Config
# ============================================================
DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
AUG_IMG_DIR = os.path.join(DATASET_ROOT, "train/images_aug")
LABEL_DIR = os.path.join(DATASET_ROOT, "train/labels_clean")  # GT labels (normalized coords)
OUT_DIR = os.path.abspath("result/classical_vs_kpt")
NUM_SAMPLES = 100
NUM_KPTS = 9

os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================
# GT ellipse from keypoints
# ============================================================
def parse_label(label_path, img_w, img_h):
    """Parse YOLO-pose label → list of (center, axes, angle) ellipses in pixel coords."""
    ellipses = []
    if not os.path.exists(label_path):
        return ellipses
    with open(label_path, "r") as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) < 5 + NUM_KPTS * 3:
                continue
            # Extract visible keypoints
            kpts_raw = vals[5:5 + NUM_KPTS * 3]
            pts = []
            for k in range(NUM_KPTS):
                kx, ky, kv = kpts_raw[k*3], kpts_raw[k*3+1], kpts_raw[k*3+2]
                if kv > 0:
                    pts.append([kx * img_w, ky * img_h])
            if len(pts) >= 5:  # fitEllipse needs >= 5 points
                pts_np = np.array(pts, dtype=np.float32)
                ellipse = cv2.fitEllipse(pts_np)  # ((cx,cy), (MA,ma), angle)
                ellipses.append(ellipse)
    return ellipses


# ============================================================
# Classical pipeline: Phase Congruency → EdgeDrawing (AAMED) ellipse
# ============================================================
def _parse_ed_ellipses(detected):
    """EdgeDrawing detectEllipses() 결과 파싱 → list of ((cx,cy), (MA,ma), angle)."""
    ellipses = []
    if detected is None:
        return ellipses
    # detected shape: (N, 1, 6) → [cx, cy, _, semi_major, semi_minor, angle]
    for row in detected.reshape(-1, 6):
        cx, cy = float(row[0]), float(row[1])
        semi_a, semi_b = float(row[3]), float(row[4])
        angle = float(row[5])
        if semi_a > 0 and semi_b > 0:
            ellipses.append(((cx, cy), (semi_a * 2, semi_b * 2), angle))
    return ellipses


def detect_ellipses_phase_cong(img_gray):
    """Phase Congruency edge → EdgeDrawing ellipse detection."""
    result = phasecong(img_gray.astype(np.float64), nscale=4, norient=6, minWaveLength=6)
    M = result[0]

    edge_map = np.clip(M * 255, 0, 255).astype(np.uint8)
    _, edge_bin = cv2.threshold(edge_map, 30, 255, cv2.THRESH_BINARY)

    ed = cv2.ximgproc.createEdgeDrawing()
    params = cv2.ximgproc.EdgeDrawing.Params()
    params.EdgeDetectionOperator = cv2.ximgproc.EDGE_DRAWING_SOBEL
    params.MinPathLength = 15
    ed.setParams(params)
    ed.detectEdges(edge_bin)
    detected = ed.detectEllipses()

    return _parse_ed_ellipses(detected), edge_map


def detect_ellipses_clahe_canny(img_gray):
    """CLAHE + Canny → EdgeDrawing ellipse detection (baseline)."""
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(img_gray)
    edges = cv2.Canny(enhanced, 50, 150)

    ed = cv2.ximgproc.createEdgeDrawing()
    params = cv2.ximgproc.EdgeDrawing.Params()
    params.MinPathLength = 15
    ed.setParams(params)
    ed.detectEdges(edges)
    detected = ed.detectEllipses()

    return _parse_ed_ellipses(detected), edges


# ============================================================
# Ellipse comparison metrics
# ============================================================
def ellipse_iou_mask(e1, e2, img_shape):
    """Compute IoU between two ellipses using mask overlap."""
    h, w = img_shape[:2]
    mask1 = np.zeros((h, w), dtype=np.uint8)
    mask2 = np.zeros((h, w), dtype=np.uint8)

    c1 = (int(round(e1[0][0])), int(round(e1[0][1])))
    a1 = (int(round(e1[1][0] / 2)), int(round(e1[1][1] / 2)))
    c2 = (int(round(e2[0][0])), int(round(e2[0][1])))
    a2 = (int(round(e2[1][0] / 2)), int(round(e2[1][1] / 2)))

    if a1[0] <= 0 or a1[1] <= 0 or a2[0] <= 0 or a2[1] <= 0:
        return 0.0

    cv2.ellipse(mask1, c1, a1, e1[2], 0, 360, 255, -1)
    cv2.ellipse(mask2, c2, a2, e2[2], 0, 360, 255, -1)

    inter = np.logical_and(mask1 > 0, mask2 > 0).sum()
    union = np.logical_or(mask1 > 0, mask2 > 0).sum()
    return inter / union if union > 0 else 0.0


def center_error(e1, e2):
    """Euclidean distance between centers (pixels)."""
    return np.sqrt((e1[0][0] - e2[0][0])**2 + (e1[0][1] - e2[0][1])**2)


def axis_error(e1, e2):
    """Relative error of semi-axes (%)."""
    a1 = sorted(e1[1], reverse=True)
    a2 = sorted(e2[1], reverse=True)
    if a1[0] == 0 or a1[1] == 0:
        return 100.0
    err_major = abs(a1[0] - a2[0]) / a1[0] * 100
    err_minor = abs(a1[1] - a2[1]) / a1[1] * 100
    return (err_major + err_minor) / 2


def match_ellipses(gt_list, det_list, img_shape):
    """Greedy match detected ellipses to GT by center distance, return per-GT best match metrics."""
    results = []
    used = set()
    for gt in gt_list:
        best_iou = 0.0
        best_idx = -1
        best_metrics = None
        for j, det in enumerate(det_list):
            if j in used:
                continue
            iou = ellipse_iou_mask(gt, det, img_shape)
            if iou > best_iou:
                best_iou = iou
                best_idx = j
                best_metrics = {
                    "iou": iou,
                    "center_err": center_error(gt, det),
                    "axis_err": axis_error(gt, det),
                }
        if best_idx >= 0 and best_iou > 0.01:
            used.add(best_idx)
            results.append(best_metrics)
        else:
            results.append(None)  # miss
    return results


# ============================================================
# Main
# ============================================================
def main():
    imgs = sorted(
        glob.glob(os.path.join(AUG_IMG_DIR, "*.png"))
        + glob.glob(os.path.join(AUG_IMG_DIR, "*.jpg"))
    )

    step = max(len(imgs) // NUM_SAMPLES, 1)
    selected = imgs[::step][:NUM_SAMPLES]
    print(f"Selected {len(selected)} images from {len(imgs)} total")

    stats = {
        "phase_cong": defaultdict(list),
        "clahe_canny": defaultdict(list),
    }
    gt_total = 0
    detect_counts = {"phase_cong": 0, "clahe_canny": 0}
    match_counts = {"phase_cong": 0, "clahe_canny": 0}

    for idx, img_path in enumerate(selected):
        fname = os.path.basename(img_path)
        stem = os.path.splitext(fname)[0]
        label_path = os.path.join(LABEL_DIR, stem + ".txt")

        img = cv2.imread(img_path)
        if img is None:
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        gt_ellipses = parse_label(label_path, w, h)
        if not gt_ellipses:
            continue
        gt_total += len(gt_ellipses)

        # Phase Congruency + AAMED
        try:
            pc_ellipses, pc_edge = detect_ellipses_phase_cong(gray)
        except Exception as e:
            pc_ellipses, pc_edge = [], np.zeros_like(gray)
            if idx < 3:
                print(f"  [PC error] {fname}: {e}")

        detect_counts["phase_cong"] += len(pc_ellipses)
        pc_matches = match_ellipses(gt_ellipses, pc_ellipses, (h, w))
        for m in pc_matches:
            if m is not None:
                match_counts["phase_cong"] += 1
                for k, v in m.items():
                    stats["phase_cong"][k].append(v)

        # CLAHE + Canny + AAMED
        cc_ellipses, cc_edge = detect_ellipses_clahe_canny(gray)
        detect_counts["clahe_canny"] += len(cc_ellipses)
        cc_matches = match_ellipses(gt_ellipses, cc_ellipses, (h, w))
        for m in cc_matches:
            if m is not None:
                match_counts["clahe_canny"] += 1
                for k, v in m.items():
                    stats["clahe_canny"][k].append(v)

        # Save visualization for first 5
        if idx < 5:
            vis = img.copy()
            # GT: green
            for e in gt_ellipses:
                c = (int(round(e[0][0])), int(round(e[0][1])))
                a = (int(round(e[1][0] / 2)), int(round(e[1][1] / 2)))
                if a[0] > 0 and a[1] > 0:
                    cv2.ellipse(vis, c, a, e[2], 0, 360, (0, 255, 0), 1)
            # PC: red
            for e in pc_ellipses:
                c = (int(round(e[0][0])), int(round(e[0][1])))
                a = (int(round(e[1][0] / 2)), int(round(e[1][1] / 2)))
                if a[0] > 0 and a[1] > 0:
                    cv2.ellipse(vis, c, a, e[2], 0, 360, (0, 0, 255), 1)
            # CC: blue
            for e in cc_ellipses:
                c = (int(round(e[0][0])), int(round(e[0][1])))
                a = (int(round(e[1][0] / 2)), int(round(e[1][1] / 2)))
                if a[0] > 0 and a[1] > 0:
                    cv2.ellipse(vis, c, a, e[2], 0, 360, (255, 0, 0), 1)

            cv2.imwrite(os.path.join(OUT_DIR, f"vis_{fname}"), vis)

        if (idx + 1) % 20 == 0:
            print(f"  Processed {idx+1}/{len(selected)}")

    # ============================================================
    # Report
    # ============================================================
    print("\n" + "=" * 70)
    print(f"GT ellipses total: {gt_total}")
    print("=" * 70)

    for method in ["phase_cong", "clahe_canny"]:
        method_label = "Phase Congruency + AAMED" if method == "phase_cong" else "CLAHE + Canny + AAMED"
        n_matched = match_counts[method]
        n_detected = detect_counts[method]

        print(f"\n[{method_label}]")
        print(f"  Detected ellipses : {n_detected}")
        print(f"  Matched to GT     : {n_matched} / {gt_total}  (Recall: {n_matched/gt_total*100:.1f}%)")

        if stats[method]["iou"]:
            ious = np.array(stats[method]["iou"])
            cerrs = np.array(stats[method]["center_err"])
            aerrs = np.array(stats[method]["axis_err"])
            print(f"  IoU          : mean={ious.mean():.3f}  std={ious.std():.3f}  median={np.median(ious):.3f}")
            print(f"  Center err   : mean={cerrs.mean():.1f}px  std={cerrs.std():.1f}px  median={np.median(cerrs):.1f}px")
            print(f"  Axis err (%) : mean={aerrs.mean():.1f}%  std={aerrs.std():.1f}%  median={np.median(aerrs):.1f}%")
            print(f"  IoU >= 0.5   : {(ious >= 0.5).sum()} / {len(ious)}  ({(ious >= 0.5).mean()*100:.1f}%)")
            print(f"  IoU >= 0.7   : {(ious >= 0.7).sum()} / {len(ious)}  ({(ious >= 0.7).mean()*100:.1f}%)")
        else:
            print(f"  No matches found.")

    # Save summary
    summary_path = os.path.join(OUT_DIR, "summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"GT ellipses total: {gt_total}\n\n")
        for method in ["phase_cong", "clahe_canny"]:
            method_label = "Phase Congruency + AAMED" if method == "phase_cong" else "CLAHE + Canny + AAMED"
            n_matched = match_counts[method]
            n_detected = detect_counts[method]
            f.write(f"[{method_label}]\n")
            f.write(f"  Detected: {n_detected}  Matched: {n_matched}/{gt_total} (Recall: {n_matched/gt_total*100:.1f}%)\n")
            if stats[method]["iou"]:
                ious = np.array(stats[method]["iou"])
                cerrs = np.array(stats[method]["center_err"])
                aerrs = np.array(stats[method]["axis_err"])
                f.write(f"  IoU: mean={ious.mean():.3f} std={ious.std():.3f} median={np.median(ious):.3f}\n")
                f.write(f"  Center err: mean={cerrs.mean():.1f}px std={cerrs.std():.1f}px\n")
                f.write(f"  Axis err: mean={aerrs.mean():.1f}% std={aerrs.std():.1f}%\n")
                f.write(f"  IoU>=0.5: {(ious>=0.5).sum()}/{len(ious)}  IoU>=0.7: {(ious>=0.7).sum()}/{len(ious)}\n")
            f.write("\n")

    print(f"\nSummary saved to {summary_path}")
    print(f"Visualizations (5) saved to {OUT_DIR}")


if __name__ == "__main__":
    main()
