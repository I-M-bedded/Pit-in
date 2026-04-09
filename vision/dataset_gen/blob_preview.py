import os
import glob
import numpy as np
import cv2
from collections import defaultdict

# ============================================================
# Config
# ============================================================
DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
AUG_IMG_DIR = os.path.join(DATASET_ROOT, "train/images_aug")
LABEL_DIR = os.path.join(DATASET_ROOT, "train/labels_clean")
OUT_DIR = os.path.abspath("result/blob_mser_vs_kpt")

NUM_SAMPLES = 100
NUM_KPTS = 9
SAVE_VIS_COUNT = 15

os.makedirs(OUT_DIR, exist_ok=True)


# ============================================================
# GT ellipse from keypoints
# ============================================================
def parse_label(label_path, img_w, img_h):
    ellipses = []
    if not os.path.exists(label_path):
        return ellipses

    with open(label_path, "r") as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) < 5 + NUM_KPTS * 3:
                continue

            kpts_raw = vals[5:5 + NUM_KPTS * 3]
            pts = []
            for k in range(NUM_KPTS):
                kx, ky, kv = kpts_raw[k * 3], kpts_raw[k * 3 + 1], kpts_raw[k * 3 + 2]
                if kv > 0:
                    pts.append([kx * img_w, ky * img_h])

            if len(pts) >= 5:
                pts_np = np.array(pts, dtype=np.float32)
                ellipse = cv2.fitEllipse(pts_np)
                ellipses.append(ellipse)

    return ellipses


# ============================================================
# Ellipse utils
# ============================================================
def normalize_ellipse(e):
    (cx, cy), (w, h), ang = e
    if h > w:
        w, h = h, w
        ang += 90.0
    ang = ang % 180.0
    return ((float(cx), float(cy)), (float(w), float(h)), float(ang))


def ellipse_iou_mask(e1, e2, img_shape):
    h, w = img_shape[:2]
    mask1 = np.zeros((h, w), dtype=np.uint8)
    mask2 = np.zeros((h, w), dtype=np.uint8)

    e1 = normalize_ellipse(e1)
    e2 = normalize_ellipse(e2)

    c1 = (int(round(e1[0][0])), int(round(e1[0][1])))
    a1 = (int(round(e1[1][0] / 2.0)), int(round(e1[1][1] / 2.0)))
    c2 = (int(round(e2[0][0])), int(round(e2[0][1])))
    a2 = (int(round(e2[1][0] / 2.0)), int(round(e2[1][1] / 2.0)))

    if a1[0] <= 0 or a1[1] <= 0 or a2[0] <= 0 or a2[1] <= 0:
        return 0.0

    cv2.ellipse(mask1, c1, a1, e1[2], 0, 360, 255, -1)
    cv2.ellipse(mask2, c2, a2, e2[2], 0, 360, 255, -1)

    inter = np.count_nonzero((mask1 > 0) & (mask2 > 0))
    union = np.count_nonzero((mask1 > 0) | (mask2 > 0))
    return inter / union if union > 0 else 0.0


def center_error(e1, e2):
    return float(np.sqrt((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))


def axis_error(e1, e2):
    a1 = sorted(normalize_ellipse(e1)[1], reverse=True)
    a2 = sorted(normalize_ellipse(e2)[1], reverse=True)

    if a1[0] <= 1e-6 or a1[1] <= 1e-6:
        return 100.0

    err_major = abs(a1[0] - a2[0]) / a1[0] * 100.0
    err_minor = abs(a1[1] - a2[1]) / a1[1] * 100.0
    return float((err_major + err_minor) / 2.0)


def match_ellipses(gt_list, det_list, img_shape, iou_match_thresh=0.01):
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

        if best_idx >= 0 and best_iou > iou_match_thresh:
            used.add(best_idx)
            results.append(best_metrics)
        else:
            results.append(None)

    return results


# ============================================================
# MSER blob -> ellipse
# ============================================================
def preprocess_for_mser(gray):
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    x = clahe.apply(gray)

    x = cv2.GaussianBlur(x, (5, 5), 0)

    blackhat = cv2.morphologyEx(
        x,
        cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    )

    mix = cv2.addWeighted(x, 0.55, blackhat, 1.25, 0)
    mix = cv2.normalize(mix, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return mix, blackhat


def contour_to_ellipse(cnt):
    if len(cnt) < 5:
        return None
    try:
        e = cv2.fitEllipse(cnt)
        return normalize_ellipse(e)
    except cv2.error:
        return None


def ellipse_area(e):
    w, h = normalize_ellipse(e)[1]
    return np.pi * (w / 2.0) * (h / 2.0)


def ellipse_nms(ellipses, img_shape, iou_thr=0.35):
    if not ellipses:
        return []

    ellipses = sorted(ellipses, key=lambda e: ellipse_area(e), reverse=True)
    kept = []

    for e in ellipses:
        keep = True
        for k in kept:
            if ellipse_iou_mask(e, k, img_shape) > iou_thr:
                keep = False
                break
        if keep:
            kept.append(e)

    return kept


def detect_ellipses_mser(gray):
    h, w = gray.shape[:2]
    proc, blackhat = preprocess_for_mser(gray)

    min_area = max(40, int(0.00015 * h * w))
    max_area = max(min_area + 1, int(0.08 * h * w))

    mser = cv2.MSER_create(
        5,
        min_area,
        max_area,
        0.25,
        0.2,
        200,
        1.01,
        0.003,
        5
    )

    candidates = []
    debug_masks = []

    for mode_name, src in [
        ("dark_on_bright", proc),
        ("bright_on_dark", 255 - proc),
    ]:
        regions, _ = mser.detectRegions(src)

        mode_mask = np.zeros_like(gray)
        for pts in regions:
            pts = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
            if pts.shape[0] < 5:
                continue

            area = cv2.contourArea(pts)
            if area < min_area or area > max_area:
                continue

            hull = cv2.convexHull(pts)
            if hull.shape[0] < 5:
                continue

            x, y, bw, bh = cv2.boundingRect(hull)
            if bw < 6 or bh < 6:
                continue

            rect_area = float(bw * bh)
            if rect_area <= 0:
                continue

            extent = area / rect_area
            if extent < 0.20 or extent > 0.95:
                continue

            peri = cv2.arcLength(hull, True)
            if peri <= 1e-6:
                continue

            circularity = 4.0 * np.pi * area / (peri * peri)
            if circularity < 0.25:
                continue

            ellipse = contour_to_ellipse(hull)
            if ellipse is None:
                continue

            (_, _), (maj, minr), _ = ellipse
            if min(maj, minr) < 6:
                continue

            axis_ratio = maj / max(minr, 1e-6)
            if axis_ratio > 6.0:
                continue

            ell_area = ellipse_area(ellipse)
            fill_ratio = area / max(ell_area, 1e-6)
            if fill_ratio < 0.35 or fill_ratio > 1.30:
                continue

            candidates.append(ellipse)
            cv2.drawContours(mode_mask, [hull], -1, 255, -1)

        debug_masks.append((mode_name, mode_mask))

    ellipses = ellipse_nms(candidates, gray.shape, iou_thr=0.30)

    debug = {
        "proc": proc,
        "blackhat": blackhat,
        "mask_dark": debug_masks[0][1] if len(debug_masks) > 0 else np.zeros_like(gray),
        "mask_bright": debug_masks[1][1] if len(debug_masks) > 1 else np.zeros_like(gray),
    }
    return ellipses, debug


# ============================================================
# Visualization
# ============================================================
def draw_ellipse(vis, e, color, thickness=1):
    e = normalize_ellipse(e)
    c = (int(round(e[0][0])), int(round(e[0][1])))
    a = (int(round(e[1][0] / 2.0)), int(round(e[1][1] / 2.0)))
    if a[0] > 0 and a[1] > 0:
        cv2.ellipse(vis, c, a, e[2], 0, 360, color, thickness)


def save_visualization(img, gt_ellipses, det_ellipses, debug, out_path):
    h, w = img.shape[:2]
    canvas = np.zeros((h * 2, w * 2, 3), dtype=np.uint8)

    vis_main = img.copy()
    for e in gt_ellipses:
        draw_ellipse(vis_main, e, (0, 255, 0), 1)
    for e in det_ellipses:
        draw_ellipse(vis_main, e, (0, 0, 255), 1)

    proc_bgr = cv2.cvtColor(debug["proc"], cv2.COLOR_GRAY2BGR)
    blackhat_bgr = cv2.cvtColor(debug["blackhat"], cv2.COLOR_GRAY2BGR)
    mask_dark_bgr = cv2.cvtColor(debug["mask_dark"], cv2.COLOR_GRAY2BGR)
    mask_bright_bgr = cv2.cvtColor(debug["mask_bright"], cv2.COLOR_GRAY2BGR)

    canvas[:h, :w] = vis_main
    canvas[:h, w:] = proc_bgr
    canvas[h:, :w] = blackhat_bgr
    mix_mask = cv2.addWeighted(mask_dark_bgr, 0.5, mask_bright_bgr, 0.5, 0)
    canvas[h:, w:] = mix_mask

    cv2.putText(canvas, "Overlay: GT green / MSER red", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(canvas, "Preprocess", (w + 12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(canvas, "Blackhat", (12, h + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(canvas, "MSER region masks", (w + 12, h + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    cv2.imwrite(out_path, canvas)


# ============================================================
# Main
# ============================================================
def main():
    imgs = sorted(
        glob.glob(os.path.join(AUG_IMG_DIR, "*.png")) +
        glob.glob(os.path.join(AUG_IMG_DIR, "*.jpg")) +
        glob.glob(os.path.join(AUG_IMG_DIR, "*.jpeg"))
    )

    step = max(len(imgs) // NUM_SAMPLES, 1)
    selected = imgs[::step][:NUM_SAMPLES]
    print(f"Selected {len(selected)} images from {len(imgs)} total")

    stats = defaultdict(list)
    gt_total = 0
    detect_count = 0
    match_count = 0

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

        try:
            det_ellipses, debug = detect_ellipses_mser(gray)
        except Exception as e:
            det_ellipses = []
            debug = {
                "proc": gray.copy(),
                "blackhat": np.zeros_like(gray),
                "mask_dark": np.zeros_like(gray),
                "mask_bright": np.zeros_like(gray),
            }
            if idx < 5:
                print(f"[MSER error] {fname}: {e}")

        detect_count += len(det_ellipses)

        matches = match_ellipses(gt_ellipses, det_ellipses, (h, w), iou_match_thresh=0.01)
        for m in matches:
            if m is not None:
                match_count += 1
                for k, v in m.items():
                    stats[k].append(v)

        if idx < SAVE_VIS_COUNT:
            save_visualization(
                img,
                gt_ellipses,
                det_ellipses,
                debug,
                os.path.join(OUT_DIR, f"vis_{fname}")
            )

        if (idx + 1) % 20 == 0:
            print(f"Processed {idx + 1}/{len(selected)}")

    print("\n" + "=" * 70)
    print("[MSER blob + ellipse fit]")
    print(f"GT ellipses total : {gt_total}")
    print(f"Detected ellipses : {detect_count}")
    print(f"Matched to GT     : {match_count}/{gt_total} (Recall: {match_count / max(gt_total, 1) * 100:.1f}%)")

    summary_path = os.path.join(OUT_DIR, "summary.txt")
    with open(summary_path, "w") as f:
        f.write("[MSER blob + ellipse fit]\n")
        f.write(f"GT ellipses total : {gt_total}\n")
        f.write(f"Detected ellipses : {detect_count}\n")
        f.write(f"Matched to GT     : {match_count}/{gt_total} (Recall: {match_count / max(gt_total, 1) * 100:.1f}%)\n")

        if stats["iou"]:
            ious = np.array(stats["iou"], dtype=np.float32)
            cerrs = np.array(stats["center_err"], dtype=np.float32)
            aerrs = np.array(stats["axis_err"], dtype=np.float32)

            lines = [
                f"IoU          : mean={ious.mean():.3f} std={ious.std():.3f} median={np.median(ious):.3f}",
                f"Center err   : mean={cerrs.mean():.1f}px std={cerrs.std():.1f}px median={np.median(cerrs):.1f}px",
                f"Axis err (%) : mean={aerrs.mean():.1f}% std={aerrs.std():.1f}% median={np.median(aerrs):.1f}%",
                f"IoU >= 0.5   : {(ious >= 0.5).sum()}/{len(ious)} ({(ious >= 0.5).mean() * 100:.1f}%)",
                f"IoU >= 0.7   : {(ious >= 0.7).sum()}/{len(ious)} ({(ious >= 0.7).mean() * 100:.1f}%)",
            ]
            for line in lines:
                print(line)
                f.write(line + "\n")
        else:
            print("No matches found.")
            f.write("No matches found.\n")

    print(f"\nSummary saved to {summary_path}")
    print(f"Visualizations saved to {OUT_DIR}")


if __name__ == "__main__":
    main()