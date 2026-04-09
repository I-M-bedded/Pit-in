import cv2
import os
import json
import numpy as np
import random
import shutil

# --- 설정 ---
INPUT_DIR = "output_paired"
OUTPUT_DIR = "yolo_pose_dataset"

TRAIN_RATIO = 0.8
VAL_RATIO = 0.1
TEST_RATIO = 0.1

IMG_W, IMG_H = 1280, 720
NUM_BOUNDARY = 8
NUM_KP = 9  # center(1) + boundary(8)

# 4 classes
CLASS_MAP = {
    "mask_CenterHole.png": 0,
    "mask_CenterHole_B.png": 1,
    "mask_Hole.png": 2,
    "mask_Hole_B.png": 3,
}

HOLE_KP_LABELS = ["kp_1", "kp_2", "kp_3", "kp_4", "kp_5", "kp_6"]
CENTER_KP_LABELS = ["kp_7", "kp_8", "kp_9"]


def sample_contour_keypoints(contour):
    """contour에서 center(1) + boundary(8) = 9개 keypoint 반환"""
    pts = contour.reshape(-1, 2).astype(np.float64)

    M = cv2.moments(contour)
    if M["m00"] > 0:
        center = (M["m10"] / M["m00"], M["m01"] / M["m00"])
    else:
        center = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))

    if len(pts) < 2:
        return [center] + [center] * NUM_BOUNDARY

    diffs = np.diff(pts, axis=0)
    dists = np.sqrt((diffs ** 2).sum(axis=1))
    cum = np.concatenate([[0.0], np.cumsum(dists)])
    total_len = cum[-1]

    if total_len < 1e-6:
        return [center] + [center] * NUM_BOUNDARY

    targets = np.linspace(0, total_len, NUM_BOUNDARY, endpoint=False)
    boundary = []
    for t in targets:
        idx = int(np.searchsorted(cum, t, side="right")) - 1
        idx = min(idx, len(pts) - 2)
        seg_len = cum[idx + 1] - cum[idx]
        frac = (t - cum[idx]) / seg_len if seg_len > 0 else 0.0
        pt = pts[idx] + frac * (pts[idx + 1] - pts[idx])
        boundary.append((float(pt[0]), float(pt[1])))

    return [center] + boundary


def extract_objects_from_mask(mask_path, class_id):
    """마스크에서 contour별 object 추출"""
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        return []
    _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    objects = []
    for c in contours:
        if cv2.contourArea(c) < 5:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        centroid_x = M["m10"] / M["m00"]

        x, y, w, h = cv2.boundingRect(c)
        cx = (x + w / 2.0) / IMG_W
        cy = (y + h / 2.0) / IMG_H
        nw = w / IMG_W
        nh = h / IMG_H

        kps = sample_contour_keypoints(c)
        kps_norm = [(px / IMG_W, py / IMG_H) for px, py in kps]

        objects.append({
            "class_id": class_id,
            "bbox": (cx, cy, nw, nh),
            "keypoints": kps_norm,
            "centroid_x": centroid_x,
        })

    objects.sort(key=lambda o: o["centroid_x"])
    return objects


def get_visibility_for_object(class_id, sorted_idx, visibility):
    is_hole = class_id in (2, 3)
    if is_hole:
        if sorted_idx < len(HOLE_KP_LABELS):
            return visibility.get(HOLE_KP_LABELS[sorted_idx], 2)
        return 0
    else:
        return min(visibility.get(k, 2) for k in CENTER_KP_LABELS)


def make_label_line(obj, vis):
    cid = obj["class_id"]
    bx, by, bw, bh = obj["bbox"]
    parts = [f"{cid} {bx:.6f} {by:.6f} {bw:.6f} {bh:.6f}"]
    for kx, ky in obj["keypoints"]:
        parts.append(f"{kx:.6f} {ky:.6f} {vis}")
    return " ".join(parts)


def process_sample(folder_path):
    """하나의 sample → pair 단위로 처리 (clean/aug 공통 mask, 각각 label)"""
    meta_path = os.path.join(folder_path, "meta.json")
    if not os.path.exists(meta_path):
        return None

    clean_path = os.path.join(folder_path, "image_clean.png")
    aug_path = os.path.join(folder_path, "image_aug.png")

    # pair 둘 다 있어야 유효
    if not os.path.exists(clean_path) or not os.path.exists(aug_path):
        return None

    # 검은 화면 필터링
    clean_img = cv2.imread(clean_path, cv2.IMREAD_GRAYSCALE)
    aug_img = cv2.imread(aug_path, cv2.IMREAD_GRAYSCALE)
    if clean_img is None or np.max(clean_img) < 10:
        return None
    if aug_img is None or np.max(aug_img) < 10:
        return None

    with open(meta_path, "r") as f:
        meta = json.load(f)
    visibility = meta.get("visibility", {})

    # 4개 마스크에서 object 추출
    all_objects = []
    for mask_file, class_id in CLASS_MAP.items():
        mask_path = os.path.join(folder_path, mask_file)
        if os.path.exists(mask_path):
            objs = extract_objects_from_mask(mask_path, class_id)
            all_objects.extend(objs)

    if not all_objects:
        return None

    # class별 sorted index (visibility 매핑용)
    class_counters = {}
    for obj in all_objects:
        cid = obj["class_id"]
        obj["sorted_idx"] = class_counters.get(cid, 0)
        class_counters[cid] = class_counters.get(cid, 0) + 1

    # Clean label: 모든 keypoint vis=2
    clean_lines = [make_label_line(obj, 2) for obj in all_objects]

    # Aug label: meta.json visibility 반영
    aug_lines = []
    for obj in all_objects:
        vis = get_visibility_for_object(obj["class_id"], obj["sorted_idx"], visibility)
        aug_lines.append(make_label_line(obj, vis))

    return {
        "clean_img": clean_path,
        "aug_img": aug_path,
        "clean_label": "\n".join(clean_lines),
        "aug_label": "\n".join(aug_lines),
    }


def main():
    print("Processing paired samples...")
    all_pairs = []

    folders = sorted([
        d for d in os.listdir(INPUT_DIR)
        if os.path.isdir(os.path.join(INPUT_DIR, d))
    ])

    for i, folder_name in enumerate(folders):
        folder_path = os.path.join(INPUT_DIR, folder_name)
        result = process_sample(folder_path)
        if result:
            result["name"] = folder_name
            all_pairs.append(result)

        if (i + 1) % 500 == 0:
            print(f"  {i+1}/{len(folders)} processed, {len(all_pairs)} valid pairs")

    print(f"Total valid pairs: {len(all_pairs)}")

    # --- Train/Val/Test Split (pair 단위) ---
    random.seed(42)
    random.shuffle(all_pairs)

    total = len(all_pairs)
    train_end = int(total * TRAIN_RATIO)
    val_end = train_end + int(total * VAL_RATIO)

    splits = {
        "train": all_pairs[:train_end],
        "val": all_pairs[train_end:val_end],
        "test": all_pairs[val_end:],
    }

    print(f"Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])} pairs")

    # --- 파일 복사 ---
    # 구조: {split}/images_clean/, {split}/images_aug/, {split}/labels_clean/, {split}/labels_aug/
    # 같은 파일명으로 pair 매칭
    for split_name, pairs in splits.items():
        dir_clean_img = os.path.join(OUTPUT_DIR, split_name, "images_clean")
        dir_aug_img = os.path.join(OUTPUT_DIR, split_name, "images_aug")
        dir_clean_lbl = os.path.join(OUTPUT_DIR, split_name, "labels_clean")
        dir_aug_lbl = os.path.join(OUTPUT_DIR, split_name, "labels_aug")
        os.makedirs(dir_clean_img, exist_ok=True)
        os.makedirs(dir_aug_img, exist_ok=True)
        os.makedirs(dir_clean_lbl, exist_ok=True)
        os.makedirs(dir_aug_lbl, exist_ok=True)

        for pair in pairs:
            name = pair["name"]
            shutil.copy(pair["clean_img"], os.path.join(dir_clean_img, f"{name}.png"))
            shutil.copy(pair["aug_img"], os.path.join(dir_aug_img, f"{name}.png"))
            with open(os.path.join(dir_clean_lbl, f"{name}.txt"), "w") as f:
                f.write(pair["clean_label"])
            with open(os.path.join(dir_aug_lbl, f"{name}.txt"), "w") as f:
                f.write(pair["aug_label"])

    # --- data.yaml ---
    yaml_content = f"""path: {os.path.abspath(OUTPUT_DIR)}

# Paired dataset: clean(teacher) + aug(student)
train_clean: train/images_clean
train_aug: train/images_aug
val_clean: val/images_clean
val_aug: val/images_aug
test_clean: test/images_clean
test_aug: test/images_aug

# Standard YOLO paths (aug 기준, 단독 추론/평가용)
train: train/images_aug
val: val/images_aug
test: test/images_aug

kpt_shape: [{NUM_KP}, 3]

names:
  0: CenterHole
  1: CenterHole_B
  2: Hole
  3: Hole_B

# Keypoint order per object:
#   kp_0: center (centroid)
#   kp_1~kp_8: boundary (equidistant contour points)
"""
    with open(os.path.join(OUTPUT_DIR, "data.yaml"), "w") as f:
        f.write(yaml_content)

    print(f"\nDataset saved to {OUTPUT_DIR}/")
    print(f"  Structure: images_clean/ + images_aug/ + labels_clean/ + labels_aug/")
    print(f"  Same filename = same pair (e.g. 000000.png)")
    print(f"  4 classes, {NUM_KP} keypoints per object (center 1 + boundary 8)")


if __name__ == "__main__":
    main()
