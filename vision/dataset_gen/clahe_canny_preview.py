"""
images_aug에서 10장 샘플링 → CLAHE → Canny edge 추출 → 저장
"""
import os
import glob
import cv2
import numpy as np

SRC_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/train/images_aug")
OUT_DIR = os.path.abspath("result/clahe_canny_preview")
NUM_SAMPLES = 10

os.makedirs(OUT_DIR, exist_ok=True)

imgs = sorted(glob.glob(os.path.join(SRC_DIR, "*.png")) + glob.glob(os.path.join(SRC_DIR, "*.jpg")))

# 균등 간격 샘플링
step = max(len(imgs) // NUM_SAMPLES, 1)
selected = imgs[::step][:NUM_SAMPLES]

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

for path in selected:
    fname = os.path.basename(path)
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # CLAHE
    enhanced = clahe.apply(gray)

    # Canny
    edges = cv2.Canny(enhanced, 50, 150)

    # 원본 | CLAHE | Canny 비교 이미지
    gray_3ch = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    enhanced_3ch = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    concat = np.hstack([gray_3ch, enhanced_3ch, edges_3ch])

    out_path = os.path.join(OUT_DIR, f"edge_{fname}")
    cv2.imwrite(out_path, concat)
    print(f"Saved: {out_path}")

print(f"\nDone. {len(selected)} images saved to {OUT_DIR}")
