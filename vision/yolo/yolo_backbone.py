"""
YOLO26n-pose 백본 추출 & 단독 추론 모듈
========================================
TD-DRP v4 파이프라인용 백본 추출 및 단독 성능 평가.

YOLO26n-pose 레이어 구조 (640x640 입력, nano scale width=0.25):
  ── Backbone ──
  [0]  Conv    16ch  320x320  (P1/2)
  [1]  Conv    32ch  160x160  (P2/4)
  [2]  C3k2    64ch  160x160
  [3]  Conv    64ch   80x80   (P3/8)
  [4]  C3k2   128ch   80x80   ← P3 feature
  [5]  Conv   128ch   40x40   (P4/16)
  [6]  C3k2   128ch   40x40   ← P4 feature (기본 추출 지점)
  [7]  Conv   256ch   20x20   (P5/32)
  [8]  C3k2   256ch   20x20
  [9]  SPPF   256ch   20x20
  [10] C2PSA  256ch   20x20   ← P5 feature
  ── Head (FPN + PAN) ──
  [11-22] Upsample + Concat + C3k2 + Conv
  [23] Pose26 → end-to-end pose output

사용법:
  python yolo_backbone.py --mode eval --data_root ./data/LIS
"""

import os
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError(
        "ultralytics 패키지가 필요합니다. "
        "'pip install ultralytics' 로 설치하세요."
    )


# ─────────────────────────────────────────────────────────────────────────────
# 1. YOLO26n-pose 백본 추출 (TD-DRP v4 파이프라인용)
# ─────────────────────────────────────────────────────────────────────────────

# 레이어별 출력 채널/해상도 (nano scale, 640x640 입력)
YOLO26N_LAYER_INFO = {
    4:  {'ch': 128, 'stride':  8, 'name': 'P3'},
    6:  {'ch': 128, 'stride': 16, 'name': 'P4'},
    10: {'ch': 256, 'stride': 32, 'name': 'P5'},
}


def load_yolo26_pose_backbone(
    model_name: str = "yolo26n-pose.pt",
    feature_channels: int = 128,
    extract_layer: int = 6,
) -> nn.Module:
    """
    YOLO26n-pose 사전학습 가중치에서 백본을 추출합니다.

    Args:
        model_name      : 모델 이름 (기본: 'yolo26n-pose.pt')
        feature_channels: 원하는 출력 채널 수 (기본: 128, P4와 동일)
        extract_layer   : 추출 레이어 인덱스 (기본: 6 = P4)

    Returns:
        nn.Module : (B, 3, H, W) → (B, feature_channels, H/stride, W/stride)
    """
    print(f"[YOLO26n-pose] '{model_name}' 백본 로딩 중...")
    yolo_model = YOLO(model_name)
    print(f"[YOLO26n-pose] 모델 로딩 완료!")

    backbone_layers = nn.Sequential(
        *[yolo_model.model.model[i] for i in range(extract_layer + 1)]
    )

    # 출력 채널/stride 검증
    with torch.no_grad():
        dummy = torch.randn(1, 3, 64, 64)
        out = backbone_layers(dummy)
        actual_channels = out.shape[1]
        actual_stride = 64 // out.shape[2]

    print(f"[YOLO26n-pose] 백본 추출: layer 0~{extract_layer}, "
          f"ch={actual_channels}, stride={actual_stride}")

    if actual_channels != feature_channels:
        print(f"[YOLO26n-pose] 채널 프로젝션: {actual_channels} → {feature_channels}")
        projection = nn.Sequential(
            nn.Conv2d(actual_channels, feature_channels, 1, bias=False),
            nn.BatchNorm2d(feature_channels),
            nn.ReLU(inplace=True),
        )
        backbone = nn.Sequential(backbone_layers, projection)
    else:
        backbone = backbone_layers

    return backbone


# ─────────────────────────────────────────────────────────────────────────────
# 2. YOLO26n-pose 단독 추론 클래스
# ─────────────────────────────────────────────────────────────────────────────

class YOLOPoseInference:
    """YOLO26n-pose 단독 추론 래퍼"""

    def __init__(
        self,
        model_name: str = "yolo26n-pose.pt",
        conf_thres: float = 0.25,
        device: str = "auto",
    ):
        self.model = YOLO(model_name)
        self.conf_thres = conf_thres
        self.device = device

    def predict_single(self, image_path: str) -> dict:
        """단일 이미지 pose 추론"""
        start_time = time.time()
        results = self.model.predict(
            source=image_path,
            conf=self.conf_thres,
            device=self.device,
            verbose=False,
        )
        inference_time = time.time() - start_time

        result = results[0]
        output = {
            "inference_time": inference_time,
            "boxes": np.array([]),
            "scores": np.array([]),
            "keypoints": np.array([]),
        }

        if result.boxes is not None and len(result.boxes) > 0:
            output["boxes"] = result.boxes.xyxy.cpu().numpy()
            output["scores"] = result.boxes.conf.cpu().numpy()

        if result.keypoints is not None and len(result.keypoints) > 0:
            output["keypoints"] = result.keypoints.data.cpu().numpy()

        return output


# ─────────────────────────────────────────────────────────────────────────────
# 3. 평가 함수
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_on_lis(
    model_name: str = "yolo26n-pose.pt",
    data_root: str = "./data/LIS",
    max_images: int = 100,
) -> Dict[str, dict]:
    """LIS 데이터셋에서 정상/저조도 성능 비교"""
    root = Path(data_root)
    inference = YOLOPoseInference(model_name=model_name)

    results = {}
    for condition, img_dir_name in [
        ("normal", "RGB-normal"),
        ("dark", "RGB-dark"),
    ]:
        img_dir = root / img_dir_name / "JPEGImages"
        if not img_dir.exists():
            print(f"[경고] {img_dir} 없음, 건너뜀")
            continue

        image_files = sorted(
            [f for f in img_dir.iterdir()
             if f.suffix.lower() in [".jpg", ".jpeg", ".png", ".bmp"]],
        )[:max_images]

        if not image_files:
            continue

        print(f"\n[평가] {condition}: {len(image_files)}장")

        total_conf = 0.0
        total_det = 0
        total_time = 0.0

        for i, img_path in enumerate(image_files):
            pred = inference.predict_single(str(img_path))
            n_det = len(pred["scores"])
            total_det += n_det
            total_conf += pred["scores"].sum() if n_det > 0 else 0
            total_time += pred["inference_time"]

            if (i + 1) % 20 == 0:
                print(f"  {i + 1}/{len(image_files)}")

        n = len(image_files)
        results[condition] = {
            "n_images": n,
            "total_detections": total_det,
            "avg_det": round(total_det / n, 2),
            "avg_conf": round(total_conf / max(total_det, 1), 4),
            "avg_time_ms": round(total_time / n * 1000, 1),
        }
        print(f"  결과: {results[condition]}")

    return results


# ─────────────────────────────────────────────────────────────────────────────
# 4. CLI
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="YOLO26n-pose 백본 & 추론")
    parser.add_argument("--mode", type=str, default="demo",
                        choices=["demo", "eval", "layers"])
    parser.add_argument("--model", type=str, default="yolo26n-pose.pt")
    parser.add_argument("--data_root", type=str, default="./data/LIS")
    parser.add_argument("--image", type=str, default=None)
    parser.add_argument("--max_images", type=int, default=100)
    args = parser.parse_args()

    if args.mode == "layers":
        # 레이어 구조 출력
        print("=== YOLO26n-pose Layer Inspection ===")
        yolo = YOLO(args.model)
        x = torch.randn(1, 3, 640, 640)
        for i, layer in enumerate(yolo.model.model):
            name = type(layer).__name__
            print(f"  [{i:2d}] {name}")

    elif args.mode == "eval":
        evaluate_on_lis(
            model_name=args.model,
            data_root=args.data_root,
            max_images=args.max_images,
        )

    elif args.mode == "demo":
        inference = YOLOPoseInference(model_name=args.model)
        if args.image:
            result = inference.predict_single(args.image)
            print(f"Detections: {len(result['scores'])}")
            print(f"Keypoints shape: {result['keypoints'].shape}")
            print(f"Time: {result['inference_time']*1000:.1f}ms")
        else:
            print("--image 경로를 지정하세요.")
