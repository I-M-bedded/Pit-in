"""
YOLOv8-seg 단독 추론 & 성능 평가 모듈
=======================================
TD-DRP 없이 YOLOv8-seg 모델만으로 LIS 데이터셋에서 추론하고,
저조도/정상조도 환경별 성능 차이를 측정합니다.

사용법:
  # 데모 추론
  python yolo_seg_backbone.py --mode demo --data_root ./data/LIS

  # 정상조도 vs 저조도 성능 비교
  python yolo_seg_backbone.py --mode eval --data_root ./data/LIS

  # TD-DRP 모델과 비교
  python yolo_seg_backbone.py --mode compare --data_root ./data/LIS --tddrp_ckpt ./checkpoints/tddrp.pt
"""

import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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
# 1. YOLOv8-seg 백본 추출 (TD-DRP 파이프라인용)
# ─────────────────────────────────────────────────────────────────────────────

def load_yolov8_seg_backbone(
    model_name: str = "yolov8n-seg.pt",
    feature_channels: int = 256,
    extract_layer: int = 6,
) -> nn.Module:
    """
    YOLOv8-seg 사전학습 가중치에서 백본만 추출합니다.

    YOLOv8-seg 레이어 구조 (640×640 입력 기준):
      ── Backbone ──
      Layer 0: Conv   ch=16   stride=2   (Stem)
      Layer 1: Conv   ch=32   stride=4
      Layer 2: C2f    ch=32   stride=4
      Layer 3: Conv   ch=64   stride=8
      Layer 4: C2f    ch=64   stride=8   ← P3
      Layer 5: Conv   ch=128  stride=16
      Layer 6: C2f    ch=128  stride=16  ← P4 (기본 추출 지점)
      Layer 7: Conv   ch=256  stride=32
      Layer 8: C2f    ch=256  stride=32
      Layer 9: SPPF   ch=256  stride=32  ← P5
      ── Neck (FPN) ──
      Layer 10-21: Upsample + Concat + C2f + Conv
      ── Head ──
      Layer 22: Segment

    Args:
        model_name      : 모델 이름 (기본: 'yolov8n-seg.pt')
        feature_channels: 출력 채널 수 (기본: 256)
        extract_layer   : 추출 레이어 (기본: 6, P4)

    Returns:
        nn.Module : (B, 3, H, W) → (B, feature_channels, H/stride, W/stride)
    """
    print(f"[YOLOv8-seg] '{model_name}' 백본 로딩 중...")
    yolo_model = YOLO(model_name)
    print(f"[YOLOv8-seg] 모델 로딩 완료!")

    # 백본 레이어 추출
    backbone_layers = nn.Sequential(
        *[yolo_model.model.model[i] for i in range(extract_layer + 1)]
    )

    # 출력 채널 확인
    with torch.no_grad():
        dummy = torch.randn(1, 3, 64, 64)
        out = backbone_layers(dummy)
        actual_channels = out.shape[1]
        actual_stride = 64 // out.shape[2]

    print(f"[YOLOv8-seg] 백본 추출: layer 0~{extract_layer}, "
          f"ch={actual_channels}, stride={actual_stride}")

    # 채널 프로젝션
    if actual_channels != feature_channels:
        print(f"[YOLOv8-seg] 채널 프로젝션: {actual_channels} → {feature_channels}")
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
# 2. YOLOv8-seg 단독 추론 클래스
# ─────────────────────────────────────────────────────────────────────────────

class YOLOSegInference:
    """
    YOLOv8-seg 모델 단독 추론 래퍼

    LIS 데이터셋에서 세그멘테이션 추론을 수행하고 성능을 평가합니다.

    Args:
        model_name : ultralytics 모델 이름 (기본: 'yolov8n-seg.pt')
        conf_thres : 신뢰도 임계값 (기본: 0.25)
        iou_thres  : NMS IoU 임계값 (기본: 0.7)
        device     : 장치 ('cuda' or 'cpu')
    """

    def __init__(
        self,
        model_name: str = "yolov8n-seg.pt",
        conf_thres: float = 0.25,
        iou_thres: float = 0.7,
        device: str = "auto",
    ):
        self.model = YOLO(model_name)
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.device = device

        # LIS 클래스 → COCO 클래스 매핑
        # LIS 8클래스는 COCO 클래스의 부분 집합
        self.lis_classes = [
            "bicycle", "car", "motorcycle", "bus",
            "bottle", "chair", "dining table", "tv",
        ]
        # COCO 클래스 인덱스
        self.coco_class_names = self.model.names
        self.lis_to_coco_map = self._build_class_map()

    def _build_class_map(self) -> Dict[str, int]:
        """LIS 클래스 이름 → COCO 인덱스 매핑"""
        mapping = {}
        for lis_cls in self.lis_classes:
            for coco_idx, coco_name in self.coco_class_names.items():
                if coco_name.lower() == lis_cls.lower():
                    mapping[lis_cls] = coco_idx
                    break
        return mapping

    def predict_single(
        self,
        image_path: str,
        save_vis: bool = False,
        vis_dir: Optional[str] = None,
    ) -> dict:
        """
        단일 이미지 세그멘테이션 추론

        Args:
            image_path : 이미지 경로
            save_vis   : 시각화 결과 저장 여부
            vis_dir    : 시각화 저장 디렉토리

        Returns:
            dict: {
                "boxes": np.ndarray (N, 4),    # xyxy
                "scores": np.ndarray (N,),
                "classes": np.ndarray (N,),
                "masks": np.ndarray (N, H, W), # 바이너리 마스크
                "class_names": list[str],
                "inference_time": float,
            }
        """
        start_time = time.time()
        results = self.model.predict(
            source=image_path,
            conf=self.conf_thres,
            iou=self.iou_thres,
            device=self.device,
            verbose=False,
        )
        inference_time = time.time() - start_time

        result = results[0]
        output = {
            "inference_time": inference_time,
            "boxes": np.array([]),
            "scores": np.array([]),
            "classes": np.array([]),
            "masks": np.array([]),
            "class_names": [],
        }

        if result.boxes is not None and len(result.boxes) > 0:
            output["boxes"] = result.boxes.xyxy.cpu().numpy()
            output["scores"] = result.boxes.conf.cpu().numpy()
            output["classes"] = result.boxes.cls.cpu().numpy().astype(int)
            output["class_names"] = [
                self.coco_class_names[c] for c in output["classes"]
            ]

        if result.masks is not None and len(result.masks) > 0:
            output["masks"] = result.masks.data.cpu().numpy()

        # 시각화 저장
        if save_vis and vis_dir:
            os.makedirs(vis_dir, exist_ok=True)
            vis_img = result.plot()
            stem = Path(image_path).stem
            cv2.imwrite(
                os.path.join(vis_dir, f"{stem}_yolo_seg.jpg"), vis_img
            )

        return output

    def predict_batch(
        self,
        image_paths: List[str],
    ) -> List[dict]:
        """배치 추론"""
        return [self.predict_single(p) for p in image_paths]

    def evaluate_iou(
        self,
        pred_mask: np.ndarray,
        gt_mask: np.ndarray,
    ) -> float:
        """단일 마스크 쌍의 IoU 계산"""
        intersection = np.logical_and(pred_mask > 0.5, gt_mask > 0.5).sum()
        union = np.logical_or(pred_mask > 0.5, gt_mask > 0.5).sum()
        if union == 0:
            return 0.0
        return float(intersection) / float(union)


# ─────────────────────────────────────────────────────────────────────────────
# 3. LIS 데이터셋 평가 함수
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_on_lis(
    model_name: str = "yolov8n-seg.pt",
    data_root: str = "./data/LIS",
    max_images: int = 100,
    save_vis: bool = False,
) -> Dict[str, dict]:
    """
    LIS 데이터셋에서 YOLOv8-seg 성능 평가

    정상조도와 저조도 이미지에서 각각 추론하여
    환경별 성능 차이를 측정합니다.

    Args:
        model_name  : 모델 이름
        data_root   : LIS 루트 경로
        max_images  : 평가할 최대 이미지 수
        save_vis    : 시각화 저장 여부

    Returns:
        dict: {
            "normal": {"avg_conf": float, "avg_detections": float, ...},
            "dark":   {"avg_conf": float, "avg_detections": float, ...},
        }
    """
    root = Path(data_root)
    inference = YOLOSegInference(model_name=model_name)

    vis_dir = str(root / "vis_results") if save_vis else None

    results = {}

    for condition, img_dir_name in [
        ("normal", "RGB-normal"),
        ("dark", "RGB-dark"),
    ]:
        img_dir = root / img_dir_name / "JPEGImages"
        if not img_dir.exists():
            print(f"[경고] {img_dir} 디렉토리 없음, 건너뜀")
            continue

        # 이미지 파일 수집
        image_files = sorted(
            [f for f in img_dir.iterdir()
             if f.suffix.lower() in [".jpg", ".jpeg", ".png", ".bmp"]],
        )[:max_images]

        if not image_files:
            print(f"[경고] {img_dir}에 이미지가 없음")
            continue

        print(f"\n{'─' * 40}")
        print(f"[평가] {condition} 이미지: {len(image_files)}장")
        print(f"{'─' * 40}")

        total_conf = 0.0
        total_detections = 0
        total_time = 0.0
        total_masks = 0
        n_images = len(image_files)

        # LIS 클래스만 필터링
        lis_coco_ids = set(inference.lis_to_coco_map.values())

        for i, img_path in enumerate(image_files):
            pred = inference.predict_single(
                str(img_path),
                save_vis=save_vis,
                vis_dir=vis_dir,
            )

            # LIS 클래스만 필터링
            if len(pred["classes"]) > 0:
                lis_mask = np.isin(pred["classes"], list(lis_coco_ids))
                n_lis = lis_mask.sum()
                lis_confs = pred["scores"][lis_mask] if n_lis > 0 else []

                total_detections += n_lis
                total_conf += sum(lis_confs)
                if len(pred["masks"]) > 0:
                    total_masks += lis_mask.sum()

            total_time += pred["inference_time"]

            if (i + 1) % 20 == 0:
                print(f"  진행: {i + 1}/{n_images}")

        avg_conf = total_conf / max(total_detections, 1)
        avg_det = total_detections / n_images
        avg_time = total_time / n_images

        results[condition] = {
            "n_images": n_images,
            "total_detections": int(total_detections),
            "avg_detections_per_image": round(avg_det, 2),
            "avg_confidence": round(avg_conf, 4),
            "avg_inference_time_ms": round(avg_time * 1000, 1),
            "total_masks": int(total_masks),
        }

        print(f"\n  결과:")
        for k, v in results[condition].items():
            print(f"    {k}: {v}")

    return results


def compare_with_tddrp(
    data_root: str = "./data/LIS",
    tddrp_ckpt: Optional[str] = None,
    max_images: int = 50,
) -> None:
    """
    YOLO-seg 단독 vs TD-DRP 강화 모델 비교

    저조도 이미지에서 두 모델의 성능 차이를 테이블로 출력합니다.

    Args:
        data_root  : LIS 데이터셋 경로
        tddrp_ckpt : TD-DRP 체크포인트 경로
        max_images : 평가 이미지 수
    """
    print("=" * 60)
    print("YOLO-seg 단독 vs TD-DRP 강화 모델 비교")
    print("=" * 60)

    # 1. YOLO-seg 단독 평가
    print("\n[1/2] YOLO-seg 단독 추론...")
    yolo_results = evaluate_on_lis(
        model_name="yolov8n-seg.pt",
        data_root=data_root,
        max_images=max_images,
    )

    # 2. TD-DRP 모델 평가 (체크포인트가 있는 경우)
    if tddrp_ckpt and os.path.exists(tddrp_ckpt):
        print("\n[2/2] TD-DRP 강화 모델 추론...")
        # TD-DRP 모델 로드 및 추론은 train.py에서 학습 완료 후 사용
        from td_drp_v3plus import TDDRPv3Plus, load_yolov8_backbone

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        backbone = load_yolov8_seg_backbone("yolov8n-seg.pt")
        model = TDDRPv3Plus(yolo_backbone=backbone).to(device)
        model.load_state_dict(torch.load(tddrp_ckpt, map_location=device))
        model.eval()
        print("  TD-DRP 모델 로드 완료")
        print("  (TD-DRP 추론은 train.py에서 학습 완료 후 비교 가능)")
    else:
        print("\n[2/2] TD-DRP 체크포인트 없음 — 학습 후 비교 가능")
        print("  사용법: python yolo_seg_backbone.py --mode compare "
              "--tddrp_ckpt ./checkpoints/tddrp.pt")

    # 결과 테이블
    print("\n" + "=" * 60)
    print("비교 결과 (YOLO-seg 단독)")
    print("=" * 60)
    if "normal" in yolo_results and "dark" in yolo_results:
        normal = yolo_results["normal"]
        dark = yolo_results["dark"]

        print(f"\n{'메트릭':<30} {'정상조도':>12} {'저조도':>12} {'차이':>10}")
        print("-" * 66)

        for key in ["avg_detections_per_image", "avg_confidence",
                     "avg_inference_time_ms"]:
            n_val = normal.get(key, 0)
            d_val = dark.get(key, 0)
            diff = d_val - n_val
            diff_str = f"{diff:+.2f}" if isinstance(diff, float) else f"{diff:+d}"
            print(f"  {key:<28} {n_val:>12} {d_val:>12} {diff_str:>10}")

        print()
        print("💡 저조도에서 성능이 하락한다면 TD-DRP가 필요한 근거입니다!")
        print("   TD-DRP 학습 후 다시 비교하세요:")
        print("   python train.py --data_root ./data/LIS --phase 1 --epochs 50")
    else:
        print("정상/저조도 데이터가 모두 필요합니다.")


# ─────────────────────────────────────────────────────────────────────────────
# 4. CLI 인터페이스
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="YOLOv8-seg 단독 추론 & 성능 평가"
    )
    parser.add_argument(
        "--mode", type=str, default="demo",
        choices=["demo", "eval", "compare"],
        help="실행 모드",
    )
    parser.add_argument(
        "--model", type=str, default="yolov8n-seg.pt",
        help="YOLO 모델 이름",
    )
    parser.add_argument(
        "--data_root", type=str, default="./data/LIS",
        help="LIS 데이터셋 루트 경로",
    )
    parser.add_argument(
        "--image", type=str, default=None,
        help="단일 이미지 추론 경로 (demo 모드)",
    )
    parser.add_argument(
        "--max_images", type=int, default=100,
        help="평가할 최대 이미지 수",
    )
    parser.add_argument(
        "--save_vis", action="store_true",
        help="시각화 결과 저장",
    )
    parser.add_argument(
        "--tddrp_ckpt", type=str, default=None,
        help="TD-DRP 체크포인트 경로 (compare 모드)",
    )
    args = parser.parse_args()

    if args.mode == "demo":
        print("=" * 60)
        print("YOLOv8-seg 단독 추론 데모")
        print("=" * 60)

        inference = YOLOSegInference(model_name=args.model)

        if args.image:
            # 단일 이미지 추론
            result = inference.predict_single(
                args.image, save_vis=True, vis_dir="./demo_output"
            )
            print(f"\n추론 결과:")
            print(f"  탐지 수: {len(result['classes'])}")
            print(f"  클래스: {result['class_names']}")
            print(f"  신뢰도: {result['scores']}")
            print(f"  마스크 수: {len(result['masks'])}")
            print(f"  추론 시간: {result['inference_time']*1000:.1f}ms")
        else:
            # LIS 데이터셋에서 샘플 추론
            root = Path(args.data_root)
            for condition in ["RGB-normal", "RGB-dark"]:
                img_dir = root / condition / "JPEGImages"
                if not img_dir.exists():
                    print(f"\n{img_dir} 디렉토리 없음, 건너뜀")
                    continue

                images = sorted(
                    [f for f in img_dir.iterdir()
                     if f.suffix.lower() in [".jpg", ".jpeg", ".png"]],
                )[:3]

                print(f"\n── {condition} 샘플 추론 ({len(images)}장) ──")
                for img_path in images:
                    result = inference.predict_single(
                        str(img_path), save_vis=args.save_vis,
                        vis_dir=str(root / "vis_results"),
                    )
                    n_det = len(result["classes"])
                    avg_conf = (
                        result["scores"].mean() if n_det > 0 else 0
                    )
                    print(f"  {img_path.name}: "
                          f"{n_det}개 탐지, "
                          f"avg_conf={avg_conf:.3f}, "
                          f"{result['inference_time']*1000:.0f}ms")

    elif args.mode == "eval":
        evaluate_on_lis(
            model_name=args.model,
            data_root=args.data_root,
            max_images=args.max_images,
            save_vis=args.save_vis,
        )

    elif args.mode == "compare":
        compare_with_tddrp(
            data_root=args.data_root,
            tddrp_ckpt=args.tddrp_ckpt,
            max_images=args.max_images,
        )
