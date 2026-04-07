"""
LIS (Low-light Instance Segmentation) Dataset DataLoader
=========================================================
IJCV 2023 / CVPR 2024 PBDL Challenge 공인 벤치마크

데이터셋 구조:
  LIS/
  ├── RGB-normal/JPEGImages/   ← 정상 조도 이미지 (1.png, 3.png, ...)
  ├── RGB-dark/JPEGImages/     ← 저조도 이미지   (2.JPG, 4.JPG, ...)
  └── annotations/
      ├── lis_coco_png_train.json     ← 정상 이미지용 train 어노테이션
      ├── lis_coco_png_test.json      ← 정상 이미지용 test 어노테이션
      ├── lis_coco_png_train+1.json   ← 저조도 이미지용 train 어노테이션
      └── lis_coco_png_test+1.json    ← 저조도 이미지용 test 어노테이션

클래스 (8개):
  bicycle, car, motorcycle, bus, bottle, chair, dining table, tv

다운로드:
  Google Drive: https://drive.google.com/drive/folders/1KpC82G_H1CI35lmnB2LYr9aK3FQcahAC
"""

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader


# ─────────────────────────────────────────────────────────────────────────────
# 1. COCO → YOLO-seg 변환 유틸리티
# ─────────────────────────────────────────────────────────────────────────────

def coco_to_yolo_seg(
    coco_json_path: str,
    output_dir: str,
    image_dir: str,
) -> Dict[str, str]:
    """
    COCO JSON 어노테이션을 YOLO-seg 텍스트 형식으로 변환합니다.

    YOLO-seg 형식 (각 이미지당 .txt 파일):
      <class_id> <x1> <y1> <x2> <y2> ... <xn> <yn>
      (좌표는 이미지 크기로 정규화된 0~1 범위)

    Args:
        coco_json_path : COCO 어노테이션 JSON 경로
        output_dir     : YOLO 텍스트 파일 저장 디렉토리
        image_dir      : 이미지 디렉토리 (크기 확인용)

    Returns:
        dict : {이미지_파일명: 라벨_파일_경로} 매핑
    """
    os.makedirs(output_dir, exist_ok=True)

    with open(coco_json_path, "r") as f:
        coco = json.load(f)

    # 카테고리 ID → 연속 인덱스 매핑
    cat_id_to_idx = {}
    for i, cat in enumerate(coco["categories"]):
        cat_id_to_idx[cat["id"]] = i

    # 이미지 ID → 이미지 정보 매핑
    img_id_to_info = {}
    for img in coco["images"]:
        img_id_to_info[img["id"]] = img

    # 이미지 ID → 어노테이션 리스트 매핑
    img_id_to_annos = {}
    for anno in coco["annotations"]:
        img_id = anno["image_id"]
        if img_id not in img_id_to_annos:
            img_id_to_annos[img_id] = []
        img_id_to_annos[img_id].append(anno)

    result_map = {}
    converted = 0

    for img_id, img_info in img_id_to_info.items():
        file_name = img_info["file_name"]
        w = img_info["width"]
        h = img_info["height"]

        # 어노테이션이 없는 이미지는 빈 파일 생성
        annos = img_id_to_annos.get(img_id, [])

        # 라벨 파일 이름 (확장자 → .txt)
        stem = Path(file_name).stem
        label_path = os.path.join(output_dir, f"{stem}.txt")

        lines = []
        for anno in annos:
            cat_idx = cat_id_to_idx.get(anno["category_id"])
            if cat_idx is None:
                continue

            # COCO segmentation은 폴리곤 리스트
            if "segmentation" not in anno or not anno["segmentation"]:
                continue
            if isinstance(anno["segmentation"], dict):
                # RLE 형식은 건너뜀 (polygon만 사용)
                continue

            for seg in anno["segmentation"]:
                if len(seg) < 6:  # 최소 3점
                    continue

                # 좌표 정규화 (x/w, y/h)
                normalized = []
                for j in range(0, len(seg), 2):
                    nx = seg[j] / w
                    ny = seg[j + 1] / h
                    normalized.extend([f"{nx:.6f}", f"{ny:.6f}"])

                line = f"{cat_idx} " + " ".join(normalized)
                lines.append(line)

        with open(label_path, "w") as f:
            f.write("\n".join(lines))

        result_map[file_name] = label_path
        converted += 1

    print(f"[COCO→YOLO-seg] 변환 완료: {converted}개 이미지, "
          f"{len(cat_id_to_idx)}개 클래스")
    print(f"  카테고리: {list(cat_id_to_idx.keys())}")
    return result_map


def get_lis_class_names() -> List[str]:
    """LIS 데이터셋 클래스 이름 반환"""
    return [
        "bicycle", "car", "motorcycle", "bus",
        "bottle", "chair", "dining table", "tv",
    ]


# ─────────────────────────────────────────────────────────────────────────────
# 2. LIS Dataset 클래스
# ─────────────────────────────────────────────────────────────────────────────

class LISDataset(Dataset):
    """
    LIS 데이터셋 PyTorch Dataset

    동작 모드:
      - phase=1 (SSL Warm-up):
          반환: {"I_clean": Tensor, "I_aug": Tensor}
          정상 이미지에서 합성 열화 이미지를 동적 생성

      - phase=2 (Task Tuning):
          반환: {"image": Tensor, "masks": Tensor, "labels": Tensor,
                 "I_clean": Tensor (optional)}
          저조도 이미지 + 세그멘테이션 마스크

    Args:
        data_root    : LIS 데이터셋 루트 디렉토리
        split        : "train" or "test"
        phase        : 학습 Phase (1 또는 2)
        img_size     : 이미지 리사이즈 크기 (기본: 640)
        use_dark     : 저조도 이미지 사용 여부 (기본: True, phase 2에서)
    """

    def __init__(
        self,
        data_root: str,
        split: str = "train",
        phase: int = 1,
        img_size: int = 640,
        use_dark: bool = True,
    ):
        super().__init__()
        self.data_root = Path(data_root)
        self.split = split
        self.phase = phase
        self.img_size = img_size
        self.use_dark = use_dark

        # ── 경로 설정
        self.normal_img_dir = self.data_root / "RGB-normal" / "JPEGImages"
        self.dark_img_dir = self.data_root / "RGB-dark" / "JPEGImages"
        self.anno_dir = self.data_root / "annotations"

        # ── COCO 어노테이션 로드
        # 정상 이미지용 어노테이션
        normal_json = self.anno_dir / f"lis_coco_png_{split}.json"
        # 저조도 이미지용 어노테이션
        dark_json = self.anno_dir / f"lis_coco_png_{split}+1.json"

        self.normal_coco = self._load_coco(normal_json)
        self.dark_coco = self._load_coco(dark_json) if dark_json.exists() else None

        # ── 이미지 리스트 구성
        self.samples = self._build_sample_list()
        print(f"[LISDataset] {split}, phase={phase}: {len(self.samples)}개 샘플 로드")

    def _load_coco(self, json_path: Path) -> Optional[dict]:
        """COCO JSON 로드 및 인덱싱"""
        if not json_path.exists():
            print(f"[경고] 어노테이션 파일 없음: {json_path}")
            return None

        with open(json_path, "r") as f:
            coco = json.load(f)

        # 카테고리 매핑
        coco["_cat_map"] = {
            cat["id"]: i for i, cat in enumerate(coco["categories"])
        }
        coco["_num_classes"] = len(coco["categories"])

        # 이미지 ID → 정보 매핑
        coco["_img_map"] = {
            img["id"]: img for img in coco["images"]
        }

        # 이미지 ID → 어노테이션 리스트 매핑
        coco["_anno_map"] = {}
        for anno in coco["annotations"]:
            img_id = anno["image_id"]
            if img_id not in coco["_anno_map"]:
                coco["_anno_map"][img_id] = []
            coco["_anno_map"][img_id].append(anno)

        return coco

    def _build_sample_list(self) -> List[dict]:
        """이미지-어노테이션 쌍 리스트 구성"""
        samples = []

        if self.phase == 1:
            # Phase 1: 정상 이미지만 사용 (합성 열화)
            if self.normal_coco is None:
                return samples
            for img_id, img_info in self.normal_coco["_img_map"].items():
                # 정상 이미지 경로 확인
                img_path = self.normal_img_dir / img_info["file_name"]
                if not img_path.exists():
                    # 확장자 대소문자 차이 대응
                    alt_paths = list(self.normal_img_dir.glob(
                        f"{img_path.stem}.*"
                    ))
                    if alt_paths:
                        img_path = alt_paths[0]
                    else:
                        continue

                samples.append({
                    "normal_path": str(img_path),
                    "img_id": img_id,
                    "coco": self.normal_coco,
                })

        elif self.phase == 2:
            # Phase 2: 저조도 이미지 + 세그멘테이션 라벨
            target_coco = self.dark_coco if self.use_dark else self.normal_coco
            img_dir = self.dark_img_dir if self.use_dark else self.normal_img_dir

            if target_coco is None:
                return samples

            for img_id, img_info in target_coco["_img_map"].items():
                img_path = img_dir / img_info["file_name"]
                if not img_path.exists():
                    alt_paths = list(img_dir.glob(f"{img_path.stem}.*"))
                    if alt_paths:
                        img_path = alt_paths[0]
                    else:
                        continue

                # 대응하는 정상 이미지 (SSL 손실 계산용)
                # LIS에서 dark 이미지 번호 = normal 이미지 번호 - 1
                normal_path = None
                if self.use_dark and self.normal_coco:
                    stem = img_path.stem
                    try:
                        normal_num = int(stem) - 1
                        np_candidates = list(
                            self.normal_img_dir.glob(f"{normal_num}.*")
                        )
                        if np_candidates:
                            normal_path = str(np_candidates[0])
                    except ValueError:
                        pass

                samples.append({
                    "img_path": str(img_path),
                    "normal_path": normal_path,
                    "img_id": img_id,
                    "img_info": img_info,
                    "coco": target_coco,
                })

        return samples

    def _load_image(self, path: str) -> np.ndarray:
        """이미지 로드 + 리사이즈 (BGR → RGB)"""
        img = cv2.imread(path)
        if img is None:
            raise FileNotFoundError(f"이미지 로드 실패: {path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self.img_size, self.img_size))
        return img

    def _image_to_tensor(self, img: np.ndarray) -> torch.Tensor:
        """numpy 이미지를 정규화된 PyTorch 텐서로 변환"""
        # (H, W, C) → (C, H, W), [0, 255] → [0, 1]
        tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        return tensor

    def _parse_masks(
        self, sample: dict
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        COCO 어노테이션에서 세그멘테이션 마스크 & 클래스 라벨 추출

        Returns:
            masks  : (N, H, W) 바이너리 마스크 텐서
            labels : (N,) 클래스 인덱스 텐서
        """
        coco = sample["coco"]
        img_id = sample["img_id"]
        img_info = sample.get("img_info", coco["_img_map"].get(img_id, {}))
        orig_w = img_info.get("width", self.img_size)
        orig_h = img_info.get("height", self.img_size)
        annos = coco["_anno_map"].get(img_id, [])

        masks_list = []
        labels_list = []

        for anno in annos:
            cat_idx = coco["_cat_map"].get(anno["category_id"])
            if cat_idx is None:
                continue

            if "segmentation" not in anno or not anno["segmentation"]:
                continue
            if isinstance(anno["segmentation"], dict):
                continue

            # 폴리곤 → 바이너리 마스크
            mask = np.zeros((orig_h, orig_w), dtype=np.uint8)
            for seg in anno["segmentation"]:
                if len(seg) < 6:
                    continue
                pts = np.array(seg, dtype=np.float32).reshape(-1, 2)
                pts = pts.astype(np.int32)
                cv2.fillPoly(mask, [pts], 1)

            # 리사이즈
            mask = cv2.resize(
                mask, (self.img_size, self.img_size),
                interpolation=cv2.INTER_NEAREST
            )

            masks_list.append(torch.from_numpy(mask).float())
            labels_list.append(cat_idx)

        if masks_list:
            masks = torch.stack(masks_list)        # (N, H, W)
            labels = torch.tensor(labels_list)     # (N,)
        else:
            masks = torch.zeros(0, self.img_size, self.img_size)
            labels = torch.zeros(0, dtype=torch.long)

        return masks, labels

    def _synthetic_augment(self, img_tensor: torch.Tensor) -> torch.Tensor:
        """합성 조명 열화 (Phase 1 SSL용)"""
        r = torch.rand(1).item()
        if r < 0.4:
            # 저조도: Gamma 증가 + 노이즈
            gamma = 2.0 + torch.rand(1).item() * 1.5  # 2.0 ~ 3.5
            aug = img_tensor.pow(gamma)
            noise = torch.randn_like(aug) * 0.02
            aug = (aug + noise).clamp(0.0, 1.0)
        elif r < 0.8:
            # 과노출: 스케일링 + 클리핑
            scale = 1.5 + torch.rand(1).item() * 1.5  # 1.5 ~ 3.0
            aug = (img_tensor * scale).clamp(0.0, 1.0)
            # 랜덤 글레어 패턴
            if torch.rand(1).item() < 0.5:
                C, H, W = aug.shape
                x1 = torch.randint(0, W // 2, (1,)).item()
                y1 = torch.randint(0, H // 2, (1,)).item()
                x2 = min(x1 + torch.randint(W // 4, W // 2, (1,)).item(), W)
                y2 = min(y1 + torch.randint(H // 4, H // 2, (1,)).item(), H)
                aug[:, y1:y2, x1:x2] = 1.0
        else:
            # 혼합: 약한 저조도 + 컬러 시프트
            gamma = 1.3 + torch.rand(1).item() * 0.7
            aug = img_tensor.pow(gamma)
            # 채널별 랜덤 스케일링 (컬러 시프트)
            color_shift = 0.8 + torch.rand(3, 1, 1) * 0.4  # 0.8 ~ 1.2
            aug = (aug * color_shift).clamp(0.0, 1.0)

        return aug

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict:
        sample = self.samples[idx]

        if self.phase == 1:
            # Phase 1: 정상 이미지 + 합성 열화
            img = self._load_image(sample["normal_path"])
            I_clean = self._image_to_tensor(img)
            I_aug = self._synthetic_augment(I_clean)

            return {
                "I_clean": I_clean,        # (3, H, W)
                "I_aug": I_aug,            # (3, H, W)
                "image": I_clean,          # 호환용
            }

        elif self.phase == 2:
            # Phase 2: 이미지 + 마스크 + (옵션) 정상 이미지
            img = self._load_image(sample["img_path"])
            image = self._image_to_tensor(img)
            masks, labels = self._parse_masks(sample)

            result = {
                "image": image,            # (3, H, W)
                "masks": masks,            # (N, H, W)
                "labels": labels,          # (N,)
            }

            # 정상 이미지 로드 (SSL 손실용)
            if sample.get("normal_path") and os.path.exists(sample["normal_path"]):
                normal_img = self._load_image(sample["normal_path"])
                result["I_clean"] = self._image_to_tensor(normal_img)

            return result


def collate_fn(batch: List[dict]) -> dict:
    """
    배치 콜레이트 함수

    마스크/라벨은 이미지마다 개수가 다르므로 리스트로 유지합니다.
    """
    result = {}

    # 텐서 필드는 스택
    for key in ["image", "I_clean", "I_aug"]:
        if key in batch[0]:
            result[key] = torch.stack([b[key] for b in batch])

    # 마스크/라벨은 리스트로 유지 (이미지마다 개수 다름)
    for key in ["masks", "labels"]:
        if key in batch[0]:
            result[key] = [b[key] for b in batch]

    return result


# ─────────────────────────────────────────────────────────────────────────────
# 3. DataLoader 팩토리
# ─────────────────────────────────────────────────────────────────────────────

def get_dataloader(
    data_root: str,
    split: str = "train",
    phase: int = 1,
    batch_size: int = 4,
    img_size: int = 640,
    num_workers: int = 2,
    use_dark: bool = True,
) -> DataLoader:
    """
    LIS 데이터셋 DataLoader 생성

    Args:
        data_root   : LIS 데이터셋 루트 경로
        split       : "train" or "test"
        phase       : 학습 Phase (1 or 2)
        batch_size  : 배치 크기
        img_size    : 이미지 리사이즈 크기
        num_workers : 데이터 로딩 워커 수
        use_dark    : Phase 2에서 저조도 이미지 사용 여부

    Returns:
        DataLoader
    """
    dataset = LISDataset(
        data_root=data_root,
        split=split,
        phase=phase,
        img_size=img_size,
        use_dark=use_dark,
    )

    shuffle = (split == "train")

    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        collate_fn=collate_fn,
        pin_memory=True,
        drop_last=(split == "train"),
    )


# ─────────────────────────────────────────────────────────────────────────────
# 4. COCO→YOLO-seg 변환 CLI & 유틸리티
# ─────────────────────────────────────────────────────────────────────────────

def setup_lis_dataset(data_root: str) -> None:
    """
    LIS 데이터셋 디렉토리 구조를 확인하고 YOLO-seg 라벨을 생성합니다.

    Args:
        data_root : LIS 데이터셋 루트 경로
    """
    root = Path(data_root)

    print("=" * 60)
    print("LIS 데이터셋 설정")
    print("=" * 60)

    # 디렉토리 존재 확인
    required = [
        root / "RGB-normal" / "JPEGImages",
        root / "RGB-dark" / "JPEGImages",
        root / "annotations",
    ]
    for d in required:
        if d.exists():
            n_files = len(list(d.iterdir()))
            print(f"  ✅ {d.relative_to(root)} ({n_files}개 파일)")
        else:
            print(f"  ❌ {d.relative_to(root)} — 없음!")

    # COCO → YOLO-seg 변환
    anno_dir = root / "annotations"
    yolo_label_dir = root / "labels"
    os.makedirs(yolo_label_dir, exist_ok=True)

    json_files = list(anno_dir.glob("*.json"))
    if json_files:
        print(f"\n어노테이션 파일 {len(json_files)}개 발견:")
        for jf in json_files:
            print(f"  - {jf.name}")

        # 정상 이미지용 변환
        for split in ["train", "test"]:
            normal_json = anno_dir / f"lis_coco_png_{split}.json"
            dark_json = anno_dir / f"lis_coco_png_{split}+1.json"

            if normal_json.exists():
                out = yolo_label_dir / f"normal_{split}"
                print(f"\n[변환] {normal_json.name} → {out}")
                coco_to_yolo_seg(
                    str(normal_json), str(out),
                    str(root / "RGB-normal" / "JPEGImages"),
                )

            if dark_json.exists():
                out = yolo_label_dir / f"dark_{split}"
                print(f"\n[변환] {dark_json.name} → {out}")
                coco_to_yolo_seg(
                    str(dark_json), str(out),
                    str(root / "RGB-dark" / "JPEGImages"),
                )
    else:
        print("\n⚠️  어노테이션 파일을 찾을 수 없습니다.")
        print("   LIS 데이터셋을 다운로드해 주세요:")
        print("   https://drive.google.com/drive/folders/"
              "1KpC82G_H1CI35lmnB2LYr9aK3FQcahAC")

    print("\n설정 완료!")


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="LIS Dataset Setup & Test")
    parser.add_argument(
        "--data_root", type=str,
        default="./data/LIS",
        help="LIS 데이터셋 루트 경로",
    )
    parser.add_argument(
        "--setup", action="store_true",
        help="COCO→YOLO-seg 변환 실행",
    )
    parser.add_argument(
        "--test_phase", type=int, default=1, choices=[1, 2],
        help="테스트할 Phase",
    )
    args = parser.parse_args()

    if args.setup:
        setup_lis_dataset(args.data_root)
    else:
        # DataLoader 테스트
        print(f"Phase {args.test_phase} 데이터 로더 테스트")
        try:
            loader = get_dataloader(
                data_root=args.data_root,
                split="train",
                phase=args.test_phase,
                batch_size=2,
                img_size=640,
                num_workers=0,
            )
            for batch in loader:
                print("\n배치 내용:")
                for k, v in batch.items():
                    if isinstance(v, torch.Tensor):
                        print(f"  {k}: {v.shape}")
                    elif isinstance(v, list):
                        shapes = [x.shape for x in v]
                        print(f"  {k}: list of {len(v)} tensors, "
                              f"shapes={shapes}")
                break
            print("\n✅ DataLoader 테스트 성공!")
        except FileNotFoundError as e:
            print(f"\n❌ {e}")
            print("   데이터셋이 올바른 경로에 있는지 확인하세요.")
