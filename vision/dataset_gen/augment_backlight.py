#!/usr/bin/env python
"""
Backlight augmentation for YOLO-Pose datasets.

Simulates veiling glare from backlight through holes using:
  1. Cauchy (Lorentzian) PSF — physically models lens flare with long tails
  2. Screen blend — transparent light overlay without hard saturation

Usage:
    python vision/dataset_gen/augment_backlight.py --mode distill --preview
    python vision/dataset_gen/augment_backlight.py --mode both
"""

import argparse
import shutil
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

# ============================================================
# Defaults
# ============================================================

DISTILL_ROOT = Path("vision/dataset/yolo_pose_distill_dataset")
BASELINE_ROOT = Path("vision/dataset/yolo_pose_baseline_dataset")
PREVIEW_DIR = Path("vision/dataset_gen/backlight_preview")

SEED = 42

# --- Veiling glare parameters ---

# Source intensity per hole (0-1 scale, peak value at source center)
# With ~6-10 holes per image, each contributes independently
SOURCE_INTENSITY_MIN = 0.15
SOURCE_INTENSITY_MAX = 0.55

# Cauchy PSF gamma (half-width at half-max in pixels)
# Small gamma → bright tight core + gentle long tail (veiling)
# Real hole radius ~6-15px, gamma slightly larger gives realistic spread
CAUCHY_GAMMA_MIN = 5
CAUCHY_GAMMA_MAX = 15

# Global veil: fraction of MEAN veil added uniformly
GLOBAL_VEIL_FRACTION = 0.03

# Global darkening factor (simulates dark environment)
DARKEN_MIN = 0.15
DARKEN_MAX = 0.50

# Per-hole backlight probability
PER_HOLE_PROB = 0.85

# Color temperature (BGR multipliers)
COLOR_TEMPS = [
    (1.0, 1.0, 1.0),     # neutral white
    (0.85, 0.95, 1.0),   # warm white
    (0.7, 0.9, 1.0),     # warm yellow
    (0.9, 1.0, 1.0),     # slightly warm
]


# ============================================================
# Label parsing
# ============================================================

def parse_yolo_pose_label(label_path: Path, img_w: int, img_h: int
                          ) -> List[Tuple[int, float, float, float, float,
                                          List[Tuple[float, float, int]]]]:
    """Parse YOLO pose label file.

    Returns list of (class_id, cx_px, cy_px, w_px, h_px, [(kx_px, ky_px, vis), ...])
    """
    objects = []
    if not label_path.exists():
        return objects

    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5 + 9 * 3:
                continue
            cls_id = int(parts[0])
            cx = float(parts[1]) * img_w
            cy = float(parts[2]) * img_h
            bw = float(parts[3]) * img_w
            bh = float(parts[4]) * img_h

            keypoints = []
            for ki in range(9):
                base = 5 + ki * 3
                kx = float(parts[base]) * img_w
                ky = float(parts[base + 1]) * img_h
                vis = int(float(parts[base + 2]))
                keypoints.append((kx, ky, vis))
            objects.append((cls_id, cx, cy, bw, bh, keypoints))
    return objects


# ============================================================
# Cauchy PSF veiling glare
# ============================================================

def build_cauchy_veil(h: int, w: int, sources: List[Tuple[float, float, float, float]]
                      ) -> np.ndarray:
    """Build veiling glare map from point light sources using Cauchy PSF.

    Cauchy (Lorentzian) PSF: I(r) = intensity * gamma^2 / (r^2 + gamma^2)
    - Long tails: light spreads across entire image (1/r^2 falloff)
    - Physically models lens flare / veiling glare

    sources: list of (x, y, intensity, gamma)
    Returns: (h, w) float32 glare map
    """
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)

    veil = np.zeros((h, w), dtype=np.float32)

    for sx, sy, intensity, gamma in sources:
        dist_sq = (xx - sx) ** 2 + (yy - sy) ** 2
        gamma_sq = gamma ** 2
        psf = intensity * gamma_sq / (dist_sq + gamma_sq)
        veil += psf

    return veil


def build_streak_flare(h: int, w: int, sources: List[Tuple[float, float, float, float]],
                       angle_deg: float, streak_width: float = 3.0
                       ) -> np.ndarray:
    """Build directional streak flare from point sources.

    Simulates lens diffraction spikes / anamorphic flare.
    Each source emits a narrow line of light at the given angle.

    streak_width: Gaussian sigma perpendicular to streak direction (pixels)
    """
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    angle_rad = np.deg2rad(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    streak = np.zeros((h, w), dtype=np.float32)

    for sx, sy, intensity, gamma in sources:
        dx = xx - sx
        dy = yy - sy

        # Distance along the streak direction
        d_along = dx * cos_a + dy * sin_a
        # Distance perpendicular to streak
        d_perp = -dx * sin_a + dy * cos_a

        # Perpendicular: tight Gaussian falloff (narrow streak)
        perp_falloff = np.exp(-d_perp ** 2 / (2 * streak_width ** 2))
        # Along: Cauchy falloff (long reach)
        along_falloff = intensity * gamma ** 2 / (d_along ** 2 + gamma ** 2)

        streak += perp_falloff * along_falloff

    return streak


def screen_blend(base: np.ndarray, light: np.ndarray) -> np.ndarray:
    """Screen blend: result = 1 - (1-base) * (1-light)

    Simulates additive light without harsh clipping.
    Both inputs should be in [0, 1] float range.
    """
    return 1.0 - (1.0 - base) * (1.0 - light)


def _collect_sources(objects: list, rng: np.random.RandomState
                     ) -> List[Tuple[float, float, float, float]]:
    """Collect point light sources from objects, respecting per-keypoint visibility.

    Uses the centroid of VISIBLE keypoints (vis > 0) as the source position.
    Objects with ALL keypoints vis=0 are skipped entirely.
    """
    sources = []
    for cls_id, cx, cy, bw, bh, keypoints in objects:
        # Only use visible keypoints
        visible_kps = [(kx, ky) for kx, ky, vis in keypoints if vis > 0]
        if not visible_kps:
            continue
        if rng.random() > PER_HOLE_PROB:
            continue

        # Source position = centroid of visible keypoints
        src_x = sum(kx for kx, ky in visible_kps) / len(visible_kps)
        src_y = sum(ky for kx, ky in visible_kps) / len(visible_kps)

        intensity = rng.uniform(SOURCE_INTENSITY_MIN, SOURCE_INTENSITY_MAX)
        gamma = rng.uniform(CAUCHY_GAMMA_MIN, CAUCHY_GAMMA_MAX)
        sources.append((src_x, src_y, intensity, gamma))

    return sources


def apply_backlight(img: np.ndarray,
                    objects: list,
                    rng: np.random.RandomState,
                    darken_factor: float = None,
                    ) -> np.ndarray:
    """Apply physically-based veiling glare backlight augmentation.

    Randomly chooses between two flare styles:
      - 'radial': Cauchy PSF veiling (isotropic spread)
      - 'streak': directional streak + Cauchy veil (anamorphic flare)

    vis=0 keypoints are excluded from source positioning.
    """
    h, w = img.shape[:2]
    img_f = img.astype(np.float32) / 255.0

    # Global darkening
    if darken_factor is None:
        darken_factor = rng.uniform(DARKEN_MIN, DARKEN_MAX)
    img_f *= darken_factor

    # Color temperature
    color_temp = np.array(COLOR_TEMPS[rng.randint(0, len(COLOR_TEMPS))], dtype=np.float32)

    # Collect sources (respecting visibility)
    sources = _collect_sources(objects, rng)

    if not sources:
        return (np.clip(img_f, 0, 1) * 255).astype(np.uint8)

    # Build veiling glare (always present)
    veil = build_cauchy_veil(h, w, sources)

    # 50% chance: add vertical streak flare
    if rng.random() < 0.5:
        angle = 90.0 + rng.uniform(-10, 10)  # near-vertical (±10°)
        streak_w = rng.uniform(1.5, 4.0)
        streak_intensity = rng.uniform(0.3, 0.7)
        streak = build_streak_flare(h, w, sources, angle, streak_w)
        veil += streak * streak_intensity

    # Add global uniform veil
    global_veil = veil.mean() * GLOBAL_VEIL_FRACTION
    veil += global_veil

    np.clip(veil, 0, 1, out=veil)

    # Screen blend per channel with color temperature
    result = np.empty_like(img_f)
    for c in range(3):
        result[:, :, c] = screen_blend(img_f[:, :, c], veil * color_temp[c])

    np.clip(result, 0, 1, out=result)
    return (result * 255).astype(np.uint8)


# ============================================================
# Dataset augmentation
# ============================================================

def augment_distill_dataset(root: Path, splits: List[str], rng: np.random.RandomState,
                            preview_only: bool = False, preview_dir: Path = None):
    """Add _backlight variant to distill dataset based on _dark images."""
    total_created = 0

    for split in splits:
        img_dir = root / split / "images"
        lbl_dir = root / split / "labels"
        if not img_dir.exists():
            continue

        dark_imgs = sorted(img_dir.glob("*_dark.png"))
        print(f"[distill/{split}] Found {len(dark_imgs)} dark images")

        for i, dark_path in enumerate(dark_imgs):
            base_id = dark_path.stem.replace("_dark", "")
            out_img_path = img_dir / f"{base_id}_backlight.png"
            out_lbl_path = lbl_dir / f"{base_id}_backlight.txt"

            if out_img_path.exists() and not preview_only:
                continue

            img = cv2.imread(str(dark_path))
            if img is None:
                continue
            h, w = img.shape[:2]

            dark_lbl = lbl_dir / f"{base_id}_dark.txt"
            objects = parse_yolo_pose_label(dark_lbl, w, h)
            if not objects:
                continue

            backlight_img = apply_backlight(img, objects, rng, darken_factor=1.0)

            if preview_only:
                if total_created < 10:
                    preview_dir.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(preview_dir / f"distill_{split}_{base_id}_dark_orig.png"), img)
                    cv2.imwrite(str(preview_dir / f"distill_{split}_{base_id}_backlight.png"), backlight_img)
                    clean_path = img_dir / f"{base_id}_clean.png"
                    if clean_path.exists():
                        cv2.imwrite(str(preview_dir / f"distill_{split}_{base_id}_clean.png"),
                                    cv2.imread(str(clean_path)))
            else:
                cv2.imwrite(str(out_img_path), backlight_img)
                if dark_lbl.exists():
                    shutil.copy2(dark_lbl, out_lbl_path)

            total_created += 1
            if preview_only and total_created >= 10:
                break
            if not preview_only and (i + 1) % 500 == 0:
                print(f"  [{split}] {i + 1}/{len(dark_imgs)} processed")

        if not preview_only:
            print(f"  [{split}] done: {total_created} backlight images created")

    return total_created


def augment_baseline_dataset(root: Path, splits: List[str], rng: np.random.RandomState,
                             preview_only: bool = False, preview_dir: Path = None):
    """Add backlight variant to baseline dataset."""
    total_created = 0

    for split in splits:
        img_dir = root / split / "images"
        lbl_dir = root / split / "labels"
        if not img_dir.exists():
            continue

        all_imgs = sorted(img_dir.glob("*.png"))
        orig_imgs = [p for p in all_imgs if "_backlight" not in p.stem]
        print(f"[baseline/{split}] Found {len(orig_imgs)} original images")

        for i, img_path in enumerate(orig_imgs):
            out_img_path = img_dir / f"{img_path.stem}_backlight.png"

            if out_img_path.exists() and not preview_only:
                continue

            img = cv2.imread(str(img_path))
            if img is None:
                continue
            h, w = img.shape[:2]

            stem = img_path.stem
            lbl_path = lbl_dir / f"{stem}.txt"
            if not lbl_path.exists():
                parts = stem.split("_")
                lbl_path = lbl_dir / f"{parts[-1]}.txt"

            objects = parse_yolo_pose_label(lbl_path, w, h)
            if not objects:
                continue

            backlight_img = apply_backlight(img, objects, rng, darken_factor=None)

            if preview_only:
                if total_created < 10:
                    preview_dir.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(preview_dir / f"baseline_{split}_{stem}_orig.png"), img)
                    cv2.imwrite(str(preview_dir / f"baseline_{split}_{stem}_backlight.png"), backlight_img)
            else:
                cv2.imwrite(str(out_img_path), backlight_img)
                backlight_lbl = lbl_dir / f"{img_path.stem}_backlight.txt"
                if lbl_path.exists():
                    shutil.copy2(lbl_path, backlight_lbl)

            total_created += 1
            if preview_only and total_created >= 10:
                break
            if not preview_only and (i + 1) % 500 == 0:
                print(f"  [{split}] {i + 1}/{len(orig_imgs)} processed")

        if not preview_only:
            print(f"  [{split}] done: {total_created} backlight images created")

    return total_created


def main():
    ap = argparse.ArgumentParser(description="Backlight augmentation (Cauchy PSF veiling glare)")
    ap.add_argument("--mode", choices=["distill", "baseline", "both"], default="both")
    ap.add_argument("--distill-root", type=Path, default=DISTILL_ROOT)
    ap.add_argument("--baseline-root", type=Path, default=BASELINE_ROOT)
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("--preview-dir", type=Path, default=PREVIEW_DIR)
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--splits", nargs="*", default=["train", "val", "test"])
    args = ap.parse_args()

    rng = np.random.RandomState(args.seed)
    if args.preview:
        print(f"[preview mode] Saving samples to {args.preview_dir}")

    if args.mode in ("distill", "both"):
        if not args.distill_root.exists():
            print(f"[ERROR] Distill root not found: {args.distill_root}")
        else:
            print(f"\n{'='*60}")
            print(f"Augmenting DISTILL dataset: {args.distill_root}")
            print(f"{'='*60}")
            n = augment_distill_dataset(args.distill_root, args.splits, rng,
                                        preview_only=args.preview, preview_dir=args.preview_dir)
            print(f"[distill] Total: {n} backlight images")

    if args.mode in ("baseline", "both"):
        if not args.baseline_root.exists():
            print(f"[ERROR] Baseline root not found: {args.baseline_root}")
        else:
            print(f"\n{'='*60}")
            print(f"Augmenting BASELINE dataset: {args.baseline_root}")
            print(f"{'='*60}")
            n = augment_baseline_dataset(args.baseline_root, args.splits, rng,
                                         preview_only=args.preview, preview_dir=args.preview_dir)
            print(f"[baseline] Total: {n} backlight images")

    print("\nDone.")


if __name__ == "__main__":
    main()
