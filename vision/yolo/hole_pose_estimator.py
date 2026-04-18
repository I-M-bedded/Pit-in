#!/usr/bin/env python
"""
Estimate hole center pose from YOLO-Pose keypoints.

This script supports:
- plain YOLO-Pose weights
- COIN-patched YOLO-Pose weights
- raw / CLAHE / MSRCR preprocessing

Geometry is mirrored from `vision/dataset_gen/auto_annotator.py`.
For each detection it:
1. collects 9 keypoints (kp_0 center + 8 boundary points),
2. fits an ellipse on the 8 boundary points,
3. optionally replaces kp_0 with the fitted ellipse center,
4. solves camera-frame pose with solvePnP using the known 3D template.

Notes on geometry:
- The projection of a 3D planar circle is generally an ellipse.
- The fitted image ellipse center is not, in general, identical to the
  perspective projection of the true 3D circle center.
- They coincide only in affine / weak-perspective conditions, or when the
  circle plane is close to fronto-parallel.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from train_clahe_baseline import CLAHE_CLIP, CLAHE_GRID, apply_clahe
# Lazy import — coin_pose may not exist outside training environments
# from train_coin_pose import patch_yolo_with_coin
from train_retinex_baseline import (
    COLOR_RESTORE_ALPHA,
    COLOR_RESTORE_BETA,
    RETINEX_GAIN,
    RETINEX_OFFSET,
    RETINEX_SIGMAS,
    apply_msrcr,
)


CLASS_ID_TO_NAME = {
    0: "center_hole_outer",
    1: "center_hole_inner",
    2: "guide_hole_outer",
    3: "guide_hole_inner",
}

# Mirrored from vision/dataset_gen/auto_annotator.py.
# The center outer target is modeled as an ellipse-like slot in that file.
GEOMETRY_SPECS = {
    "center_hole_outer": {"shape": "slot", "size": (0.072, 0.040)},
    "center_hole_inner": {"shape": "circle", "size": 0.026},
    "guide_hole_outer": {"shape": "circle", "size": 0.040},
    "guide_hole_inner": {"shape": "circle", "size": 0.026},
}

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
PNP_METHOD_NAMES = {
    int(cv2.SOLVEPNP_ITERATIVE): "ITERATIVE",
    int(cv2.SOLVEPNP_EPNP): "EPNP",
    int(cv2.SOLVEPNP_P3P): "P3P",
    int(cv2.SOLVEPNP_AP3P): "AP3P",
    int(cv2.SOLVEPNP_IPPE): "IPPE",
    int(cv2.SOLVEPNP_IPPE_SQUARE): "IPPE_SQUARE",
    int(cv2.SOLVEPNP_SQPNP): "SQPNP",
}


def load_intrinsics(path: Optional[Path], fx: float, fy: float, cx: float, cy: float,
                    dist: list[float]) -> tuple[np.ndarray, np.ndarray]:
    if path is None:
        if min(fx, fy) <= 0:
            raise ValueError("Either --intrinsics or positive --fx/--fy/--cx/--cy must be provided.")
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist_coeffs = np.array(dist, dtype=np.float64)
        return camera_matrix, dist_coeffs

    suffix = path.suffix.lower()
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f) if suffix == ".json" else yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Unsupported intrinsics payload in {path}")

    if "camera_matrix" in data:
        cm = data["camera_matrix"]
        if isinstance(cm, dict) and "data" in cm:
            camera_matrix = np.array(cm["data"], dtype=np.float64).reshape(3, 3)
        else:
            camera_matrix = np.array(cm, dtype=np.float64).reshape(3, 3)
    elif "K" in data:
        camera_matrix = np.array(data["K"], dtype=np.float64).reshape(3, 3)
    elif all(k in data for k in ("fx", "fy", "cx", "cy")):
        camera_matrix = np.array(
            [[data["fx"], 0.0, data["cx"]], [0.0, data["fy"], data["cy"]], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    else:
        raise ValueError(f"Could not parse camera matrix from {path}")

    if "dist_coeffs" in data:
        dc = data["dist_coeffs"]
        if isinstance(dc, dict) and "data" in dc:
            dist_coeffs = np.array(dc["data"], dtype=np.float64).reshape(-1)
        else:
            dist_coeffs = np.array(dc, dtype=np.float64).reshape(-1)
    elif "D" in data:
        dist_coeffs = np.array(data["D"], dtype=np.float64).reshape(-1)
    elif "distortion_coefficients" in data:
        dc = data["distortion_coefficients"]
        if isinstance(dc, dict) and "data" in dc:
            dist_coeffs = np.array(dc["data"], dtype=np.float64).reshape(-1)
        else:
            dist_coeffs = np.array(dc, dtype=np.float64).reshape(-1)
    else:
        dist_coeffs = np.zeros(5, dtype=np.float64)

    return camera_matrix, dist_coeffs


def build_object_keypoints(class_id: int) -> np.ndarray:
    class_name = CLASS_ID_TO_NAME[int(class_id)]
    spec = GEOMETRY_SPECS[class_name]
    points = [[0.0, 0.0, 0.0]]

    if spec["shape"] == "circle":
        radius = float(spec["size"]) / 2.0
        rx = radius
        ry = radius
    else:
        rx = float(spec["size"][0]) / 2.0
        ry = float(spec["size"][1]) / 2.0

    for idx in range(8):
        theta = 2.0 * np.pi * idx / 8.0
        points.append([rx * np.cos(theta), ry * np.sin(theta), 0.0])

    return np.array(points, dtype=np.float32)


def preprocess_image(img_bgr: np.ndarray, mode: str) -> np.ndarray:
    if mode == "raw":
        return img_bgr
    if mode == "clahe":
        return apply_clahe(img_bgr, CLAHE_CLIP, CLAHE_GRID)
    if mode == "msrcr":
        return apply_msrcr(
            img_bgr,
            sigmas=RETINEX_SIGMAS,
            gain=RETINEX_GAIN,
            offset=RETINEX_OFFSET,
            alpha=COLOR_RESTORE_ALPHA,
            beta=COLOR_RESTORE_BETA,
        )
    raise ValueError(f"Unsupported preprocessing mode: {mode}")


def iter_images(source: Path) -> Iterable[Path]:
    if source.is_file():
        yield source
        return
    if not source.is_dir():
        raise FileNotFoundError(f"Source not found: {source}")
    for path in sorted(source.iterdir()):
        if path.suffix.lower() in IMAGE_EXTS:
            yield path


def fit_boundary_ellipse(kpts_xy: np.ndarray, kpts_conf: np.ndarray, kp_conf: float) -> Optional[dict]:
    keep = kpts_conf >= kp_conf
    points = kpts_xy[keep]
    if len(points) < 5:
        return None
    ellipse = cv2.fitEllipse(points.astype(np.float32).reshape(-1, 1, 2))
    center_xy = np.array(ellipse[0], dtype=np.float64)
    axes = np.array(ellipse[1], dtype=np.float64)
    angle_deg = float(ellipse[2])
    return {
        "center_xy": center_xy,
        "axes_px": axes,
        "angle_deg": angle_deg,
        "num_points": int(len(points)),
    }


def ellipse_to_conic_matrix(ellipse: dict) -> np.ndarray:
    """Convert ellipse params (from fit_boundary_ellipse) to 3x3 conic matrix.

    The conic C satisfies  [x y 1] C [x y 1]^T = 0  for points on the ellipse.
    """
    cx, cy = ellipse["center_xy"]
    w, h = ellipse["axes_px"]
    a, b = w / 2.0, h / 2.0
    theta = np.deg2rad(ellipse["angle_deg"])
    ct, st = np.cos(theta), np.sin(theta)

    M = np.array([
        [ct ** 2 / a ** 2 + st ** 2 / b ** 2, ct * st * (1 / a ** 2 - 1 / b ** 2)],
        [ct * st * (1 / a ** 2 - 1 / b ** 2), st ** 2 / a ** 2 + ct ** 2 / b ** 2],
    ])
    c = np.array([cx, cy])
    Mc = M @ c

    C = np.zeros((3, 3))
    C[:2, :2] = M
    C[:2, 2] = -Mc
    C[2, :2] = -Mc
    C[2, 2] = c @ M @ c - 1.0
    return C


def conic_center_recovery(
    ellipse: dict, K: np.ndarray
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Recover projected 3D-circle center from its image ellipse via conic back-projection.

    Under perspective projection the ellipse center != projected circle center.
    This uses the pole-polar relationship on the back-projected cone to correct the bias.

    Returns (corrected_center_xy, normal_camera) or None on degeneracy.
    """
    C = ellipse_to_conic_matrix(ellipse)
    K_inv = np.linalg.inv(K)
    Q = K_inv.T @ C @ K_inv
    Q = (Q + Q.T) / 2.0

    eigvals, eigvecs = np.linalg.eigh(Q)  # ascending
    l1, l2, l3 = eigvals

    # Valid circle projection: signature (+,+,-) → after eigh: l1 < 0 < l2 <= l3
    if l1 >= 0 or l3 <= 0 or abs(l2) < 1e-12:
        return None

    s1 = np.sqrt(abs((l2 - l1) / (l3 - l1)))
    s3 = np.sqrt(abs((l3 - l2) / (l3 - l1)))
    v1, v3 = eigvecs[:, 0], eigvecs[:, 2]

    candidates = []
    for n in [s1 * v3 + s3 * v1, s1 * v3 - s3 * v1]:
        vanishing_line = K_inv.T @ n
        try:
            C_inv = np.linalg.inv(C)
        except np.linalg.LinAlgError:
            continue
        ch = C_inv @ vanishing_line
        if abs(ch[2]) < 1e-10:
            continue
        candidates.append((ch[:2] / ch[2], n))

    if not candidates:
        return None

    ec = np.array(ellipse["center_xy"])
    return min(candidates, key=lambda r: np.linalg.norm(r[0] - ec))


# ---------------------------------------------------------------------------
#  Real-time estimator
# ---------------------------------------------------------------------------

@dataclass
class CenterHoleResult:
    """Output of HolePoseEstimator.estimate()."""
    center_xy: Optional[np.ndarray] = None   # pixel coords of center hole
    confidence: float = 0.0                   # 0-1 integrated belief
    source: str = "none"                      # "conic" | "ellipse" | "line" | "ema_*" | "none"
    pose: Optional[dict] = None               # 3D pose from solvePnP
    detections: List[dict] = field(default_factory=list)


class HolePoseEstimator:
    """Two-tier real-time center-hole pose estimator.

    Tier 1 (reliable): CenterHole + CenterHole_B both detected with high
        reliability → conic-based center recovery on the inner circle.
    Tier 2 (fallback): Fit ellipses on all detections, use line fitting
        to estimate center position from the point cloud.

    Temporal EMA filter smooths the output with belief-weighted alpha.
    """

    CENTER_OUTER = 0   # CenterHole_B  (slot 0.072x0.040)
    CENTER_INNER = 1   # CenterHole    (circle d=0.026)
    GUIDE_OUTER = 2
    GUIDE_INNER = 3

    OUTER_AR_THRESHOLD = 1.3  # aspect-ratio gate for slot verification

    def __init__(
        self,
        weights_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        device: str = "cuda:0",
        conf_threshold: float = 0.5,
        kp_conf: float = 0.15,
        reliability_threshold: float = 0.6,
        sigma: float = 20.0,
        half: bool = False,
        imgsz: int = 640,
    ):
        self.model = YOLO(weights_path)
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.device = device
        self.conf_threshold = conf_threshold
        self.kp_conf = kp_conf
        self.reliability_threshold = reliability_threshold
        self.sigma = sigma
        self.half = half
        self.imgsz = imgsz

        # Warm-up: compile CUDA kernels and TensorRT context before live frames
        _dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
        for _ in range(3):
            self.model.predict(
                source=_dummy, conf=conf_threshold,
                device=device, half=half, imgsz=imgsz, verbose=False,
            )

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def estimate(
        self,
        img_bgr: np.ndarray,
        prior_xy: Optional[np.ndarray] = None,
    ) -> CenterHoleResult:
        """Estimate center-hole position from a single BGR frame (stateless).

        Args:
            img_bgr:  BGR image from RealSense color stream.
            prior_xy: Previous center-hole pixel position (u, v) projected from
                      WorldTracker world estimate.  Used by Tier-2b to resolve
                      the line-direction ambiguity caused by the symmetric hole
                      layout.  Pass None when no prior is available.
        """
        results = self.model.predict(
            source=img_bgr, conf=self.conf_threshold,
            device=self.device, half=self.half, imgsz=self.imgsz, verbose=False,
        )
        r = results[0]

        if r.boxes is None or r.keypoints is None or len(r.boxes) == 0:
            return CenterHoleResult()

        detections = self._parse_detections(r)

        # --- Tier 1 ---
        t1 = self._tier1_conic(detections)
        if t1 is not None:
            t1.detections = detections
            return t1

        # --- Tier 2 ---
        t2 = self._tier2_line_fitting(detections, prior_xy)
        t2.detections = detections
        return t2

    # ------------------------------------------------------------------
    #  Detection parsing
    # ------------------------------------------------------------------

    def _parse_detections(self, result) -> List[dict]:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        keypoints = result.keypoints.data.cpu().numpy()

        # Build all detections
        all_dets: dict[int, list] = {}
        for i in range(len(boxes)):
            kxy = keypoints[i, :, :2].astype(np.float64)
            kc = keypoints[i, :, 2].astype(np.float64)
            ell = fit_boundary_ellipse(kxy[1:], kc[1:], self.kp_conf)
            d = {
                "class_id": int(classes[i]),
                "conf": float(scores[i]),
                "bbox": boxes[i],
                "kp0_xy": kxy[0],
                "kp0_conf": float(kc[0]),
                "kpts_xy": kxy,
                "kpts_conf": kc,
                "ellipse": ell,
            }
            all_dets.setdefault(d["class_id"], []).append(d)

        # Keep only the highest-conf detection per class
        dets = []
        for cls_dets in all_dets.values():
            dets.append(max(cls_dets, key=lambda d: d["conf"]))
        return dets

    # ------------------------------------------------------------------
    #  Tier 1 — conic recovery (inner circle)
    # ------------------------------------------------------------------

    def _tier1_conic(self, dets: List[dict]) -> Optional[CenterHoleResult]:
        outer = self._best_of_class(dets, self.CENTER_OUTER, verify_ar=True)
        inner = self._best_of_class(dets, self.CENTER_INNER)
        if outer is None or inner is None:
            return None
        if outer["ellipse"] is None or inner["ellipse"] is None:
            return None

        oc = outer["ellipse"]["center_xy"]
        ic = inner["ellipse"]["center_xy"]
        dist_px = float(np.linalg.norm(oc - ic))

        reliability = outer["conf"] * inner["conf"] / (1.0 + dist_px / self.sigma)
        if reliability < self.reliability_threshold:
            return None

        # Conic recovery on inner (it IS a circle → formula valid)
        corrected = conic_center_recovery(inner["ellipse"], self.K)
        if corrected is not None:
            center_xy, _normal = corrected
            source = "conic"
        else:
            center_xy = ic.copy()
            source = "ellipse"

        pose = self._solve_for(inner, center_xy)
        return CenterHoleResult(center_xy=center_xy, confidence=reliability,
                                source=source, pose=pose)

    # ------------------------------------------------------------------
    #  Tier 2 — line fitting from all detections
    # ------------------------------------------------------------------

    def _tier2_line_fitting(
        self,
        dets: List[dict],
        prior_xy: Optional[np.ndarray] = None,
    ) -> CenterHoleResult:
        # ------------------------------------------------------------------
        # Case 2a: CenterHole_B (outer) present and AR-verified → conic recovery
        # ------------------------------------------------------------------
        outer = self._best_of_class(dets, self.CENTER_OUTER, verify_ar=True)
        if outer is not None and outer["ellipse"] is not None:
            corrected = conic_center_recovery(outer["ellipse"], self.K)
            if corrected is not None:
                center, _ = corrected
                source = "conic"
            else:
                center = outer["ellipse"]["center_xy"].copy()
                source = "ellipse"
            pose = self._solve_for(outer, center)
            return CenterHoleResult(center_xy=center, confidence=outer["conf"] * 0.7,
                                    source=source, pose=pose)

        # ------------------------------------------------------------------
        # Case 2b: no reliable single detection → line fitting on all keypoints
        #
        # Each detection contributes up to 2 points to the cloud:
        #   · boundary kp1-kp8 → ellipse fitting → ellipse center
        #   · center kp0       → used directly
        #
        # Symmetric hole layout causes direction ambiguity on the fitted line.
        # prior_xy (previous center-hole pixel, from WorldTracker back-projection)
        # is used to resolve which end of the line is the correct center direction.
        # Exact hole-position offsets along the line are TODO (user to define).
        # ------------------------------------------------------------------
        pts = []
        valid_dets_for_conf = []
        for d in dets:
            added = False
            if d["ellipse"] is not None:
                pts.append(d["ellipse"]["center_xy"])
                added = True
            if d["kp0_conf"] >= self.kp_conf:
                pts.append(d["kp0_xy"])
                added = True
            if added:
                valid_dets_for_conf.append(d)

        if len(pts) < 2:
            return CenterHoleResult()

        pts = np.array(pts, dtype=np.float64)
        mean = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - mean)
        direction = Vt[0]
        projs = (pts - mean) @ direction

        # Midpoint of projected range as initial center estimate
        mid = (projs.max() + projs.min()) / 2.0
        estimated = mean + mid * direction

        # Direction disambiguation using world prior
        # The symmetric layout produces a mirror candidate:
        #   candidate A = mean + mid * direction
        #   candidate B = mean - mid * direction (reflection through mean)
        # Pick the one closer to prior_xy.
        if prior_xy is not None:
            candidate_b = mean - mid * direction
            if (np.linalg.norm(candidate_b - prior_xy) <
                    np.linalg.norm(estimated - prior_xy)):
                estimated = candidate_b

        avg_conf = float(np.mean([d["conf"] for d in valid_dets_for_conf]))
        return CenterHoleResult(center_xy=estimated, confidence=avg_conf * 0.4,
                                source="line")

    # ------------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------------

    def _best_of_class(self, dets: List[dict], cls: int,
                       verify_ar: bool = False) -> Optional[dict]:
        cands = [d for d in dets if d["class_id"] == cls]
        if verify_ar:
            verified = []
            for c in cands:
                if c["ellipse"] is not None:
                    ax = c["ellipse"]["axes_px"]
                    ar = max(ax) / (min(ax) + 1e-6)
                    if ar >= self.OUTER_AR_THRESHOLD:
                        verified.append(c)
            cands = verified
        return max(cands, key=lambda d: d["conf"]) if cands else None

    def _solve_for(self, det: dict, center_xy: np.ndarray) -> Optional[dict]:
        obj = build_object_keypoints(det["class_id"])
        img = det["kpts_xy"].copy()
        img[0] = center_xy
        valid = det["kpts_conf"] >= self.kp_conf
        valid[0] = True
        if valid.sum() < 4:
            return None
        return solve_pose(obj[valid], img[valid], self.K, self.dist)


def solve_pose(object_points: np.ndarray, image_points: np.ndarray,
               camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Optional[dict]:
    methods = []
    if len(object_points) >= 4 and np.allclose(object_points[:, 2], object_points[0, 2]):
        methods.append(cv2.SOLVEPNP_IPPE)
    methods.extend([cv2.SOLVEPNP_ITERATIVE, cv2.SOLVEPNP_SQPNP])

    obj = np.ascontiguousarray(object_points.astype(np.float32))
    img = np.ascontiguousarray(image_points.astype(np.float32))

    for method in methods:
        try:
            ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs, flags=method)
        except cv2.error:
            continue
        if not ok:
            continue

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        reproj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        reproj = reproj.reshape(-1, 2)
        reproj_err = float(np.sqrt(np.mean(np.sum((reproj - img) ** 2, axis=1))))

        return {
            "solver": int(method),
            "solver_name": PNP_METHOD_NAMES.get(int(method), f"METHOD_{int(method)}"),
            "rvec": rvec.reshape(-1).astype(float).tolist(),
            "tvec_m": tvec.reshape(-1).astype(float).tolist(),
            "xyz_m": tvec.reshape(-1).astype(float).tolist(),
            "normal_camera": rotation_matrix[:, 2].astype(float).tolist(),
            "reprojection_error_px": reproj_err,
            "num_points": int(len(obj)),
        }

    return None


def estimate_detection(
    class_id: int,
    box_conf: float,
    bbox_xyxy: np.ndarray,
    keypoints: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    center_source: str,
    kp_conf: float,
) -> dict:
    class_name = CLASS_ID_TO_NAME.get(int(class_id), f"class_{int(class_id)}")
    object_points = build_object_keypoints(int(class_id))

    kpts_xy = keypoints[:, :2].astype(np.float64)
    kpts_conf = keypoints[:, 2].astype(np.float64)

    ellipse = fit_boundary_ellipse(kpts_xy[1:], kpts_conf[1:], kp_conf)
    kp0_xy = kpts_xy[0]
    kp0_conf = float(kpts_conf[0])

    chosen_center_xy = kp0_xy.copy()
    chosen_center_source = "kp0"
    if center_source == "ellipse" and ellipse is not None:
        chosen_center_xy = ellipse["center_xy"].copy()
        chosen_center_source = "ellipse"

    image_points = kpts_xy.copy()
    image_points[0] = chosen_center_xy

    valid_mask = kpts_conf >= kp_conf
    if chosen_center_source == "ellipse":
        valid_mask[0] = True
    if valid_mask.sum() < 4:
        pose = None
    else:
        pose = solve_pose(
            object_points=object_points[valid_mask],
            image_points=image_points[valid_mask],
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )

    result = {
        "class_id": int(class_id),
        "class_name": class_name,
        "box_conf": float(box_conf),
        "bbox_xyxy": bbox_xyxy.astype(float).tolist(),
        "kp0_xy": kp0_xy.astype(float).tolist(),
        "kp0_conf": kp0_conf,
        "center_xy": chosen_center_xy.astype(float).tolist(),
        "center_source": chosen_center_source,
        "num_valid_keypoints": int(valid_mask.sum()),
        "pose": pose,
    }

    if ellipse is not None:
        result["ellipse"] = {
            "center_xy": ellipse["center_xy"].astype(float).tolist(),
            "axes_px": ellipse["axes_px"].astype(float).tolist(),
            "angle_deg": float(ellipse["angle_deg"]),
            "num_points": int(ellipse["num_points"]),
        }
        result["kp0_vs_ellipse_px"] = float(np.linalg.norm(kp0_xy - ellipse["center_xy"]))
    else:
        result["ellipse"] = None
        result["kp0_vs_ellipse_px"] = None

    return result


def draw_detection(image: np.ndarray, detection: dict) -> np.ndarray:
    out = image.copy()
    x1, y1, x2, y2 = [int(round(v)) for v in detection["bbox_xyxy"]]
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 180), 2)

    center_xy = tuple(int(round(v)) for v in detection["center_xy"])
    kp0_xy = tuple(int(round(v)) for v in detection["kp0_xy"])

    if detection["ellipse"] is not None:
        ecx, ecy = detection["ellipse"]["center_xy"]
        ew, eh = detection["ellipse"]["axes_px"]
        angle = detection["ellipse"]["angle_deg"]
        cv2.ellipse(
            out,
            (int(round(ecx)), int(round(ecy))),
            (max(1, int(round(ew / 2.0))), max(1, int(round(eh / 2.0)))),
            angle,
            0,
            360,
            (255, 180, 0),
            1,
        )
        cv2.circle(out, (int(round(ecx)), int(round(ecy))), 4, (255, 180, 0), -1)

    cv2.circle(out, kp0_xy, 4, (0, 0, 255), -1)
    cv2.circle(out, center_xy, 5, (255, 255, 255), 1)

    label = f"{detection['class_name']} {detection['box_conf']:.2f}"
    cv2.putText(out, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 180), 2)

    pose = detection.get("pose")
    if pose is not None:
        x_m, y_m, z_m = pose["xyz_m"]
        pose_text = f"x={x_m:.4f} y={y_m:.4f} z={z_m:.4f} m"
        cv2.putText(out, pose_text, (x1, min(out.shape[0] - 10, y2 + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1)

    return out


def load_model(yolo_weights: Path, device: str, coin_weights: Optional[Path]) -> YOLO:
    model = YOLO(str(yolo_weights))
    if coin_weights is not None:
        from train_coin_pose import patch_yolo_with_coin
        patch_yolo_with_coin(model, coin_ckpt=str(coin_weights), anchor_weight=0.0, aux_tv_weight=0.0)
    return model


def run_on_image(
    model: YOLO,
    image_path: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    preproc: str,
    conf: float,
    device: str,
    center_source: str,
    kp_conf: float,
) -> tuple[dict, np.ndarray]:
    img_bgr = cv2.imread(str(image_path))
    if img_bgr is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    input_bgr = preprocess_image(img_bgr, preproc)
    results = model.predict(source=input_bgr, conf=conf, device=device, verbose=False)
    result = results[0]

    detections = []
    visual = input_bgr.copy()

    if result.boxes is not None and result.keypoints is not None and len(result.boxes) > 0:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        keypoints = result.keypoints.data.cpu().numpy()

        for det_idx in range(len(boxes)):
            detection = estimate_detection(
                class_id=int(classes[det_idx]),
                box_conf=float(scores[det_idx]),
                bbox_xyxy=boxes[det_idx],
                keypoints=keypoints[det_idx],
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                center_source=center_source,
                kp_conf=kp_conf,
            )
            detections.append(detection)
            visual = draw_detection(visual, detection)

    payload = {
        "image_path": str(image_path.resolve()),
        "preproc": preproc,
        "center_source": center_source,
        "detections": detections,
    }
    return payload, visual


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Estimate hole camera-frame pose from YOLO-Pose keypoints")
    ap.add_argument("--source", type=Path, required=True, help="Image file or directory")
    ap.add_argument("--yolo-weights", type=Path, required=True, help="YOLO pose checkpoint")
    ap.add_argument("--coin-weights", type=Path, default=None, help="COIN module checkpoint")
    ap.add_argument("--intrinsics", type=Path, default=None, help="JSON/YAML with K and D")
    ap.add_argument("--fx", type=float, default=-1.0)
    ap.add_argument("--fy", type=float, default=-1.0)
    ap.add_argument("--cx", type=float, default=-1.0)
    ap.add_argument("--cy", type=float, default=-1.0)
    ap.add_argument("--dist", nargs="*", type=float, default=[0.0, 0.0, 0.0, 0.0, 0.0])
    ap.add_argument("--preproc", choices=["raw", "clahe", "msrcr"], default="raw")
    ap.add_argument("--conf", type=float, default=0.25, help="Box confidence threshold")
    ap.add_argument("--kp-conf", type=float, default=0.15, help="Keypoint confidence threshold for ellipse/PnP")
    ap.add_argument("--center-source", choices=["kp0", "ellipse"], default="ellipse")
    ap.add_argument("--device", type=str, default="0")
    ap.add_argument("--out-dir", type=Path, default=Path("vision/result/hole_pose_estimator"))
    ap.add_argument("--save-vis", action="store_true", help="Save visualization images")
    ap.add_argument("--save-json", action="store_true", help="Save per-image JSON outputs")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    camera_matrix, dist_coeffs = load_intrinsics(args.intrinsics, args.fx, args.fy, args.cx, args.cy, args.dist)
    model = load_model(args.yolo_weights, args.device, args.coin_weights)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    all_results = []

    for image_path in iter_images(args.source):
        payload, visual = run_on_image(
            model=model,
            image_path=image_path,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            preproc=args.preproc,
            conf=args.conf,
            device=args.device,
            center_source=args.center_source,
            kp_conf=args.kp_conf,
        )
        all_results.append(payload)

        if args.save_json:
            out_json = args.out_dir / f"{image_path.stem}.json"
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)

        if args.save_vis:
            out_img = args.out_dir / f"{image_path.stem}_pose.png"
            cv2.imwrite(str(out_img), visual)

        print(json.dumps(payload, ensure_ascii=False, indent=2))

    if len(all_results) > 1:
        summary_path = args.out_dir / "summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(all_results, f, indent=2)


if __name__ == "__main__":
    main()
