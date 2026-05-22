#!/usr/bin/env python
"""
Estimate hole-center pose from YOLO-Pose keypoints.

This module supports:
- plain YOLO-Pose weights
- COIN-patched YOLO-Pose weights
- raw / CLAHE / MSRCR preprocessing
- tiered real-time center-hole estimation for online servoing

Geometry is mirrored from `vision/dataset_gen/auto_annotator.py`.
For each detection it keeps:
1. the detector center keypoint (`kp_0`),
2. eight boundary keypoints,
3. a fitted ellipse when enough boundary points survive,
4. pose hypotheses from conic recovery and/or solvePnP depending on the tier.

Notes on geometry:
- The projection of a 3D planar circle is generally an ellipse.
- The fitted image ellipse center is not, in general, identical to the
  perspective projection of the true 3D circle center.
- They coincide only in affine / weak-perspective conditions, or when the
  circle plane is close to fronto-parallel.
"""

from __future__ import annotations

import argparse
import itertools
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
# Keep COIN-Pose optional so this file still imports in lighter environments.
# from train_coin_pose import patch_yolo_with_coin
from train_retinex_baseline import (
    COLOR_RESTORE_ALPHA,
    COLOR_RESTORE_BETA,
    RETINEX_GAIN,
    RETINEX_OFFSET,
    RETINEX_SIGMAS,
    apply_msrcr,
)


# Class naming convention used in the training data:
#   suffix "_B" -> outer rim (slot for center target / 4 cm circle for guides)
#   no suffix   -> inner hole (26 mm circle)
# Training class IDs: 0 = CenterHole (inner), 1 = CenterHole_B (outer),
#                     2 = Hole        (inner), 3 = Hole_B        (outer).
CLASS_ID_TO_NAME = {
    0: "center_hole_inner",
    1: "center_hole_outer",
    2: "guide_hole_inner",
    3: "guide_hole_outer",
}

# Mirrored from vision/dataset_gen/auto_annotator.py.
# The center outer target is modeled as an ellipse-like slot in that file.
GEOMETRY_SPECS = {
    "center_hole_outer": {"shape": "slot", "size": (0.072, 0.040)},
    "center_hole_inner": {"shape": "circle", "size": 0.026},
    "guide_hole_outer": {"shape": "circle", "size": 0.040},
    "guide_hole_inner": {"shape": "circle", "size": 0.026},
}

# Dataset geometry uses LV1=2.5 cm for outer contours and LV2=5.5 cm for the
# actual hole centers. Case-2b shifts every observed surface center onto this
# common LV2 reference so inner/outer detections can be mixed consistently.
_LV1_TO_LV2_OFFSET_M = 0.055 - 0.025
_CANONICAL_CENTER_OFFSET_M = {
    "center_hole_outer": _LV1_TO_LV2_OFFSET_M,
    "center_hole_inner": 0.0,
    "guide_hole_outer": _LV1_TO_LV2_OFFSET_M,
    "guide_hole_inner": 0.0,
}

# Experimental board layout: H - 14 cm - H - 8 cm - C - 8 cm - H - 14 cm - H
# Offsets of every hole CENTER from the board centre along the row [m].
# Case-2b enumerates subsets of these to match any mixed detection cloud
# (guides + low-confidence center all count as observation candidates).
_BOARD_OFFSETS_M = np.array([-0.22, -0.08, 0.0, 0.08, 0.22], dtype=np.float64)
_BOARD_CENTER_OFFSET_TOL_M = 1e-9
_BOARD_CLASS_MISMATCH_PX = 8.0
_BOARD_LINE_WEIGHT = 0.05
_BOARD_PRIOR_WEIGHT = 0.35
_CASE2B_CLUSTER_DIST_M = 0.04
_CASE2B_RESCUE_WEIGHT = 0.65

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


def _read_opencv_yaml_intrinsics(path: Path) -> Optional[tuple[np.ndarray, np.ndarray, str]]:
    try:
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    except cv2.error:
        return None
    if not fs.isOpened():
        return None
    try:
        k_node = fs.getNode("K")
        d_node = fs.getNode("D")
        if k_node.empty() or d_node.empty():
            return None
        camera_matrix = k_node.mat().astype(np.float64)
        dist_coeffs = d_node.mat().astype(np.float64).reshape(-1)
        model_node = fs.getNode("camera_model")
        camera_model = model_node.string() if not model_node.empty() else "pinhole"
        return camera_matrix, dist_coeffs, camera_model
    finally:
        fs.release()


def load_intrinsics_with_model(path: Optional[Path], fx: float, fy: float, cx: float, cy: float,
                               dist: list[float]) -> tuple[np.ndarray, np.ndarray, str]:
    if path is None:
        if min(fx, fy) <= 0:
            raise ValueError("Either --intrinsics or positive --fx/--fy/--cx/--cy must be provided.")
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist_coeffs = np.array(dist, dtype=np.float64)
        return camera_matrix, dist_coeffs, "pinhole"

    suffix = path.suffix.lower()
    opencv_payload = _read_opencv_yaml_intrinsics(path)
    if opencv_payload is not None:
        return opencv_payload

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

    camera_model = str(data.get("camera_model", data.get("distortion_model", "pinhole")))
    return camera_matrix, dist_coeffs, camera_model


def load_intrinsics(path: Optional[Path], fx: float, fy: float, cx: float, cy: float,
                    dist: list[float]) -> tuple[np.ndarray, np.ndarray]:
    camera_matrix, dist_coeffs, _ = load_intrinsics_with_model(path, fx, fy, cx, cy, dist)
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


def conic_to_pose(ellipse: dict, K: np.ndarray, radius: float) -> Optional[dict]:
    """Closed-form 3-D pose for a **circle** target from its image ellipse.

    No PnP is used. Pipeline:
      1. ``conic_center_recovery`` -> corrected pixel + plane normal.
      2. Estimate depth from the major-semi-axis relation
             a_major_px ~= f_eff * r / Z
         so Z ~= f_eff * r / a_major_px.
         This is exact for fronto-parallel circles and becomes biased under
         strong oblique viewing.
      3. Back-project the corrected pixel at depth Z to recover ``tvec``.
      4. Build ``rvec`` from the plane normal. Rotation about the normal is
         undetermined for a plain circle, so an arbitrary azimuth is used.

    The returned dict matches ``solve_pose``'s schema and adds
    ``solver_name='CONIC_CLOSED_FORM'`` plus ``corrected_xy``.
    ``reprojection_error_px`` is left as ``None`` because no PnP reprojection
    residual is available in this closed-form branch.
    """

    recovered = conic_center_recovery(ellipse, K)
    if recovered is None:
        return None
    corrected_xy, normal = recovered

    w, h = ellipse["axes_px"]
    a_major_px = max(float(w), float(h)) / 2.0
    if a_major_px < 1e-6:
        return None

    fx = float(K[0, 0]); fy = float(K[1, 1])
    cx = float(K[0, 2]); cy = float(K[1, 2])
    f_eff = float(np.sqrt(fx * fy))
    Z = f_eff * float(radius) / a_major_px
    if Z <= 0.0:
        return None

    u, v = float(corrected_xy[0]), float(corrected_xy[1])
    tvec = np.array(
        [(u - cx) / fx * Z, (v - cy) / fy * Z, Z], dtype=np.float64,
    )

    n_unit = normal / (np.linalg.norm(normal) + 1e-12)
    if float(np.dot(n_unit, tvec)) > 0.0:
        n_unit = -n_unit

    z_axis = -n_unit
    z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-12)
    x_hint = np.array([1.0, 0.0, 0.0])
    x_axis = x_hint - float(np.dot(x_hint, z_axis)) * z_axis
    if np.linalg.norm(x_axis) < 1e-6:
        x_hint = np.array([0.0, 1.0, 0.0])
        x_axis = x_hint - float(np.dot(x_hint, z_axis)) * z_axis
    x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-12)
    y_axis = np.cross(z_axis, x_axis)
    R = np.column_stack([x_axis, y_axis, z_axis])
    rvec, _ = cv2.Rodrigues(R)

    return {
        "solver": -1,
        "solver_name": "CONIC_CLOSED_FORM",
        "rvec": rvec.reshape(-1).astype(float).tolist(),
        "tvec_m": tvec.astype(float).tolist(),
        "xyz_m": tvec.astype(float).tolist(),
        "normal_camera": n_unit.astype(float).tolist(),
        "reprojection_error_px": None,
        "num_points": int(ellipse["num_points"]),
        "corrected_xy": corrected_xy.astype(float).tolist(),
    }


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

    # Valid circle projection: signature (+,+,-) after eigh means l1 < 0 < l2 <= l3.
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
    source: str = "none"                      # "conic" | "ellipse" | "kp0" | "line" | "none"
    pose: Optional[dict] = None               # 3D pose estimate in camera frame
    detections: List[dict] = field(default_factory=list)


class HolePoseEstimator:
    """Tiered real-time center-hole pose estimator.

    Current runtime order:
    1. reliable center target: pinhole may use conic, otherwise IPPE
    2. weak/missing center: assign visible detections to the board template and
       solve one board-plane IPPE pose
    3. legacy fallback: per-detection 3-D anchors + weighted row geometry
    """


    CENTER_INNER = 0   # CenterHole     (circle d=0.026)
    CENTER_OUTER = 1   # CenterHole_B   (slot 0.072 x 0.040)
    GUIDE_INNER  = 2   # Hole           (circle d=0.026)
    GUIDE_OUTER  = 3   # Hole_B         (circle d=0.040)

    OUTER_AR_THRESHOLD = 1.3   # aspect-ratio gate for slot verification
    INNER_ALONE_CONF   = 0.8   # high-confidence center target trigger
    CONIC_CENTER_GATE_PX = 18.0

    def __init__(
        self,
        weights_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        device: str = "cuda:0",
        conf_threshold: float = 0.5,
        kp_conf: float = 0.15,
        reliability_threshold: float = 0.4,
        sigma: float = 20.0,
        half: bool = False,
        imgsz: int = 640,
        camera_model: str = "pinhole",
    ):
        self.model = YOLO(weights_path)
        self.K = np.asarray(camera_matrix, dtype=np.float64)
        self.dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        self.device = device
        self.conf_threshold = conf_threshold
        self.kp_conf = kp_conf
        self.reliability_threshold = reliability_threshold
        self.sigma = sigma
        self.half = half
        self.imgsz = imgsz
        self.camera_model = camera_model
        self.is_fisheye = camera_model.lower() in {"opencv_fisheye", "fisheye"}

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
        prior_tvec_cam: Optional[np.ndarray] = None,
    ) -> CenterHoleResult:
        """Estimate center-hole position from a single BGR frame (stateless).

        Args:
            img_bgr:  BGR image from RealSense color stream.
            prior_xy: Previous center-hole pixel position (u, v) projected from
                      WorldTracker world estimate.  Used by Tier-2b to resolve
                      the line-direction ambiguity caused by the symmetric hole
                      layout.  Pass None when no prior is available.
            prior_tvec_cam: Previous world estimate transformed into the current
                            camera frame.  When available, Tier-2b uses this
                            metric prior to score all board-offset combos.
        """
        results = self.model.predict(
            source=img_bgr, conf=self.conf_threshold,
            device=self.device, half=self.half, imgsz=self.imgsz, verbose=False,
        )
        r = results[0]

        if r.boxes is None or r.keypoints is None or len(r.boxes) == 0:
            return CenterHoleResult()

        detections = self._parse_detections(r)

        # --- Tier C: reliable center target ---
        center_result = self._estimate_center_high(detections)
        if center_result is not None:
            center_result.detections = detections
            return center_result

        # --- Tier B: weak/missing center, solve the whole board plane ---
        board_result = self._estimate_board_ippe(detections, prior_xy, prior_tvec_cam)
        if board_result is not None:
            board_result.detections = detections
            return board_result

        # --- Legacy fallback: per-detection anchors + 3-D row geometry ---
        legacy_result = self._tier2_line_fitting(detections, prior_xy, prior_tvec_cam)
        legacy_result.detections = detections
        return legacy_result

    def project_cam_to_pixel(self, pt_cam: np.ndarray) -> Optional[np.ndarray]:
        return self._project_cam_to_pixel(pt_cam)

    # ------------------------------------------------------------------
    #  Common top-level tiers
    # ------------------------------------------------------------------

    def _estimate_center_high(self, dets: List[dict]) -> Optional[CenterHoleResult]:
        """Use the center target when the detector says it is reliable.

        Pinhole keeps the conic option for the inner circle because it can be
        more accurate than point-template PnP. Fisheye skips conics entirely
        and uses IPPE on undistorted keypoints.
        """
        inner = self._best_of_class(dets, self.CENTER_INNER)
        outer = self._best_of_class(dets, self.CENTER_OUTER, verify_ar=True)

        pair_reliability = 0.0
        if inner is not None and outer is not None:
            pair_dist_px = float(np.linalg.norm(outer["kp0_xy"] - inner["kp0_xy"]))
            pair_reliability = (
                float(outer["conf"]) * float(inner["conf"])
                / (1.0 + pair_dist_px / self.sigma)
            )

        target = None
        confidence = 0.0
        if inner is not None and pair_reliability >= self.reliability_threshold:
            target = inner
            confidence = pair_reliability
        elif inner is not None and float(inner["conf"]) >= self.INNER_ALONE_CONF:
            target = inner
            confidence = float(inner["conf"]) * 0.8
        elif outer is not None and float(outer["conf"]) >= self.INNER_ALONE_CONF:
            target = outer
            confidence = float(outer["conf"]) * 0.7

        if target is None:
            return None

        if not self.is_fisheye and target["class_id"] == self.CENTER_INNER:
            conic_result = self._estimate_center_conic(target, confidence)
            if conic_result is not None:
                return conic_result

        pose = self._solve_for(target, target["kp0_xy"], include_center=True, ippe_only=True)
        return self._center_result_from_pose(
            target,
            pose,
            confidence=confidence,
            source="center_ippe",
        )

    def _estimate_center_conic(
        self,
        det: dict,
        confidence: float,
    ) -> Optional[CenterHoleResult]:
        if det["ellipse"] is None:
            return None
        spec = GEOMETRY_SPECS[CLASS_ID_TO_NAME[det["class_id"]]]
        if spec["shape"] != "circle":
            return None
        radius = float(spec["size"]) / 2.0
        pose = conic_to_pose(det["ellipse"], self.K, radius)
        if pose is None or not self._is_conic_center_sane(det, pose):
            return None
        center_xy = np.array(pose["corrected_xy"], dtype=np.float64)
        pose = dict(pose)
        pose["source"] = "center_conic"
        pose["camera_model"] = self.camera_model
        return CenterHoleResult(
            center_xy=center_xy,
            confidence=float(confidence),
            source="center_conic",
            pose=pose,
        )

    def _center_result_from_pose(
        self,
        det: dict,
        pose: Optional[dict],
        *,
        confidence: float,
        source: str,
    ) -> Optional[CenterHoleResult]:
        if pose is None:
            return None
        class_name = CLASS_ID_TO_NAME[det["class_id"]]
        canonical = self._canonical_center_from_pose(class_name, pose)
        if canonical is None:
            return None

        _, hole_center = canonical
        center_xy = self._project_cam_to_pixel(hole_center)
        if center_xy is None:
            return None

        pose = dict(pose)
        pose["surface_tvec_m"] = pose["tvec_m"]
        pose["tvec_m"] = hole_center.tolist()
        pose["xyz_m"] = hole_center.tolist()
        pose["source"] = source
        pose["camera_model"] = self.camera_model
        return CenterHoleResult(
            center_xy=center_xy,
            confidence=float(confidence),
            source=source,
            pose=pose,
        )

    # ------------------------------------------------------------------
    #  Detection parsing
    # ------------------------------------------------------------------

    def _parse_detections(self, result) -> List[dict]:
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        keypoints = result.keypoints.data.cpu().numpy()

        # Build all detections
        all_dets: List[dict] = []
        for i in range(len(boxes)):
            kxy = keypoints[i, :, :2].astype(np.float64)
            kc = keypoints[i, :, 2].astype(np.float64)
            ell = fit_boundary_ellipse(kxy[1:], kc[1:], self.kp_conf)
            all_dets.append({
                "class_id": int(classes[i]),
                "conf": float(scores[i]),
                "bbox": boxes[i],
                "kp0_xy": kxy[0],
                "kp0_conf": float(kc[0]),
                "kpts_xy": kxy,
                "kpts_conf": kc,
                "ellipse": ell,
            })

        # Dedup ONLY the CENTER classes (single center target per frame).
        # GUIDE detections are kept because the board contains multiple holes.
        center_by_cls: dict[int, dict] = {}
        guides: List[dict] = []
        for d in all_dets:
            cid = d["class_id"]
            if cid in (self.CENTER_OUTER, self.CENTER_INNER):
                cur = center_by_cls.get(cid)
                if cur is None or d["conf"] > cur["conf"]:
                    center_by_cls[cid] = d
            else:
                guides.append(d)
        return list(center_by_cls.values()) + guides

    # ------------------------------------------------------------------
    #  Legacy fallback hierarchy
    # ------------------------------------------------------------------

    def _tier2_line_fitting(
        self,
        dets: List[dict],
        prior_xy: Optional[np.ndarray] = None,
        prior_tvec_cam: Optional[np.ndarray] = None,
    ) -> CenterHoleResult:
        # ------------------------------------------------------------------
        # Final fallback for weak center/board observations.
        #
        # Each detection first tries a boundary-only pose. When contour support
        # is weak, kp0 can rescue the anchor through a separate PnP solve.
        # Every anchor is converted to a canonical LV2 hole center, duplicate
        # detections are clustered by physical hole, and the surviving 3-D hole
        # centers are matched against `_BOARD_OFFSETS_M` with weighted PCA and
        # weighted least squares.
        # ------------------------------------------------------------------
        anchors: List[Tuple[np.ndarray, float, float, dict]] = []
        for det in dets:
            anchor = self._estimate_case2b_hole_center(det)
            if anchor is not None:
                anchors.append(anchor)

        if len(anchors) < 2:
            return CenterHoleResult()

        holes = self._cluster_case2b_centers(anchors)
        if len(holes) < 2:
            return CenterHoleResult()

        tvecs_hat = np.stack([h[0] for h in holes], axis=0)   # (k, 3) meters
        weights   = np.array([h[1] for h in holes], dtype=np.float64)

        matched = self._case2b_weighted_geometry(
            tvecs_hat,
            weights,
            prior_xy,
            prior_tvec_cam,
        )
        if matched is None:
            return CenterHoleResult()

        center_tvec_cam, residual_m, scale = matched

        center_xy = self._project_cam_to_pixel(center_tvec_cam)
        if center_xy is None:
            return CenterHoleResult()

        best = max(holes, key=lambda h: h[1])
        pose = dict(best[3])
        pose["tvec_m"] = center_tvec_cam.tolist()
        pose["xyz_m"] = center_tvec_cam.tolist()
        pose["source"] = "case2b_hybrid_line"
        pose["line_residual_m"] = residual_m
        pose["line_scale"] = scale

        avg_conf = float(np.mean([h[2] for h in holes]))
        residual_gain = 1.0 / (1.0 + residual_m / 0.005)
        return CenterHoleResult(
            center_xy=center_xy,
            confidence=avg_conf * 0.4 * residual_gain,
            source="line",
            pose=pose,
        )

    # ------------------------------------------------------------------
    #  Board-level IPPE fallback
    # ------------------------------------------------------------------

    def _estimate_board_ippe(
        self,
        dets: List[dict],
        prior_xy: Optional[np.ndarray] = None,
        prior_tvec_cam: Optional[np.ndarray] = None,
    ) -> Optional[CenterHoleResult]:
        candidates = self._board_observation_candidates(dets)
        if len(candidates) < 2:
            return None

        centers_rect = np.stack([c["center_rect"] for c in candidates], axis=0)
        weights = np.array([c["weight"] for c in candidates], dtype=np.float64)
        line = self._fit_weighted_image_line(centers_rect, weights)
        if line is None:
            return None
        line_origin, line_dir = line

        max_subset = min(len(candidates), len(_BOARD_OFFSETS_M))
        subset_sizes = range(max_subset, 1, -1)
        best: Optional[tuple] = None

        for subset_size in subset_sizes:
            for subset in itertools.combinations(candidates, subset_size):
                ordered = sorted(
                    subset,
                    key=lambda item: float(np.dot(item["center_rect"] - line_origin, line_dir)),
                )
                for ordered_try in (ordered, list(reversed(ordered))):
                    for offsets in itertools.combinations(_BOARD_OFFSETS_M, subset_size):
                        hypothesis = self._solve_board_assignment(
                            ordered_try,
                            np.array(offsets, dtype=np.float64),
                            line_origin,
                            line_dir,
                            prior_xy,
                            prior_tvec_cam,
                        )
                        if hypothesis is None:
                            continue
                        if best is None or hypothesis[0] < best[0]:
                            best = hypothesis
            if best is not None:
                break

        if best is None:
            return None

        score, pose, center_xy, mean_conf = best
        reproj = float(pose.get("reprojection_error_px", 10.0))
        reproj_gain = 1.0 / (1.0 + reproj / 2.0)
        score_gain = 1.0 / (1.0 + score / 10.0)
        confidence = mean_conf * reproj_gain * score_gain
        return CenterHoleResult(
            center_xy=center_xy,
            confidence=float(confidence),
            source="board_ippe",
            pose=pose,
        )

    def _board_observation_candidates(self, dets: List[dict]) -> List[dict]:
        candidates = []
        for idx, det in enumerate(dets):
            if det["kp0_conf"] < self.kp_conf:
                continue
            center_rect = self._undistorted_pixel_points(det["kp0_xy"].reshape(1, 2))
            if center_rect is None:
                continue
            boundary_support = float(np.count_nonzero(det["kpts_conf"][1:] >= self.kp_conf)) / 8.0
            weight = float(det["conf"]) * (0.35 + 0.65 * boundary_support)
            candidates.append({
                "idx": idx,
                "det": det,
                "center_rect": center_rect.reshape(2),
                "weight": max(weight, 1e-6),
            })
        return candidates

    def _fit_weighted_image_line(
        self,
        points: np.ndarray,
        weights: np.ndarray,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        w_sum = float(weights.sum())
        if len(points) < 2 or w_sum <= 0.0:
            return None
        w = weights / w_sum
        mean = (w[:, None] * points).sum(axis=0)
        centered = points - mean
        cov = np.einsum("i,ij,ik->jk", w, centered, centered)
        eigvals, eigvecs = np.linalg.eigh(cov)
        if float(eigvals[-1]) <= 1e-12:
            return None
        direction = eigvecs[:, -1]
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        return mean, direction

    def _solve_board_assignment(
        self,
        ordered: List[dict],
        offsets: np.ndarray,
        line_origin: np.ndarray,
        line_dir: np.ndarray,
        prior_xy: Optional[np.ndarray],
        prior_tvec_cam: Optional[np.ndarray],
    ) -> Optional[tuple]:
        object_points = []
        image_points = []
        confs = []
        class_penalty = 0.0
        line_residual = 0.0

        for item, offset in zip(ordered, offsets):
            det = item["det"]
            valid = det["kpts_conf"] >= self.kp_conf
            if np.count_nonzero(valid) < 2:
                continue
            local_obj = build_object_keypoints(det["class_id"]).astype(np.float64)
            board_obj = local_obj.copy()
            board_obj[:, 0] += float(offset)
            object_points.append(board_obj[valid])
            image_points.append(det["kpts_xy"][valid])
            confs.append(float(det["conf"]))

            residual_vec = item["center_rect"] - line_origin
            along = float(np.dot(residual_vec, line_dir))
            closest = line_origin + along * line_dir
            line_residual += item["weight"] * float(np.linalg.norm(item["center_rect"] - closest))
            class_penalty += self._board_class_assignment_penalty(det, float(offset))

        if not object_points:
            return None

        obj = np.vstack(object_points)
        img = np.vstack(image_points)
        if len(obj) < 4:
            return None

        pose = solve_pose(
            obj,
            img,
            self.K,
            self.dist,
            self.camera_model,
            ippe_only=True,
        )
        if pose is None:
            return None

        center_tvec = np.array(pose["tvec_m"], dtype=np.float64)
        center_xy = self._project_cam_to_pixel(center_tvec)
        if center_xy is None:
            return None

        reproj = float(pose.get("reprojection_error_px", 10.0))
        normalized_line_residual = line_residual / max(sum(item["weight"] for item in ordered), 1e-9)
        score = reproj + _BOARD_LINE_WEIGHT * normalized_line_residual + class_penalty
        if prior_tvec_cam is not None:
            score += _BOARD_PRIOR_WEIGHT * float(np.linalg.norm(center_tvec - prior_tvec_cam))
        elif prior_xy is not None:
            score += 0.0005 * min(float(np.linalg.norm(center_xy - prior_xy)), 200.0)

        pose = dict(pose)
        pose["source"] = "board_ippe"
        pose["camera_model"] = self.camera_model
        pose["board_offsets_m"] = [float(v) for v in offsets]
        pose["line_residual_px"] = float(normalized_line_residual)
        pose["assignment_class_penalty"] = float(class_penalty)
        pose["tvec_m"] = center_tvec.tolist()
        pose["xyz_m"] = center_tvec.tolist()
        mean_conf = float(np.mean(confs)) if confs else 0.0
        return score, pose, center_xy, mean_conf

    def _board_class_assignment_penalty(self, det: dict, offset: float) -> float:
        is_center_class = det["class_id"] in (self.CENTER_INNER, self.CENTER_OUTER)
        is_center_offset = abs(offset) <= _BOARD_CENTER_OFFSET_TOL_M
        if is_center_class != is_center_offset:
            return _BOARD_CLASS_MISMATCH_PX
        return 0.0

    def _undistorted_pixel_points(self, points_xy: np.ndarray) -> Optional[np.ndarray]:
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 1, 2)
        try:
            if self.is_fisheye:
                out = cv2.fisheye.undistortPoints(
                    pts,
                    self.K,
                    self.dist.reshape(-1, 1),
                    R=np.eye(3, dtype=np.float64),
                    P=self.K,
                )
            elif self.dist.size > 0 and np.any(np.abs(self.dist) > 1e-12):
                out = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
            else:
                out = pts
        except cv2.error:
            return None
        return out.reshape(-1, 2).astype(np.float64)

    # ------------------------------------------------------------------
    #  Case 2b - hybrid anchor and weighted geometry helpers
    # ------------------------------------------------------------------

    def _estimate_case2b_hole_center(
        self, det: dict,
    ) -> Optional[Tuple[np.ndarray, float, float, dict]]:
        """Recover one physical-hole center for Case-2b from one detection.

        The final fallback should survive broken contours, so we build:
        1. a boundary-only anchor when the contour is still usable
        2. a kp0-assisted rescue anchor when boundary support collapses

        Both anchors are converted onto the common LV2 hole-center reference.
        When both exist, they are blended with a preference for the boundary
        anchor unless it disagrees strongly with kp0 in image space.
        """
        class_name = CLASS_ID_TO_NAME[det["class_id"]]

        boundary_pose = self._solve_for(det, include_center=False)
        boundary_candidate = self._case2b_candidate_from_pose(
            det, class_name, boundary_pose, use_kp0=False, source="boundary"
        )

        kp0_candidate = None
        if det["kp0_conf"] >= self.kp_conf:
            kp0_pose = self._solve_for(det, det["kp0_xy"], include_center=True)
            kp0_candidate = self._case2b_candidate_from_pose(
                det, class_name, kp0_pose, use_kp0=True, source="kp0_rescue"
            )

        if boundary_candidate is not None and kp0_candidate is not None:
            return self._blend_case2b_candidates(det, boundary_candidate, kp0_candidate)
        if boundary_candidate is not None:
            center_3d, weight, conf, pose = boundary_candidate
            pose = dict(pose)
            pose["case2b_mode"] = "boundary_only"
            return center_3d, weight, conf, pose
        if kp0_candidate is not None:
            center_3d, weight, conf, pose = kp0_candidate
            pose = dict(pose)
            pose["case2b_mode"] = "kp0_only"
            return center_3d, weight, conf, pose
        return None

    def _case2b_candidate_from_pose(
        self,
        det: dict,
        class_name: str,
        pose: Optional[dict],
        *,
        use_kp0: bool,
        source: str,
    ) -> Optional[Tuple[np.ndarray, float, float, dict]]:
        if pose is None:
            return None

        canonical = self._canonical_center_from_pose(class_name, pose)
        if canonical is None:
            return None
        surface_center, hole_center = canonical

        conf = float(det["conf"])
        kp_strength = float(np.clip(det["kp0_conf"], 0.0, 1.0))
        num_boundary = float(np.count_nonzero(det["kpts_conf"][1:] >= self.kp_conf))
        boundary_support = min(num_boundary / 8.0, 1.0)
        reproj = float(pose.get("reprojection_error_px", 10.0))
        reproj_gain = 1.0 / (1.0 + reproj / 2.0)

        if use_kp0:
            support = np.clip(0.20 + 0.30 * boundary_support + 0.50 * kp_strength, 0.0, 1.0)
            weight = (conf ** 2) * reproj_gain * support * _CASE2B_RESCUE_WEIGHT
        else:
            support = np.clip(0.25 + 0.75 * boundary_support, 0.0, 1.0)
            weight = (conf ** 2) * reproj_gain * support

        pose = dict(pose)
        pose["surface_tvec_m"] = surface_center.tolist()
        pose["canonical_center_tvec_m"] = hole_center.tolist()
        pose["case2b_anchor_source"] = source
        pose["case2b_support"] = float(support)
        pose["case2b_reproj_gain"] = float(reproj_gain)
        return hole_center, float(weight), conf, pose

    def _blend_case2b_candidates(
        self,
        det: dict,
        boundary_candidate: Tuple[np.ndarray, float, float, dict],
        kp0_candidate: Tuple[np.ndarray, float, float, dict],
    ) -> Tuple[np.ndarray, float, float, dict]:
        b_center, b_weight, conf, b_pose = boundary_candidate
        k_center, k_weight, _, k_pose = kp0_candidate

        b_px = self._project_cam_to_pixel(b_center)
        k_px = self._project_cam_to_pixel(k_center)
        kp0_xy = det["kp0_xy"]

        b_consistency = self._pixel_consistency_gain(b_px, kp0_xy)
        k_consistency = self._pixel_consistency_gain(k_px, kp0_xy)

        b_score = b_weight * (0.55 + 0.45 * b_consistency)
        k_score = k_weight * (0.35 + 0.65 * k_consistency)
        score_sum = b_score + k_score
        alpha = b_score / score_sum if score_sum > 1e-12 else 0.5

        hole_center = alpha * b_center + (1.0 - alpha) * k_center
        agreement = max(b_consistency, k_consistency)
        weight = (alpha * b_weight + (1.0 - alpha) * k_weight) * (0.60 + 0.40 * agreement)

        pose = dict(b_pose if alpha >= 0.5 else k_pose)
        pose["canonical_center_tvec_m"] = hole_center.tolist()
        pose["case2b_mode"] = "hybrid"
        pose["case2b_blend_alpha"] = float(alpha)
        pose["case2b_boundary_weight"] = float(b_weight)
        pose["case2b_kp0_weight"] = float(k_weight)
        pose["case2b_boundary_consistency"] = float(b_consistency)
        pose["case2b_kp0_consistency"] = float(k_consistency)
        return hole_center, float(weight), conf, pose

    def _cluster_case2b_centers(
        self,
        anchors: List[Tuple[np.ndarray, float, float, dict]],
    ) -> List[Tuple[np.ndarray, float, float, dict]]:
        """Merge duplicate detections that belong to the same physical hole."""
        clusters: List[dict] = []

        for center_3d, weight, conf, pose in sorted(anchors, key=lambda a: a[1], reverse=True):
            best_idx = None
            best_dist = float("inf")
            for idx, cluster in enumerate(clusters):
                dist = float(np.linalg.norm(center_3d - cluster["center_3d"]))
                if dist < _CASE2B_CLUSTER_DIST_M and dist < best_dist:
                    best_idx = idx
                    best_dist = dist

            if best_idx is None:
                clusters.append({
                    "center_3d": center_3d.copy(),
                    "weight": float(weight),
                    "conf_sum": float(conf),
                    "count": 1,
                    "best_weight": float(weight),
                    "best_pose": pose,
                })
                continue

            cluster = clusters[best_idx]
            prev_weight = cluster["weight"]
            new_weight = prev_weight + float(weight)
            if new_weight > 0.0:
                cluster["center_3d"] = (
                    prev_weight * cluster["center_3d"] + float(weight) * center_3d
                ) / new_weight
            cluster["weight"] = new_weight
            cluster["conf_sum"] += float(conf)
            cluster["count"] += 1
            if float(weight) > cluster["best_weight"]:
                cluster["best_weight"] = float(weight)
                cluster["best_pose"] = pose

        merged: List[Tuple[np.ndarray, float, float, dict]] = []
        for cluster in clusters:
            merged.append((
                cluster["center_3d"],
                float(cluster["weight"]),
                float(cluster["conf_sum"] / max(cluster["count"], 1)),
                cluster["best_pose"],
            ))
        return merged

    def _case2b_weighted_geometry(
        self,
        tvecs: np.ndarray,
        weights: np.ndarray,
        prior_xy: Optional[np.ndarray],
        prior_tvec_cam: Optional[np.ndarray] = None,
    ) -> Optional[Tuple[np.ndarray, float, float]]:
        """Weighted PCA + weighted LSQ against ``_BOARD_OFFSETS_M`` in m.

        Returns (center_tvec_cam, residual_m, scale) or None.
        """
        k = tvecs.shape[0]
        if k < 2:
            return None

        w_sum = float(weights.sum())
        if w_sum <= 0.0:
            return None
        w = weights / w_sum

        # Weighted mean and covariance in camera-metric space.
        mean_3d  = (w[:, None] * tvecs).sum(axis=0)
        centered = tvecs - mean_3d
        cov = np.einsum("i,ij,ik->jk", w, centered, centered)

        eigvecs = np.linalg.eigh(cov)[1]
        direction_3d = eigvecs[:, -1]               # principal axis (row dir)
        projs_1d = centered @ direction_3d          # meters

        order = np.argsort(projs_1d)
        projs_sorted = projs_1d[order]
        w_sorted     = w[order]

        SCALE_WEIGHT = 0.01
        PRIOR_WEIGHT = 0.35
        best: Optional[tuple] = None

        for combo in itertools.combinations(_BOARD_OFFSETS_M, k):
            offsets = np.array(combo, dtype=np.float64)   # ascending
            A = np.column_stack([offsets, np.ones_like(offsets)])
            AtW = A.T * w_sorted
            try:
                sol = np.linalg.solve(AtW @ A, AtW @ projs_sorted)
            except np.linalg.LinAlgError:
                continue
            s, c = float(sol[0]), float(sol[1])

            if s < 0.0:
                s, c = -s, -c
                direction_try = -direction_3d
            else:
                direction_try = direction_3d

            residuals = projs_sorted - (s * offsets + c)
            weighted_rms = float(np.sqrt(np.sum(w_sorted * residuals ** 2)))
            center_3d = mean_3d + c * direction_try
            score = weighted_rms + SCALE_WEIGHT * abs(s - 1.0)

            # With only two visible holes, LSQ residuals are nearly always
            # zero, so the previous world estimate must participate in combo
            # selection itself instead of only resolving the final mirror.
            if prior_tvec_cam is not None:
                c_proj = float(np.dot(center_3d - mean_3d, direction_try))
                mirror_3d = mean_3d - c_proj * direction_try
                center_dist = float(np.linalg.norm(center_3d - prior_tvec_cam))
                mirror_dist = float(np.linalg.norm(mirror_3d - prior_tvec_cam))
                if mirror_dist < center_dist:
                    center_3d = mirror_3d
                    prior_dist = mirror_dist
                else:
                    prior_dist = center_dist
                score += PRIOR_WEIGHT * prior_dist
            elif prior_xy is not None:
                center_px = self._project_cam_to_pixel(center_3d)
                if center_px is not None:
                    prior_px = float(np.linalg.norm(center_px - prior_xy))
                    score += 0.0005 * min(prior_px, 200.0)

            if best is None or score < best[0]:
                best = (score, weighted_rms, center_3d, direction_try, s)

        if best is None:
            return None

        _, residual, center_3d, direction, scale = best

        if prior_tvec_cam is None and prior_xy is not None:
            c_proj = float(np.dot(center_3d - mean_3d, direction))
            mirror_3d = mean_3d - c_proj * direction
            center_px = self._project_cam_to_pixel(center_3d)
            mirror_px = self._project_cam_to_pixel(mirror_3d)
            if (center_px is not None and mirror_px is not None
                    and np.linalg.norm(mirror_px - prior_xy)
                        < np.linalg.norm(center_px - prior_xy)):
                center_3d = mirror_3d

        return center_3d, residual, scale

    def _project_cam_to_pixel(self, pt_cam: np.ndarray) -> Optional[np.ndarray]:
        """Project a 3-D camera-frame point to raw image pixel (u, v)."""
        Z = float(pt_cam[2])
        if Z <= 0.0:
            return None

        if self.is_fisheye:
            obj = np.asarray(pt_cam, dtype=np.float64).reshape(1, 1, 3)
            rvec = np.zeros((3, 1), dtype=np.float64)
            tvec = np.zeros((3, 1), dtype=np.float64)
            try:
                projected, _ = cv2.fisheye.projectPoints(
                    obj,
                    rvec,
                    tvec,
                    self.K,
                    self.dist.reshape(-1, 1),
                )
            except cv2.error:
                return None
            return projected.reshape(2).astype(np.float64)

        fx = float(self.K[0, 0]); fy = float(self.K[1, 1])
        cx = float(self.K[0, 2]); cy = float(self.K[1, 2])
        return np.array(
            [fx * pt_cam[0] / Z + cx, fy * pt_cam[1] / Z + cy],
            dtype=np.float64,
        )

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

    def _is_conic_center_sane(self, det: dict, pose: dict) -> bool:
        """Reject conic corrections that jump too far from detector center."""
        if pose is None or "corrected_xy" not in pose:
            return False
        corrected = np.array(pose["corrected_xy"], dtype=np.float64)
        kp0 = np.array(det["kp0_xy"], dtype=np.float64)
        if not np.all(np.isfinite(corrected)) or not np.all(np.isfinite(kp0)):
            return False

        return float(np.linalg.norm(corrected - kp0)) <= self.CONIC_CENTER_GATE_PX

    def _canonical_center_from_pose(
        self,
        class_name: str,
        pose: dict,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        surface_center = np.array(pose["tvec_m"], dtype=np.float64)
        if surface_center[2] <= 0.0:
            return None
        z_axis = self._pose_z_axis(pose)
        hole_center = surface_center + self._canonical_center_offset_m(class_name) * z_axis
        if hole_center[2] <= 0.0:
            return None
        return surface_center, hole_center

    def _pixel_consistency_gain(
        self,
        predicted_xy: Optional[np.ndarray],
        observed_xy: np.ndarray,
    ) -> float:
        if predicted_xy is None:
            return 0.0
        sigma_px = max(float(self.sigma), 1.0)
        dist_px = float(np.linalg.norm(predicted_xy - observed_xy))
        return float(np.exp(-0.5 * (dist_px / sigma_px) ** 2))

    def _canonical_center_offset_m(self, class_name: str) -> float:
        return float(_CANONICAL_CENTER_OFFSET_M[class_name])

    def _pose_z_axis(self, pose: dict) -> np.ndarray:
        rvec = np.array(pose["rvec"], dtype=np.float64).reshape(3, 1)
        tvec = np.array(pose["tvec_m"], dtype=np.float64)
        R, _ = cv2.Rodrigues(rvec)
        z_axis = R[:, 2].astype(np.float64)
        z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-12)
        if float(np.dot(z_axis, tvec)) < 0.0:
            z_axis = -z_axis
        return z_axis

    def _solve_for(
        self,
        det: dict,
        center_xy: Optional[np.ndarray] = None,
        include_center: bool = True,
        ippe_only: bool = False,
    ) -> Optional[dict]:
        obj = build_object_keypoints(det["class_id"])
        img = det["kpts_xy"].copy()
        if center_xy is not None:
            img[0] = center_xy
        valid = det["kpts_conf"] >= self.kp_conf
        valid[0] = bool(include_center)
        if valid.sum() < 4:
            return None
        return solve_pose(obj[valid], img[valid], self.K, self.dist, self.camera_model, ippe_only=ippe_only)


def solve_pose(object_points: np.ndarray, image_points: np.ndarray,
               camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
               camera_model: str = "pinhole",
               ippe_only: bool = False) -> Optional[dict]:
    obj = np.ascontiguousarray(object_points.astype(np.float32))
    img = np.ascontiguousarray(image_points.astype(np.float32))
    camera_model_l = camera_model.lower()
    is_fisheye = camera_model_l in {"opencv_fisheye", "fisheye"}
    is_planar = len(object_points) >= 4 and np.allclose(object_points[:, 2], object_points[0, 2])

    if is_fisheye or ippe_only:
        if not is_planar:
            return None
        methods = [cv2.SOLVEPNP_IPPE]
    else:
        methods = []
        if is_planar:
            methods.append(cv2.SOLVEPNP_IPPE)
        methods.extend([cv2.SOLVEPNP_ITERATIVE, cv2.SOLVEPNP_SQPNP])

    if is_fisheye:
        try:
            img_for_pnp = cv2.fisheye.undistortPoints(
                img.reshape(-1, 1, 2).astype(np.float64),
                camera_matrix.astype(np.float64),
                np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1),
                R=np.eye(3, dtype=np.float64),
                P=None,
            ).reshape(-1, 2).astype(np.float32)
        except cv2.error:
            return None
        pnp_camera_matrix = np.eye(3, dtype=np.float64)
        pnp_dist_coeffs = np.zeros(4, dtype=np.float64)
    else:
        img_for_pnp = img
        pnp_camera_matrix = camera_matrix
        pnp_dist_coeffs = dist_coeffs

    for method in methods:
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj,
                img_for_pnp,
                pnp_camera_matrix,
                pnp_dist_coeffs,
                flags=method,
            )
        except cv2.error:
            continue
        if not ok:
            continue

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        if is_fisheye:
            try:
                reproj, _ = cv2.fisheye.projectPoints(
                    obj.reshape(-1, 1, 3).astype(np.float64),
                    rvec,
                    tvec,
                    camera_matrix.astype(np.float64),
                    np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1),
                )
            except cv2.error:
                continue
        else:
            reproj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        reproj = reproj.reshape(-1, 2)
        reproj_err = float(np.sqrt(np.mean(np.sum((reproj - img) ** 2, axis=1))))

        return {
            "solver": int(method),
            "solver_name": PNP_METHOD_NAMES.get(int(method), f"METHOD_{int(method)}"),
            "camera_model": camera_model,
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
    camera_model: str,
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
            camera_model=camera_model,
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
    camera_model: str,
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
                camera_model=camera_model,
                center_source=center_source,
                kp_conf=kp_conf,
            )
            detections.append(detection)
            visual = draw_detection(visual, detection)

    payload = {
        "image_path": str(image_path.resolve()),
        "preproc": preproc,
        "camera_model": camera_model,
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
    ap.add_argument("--camera-model", choices=["auto", "pinhole", "opencv_fisheye", "fisheye"],
                    default="auto")
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
    camera_matrix, dist_coeffs, detected_camera_model = load_intrinsics_with_model(
        args.intrinsics,
        args.fx,
        args.fy,
        args.cx,
        args.cy,
        args.dist,
    )
    camera_model = detected_camera_model if args.camera_model == "auto" else args.camera_model
    model = load_model(args.yolo_weights, args.device, args.coin_weights)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    all_results = []

    for image_path in iter_images(args.source):
        payload, visual = run_on_image(
            model=model,
            image_path=image_path,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            camera_model=camera_model,
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
