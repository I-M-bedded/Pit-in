from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})

import os
import sys
import json
import random
import argparse
import numpy as np
from PIL import Image

import omni
import omni.kit.app
import omni.usd
import omni.replicator.core as rep

from pxr import UsdGeom, Gf, Semantics, UsdShade, Sdf


# =========================
# USER CONFIG
# =========================
USD_PATH    = r"C:/Users/JUN/Desktop/mydataset.usd"
OUTPUT_DIR  = r"C:/Users/JUN/Documents/Pit-in/vision/output_paired"
DISTILL_POSES_FILE = os.path.join(OUTPUT_DIR, "poses_distill.json")

CAMERA_PRIM    = "/World/robot/lcam_1/lcam"
PART_ROOT      = "/World/Niro_2"
ROBOT_PRIM     = "/World/robot"
ROBOT_MATERIAL = "/World/Looks/PreviewSurface"

# 9 front keypoint holes (camera-facing side, used for vis detection)
FRONT_HOLES = [
    {"prim": f"{PART_ROOT}/Hole1_B",                    "label": "kp_1"},
    {"prim": f"{PART_ROOT}/Hole2_B",                    "label": "kp_2"},
    {"prim": f"{PART_ROOT}/Hole3_B",                    "label": "kp_3"},
    {"prim": f"{PART_ROOT}/Hole4_B",                    "label": "kp_4"},
    {"prim": f"{PART_ROOT}/Hole5_B",                    "label": "kp_5"},
    {"prim": f"{PART_ROOT}/Hole6_B",                    "label": "kp_6"},
    {"prim": f"{PART_ROOT}/CenterHole_B/CenterHole_01", "label": "kp_7"},
    {"prim": f"{PART_ROOT}/CenterHole_B/CenterHole_02", "label": "kp_8"},
    {"prim": f"{PART_ROOT}/CenterHole_B/CenterHole_R",  "label": "kp_9"},
]

# Back holes (amodal GT — always unoccluded)
BACK_HOLES = [
    {"prim": f"{PART_ROOT}/Hole1",       "label": "kp_back_1"},
    {"prim": f"{PART_ROOT}/Hole2",       "label": "kp_back_2"},
    {"prim": f"{PART_ROOT}/Hole3",       "label": "kp_back_3"},
    {"prim": f"{PART_ROOT}/Hole4",       "label": "kp_back_4"},
    {"prim": f"{PART_ROOT}/Hole5",       "label": "kp_back_5"},
    {"prim": f"{PART_ROOT}/Hole6",       "label": "kp_back_6"},
    {"prim": f"{PART_ROOT}/CenterHole",  "label": "kp_back_center"},
]

ALL_FRONT_PRIMS = [h["prim"] for h in FRONT_HOLES]
ALL_BACK_PRIMS  = [h["prim"] for h in BACK_HOLES]

# Which front labels are grouped into the 'center hole' mask
CENTER_LABELS = {"kp_7", "kp_8", "kp_9"}

RESOLUTION           = (640, 480)
BASELINE_LIGHT_COUNT = 10
BASELINE_POSES_PER_LIGHT = 300
BASELINE_PART_POSES = BASELINE_POSES_PER_LIGHT // 2
BASELINE_ROBOT_POSES = BASELINE_POSES_PER_LIGHT - BASELINE_PART_POSES
BASELINE_BLOOM_BLOCKS = (
    (300, 900),
    (1200, 1500),
    (1800, 2100),
    (2700, 3000),
)
DISTILL_POSES_PER_RUN = 500
DISTILL_PART_POSES = DISTILL_POSES_PER_RUN // 2
DISTILL_ROBOT_POSES = DISTILL_POSES_PER_RUN - DISTILL_PART_POSES
RANDOM_SEED          = 7

# Render synchronization tuning.
# RGB pass can be expensive when bloom / linear lighting is enabled, so keep its
# subframes moderate and make mask passes much cheaper.
RGB_RT_SUBFRAMES     = 8
MASK_RT_SUBFRAMES    = 1
PRE_RENDER_UPDATES   = 2
POST_RENDER_UPDATES  = 2
FRAME_DRAIN_INTERVAL = 8
FRAME_DRAIN_UPDATES  = 3

# Vis thresholds (ratio = occluded_pixels / clean_pixels)
VIS_FULL_THRESH     = 0.95   # >= 95% visible → vis=2
VIS_OCCLUDED_THRESH = 0.05   # <= 5%  visible → vis=0
                             # between        → vis=1

# Per-mode capture config: RGB filename + meta keys (for occluded modes)
MODE_CONFIG = {
    "baseline": {
        "image":           "image_baseline.png",
        "hole_pixels_key": "hole_pixels_baseline",
        "visibility_key":  "visibility_baseline",
    },
    # Distill 3-phase pipeline (총 9000장)
    # 순서: distill_clean → (씬 조명 유지) distill_bright → (씬 어둡게) distill_dark
    "distill_clean": {
        "image": "image_clean.png",
    },
    "distill_occ": {
        "image":           "image_occ.png",
        "hole_pixels_key": "hole_pixels_occ",
        "visibility_key":  "visibility_occ",
    },
    "distill_illum": {
        "image":           "image_illum.png",
        "hole_pixels_key": "hole_pixels_illum",
        "visibility_key":  "visibility_illum",
    },
}

PART_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.2, 0.2),
    "tz": (0.4, 1.1),
    "rz": (-45.0, 45.0),
}

ROBOT_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.1, 0.1),
    "tz": (0.2, 0.4),
    "rz": (-45.0, 45.0),
}


def _parse_args():
    ap = argparse.ArgumentParser(description="Synthetic dataset capture")
    ap.add_argument(
        "mode",
        nargs="?",
        default="baseline",
        choices=[
            "baseline",
            "distill_clean",
            "distill_occ",
            "distill_illum",
        ],
    )
    ap.add_argument("-s", "--start", type=int, default=None, help="Start frame index for this run")
    ap.add_argument("-p", "--frames", type=int, default=None, help="Number of frames to capture in this run")
    return ap.parse_args()


_ARGS = _parse_args()
MODE = _ARGS.mode
START_OVERRIDE = _ARGS.start
FRAME_OVERRIDE = _ARGS.frames

# Global: current robot pose to re-apply after physics updates.
# Physics on the articulated robot overrides xformOps, so we restore it manually.
_active_robot_pose = None


# =========================
# UTILITY
# =========================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def _pose_file_for_mode(mode=None):
    m = mode if mode is not None else MODE
    return DISTILL_POSES_FILE

def _frame_dir_name(frame_idx, mode=None):
    m = mode if mode is not None else MODE
    stem = f"{frame_idx:06d}"
    if m == "baseline":
        for start, end in BASELINE_BLOOM_BLOCKS:
            if start <= frame_idx < end:
                return f"{stem}_b"
    return stem

def get_frame_dir(frame_idx, mode=None):
    """Per-mode frame directory: OUTPUT_DIR/<mode>/<frame_idx>/"""
    m = mode if mode is not None else MODE
    return os.path.join(OUTPUT_DIR, m, _frame_dir_name(frame_idx, mode))

def get_stage():
    return omni.usd.get_context().get_stage()

def get_prim(path):
    return get_stage().GetPrimAtPath(path)

def prim_exists(path):
    return get_prim(path).IsValid()

def save_rgb_png(rgb, path):
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    if rgb.ndim == 3 and rgb.shape[2] == 4:
        rgb = rgb[:, :, :3]
    Image.fromarray(rgb).save(path)


def save_mask_png(mask, path):
    Image.fromarray(mask.astype(np.uint8)).save(path)

def set_visibility(path, visible):
    prim = get_prim(path)
    if not prim.IsValid():
        print(f"[WARN] prim missing: {path}")
        return
    imageable = UsdGeom.Imageable(prim)
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()

def set_holes_visibility(front=False, back=False):
    for p in ALL_FRONT_PRIMS:
        set_visibility(p, front)
    for p in ALL_BACK_PRIMS:
        set_visibility(p, back)

def get_annotator_data(anno):
    data = anno.get_data()
    if isinstance(data, dict):
        return data["data"], data.get("info", {})
    return data, {}

def _pump_app_updates(count):
    app = omni.kit.app.get_app()
    for _ in range(count):
        simulation_app.update()
        app.update()

def flush_and_render(mask_pass=False):
    _pump_app_updates(PRE_RENDER_UPDATES)
    # Re-apply robot pose after physics updates
    if _active_robot_pose is not None:
        _set_robot_xform(_active_robot_pose)
    subframes = MASK_RT_SUBFRAMES if mask_pass else RGB_RT_SUBFRAMES
    rep.orchestrator.step(delta_time=0.0, rt_subframes=1, wait_for_render=True)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=subframes, wait_for_render=True)
    _pump_app_updates(POST_RENDER_UPDATES)

def reapply_material(prim_path, material_path):
    prim = get_prim(prim_path)
    mat  = get_prim(material_path)
    if prim.IsValid() and mat.IsValid():
        material = UsdShade.Material(mat)
        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(material, UsdShade.Tokens.strongerThanDescendants)

# =========================
# SEMANTICS (per-hole unique labels)
# =========================
def add_semantic_recursive(prim_path, label):
    prim = get_prim(prim_path)
    if not prim.IsValid():
        print(f"[WARN] semantic prim missing: {prim_path}")
        return

    def apply(p):
        if p.GetTypeName() == "Mesh":
            api = Semantics.SemanticsAPI.Apply(p, "Semantics")
            api.CreateSemanticTypeAttr().Set("class")
            api.CreateSemanticDataAttr().Set(label)
        for c in p.GetChildren():
            apply(c)
    apply(prim)

def setup_semantics():
    print("[INFO] Setting per-hole semantic labels...")
    for h in FRONT_HOLES + BACK_HOLES:
        add_semantic_recursive(h["prim"], h["label"])


# =========================
# POSE BLOCK GENERATION
# =========================
def _generate_coverage_poses(n, ranges, seed):
    rng = random.Random(seed)
    if n <= 0:
        return []
    tz_lo, tz_hi = ranges["tz"]
    if n == 1:
        tz_values = [(tz_lo + tz_hi) * 0.5]
    else:
        bin_edges = np.linspace(tz_lo, tz_hi, n + 1).tolist()
        tz_values = [
            rng.uniform(bin_edges[i], bin_edges[i + 1])
            for i in range(n)
        ]
        rng.shuffle(tz_values)

    poses = []
    for tz in tz_values:
        poses.append((
            rng.uniform(*ranges["tx"]),
            rng.uniform(*ranges["ty"]),
            float(tz),
            rng.uniform(*ranges["rz"]),
        ))
    return poses

def generate_baseline_pose_block(seed):
    # Baseline uses a fresh pose block each run; no pose file is reused.
    part_poses = _generate_coverage_poses(BASELINE_PART_POSES, PART_POSE_RANGES, seed)
    robot_poses = _generate_coverage_poses(BASELINE_ROBOT_POSES, ROBOT_POSE_RANGES, seed + 1)
    return part_poses, robot_poses


def generate_distill_pose_block(seed):
    # One pose block reused across distill_clean / distill_occ / distill_illum.
    part_poses = _generate_coverage_poses(DISTILL_PART_POSES, PART_POSE_RANGES, seed)
    robot_poses = _generate_coverage_poses(DISTILL_ROBOT_POSES, ROBOT_POSE_RANGES, seed + 1)
    return part_poses, robot_poses


def save_poses(part_poses, robot_poses, pose_file=None):
    pose_file = pose_file or _pose_file_for_mode()
    data = {
        "seed":        RANDOM_SEED,
        "num_part":    len(part_poses),
        "num_robot":   len(robot_poses),
        "part_poses":  [list(p) for p in part_poses],
        "robot_poses": [list(p) for p in robot_poses],
    }
    with open(pose_file, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[INFO] Poses saved to {pose_file}")

def load_poses(pose_file=None):
    pose_file = pose_file or _pose_file_for_mode()
    with open(pose_file, "r") as f:
        data = json.load(f)
    part  = [tuple(p) for p in data["part_poses"]]
    robot = [tuple(p) for p in data["robot_poses"]]
    print(f"[INFO] Poses loaded from {pose_file} ({len(part)} part, {len(robot)} robot)")
    return part, robot


def _get_total_frames():
    # Baseline reuses one 300-pose block across 5 lighting conditions.
    if MODE == "baseline":
        return BASELINE_LIGHT_COUNT * BASELINE_POSES_PER_LIGHT
    # Distill modes share one 500-pose block across clean / occ / illum.
    if MODE.startswith("distill_"):
        return DISTILL_POSES_PER_RUN
    raise ValueError(f"Unsupported mode: {MODE}")

def _get_clean_mode_for(mode):
    """occluded 모드가 참조할 clean 모드 결정"""
    return "distill_clean"

def find_start_index():
    """Find the first frame index that is missing its capture image."""
    img_name = MODE_CONFIG[MODE]["image"]
    total = _get_total_frames()
    for i in range(total):
        if not os.path.exists(os.path.join(get_frame_dir(i), img_name)):
            return i
    return total


# =========================
# POSE APPLICATION
# =========================
def apply_part_pose(pose):
    tx, ty, tz, rz = pose
    prim = get_prim(PART_ROOT)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)

    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(tx, ty, tz))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))

    r_attr = prim.GetAttribute("xformOp:rotateXYZ")
    if r_attr.IsValid():
        r_attr.Set(Gf.Vec3f(0.0, 0.0, rz))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rz))

def reset_part_pose():
    apply_part_pose((0.0, 0.0, 0.7, 0.0))

def _set_robot_xform(pose):
    tx, ty, tz, rz = pose
    prim = get_prim(ROBOT_PRIM)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)

    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(tx, ty, tz))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))

    # rz (degrees) → quaternion around Z
    quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), rz).GetQuat()

    o_attr = prim.GetAttribute("xformOp:orient")
    if o_attr.IsValid():
        o_attr.Set(quat)
    else:
        xform.AddOrientOp().Set(quat)

def apply_robot_pose(pose):
    global _active_robot_pose
    _active_robot_pose = pose
    _set_robot_xform(pose)

def reset_robot_pose():
    global _active_robot_pose
    _active_robot_pose = None
    prim = get_prim(ROBOT_PRIM)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)

    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    quat_identity = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
    o_attr = prim.GetAttribute("xformOp:orient")
    if o_attr.IsValid():
        o_attr.Set(quat_identity)
    else:
        xform.AddOrientOp().Set(quat_identity)


# =========================
# REPLICATOR SETUP
# =========================
def _ensure_api_schema(prim, schema_name):
    existing = prim.GetMetadata("apiSchemas")
    if isinstance(existing, Sdf.TokenListOp):
        tokens = set(existing.prependedItems or [])
        tokens.update(existing.appendedItems or [])
        tokens.update(existing.explicitItems or [])
        if schema_name in tokens:
            return
        tokens.add(schema_name)
        prim.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(tokens))
        return
    if existing:
        tokens = list(existing)
        if schema_name in tokens:
            return
        tokens.append(schema_name)
        prim.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(set(tokens)))
        return
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.Create({schema_name}))


def _set_render_product_attr(prim, attr_name, type_name, value):
    attr = prim.GetAttribute(attr_name)
    if not attr.IsValid():
        attr = prim.CreateAttribute(attr_name, type_name, custom=False)
    attr.Set(value)


def setup_replicator():
    rp  = rep.create.render_product(CAMERA_PRIM, RESOLUTION)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    sem = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
    rgb.attach(rp)
    sem.attach(rp)
    return rgb, sem

def build_binary_mask_from_sem(sem_map, id_to_labels, wanted_class):
    target_ids = []
    wanted_lower = wanted_class.lower()

    for k, v in id_to_labels.items():
        sid = int(k)
        if isinstance(v, dict):
            cls = v.get("class", None)
            if isinstance(cls, str):
                if wanted_lower in [c.strip().lower() for c in cls.split(",")]:
                    target_ids.append(sid)
                continue
            if isinstance(cls, list) and wanted_lower in [str(c).lower() for c in cls]:
                target_ids.append(sid)
                continue
            if isinstance(cls, dict) and wanted_lower in [str(c).lower() for c in cls]:
                target_ids.append(sid)
                continue
        if isinstance(v, str):
            if wanted_lower in [c.strip().lower() for c in v.split(",")]:
                target_ids.append(sid)

    return (np.isin(sem_map, target_ids).astype(np.uint8) * 255)


# =========================
# RENDER PASS HELPERS
# =========================
def _render_rgb(rgb_anno, out_path, robot_on):
    """Hide all holes, (optionally) show robot, render and save RGB."""
    set_holes_visibility(front=False, back=False)
    set_visibility(ROBOT_PRIM, robot_on)
    if robot_on:
        reapply_material(ROBOT_PRIM, ROBOT_MATERIAL)
    flush_and_render(mask_pass=False)
    rgb_data, _ = get_annotator_data(rgb_anno)
    save_rgb_png(rgb_data, out_path)


def _render_front_mask(sem_anno, robot_on):
    """Show front holes only (robot optionally ON for occlusion)."""
    set_holes_visibility(front=True, back=False)
    set_visibility(ROBOT_PRIM, robot_on)
    flush_and_render(mask_pass=True)
    sem_map, info = get_annotator_data(sem_anno)
    return sem_map, info.get("idToLabels", {})

def _render_back_mask(sem_anno, robot_on=False):
    """Show back holes only (robot optionally ON for occlusion check)."""
    set_holes_visibility(front=False, back=True)
    set_visibility(ROBOT_PRIM, robot_on)
    flush_and_render(mask_pass=True)
    sem_map, info = get_annotator_data(sem_anno)
    return sem_map, info.get("idToLabels", {})


def _mask_bbox(mask):
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]


def _mask_touches_border(mask):
    if mask.size == 0:
        return False
    return bool(
        np.any(mask[0, :] > 0)
        or np.any(mask[-1, :] > 0)
        or np.any(mask[:, 0] > 0)
        or np.any(mask[:, -1] > 0)
    )


def _build_target_metrics(
    clean_sem_map,
    clean_labels,
    visible_sem_map,
    visible_labels,
    targets,
):
    metrics = {}
    visibility = {}
    visible_pixels = {}

    for target in targets:
        label = target["label"]
        clean_mask = build_binary_mask_from_sem(clean_sem_map, clean_labels, label)
        visible_mask = build_binary_mask_from_sem(visible_sem_map, visible_labels, label)

        clean_px = int(np.sum(clean_mask > 0))
        visible_px = int(np.sum(visible_mask > 0))
        clean_touches_border = _mask_touches_border(clean_mask)
        visible_ratio = (visible_px / clean_px) if clean_px > 0 else 0.0
        is_truncated = clean_px == 0 or clean_touches_border
        is_occluded = (not is_truncated) and (visible_ratio < VIS_FULL_THRESH)

        if is_truncated:
            vis_label = 0
        elif visible_ratio < VIS_FULL_THRESH:
            # Any meaningful occlusion should remain inferable target supervision.
            vis_label = 1
        else:
            vis_label = 2

        metrics[label] = {
            "clean_pixels": clean_px,
            "visible_pixels": visible_px,
            "visible_ratio": visible_ratio,
            "clean_bbox_xyxy": _mask_bbox(clean_mask),
            "visible_bbox_xyxy": _mask_bbox(visible_mask),
            "clean_touches_border": clean_touches_border,
            "is_truncated": is_truncated,
            "is_occluded": is_occluded,
            "in_frame": not is_truncated,
            "in_frame_ratio": 0.0 if is_truncated else 1.0,
            "vis_label": vis_label,
        }
        visibility[label] = vis_label
        visible_pixels[label] = visible_px

    return metrics, visibility, visible_pixels


def _serialize_clean_meta(frame_idx, front_metrics, back_metrics):
    return {
        "frame_idx": frame_idx,
        "front_clean_pixels": {label: metrics["clean_pixels"] for label, metrics in front_metrics.items()},
        "back_clean_pixels": {label: metrics["clean_pixels"] for label, metrics in back_metrics.items()},
        "target_metrics_clean_front": front_metrics,
        "target_metrics_clean_back": back_metrics,
    }


def _load_clean_target_metrics(frame_idx, mode):
    clean_mode = _get_clean_mode_for(mode)
    clean_meta_path = os.path.join(get_frame_dir(frame_idx, clean_mode), "meta.json")
    with open(clean_meta_path, "r", encoding="utf-8") as f:
        clean_meta = json.load(f)
    return (
        clean_meta.get("target_metrics_clean_front", {}),
        clean_meta.get("target_metrics_clean_back", {}),
    )


def _build_metrics_from_clean(clean_metrics, visible_sem_map, visible_labels):
    metrics = {}
    visibility = {}
    visible_pixels = {}

    for label, clean_info in clean_metrics.items():
        visible_mask = build_binary_mask_from_sem(visible_sem_map, visible_labels, label)
        visible_px = int(np.sum(visible_mask > 0))
        clean_px = int(clean_info.get("clean_pixels", 0))
        visible_ratio = (visible_px / clean_px) if clean_px > 0 else 0.0
        is_truncated = bool(clean_info.get("is_truncated", clean_px == 0))
        is_occluded = (not is_truncated) and (visible_ratio < VIS_FULL_THRESH)

        if is_truncated:
            vis_label = 0
        elif visible_ratio < VIS_FULL_THRESH:
            vis_label = 1
        else:
            vis_label = 2

        metrics[label] = {
            "clean_pixels": clean_px,
            "visible_pixels": visible_px,
            "visible_ratio": visible_ratio,
            "clean_bbox_xyxy": clean_info.get("clean_bbox_xyxy"),
            "visible_bbox_xyxy": _mask_bbox(visible_mask),
            "clean_touches_border": bool(clean_info.get("clean_touches_border", False)),
            "is_truncated": is_truncated,
            "is_occluded": is_occluded,
            "in_frame": not is_truncated,
            "in_frame_ratio": 0.0 if is_truncated else 1.0,
            "vis_label": vis_label,
        }
        visibility[label] = vis_label
        visible_pixels[label] = visible_px

    return metrics, visibility, visible_pixels


# =========================
# CAPTURE: CLEAN MODE
# =========================
def capture_clean(frame_idx, rgb_anno, sem_anno, mode_override=None):
    """Bright light, robot OFF → RGB + GT masks + per-hole pixel counts."""
    # distill_clean stores the canonical per-target clean meta used by later distill modes.
    mode = mode_override or "distill_clean"
    frame_dir = get_frame_dir(frame_idx, mode)
    ensure_dir(frame_dir)

    # --- RGB (robot off) ---
    _render_rgb(
        rgb_anno,
        os.path.join(frame_dir, MODE_CONFIG[mode]["image"]),
        robot_on=False,
    )

    # --- FRONT MASK (robot off → unoccluded); also record per-hole pixel counts ---
    sem_map, labels = _render_front_mask(sem_anno, robot_on=False)

    hole_pixels   = {}
    mask_hole_b   = np.zeros(sem_map.shape[:2], dtype=np.uint8)
    mask_center_b = np.zeros(sem_map.shape[:2], dtype=np.uint8)

    for h in FRONT_HOLES:
        mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        hole_pixels[h["label"]] = int(np.sum(mask > 0))
        if h["label"] in CENTER_LABELS:
            mask_center_b = np.maximum(mask_center_b, mask)
        else:
            mask_hole_b = np.maximum(mask_hole_b, mask)

    save_mask_png(mask_hole_b,   os.path.join(frame_dir, "mask_Hole_B.png"))
    save_mask_png(mask_center_b, os.path.join(frame_dir, "mask_CenterHole_B.png"))

    # --- BACK MASK (amodal GT) ---
    sem_map2, labels2 = _render_back_mask(sem_anno)

    mask_hole   = np.zeros(sem_map2.shape[:2], dtype=np.uint8)
    mask_center = np.zeros(sem_map2.shape[:2], dtype=np.uint8)

    for h in BACK_HOLES:
        mask = build_binary_mask_from_sem(sem_map2, labels2, h["label"])
        if h["label"] == "kp_back_center":
            mask_center = np.maximum(mask_center, mask)
        else:
            mask_hole = np.maximum(mask_hole, mask)

    save_mask_png(mask_hole,   os.path.join(frame_dir, "mask_Hole.png"))
    save_mask_png(mask_center, os.path.join(frame_dir, "mask_CenterHole.png"))

    front_metrics, _, _ = _build_target_metrics(
        clean_sem_map=sem_map,
        clean_labels=labels,
        visible_sem_map=sem_map,
        visible_labels=labels,
        targets=FRONT_HOLES,
    )
    back_metrics, _, _ = _build_target_metrics(
        clean_sem_map=sem_map2,
        clean_labels=labels2,
        visible_sem_map=sem_map2,
        visible_labels=labels2,
        targets=BACK_HOLES,
    )

    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(_serialize_clean_meta(frame_idx, front_metrics, back_metrics), f, indent=2, ensure_ascii=False)

    set_holes_visibility(front=True, back=True)


# =========================
# CAPTURE: DISTILL OCC / ILLUM
# =========================
def capture_occluded(frame_idx, mode, rgb_anno, sem_anno):
    """
    Robot ON → RGB + per-hole vis labels via comparison with clean pixel counts.
    Shared between distill_occ and distill_illum using the same clean target meta.
    Returns the per-label visibility dict.
    """
    # Clean target metrics from distill_clean are the only visibility reference.
    cfg = MODE_CONFIG[mode]
    frame_dir = get_frame_dir(frame_idx, mode)
    ensure_dir(frame_dir)

    # --- RGB (robot on) ---
    _render_rgb(
        rgb_anno,
        os.path.join(frame_dir, cfg["image"]),
        robot_on=True,
    )

    sem_map_front, labels_front = _render_front_mask(sem_anno, robot_on=True)
    sem_map_back, labels_back = _render_back_mask(sem_anno, robot_on=True)
    clean_front_metrics, clean_back_metrics = _load_clean_target_metrics(frame_idx, mode)
    front_metrics, front_visibility, front_pixels_visible = _build_metrics_from_clean(
        clean_front_metrics, sem_map_front, labels_front
    )
    back_metrics, back_visibility, back_pixels_visible = _build_metrics_from_clean(
        clean_back_metrics, sem_map_back, labels_back
    )

    meta = {
        "frame_idx": frame_idx,
        cfg["hole_pixels_key"]: front_pixels_visible,
        cfg["visibility_key"]: front_visibility,
        f"{cfg['hole_pixels_key']}_back": back_pixels_visible,
        f"{cfg['visibility_key']}_back": back_visibility,
        f"target_metrics_{mode}_front": front_metrics,
        f"target_metrics_{mode}_back": back_metrics,
    }
    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    set_holes_visibility(front=True, back=True)
    return front_visibility


# =========================
# CAPTURE: BASELINE MODE
# =========================
def capture_baseline(frame_idx, rgb_anno, sem_anno):
    """
    Single-run baseline pair (user controls lighting in USD):
      1) Robot OFF → front/back GT masks + per-hole clean pixel counts
      2) Robot ON  → RGB image + per-hole vis labels
    All outputs written into one frame directory + one meta.json.
    """
    # Baseline writes front/back clean and robot-on target metrics in one folder.
    cfg = MODE_CONFIG["baseline"]
    frame_dir = get_frame_dir(frame_idx, "baseline")
    ensure_dir(frame_dir)

    # ---- Phase A: GT masks (robot off) ----
    sem_map, labels = _render_front_mask(sem_anno, robot_on=False)

    hole_pixels   = {}
    mask_hole_b   = np.zeros(sem_map.shape[:2], dtype=np.uint8)
    mask_center_b = np.zeros(sem_map.shape[:2], dtype=np.uint8)

    for h in FRONT_HOLES:
        mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        hole_pixels[h["label"]] = int(np.sum(mask > 0))
        if h["label"] in CENTER_LABELS:
            mask_center_b = np.maximum(mask_center_b, mask)
        else:
            mask_hole_b = np.maximum(mask_hole_b, mask)

    save_mask_png(mask_hole_b,   os.path.join(frame_dir, "mask_Hole_B.png"))
    save_mask_png(mask_center_b, os.path.join(frame_dir, "mask_CenterHole_B.png"))

    sem_map2, labels2 = _render_back_mask(sem_anno, robot_on=False)

    mask_hole   = np.zeros(sem_map2.shape[:2], dtype=np.uint8)
    mask_center = np.zeros(sem_map2.shape[:2], dtype=np.uint8)

    for h in BACK_HOLES:
        mask = build_binary_mask_from_sem(sem_map2, labels2, h["label"])
        if h["label"] == "kp_back_center":
            mask_center = np.maximum(mask_center, mask)
        else:
            mask_hole = np.maximum(mask_hole, mask)

    save_mask_png(mask_hole,   os.path.join(frame_dir, "mask_Hole.png"))
    save_mask_png(mask_center, os.path.join(frame_dir, "mask_CenterHole.png"))

    # ---- Phase B: RGB with robot on + visibility ----
    _render_rgb(
        rgb_anno,
        os.path.join(frame_dir, cfg["image"]),
        robot_on=True,
    )
    sem_map3, labels3 = _render_front_mask(sem_anno, robot_on=True)
    sem_map4, labels4 = _render_back_mask(sem_anno, robot_on=True)

    front_metrics, front_visibility, hole_pixels_occ = _build_target_metrics(
        clean_sem_map=sem_map,
        clean_labels=labels,
        visible_sem_map=sem_map3,
        visible_labels=labels3,
        targets=FRONT_HOLES,
    )
    back_metrics, back_visibility, back_pixels_occ = _build_target_metrics(
        clean_sem_map=sem_map2,
        clean_labels=labels2,
        visible_sem_map=sem_map4,
        visible_labels=labels4,
        targets=BACK_HOLES,
    )

    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump({
            "frame_idx":            frame_idx,
            "hole_pixels_clean":    hole_pixels,
            cfg["hole_pixels_key"]: hole_pixels_occ,
            cfg["visibility_key"]:  front_visibility,
            "back_pixels_clean":    {h["label"]: back_metrics[h["label"]]["clean_pixels"] for h in BACK_HOLES},
            "back_pixels_baseline": back_pixels_occ,
            "visibility_baseline_back": back_visibility,
            "target_metrics_baseline_front": front_metrics,
            "target_metrics_baseline_back": back_metrics,
        }, f, indent=2, ensure_ascii=False)

    set_holes_visibility(front=True, back=True)
    return front_visibility



# =========================
# CAMERA INSPECTION (read-only)
# =========================
def log_camera_intrinsics():
    """Read USD camera parameters as-is (no modification) and print resulting FOV."""
    prim = get_prim(CAMERA_PRIM)
    if not prim.IsValid():
        print(f"[WARN] Camera prim missing: {CAMERA_PRIM}")
        return

    cam = UsdGeom.Camera(prim)

    def _get(attr):
        v = attr.Get() if attr else None
        return float(v) if v is not None else None

    fl = _get(cam.GetFocalLengthAttr())
    ha = _get(cam.GetHorizontalApertureAttr())
    va = _get(cam.GetVerticalApertureAttr())

    if None in (fl, ha, va):
        print(f"[WARN] USD camera missing one of fl/ha/va  fl={fl} ha={ha} va={va}")
        return

    import math
    hfov = 2.0 * math.degrees(math.atan((ha / 2.0) / fl))
    vfov = 2.0 * math.degrees(math.atan((va / 2.0) / fl))
    print(f"[INFO] USD camera @ render {RESOLUTION[0]}x{RESOLUTION[1]}: "
          f"fl={fl:.2f}mm, ha={ha:.3f}mm, va={va:.3f}mm, "
          f"HFOV={hfov:.1f}°, VFOV={vfov:.1f}°, aperture AR={ha/va:.3f}")


# =========================
# FRAME SEQUENCING
# =========================
def apply_pose_for_frame(frame_idx, part_poses, robot_poses):
    """
    Pose scheduling by mode.
    - baseline: 300-frame light block, first half part motion and second half robot motion
    - distill_*: 500-frame triplet block, first half part motion and second half robot motion
    Returns True when the current frame is in the robot-motion half.
    """
    part_count = len(part_poses)
    robot_count = len(robot_poses)

    if MODE == "baseline":
        local_idx = frame_idx % BASELINE_POSES_PER_LIGHT
        if local_idx < part_count:
            reset_robot_pose()
            apply_part_pose(part_poses[local_idx])
            return False
        reset_part_pose()
        apply_robot_pose(robot_poses[local_idx - part_count])
        return True

    if MODE.startswith("distill_"):
        if frame_idx < part_count:
            reset_robot_pose()
            apply_part_pose(part_poses[frame_idx])
            return False
        reset_part_pose()
        apply_robot_pose(robot_poses[frame_idx - part_count])
        return True

    if frame_idx < part_count:
        reset_robot_pose()
        apply_part_pose(part_poses[frame_idx])
        return False
    reset_part_pose()
    apply_robot_pose(robot_poses[frame_idx - part_count])
    return True


# =========================
# MAIN
# =========================
def main():
    print(f"[INFO] ===== MODE: {MODE} =====")
    ensure_dir(OUTPUT_DIR)

    omni.usd.get_context().open_stage(USD_PATH)

    for _ in range(120):
        simulation_app.update()

    if not prim_exists(CAMERA_PRIM):
        raise RuntimeError(f"Camera prim not found: {CAMERA_PRIM}")

    log_camera_intrinsics()

    for _ in range(60):
        simulation_app.update()

    setup_semantics()

    rep.orchestrator.set_capture_on_play(False)
    rgb_anno, sem_anno = setup_replicator()

    # --- Pose generation / loading ---
    # baseline: generate a fresh 300-pose block every run and do not save it
    # distill_*: reuse poses_distill.json across clean / occ / illum
    if MODE == "baseline":
        seed = RANDOM_SEED + (START_OVERRIDE or 0)
        all_part_poses, all_robot_poses = generate_baseline_pose_block(seed)
    else:
        pose_file = _pose_file_for_mode(MODE)
        if os.path.exists(pose_file):
            all_part_poses, all_robot_poses = load_poses(pose_file)
        else:
            if MODE != "distill_clean":
                raise RuntimeError(
                    f"poses file not found at {pose_file}. "
                    f"Run distill_clean first to generate the shared distill pose block."
                )
            all_part_poses, all_robot_poses = generate_distill_pose_block(RANDOM_SEED)
            save_poses(all_part_poses, all_robot_poses, pose_file)

    # --- Auto-detect batch ---
    total_frames = _get_total_frames()

    if START_OVERRIDE is not None:
        start_idx = START_OVERRIDE
        print(f"[INFO] Start index manually set to {start_idx}")
    else:
        start_idx = find_start_index()

    if start_idx >= total_frames:
        print(f"[INFO] All {total_frames} frames already captured for {MODE} mode. Nothing to do.")
        return

    default_run_frames = (
        BASELINE_POSES_PER_LIGHT
        if MODE == "baseline"
        else DISTILL_POSES_PER_RUN
    )
    run_frames = FRAME_OVERRIDE if FRAME_OVERRIDE is not None else default_run_frames
    end_idx = min(start_idx + run_frames, total_frames)
    print(f"[INFO] Resuming from frame {start_idx} → {end_idx - 1}  ({end_idx - start_idx} frames this run)")

    vis_stats = {0: 0, 1: 0, 2: 0}
    phase2_announced = False

    for frame_idx in range(start_idx, end_idx):

        # Apply poses every frame to prevent physics drift
        is_phase2 = apply_pose_for_frame(frame_idx, all_part_poses, all_robot_poses)

        if not is_phase2 and frame_idx == start_idx:
            if MODE == "baseline":
                print(
                    f"[INFO] Baseline block pose coverage: "
                    f"{BASELINE_LIGHT_COUNT} light blocks x {BASELINE_POSES_PER_LIGHT} frames "
                    f"({BASELINE_PART_POSES} part + {BASELINE_ROBOT_POSES} robot per block)"
                )
            else:
                print(
                    f"[INFO] Distill triplet pose coverage: "
                    f"{DISTILL_POSES_PER_RUN} frames "
                    f"({DISTILL_PART_POSES} part + {DISTILL_ROBOT_POSES} robot)"
                )
        if is_phase2 and not phase2_announced:
            phase2_announced = True
            if MODE == "baseline":
                print(f"[INFO] Baseline robot-pose half begins within each {BASELINE_POSES_PER_LIGHT}-frame light block")
            else:
                print(f"[INFO] Distill robot-pose half begins within the {DISTILL_POSES_PER_RUN}-frame triplet block")

        # Let physics settle, then force robot pose back (physics may override xformOps)
        for _ in range(24):
            simulation_app.update()
        if _active_robot_pose is not None:
            _set_robot_xform(_active_robot_pose)

        if MODE == "distill_clean":
            capture_clean(frame_idx, rgb_anno, sem_anno,
                          mode_override=MODE)
        elif MODE == "baseline":
            visibility = capture_baseline(frame_idx, rgb_anno, sem_anno)
            for v in visibility.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1
        elif MODE in ("distill_occ", "distill_illum"):
            visibility = capture_occluded(frame_idx, MODE, rgb_anno, sem_anno)
            for v in visibility.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1
        else:
            raise ValueError(f"Unsupported mode: {MODE}")

        if ((frame_idx - start_idx + 1) % FRAME_DRAIN_INTERVAL) == 0:
            _pump_app_updates(FRAME_DRAIN_UPDATES)

        print(f"[{MODE}] {frame_idx + 1}/{total_frames}")

    # --- Post-run summary ---
    captured  = end_idx - start_idx
    remaining = total_frames - end_idx

    NEXT_MODE = {
        "distill_clean": "distill_occ",
        "distill_occ": "distill_illum",
    }
    if MODE == "distill_clean":
        if find_start_index() >= total_frames:
            print(f"[INFO] All {MODE} frames done!")
            if MODE in NEXT_MODE:
                print(f"[INFO] >> 다음 단계: python make_synthetic_dataset.py {NEXT_MODE[MODE]}")
        else:
            print(f"[INFO] Done. {captured} frames captured. {remaining} remaining — restart & run again.")
    else:
        total_labels = sum(vis_stats.values())
        if total_labels > 0:
            print(f"\n[INFO] ===== Occlusion Statistics [{MODE}] (this run) =====")
            print(f"  vis=2 (visible):   {vis_stats[2]:6d} ({vis_stats[2]/total_labels*100:.1f}%)")
            print(f"  vis=1 (partial):   {vis_stats[1]:6d} ({vis_stats[1]/total_labels*100:.1f}%)")
            print(f"  vis=0 (occluded):  {vis_stats[0]:6d} ({vis_stats[0]/total_labels*100:.1f}%)")
        if remaining > 0:
            print(f"[INFO] Done. {captured} frames captured. {remaining} remaining — restart & run again.")
        else:
            print(f"[INFO] All {MODE} frames complete.")
            if MODE in NEXT_MODE:
                print(f"[INFO] >> 다음 단계: python make_synthetic_dataset.py {NEXT_MODE[MODE]}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("[ERROR]", e)
    finally:
        simulation_app.close()
