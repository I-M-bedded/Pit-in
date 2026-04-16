from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})

import os
import sys
import json
import random
import numpy as np
from PIL import Image

import omni
import omni.usd
import omni.replicator.core as rep

from pxr import UsdGeom, Gf, Semantics, UsdShade


# =========================
# USER CONFIG
# =========================
USD_PATH    = r"C:/Users/JUN/Desktop/mydataset.usd"
OUTPUT_DIR  = r"C:/Users/JUN/Documents/synthetic_dataset/output_paired"
POSES_FILE  = os.path.join(OUTPUT_DIR, "poses.json")

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
NUM_PART_POSES       = 1500      # Phase 1 frame count
NUM_ROBOT_POSES      = 1500      # Phase 2 frame count
NUM_BRIGHT_OCC_POSES = 3000      # bright_occ reuses part+robot pose sequence
FRAMES_PER_RUN       = 3000
RANDOM_SEED          = 7

# Vis thresholds (ratio = occluded_pixels / clean_pixels)
VIS_FULL_THRESH     = 0.95   # >= 95% visible → vis=2
VIS_OCCLUDED_THRESH = 0.05   # <= 5%  visible → vis=0
                             # between        → vis=1

# Per-mode capture config: RGB filename + meta keys (for occluded modes)
MODE_CONFIG = {
    "clean": {
        "image": "image_clean.png",
    },
    "aug": {
        "image":           "image_aug.png",
        "hole_pixels_key": "hole_pixels_aug",
        "visibility_key":  "visibility",
    },
    "bright_occ": {
        "image":           "image_bright_occ.png",
        "hole_pixels_key": "hole_pixels_bright_occ",
        "visibility_key":  "visibility_bright_occ",
    },
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
    "distill_bright": {
        "image":           "image_bright.png",
        "hole_pixels_key": "hole_pixels_bright",
        "visibility_key":  "visibility_bright",
    },
    "distill_dark": {
        "image":           "image_dark.png",
        "hole_pixels_key": "hole_pixels_dark",
        "visibility_key":  "visibility_dark",
    },
}

PART_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.2, 0.2),
    "tz": (0.4, 1.0),
    "rz": (-40.0, 40.0),
}

ROBOT_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.2, 0.2),
    "tz": (0.1, 0.4),
    "rz": (-40.0, 40.0),
}

# ---- MODE & START OVERRIDE ----
# Usage: python make_synthetic_dataset.py clean          (auto-detect start)
#        python make_synthetic_dataset.py aug 1500       (force start from frame 1500)
MODE = "clean"
START_OVERRIDE = None
for arg in sys.argv[1:]:
    if arg in MODE_CONFIG:
        MODE = arg
    elif arg.isdigit():
        START_OVERRIDE = int(arg)

# Global: current robot pose to re-apply after physics updates.
# Physics on the articulated robot overrides xformOps, so we restore it manually.
_active_robot_pose = None


# =========================
# UTILITY
# =========================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def get_frame_dir(frame_idx, mode=None):
    """Per-mode frame directory: OUTPUT_DIR/<mode>/<frame_idx>/"""
    m = mode if mode is not None else MODE
    return os.path.join(OUTPUT_DIR, m, f"{frame_idx:06d}")

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

def flush_and_render():
    for _ in range(2):
        simulation_app.update()
    # Re-apply robot pose after physics updates
    if _active_robot_pose is not None:
        _set_robot_xform(_active_robot_pose)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=1,  wait_for_render=True)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=24, wait_for_render=True)

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
# RANDOM POSE GENERATION & SAVE/LOAD
# =========================
def generate_random_part_poses(n, seed):
    rng = random.Random(seed)
    r = PART_POSE_RANGES
    return [
        (rng.uniform(*r["tx"]), rng.uniform(*r["ty"]), rng.uniform(*r["tz"]),
         rng.uniform(*r["rz"]))
        for _ in range(n)
    ]

def generate_random_robot_poses(n, seed):
    rng = random.Random(seed)
    r = ROBOT_POSE_RANGES
    return [
        (rng.uniform(*r["tx"]), rng.uniform(*r["ty"]), rng.uniform(*r["tz"]),
         rng.uniform(*r["rz"]))
        for _ in range(n)
    ]

def save_poses(part_poses, robot_poses):
    data = {
        "seed":        RANDOM_SEED,
        "num_part":    len(part_poses),
        "num_robot":   len(robot_poses),
        "part_poses":  [list(p) for p in part_poses],
        "robot_poses": [list(p) for p in robot_poses],
    }
    with open(POSES_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[INFO] Poses saved to {POSES_FILE}")

def load_poses():
    with open(POSES_FILE, "r") as f:
        data = json.load(f)
    part  = [tuple(p) for p in data["part_poses"]]
    robot = [tuple(p) for p in data["robot_poses"]]
    print(f"[INFO] Poses loaded from {POSES_FILE} ({len(part)} part, {len(robot)} robot)")
    return part, robot


def _get_total_frames():
    if MODE == "bright_occ":
        return NUM_BRIGHT_OCC_POSES
    return NUM_PART_POSES + NUM_ROBOT_POSES

def _get_clean_mode_for(mode):
    """occluded 모드가 참조할 clean 모드 결정"""
    if mode.startswith("distill_"):
        return "distill_clean"
    return "clean"

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
    flush_and_render()
    rgb_data, _ = get_annotator_data(rgb_anno)
    save_rgb_png(rgb_data, out_path)

def _render_front_mask(sem_anno, robot_on):
    """Show front holes only (robot optionally ON for occlusion)."""
    set_holes_visibility(front=True, back=False)
    set_visibility(ROBOT_PRIM, robot_on)
    flush_and_render()
    sem_map, info = get_annotator_data(sem_anno)
    return sem_map, info.get("idToLabels", {})

def _render_back_mask(sem_anno):
    """Show back holes only (robot OFF, amodal GT)."""
    set_holes_visibility(front=False, back=True)
    set_visibility(ROBOT_PRIM, False)
    flush_and_render()
    sem_map, info = get_annotator_data(sem_anno)
    return sem_map, info.get("idToLabels", {})

def _classify_visibility(occluded_px, clean_px):
    if clean_px == 0:
        return 0
    ratio = occluded_px / clean_px
    if ratio <= VIS_OCCLUDED_THRESH:
        return 0
    if ratio >= VIS_FULL_THRESH:
        return 2
    return 1


# =========================
# CAPTURE: CLEAN MODE
# =========================
def capture_clean(frame_idx, rgb_anno, sem_anno, mode_override=None):
    """Bright light, robot OFF → RGB + GT masks + per-hole pixel counts."""
    mode = mode_override or "clean"
    frame_dir = get_frame_dir(frame_idx, mode)
    ensure_dir(frame_dir)

    # --- RGB (robot off) ---
    _render_rgb(rgb_anno, os.path.join(frame_dir, MODE_CONFIG[mode]["image"]), robot_on=False)

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

    # --- META: per-hole clean pixel counts (consumed by occluded-mode vis computation) ---
    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump({
            "frame_idx":         frame_idx,
            "hole_pixels_clean": hole_pixels,
        }, f, indent=2, ensure_ascii=False)

    # Restore
    set_holes_visibility(front=True, back=True)


# =========================
# CAPTURE: OCCLUDED MODES (aug, bright_occ)
# =========================
def capture_occluded(frame_idx, mode, rgb_anno, sem_anno):
    """
    Robot ON → RGB + per-hole vis labels via comparison with clean pixel counts.
    Shared between `aug` (dark light) and `bright_occ` (bright light).
    Returns the per-label visibility dict.
    """
    cfg = MODE_CONFIG[mode]
    frame_dir = get_frame_dir(frame_idx, mode)
    ensure_dir(frame_dir)

    # --- RGB (robot on) ---
    _render_rgb(rgb_anno, os.path.join(frame_dir, cfg["image"]), robot_on=True)

    # --- FRONT MASK with robot ON (partial occlusion) ---
    sem_map, labels = _render_front_mask(sem_anno, robot_on=True)

    # Load clean per-hole pixel counts (cross-mode read from clean folder)
    clean_mode = _get_clean_mode_for(mode)
    clean_meta_path = os.path.join(get_frame_dir(frame_idx, clean_mode), "meta.json")
    with open(clean_meta_path, "r") as f:
        clean_meta = json.load(f)
    clean_pixels = clean_meta["hole_pixels_clean"]

    hole_pixels_occ = {}
    visibility      = {}

    for h in FRONT_HOLES:
        occ_mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        occ_px   = int(np.sum(occ_mask > 0))
        clean_px = clean_pixels.get(h["label"], 0)
        hole_pixels_occ[h["label"]] = occ_px
        visibility[h["label"]]      = _classify_visibility(occ_px, clean_px)

    # Write meta in own mode folder
    meta = {
        "frame_idx":            frame_idx,
        cfg["hole_pixels_key"]: hole_pixels_occ,
        cfg["visibility_key"]:  visibility,
    }
    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # Restore
    set_holes_visibility(front=True, back=True)
    return visibility


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

    # ---- Phase B: RGB with robot on + visibility ----
    _render_rgb(rgb_anno, os.path.join(frame_dir, cfg["image"]), robot_on=True)
    sem_map3, labels3 = _render_front_mask(sem_anno, robot_on=True)

    hole_pixels_occ = {}
    visibility      = {}
    for h in FRONT_HOLES:
        occ_mask = build_binary_mask_from_sem(sem_map3, labels3, h["label"])
        occ_px   = int(np.sum(occ_mask > 0))
        clean_px = hole_pixels.get(h["label"], 0)
        hole_pixels_occ[h["label"]] = occ_px
        visibility[h["label"]]      = _classify_visibility(occ_px, clean_px)

    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump({
            "frame_idx":            frame_idx,
            "hole_pixels_clean":    hole_pixels,
            cfg["hole_pixels_key"]: hole_pixels_occ,
            cfg["visibility_key"]:  visibility,
        }, f, indent=2, ensure_ascii=False)

    set_holes_visibility(front=True, back=True)
    return visibility



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
    Two-phase sequence shared by all modes:
      Phase 1 (frame_idx <  NUM_PART_POSES) : vary part pose, robot at identity
      Phase 2 (frame_idx >= NUM_PART_POSES) : reset part, vary robot pose
    Returns True if we are in Phase 2 for this frame.
    """
    if frame_idx < NUM_PART_POSES:
        reset_robot_pose()
        apply_part_pose(part_poses[frame_idx])
        return False
    reset_part_pose()
    apply_robot_pose(robot_poses[frame_idx - NUM_PART_POSES])
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
    POSE_GENERATORS = {"clean", "baseline", "distill_clean"}
    if os.path.exists(POSES_FILE):
        all_part_poses, all_robot_poses = load_poses()
    else:
        if MODE not in POSE_GENERATORS:
            raise RuntimeError(
                f"poses.json not found at {POSES_FILE}. "
                f"Run clean, baseline, or distill_clean mode first to generate poses."
            )
        all_part_poses  = generate_random_part_poses(NUM_PART_POSES, RANDOM_SEED)
        all_robot_poses = generate_random_robot_poses(NUM_ROBOT_POSES, RANDOM_SEED + 1)
        save_poses(all_part_poses, all_robot_poses)

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

    end_idx = min(start_idx + FRAMES_PER_RUN, total_frames)
    print(f"[INFO] Resuming from frame {start_idx} → {end_idx - 1}  ({end_idx - start_idx} frames this run)")

    vis_stats = {0: 0, 1: 0, 2: 0}
    phase2_announced = False

    for frame_idx in range(start_idx, end_idx):

        # Apply poses every frame to prevent physics drift
        is_phase2 = apply_pose_for_frame(frame_idx, all_part_poses, all_robot_poses)

        if not is_phase2 and frame_idx == start_idx:
            print(f"[INFO] Phase 1: Part Pose (frames 0~{NUM_PART_POSES - 1})")
        if is_phase2 and not phase2_announced:
            phase2_announced = True
            print(f"[INFO] Phase 2: Robot Pose (frames {NUM_PART_POSES}~{total_frames - 1})")

        # bright_occ: keep robot visible during Phase 1 physics settle as well
        if MODE == "bright_occ" and not is_phase2:
            set_visibility(ROBOT_PRIM, True)

        # Let physics settle, then force robot pose back (physics may override xformOps)
        for _ in range(24):
            simulation_app.update()
        if _active_robot_pose is not None:
            _set_robot_xform(_active_robot_pose)

        if MODE in ("clean", "distill_clean"):
            capture_clean(frame_idx, rgb_anno, sem_anno,
                          mode_override=MODE if MODE != "clean" else None)
        elif MODE == "baseline":
            visibility = capture_baseline(frame_idx, rgb_anno, sem_anno)
            for v in visibility.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1
        elif MODE in ("distill_bright", "distill_dark"):
            visibility = capture_occluded(frame_idx, MODE, rgb_anno, sem_anno)
            for v in visibility.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1
        else:
            visibility = capture_occluded(frame_idx, MODE, rgb_anno, sem_anno)
            for v in visibility.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1

        print(f"[{MODE}] {frame_idx + 1}/{total_frames}")

    # --- Post-run summary ---
    captured  = end_idx - start_idx
    remaining = total_frames - end_idx

    NEXT_MODE = {
        "clean":          "aug",
        "distill_clean":  "distill_bright  (씬 조명 유지, 로봇 ON)",
        "distill_bright": "distill_dark    (씬 조명 어둡게 변경 후)",
    }
    if MODE in ("clean", "distill_clean"):
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
