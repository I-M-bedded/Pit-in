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

from pxr import UsdGeom, Gf, Semantics


# =========================
# USER CONFIG
# =========================
USD_PATH = r"C:/Users/JUN/Desktop/mydataset.usd"
OUTPUT_DIR = r"C:/Users/JUN/Documents/synthetic_dataset/output_paired"
POSES_FILE = os.path.join(OUTPUT_DIR, "poses.json")

CAMERA_PRIM = "/World/robot/lcam_1/lcam"
PART_ROOT = "/World/Niro_2"
ROBOT_PRIM = "/World/robot"
LIGHT_PRIM = "/Environment/warehouse/RectLigth"

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

# Which front labels are "center" (for grouped mask saving)
CENTER_LABELS = {"kp_7", "kp_8", "kp_9"}

RESOLUTION = (1280, 720)
NUM_PART_POSES  = 2500      # total across all batches
NUM_ROBOT_POSES = 2500
NUM_BRIGHT_OCC_POSES = 5000  # bright_occ: all poses with robot ON
FRAMES_PER_RUN  = 5000       # 250 part + 250 robot per run
NUM_BATCHES     = (NUM_PART_POSES + NUM_ROBOT_POSES) // FRAMES_PER_RUN  # 10
RANDOM_SEED = 42

# Vis thresholds (ratio = aug_pixels / clean_pixels)
VIS_FULL_THRESH    = 0.95   # >= 95% visible  → viss=2
VIS_OCCLUDED_THRESH = 0.05  # <= 5% visible   → vis=0
                             # between          → vis=1

# ---- MODE & START OVERRIDE ----
# Usage: python make_dataset.py clean         (auto-detect start)
#        python make_dataset.py aug 1500      (force start from frame 1500)
MODE = "clean"
START_OVERRIDE = None
for arg in sys.argv[1:]:
    if arg in ("clean", "aug", "bright_occ"):
        MODE = arg
    elif arg.isdigit():
        START_OVERRIDE = int(arg)

BRIGHT_INTENSITY = 20000.0
DARK_INTENSITY = 200.0

# Global: current robot pose to re-apply after physics updates
_active_robot_pose = None

PART_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.2, 0.2),
    "tz": (1.0, 1.3),
    "rx": (-10.0, 10.0),
    "ry": (-10.0, 10.0),
    "rz": (-30.0, 30.0),
}

ROBOT_POSE_RANGES = {
    "tx": (-0.2, 0.2),
    "ty": (-0.2, 0.2),
    "tz": (0.1, 0.6),
    "rz": (-30.0, 30.0),
}


# =========================
# UTILITY
# =========================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

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

def get_annotator_data(anno):
    data = anno.get_data()
    if isinstance(data, dict):
        return data["data"], data.get("info", {})
    return data, {}

def flush_and_render():
    for _ in range(2):
        simulation_app.update()
    # Re-apply robot pose after physics updates (physics overrides xformOps on articulated prims)
    if _active_robot_pose is not None:
        _set_robot_xform(_active_robot_pose)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=1, wait_for_render=True)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=24, wait_for_render=True)

def reapply_material(prim_path, material_path):
    from pxr import UsdShade
    st = get_stage()
    prim = st.GetPrimAtPath(prim_path)
    mat = st.GetPrimAtPath(material_path)
    if prim.IsValid() and mat.IsValid():
        material = UsdShade.Material(mat)
        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(material, UsdShade.Tokens.strongerThanDescendants)


# =========================
# LIGHTING
# =========================
def set_light_intensity(intensity):
    prim = get_prim(LIGHT_PRIM)
    if not prim.IsValid():
        print(f"[WARN] Light prim not found: {LIGHT_PRIM}")
        return
    attr = prim.GetAttribute("inputs:intensity")
    if attr.IsValid():
        attr.Set(intensity)
        print(f"[INFO] Light intensity -> {intensity}")
    else:
        print(f"[WARN] inputs:intensity not found on {LIGHT_PRIM}")


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
    for h in FRONT_HOLES:
        add_semantic_recursive(h["prim"], h["label"])
    for h in BACK_HOLES:
        add_semantic_recursive(h["prim"], h["label"])


# =========================
# RANDOM POSE GENERATION & SAVE/LOAD
# =========================
def generate_random_part_poses(n, seed):
    rng = random.Random(seed)
    r = PART_POSE_RANGES
    return [
        (rng.uniform(*r["tx"]), rng.uniform(*r["ty"]), rng.uniform(*r["tz"]),
         rng.uniform(*r["rx"]), rng.uniform(*r["ry"]), rng.uniform(*r["rz"]))
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
        "seed": RANDOM_SEED,
        "num_part": len(part_poses),
        "num_robot": len(robot_poses),
        "part_poses": [list(p) for p in part_poses],
        "robot_poses": [list(p) for p in robot_poses],
    }
    with open(POSES_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[INFO] Poses saved to {POSES_FILE}")

def load_poses():
    with open(POSES_FILE, "r") as f:
        data = json.load(f)
    part = [tuple(p) for p in data["part_poses"]]
    robot = [tuple(p) for p in data["robot_poses"]]
    print(f"[INFO] Poses loaded from {POSES_FILE} ({len(part)} part, {len(robot)} robot)")
    return part, robot


def find_start_index():
    """Find the first frame index that is missing its capture image."""
    img_map = {"clean": "image_clean.png", "aug": "image_aug.png", "bright_occ": "image_bright_occ.png"}
    img_name = img_map[MODE]
    total = NUM_BRIGHT_OCC_POSES if MODE == "bright_occ" else NUM_PART_POSES + NUM_ROBOT_POSES
    for i in range(total):
        if not os.path.exists(os.path.join(OUTPUT_DIR, f"{i:06d}", img_name)):
            return i
    return total  # all done


# =========================
# POSE APPLICATION
# =========================
def apply_part_pose(pose):
    tx, ty, tz, rx, ry, rz = pose
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
        r_attr.Set(Gf.Vec3f(rx, ry, rz))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))

def reset_part_pose():
    prim = get_prim(PART_ROOT)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)

    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(0.0, 0.0, 1.0))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))

    r_attr = prim.GetAttribute("xformOp:rotateXYZ")
    if r_attr.IsValid():
        r_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

def _set_robot_xform(pose):
    tx, ty, tz, rz = pose
    prim = get_prim(ROBOT_PRIM)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)

    # 위치(Translate) 적용
    t_attr = prim.GetAttribute("xformOp:translate")
    if t_attr.IsValid():
        t_attr.Set(Gf.Vec3d(tx, ty, tz))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))

    # 회전(Rotate) 적용: rz(Degree)를 Quaternion으로 변환하여 Orient 연산에 적용
    # Gf.Rotation은 (회전 축, 각도(도 단위))를 인자로 받습니다.
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

    # 쿼터니언 초기화 (회전 없음: W=1.0, X=0, Y=0, Z=0)
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
    rp = rep.create.render_product(CAMERA_PRIM, RESOLUTION)

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
# CAPTURE: CLEAN MODE
# =========================
def capture_clean(frame_idx, rgb_anno, sem_anno):
    """Bright light, robot OFF → RGB + GT masks + per-hole pixel counts"""
    frame_dir = os.path.join(OUTPUT_DIR, f"{frame_idx:06d}")
    ensure_dir(frame_dir)

    # --- RGB: holes hidden, robot hidden ---
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, False)

    flush_and_render()

    rgb_data, _ = get_annotator_data(rgb_anno)
    save_rgb_png(rgb_data, os.path.join(frame_dir, "image_clean.png"))

    # --- FRONT MASK (robot off → unoccluded, per-hole semantic) ---
    for p in ALL_FRONT_PRIMS:
        set_visibility(p, True)
    for p in ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, False)

    flush_and_render()

    sem_map, info = get_annotator_data(sem_anno)
    labels = info.get("idToLabels", {})

    hole_pixels = {}
    mask_hole_b = np.zeros(sem_map.shape[:2], dtype=np.uint8)
    mask_center_b = np.zeros(sem_map.shape[:2], dtype=np.uint8)

    for h in FRONT_HOLES:
        mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        hole_pixels[h["label"]] = int(np.sum(mask > 0))
        if h["label"] in CENTER_LABELS:
            mask_center_b = np.maximum(mask_center_b, mask)
        else:
            mask_hole_b = np.maximum(mask_hole_b, mask)

    save_mask_png(mask_hole_b, os.path.join(frame_dir, "mask_Hole_B.png"))
    save_mask_png(mask_center_b, os.path.join(frame_dir, "mask_CenterHole_B.png"))

    # --- BACK MASK (robot off → amodal GT) ---
    for p in ALL_FRONT_PRIMS:
        set_visibility(p, False)
    for p in ALL_BACK_PRIMS:
        set_visibility(p, True)
    set_visibility(ROBOT_PRIM, False)

    flush_and_render()

    sem_map2, info2 = get_annotator_data(sem_anno)
    labels2 = info2.get("idToLabels", {})

    mask_hole = np.zeros(sem_map2.shape[:2], dtype=np.uint8)
    mask_center = np.zeros(sem_map2.shape[:2], dtype=np.uint8)

    for h in BACK_HOLES:
        mask = build_binary_mask_from_sem(sem_map2, labels2, h["label"])
        if h["label"] == "kp_back_center":
            mask_center = np.maximum(mask_center, mask)
        else:
            mask_hole = np.maximum(mask_hole, mask)

    save_mask_png(mask_hole, os.path.join(frame_dir, "mask_Hole.png"))
    save_mask_png(mask_center, os.path.join(frame_dir, "mask_CenterHole.png"))

    # --- META: save per-hole pixel counts for aug mode ---
    with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump({
            "frame_idx": frame_idx,
            "hole_pixels_clean": hole_pixels,
        }, f, indent=2, ensure_ascii=False)

    # Restore
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, True)


# =========================
# CAPTURE: AUG MODE
# =========================
def capture_aug(frame_idx, rgb_anno, sem_anno):
    """Dark light, robot ON → RGB + semantic mask → compare with clean → vis labels"""
    frame_dir = os.path.join(OUTPUT_DIR, f"{frame_idx:06d}")
    ensure_dir(frame_dir)

    # --- RGB: holes hidden, robot ON ---
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, True)
    reapply_material(ROBOT_PRIM, "/World/Looks/PreviewSurface")

    flush_and_render()

    rgb_data, _ = get_annotator_data(rgb_anno)
    save_rgb_png(rgb_data, os.path.join(frame_dir, "image_aug.png"))

    # --- FRONT MASK with robot ON (for occlusion detection) ---
    for p in ALL_FRONT_PRIMS:
        set_visibility(p, True)
    for p in ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, True)

    flush_and_render()

    sem_map, info = get_annotator_data(sem_anno)
    labels = info.get("idToLabels", {})

    # Load clean per-hole pixel counts
    meta_path = os.path.join(frame_dir, "meta.json")
    with open(meta_path, "r") as f:
        meta = json.load(f)
    clean_pixels = meta["hole_pixels_clean"]

    # Per-hole vis computation
    hole_pixels_aug = {}
    visibility = {}

    for h in FRONT_HOLES:
        aug_mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        aug_px = int(np.sum(aug_mask > 0))
        clean_px = clean_pixels.get(h["label"], 0)
        hole_pixels_aug[h["label"]] = aug_px

        if clean_px == 0:
            visibility[h["label"]] = 0
        else:
            ratio = aug_px / clean_px
            if ratio <= VIS_OCCLUDED_THRESH:
                visibility[h["label"]] = 0
            elif ratio >= VIS_FULL_THRESH:
                visibility[h["label"]] = 2
            else:
                visibility[h["label"]] = 1

    # Update meta with aug data
    meta["hole_pixels_aug"] = hole_pixels_aug
    meta["visibility"] = visibility
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # Restore
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, True)


# =========================
# CAPTURE: BRIGHT_OCC MODE
# =========================
def capture_bright_occ(frame_idx, rgb_anno, sem_anno):
    """Bright light, robot ON → RGB + vis labels (for 2-stage distillation)"""
    frame_dir = os.path.join(OUTPUT_DIR, f"{frame_idx:06d}")
    ensure_dir(frame_dir)

    # --- RGB: holes hidden, robot ON, bright ---
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, True)
    reapply_material(ROBOT_PRIM, "/World/Looks/PreviewSurface")

    flush_and_render()

    rgb_data, _ = get_annotator_data(rgb_anno)
    save_rgb_png(rgb_data, os.path.join(frame_dir, "image_bright_occ.png"))

    # --- FRONT MASK with robot ON (for occlusion detection) ---
    for p in ALL_FRONT_PRIMS:
        set_visibility(p, True)
    for p in ALL_BACK_PRIMS:
        set_visibility(p, False)
    set_visibility(ROBOT_PRIM, True)

    flush_and_render()

    sem_map, info = get_annotator_data(sem_anno)
    labels = info.get("idToLabels", {})

    # Load clean per-hole pixel counts
    meta_path = os.path.join(frame_dir, "meta.json")
    with open(meta_path, "r") as f:
        meta = json.load(f)
    clean_pixels = meta["hole_pixels_clean"]

    # Per-hole vis computation (same logic as aug)
    hole_pixels_bocc = {}
    visibility_bocc = {}

    for h in FRONT_HOLES:
        bocc_mask = build_binary_mask_from_sem(sem_map, labels, h["label"])
        bocc_px = int(np.sum(bocc_mask > 0))
        clean_px = clean_pixels.get(h["label"], 0)
        hole_pixels_bocc[h["label"]] = bocc_px

        if clean_px == 0:
            visibility_bocc[h["label"]] = 0
        else:
            ratio = bocc_px / clean_px
            if ratio <= VIS_OCCLUDED_THRESH:
                visibility_bocc[h["label"]] = 0
            elif ratio >= VIS_FULL_THRESH:
                visibility_bocc[h["label"]] = 2
            else:
                visibility_bocc[h["label"]] = 1

    # Update meta with bright_occ data
    meta["hole_pixels_bright_occ"] = hole_pixels_bocc
    meta["visibility_bright_occ"] = visibility_bocc
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # Restore
    for p in ALL_FRONT_PRIMS + ALL_BACK_PRIMS:
        set_visibility(p, True)


# =========================
# CAMERA INTRINSICS
# =========================
def setup_camera_intrinsics():
    prim = get_prim(CAMERA_PRIM)
    if not prim.IsValid():
        print(f"[WARN] Camera prim missing: {CAMERA_PRIM}")
        return

    cam = UsdGeom.Camera(prim)

    w, h = 1280.0, 720.0
    fx, fy = 908.4711303710938, 908.1490478515625
    cx, cy = 648.7000122070312, 369.1841735839844
    fl = 10.0

    cam.CreateFocalLengthAttr().Set(fl)
    ha = w * fl / fx
    va = h * fl / fy
    cam.CreateHorizontalApertureAttr().Set(ha)
    cam.CreateVerticalApertureAttr().Set(va)
    cam.CreateHorizontalApertureOffsetAttr().Set((cx - w / 2.0) / w * ha)
    cam.CreateVerticalApertureOffsetAttr().Set(-(cy - h / 2.0) / h * va)

    print(f"[INFO] Camera intrinsics applied to {CAMERA_PRIM}")


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

    setup_camera_intrinsics()

    # Set lighting
    if MODE in ("clean", "bright_occ"):
        set_light_intensity(BRIGHT_INTENSITY)
    else:
        set_light_intensity(DARK_INTENSITY)

    for _ in range(60):
        simulation_app.update()

    setup_semantics()

    rep.orchestrator.set_capture_on_play(False)
    rgb_anno, sem_anno = setup_replicator()

    # --- Pose generation / loading ---
    if os.path.exists(POSES_FILE):
        all_part_poses, all_robot_poses = load_poses()
    else:
        if MODE != "clean":
            raise RuntimeError(f"poses.json not found at {POSES_FILE}. Run clean mode first.")
        all_part_poses = generate_random_part_poses(NUM_PART_POSES, RANDOM_SEED)
        all_robot_poses = generate_random_robot_poses(NUM_ROBOT_POSES, RANDOM_SEED + 1)
        save_poses(all_part_poses, all_robot_poses)

    # --- Auto-detect batch ---
    total_frames = NUM_BRIGHT_OCC_POSES if MODE == "bright_occ" else NUM_PART_POSES + NUM_ROBOT_POSES

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
    in_phase2 = False

    for frame_idx in range(start_idx, end_idx):

        # --- Apply poses EVERY FRAME to prevent physics drift ---
        if MODE == "bright_occ":
            # bright_occ: reuse same pose sequence as clean/aug (part then robot)
            if frame_idx < NUM_PART_POSES:
                if not in_phase2 and frame_idx == start_idx:
                    print(f"[INFO] Phase 1: Part Pose (frames 0~{NUM_PART_POSES - 1})")
                reset_robot_pose()
                apply_part_pose(all_part_poses[frame_idx])
                # robot ON at default position for occlusion
                set_visibility(ROBOT_PRIM, True)
            else:
                if not in_phase2:
                    in_phase2 = True
                    print(f"[INFO] Phase 2: Robot Pose (frames {NUM_PART_POSES}~{total_frames - 1})")
                reset_part_pose()
                apply_robot_pose(all_robot_poses[frame_idx - NUM_PART_POSES])
        else:
            if frame_idx < NUM_PART_POSES:
                if not in_phase2 and frame_idx == start_idx:
                    print(f"[INFO] Phase 1: Part Pose (frames 0~{NUM_PART_POSES - 1})")
                reset_robot_pose()
                apply_part_pose(all_part_poses[frame_idx])
            else:
                if not in_phase2:
                    in_phase2 = True
                    print(f"[INFO] Phase 2: Robot Pose (frames {NUM_PART_POSES}~{total_frames - 1})")
                reset_part_pose()
                apply_robot_pose(all_robot_poses[frame_idx - NUM_PART_POSES])

        for _ in range(24):
            simulation_app.update()

        # Re-apply robot pose after physics settling
        if _active_robot_pose is not None:
            _set_robot_xform(_active_robot_pose)

        if MODE == "clean":
            capture_clean(frame_idx, rgb_anno, sem_anno)
        elif MODE == "bright_occ":
            capture_bright_occ(frame_idx, rgb_anno, sem_anno)
            meta_path = os.path.join(OUTPUT_DIR, f"{frame_idx:06d}", "meta.json")
            with open(meta_path) as f:
                vis = json.load(f).get("visibility_bright_occ", {})
            for v in vis.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1
        else:
            capture_aug(frame_idx, rgb_anno, sem_anno)
            meta_path = os.path.join(OUTPUT_DIR, f"{frame_idx:06d}", "meta.json")
            with open(meta_path) as f:
                vis = json.load(f).get("visibility", {})
            for v in vis.values():
                vis_stats[v] = vis_stats.get(v, 0) + 1

        print(f"[{MODE}] {frame_idx + 1}/{total_frames}")

    # --- Post-run ---
    captured = end_idx - start_idx
    if MODE == "clean":
        if find_start_index() >= total_frames:
            print("[INFO] All clean frames done!")
            print("[INFO] >> Restart simulator, then run: python make_dataset.py aug")
        else:
            remaining = total_frames - end_idx
            print(f"[INFO] Done. {captured} frames captured. {remaining} remaining — restart & run again.")
    else:
        if sum(vis_stats.values()) > 0:
            total_labels = sum(vis_stats.values())
            print(f"\n[INFO] ===== Occlusion Statistics [{MODE}] (this run) =====")
            print(f"  vis=2 (visible):   {vis_stats[2]:6d} ({vis_stats[2]/total_labels*100:.1f}%)")
            print(f"  vis=1 (partial):   {vis_stats[1]:6d} ({vis_stats[1]/total_labels*100:.1f}%)")
            print(f"  vis=0 (occluded):  {vis_stats[0]:6d} ({vis_stats[0]/total_labels*100:.1f}%)")
        remaining = total_frames - end_idx
        if remaining > 0:
            print(f"[INFO] Done. {captured} frames captured. {remaining} remaining — restart & run again.")
        else:
            print(f"[INFO] All {MODE} frames complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("[ERROR]", e)
    finally:
        simulation_app.close()
