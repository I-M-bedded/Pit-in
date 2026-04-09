"""
두 모델 비교 추론:
  1) TD-DRP adapter (Phase1-A ExLPose) + yolo26n-pose-best
  2) Plain yolo26n-pose-best (foundation only)

test/images_clean + test/images_aug 각각 10장씩 추론 → result/td_drp_v2/inference_compare/ 저장
"""

import os, sys, glob
import numpy as np
import cv2
import torch
from PIL import Image
from torchvision.transforms import ToTensor

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from ultralytics import YOLO
from vision.yolo.td_drp import TDDRPEndToEndPose

# ── paths ──
FOUNDATION = os.path.abspath("vision/yolo/weights/yolo26n-pose_best.pt")
P1A_WEIGHTS = os.path.abspath("result/td_drp_v2/td_drp_phase1_A.pt")
DATASET_ROOT = os.path.abspath("vision/dataset/yolo_pose_dataset")
SAVE_DIR = os.path.abspath("result/td_drp_v2/inference_compare")
IMG_SIZE = 640
N_IMAGES = 10

# 9-keypoint skeleton (커스텀 데이터셋)
SKELETON = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,6),(6,7),(7,8)]
KPT_COLORS = [
    (255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),
    (0,255,255),(128,0,255),(255,128,0),(128,255,0),
]

TO_TENSOR = ToTensor()


def letterbox(img_pil, size=640):
    img_pil = img_pil.convert("RGB")
    ow, oh = img_pil.size
    scale = min(size / ow, size / oh)
    nw, nh = max(1, int(round(ow * scale))), max(1, int(round(oh * scale)))
    img_resized = img_pil.resize((nw, nh), Image.BILINEAR)
    canvas = Image.new("RGB", (size, size), (0, 0, 0))
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas.paste(img_resized, (px, py))
    return canvas, scale, px, py


def load_tddrp_model(device):
    """Phase1-A adapter + foundation YOLO"""
    yolo = YOLO(FOUNDATION)
    model = TDDRPEndToEndPose(yolo).to(device)

    ADAPTER_PREFIXES = ("illumination_encoder.", "soft_router.", "multi_scale_rsv.", "criterion.")
    p1_state = torch.load(P1A_WEIGHTS, map_location=device)
    model_state = model.state_dict()
    loaded = {k: v for k, v in p1_state.items() if k.startswith(ADAPTER_PREFIXES)}
    model_state.update(loaded)
    model.load_state_dict(model_state)
    model.eval()
    print(f"[TD-DRP] Loaded {len(loaded)} adapter keys from Phase1-A")
    return model


def load_plain_yolo():
    """Plain yolo26n-pose-best"""
    model = YOLO(FOUNDATION)
    return model


def draw_pose(img_np, boxes, kpts_list, confs, title=""):
    """boxes: list of (x1,y1,x2,y2), kpts_list: list of (9,3), confs: list of float"""
    vis = img_np.copy()
    for box, kpts, conf in zip(boxes, kpts_list, confs):
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, f"{conf:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        for ki, (kx, ky, kv) in enumerate(kpts):
            if kv > 0.3:
                c = KPT_COLORS[ki % len(KPT_COLORS)]
                cv2.circle(vis, (int(kx), int(ky)), 4, c, -1)
        for (a, b) in SKELETON:
            if kpts[a][2] > 0.3 and kpts[b][2] > 0.3:
                pt1 = (int(kpts[a][0]), int(kpts[a][1]))
                pt2 = (int(kpts[b][0]), int(kpts[b][1]))
                cv2.line(vis, pt1, pt2, (200, 200, 200), 1)
    if title:
        cv2.putText(vis, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return vis


@torch.no_grad()
def infer_tddrp(model, img_pil, device):
    """Returns boxes(N,4), kpts(N,9,3), confs(N,)  in letterboxed 640 coords"""
    lb_img, scale, px, py = letterbox(img_pil, IMG_SIZE)
    tensor = TO_TENSOR(lb_img).unsqueeze(0).to(device)
    out = model(tensor)
    # x_aug is the YOLO head output
    preds = out["x_aug"]

    # preds shape depends on YOLO head: could be list or tensor
    if isinstance(preds, (list, tuple)):
        preds = preds[0]  # first element

    # preds: [batch, num_dets, dim] or [batch, dim, num_dets]
    if preds.dim() == 3:
        # YOLO v11 head output: [1, 36, 8400] (bbox4 + cls1 + kpt9*3 = 32 → nope)
        # Actually need to check the shape
        b, d1, d2 = preds.shape
        if d1 < d2:
            preds = preds.permute(0, 2, 1)  # → [1, 8400, d1]
        preds = preds[0]  # [num_dets, dim]

    # Parse: first 4 = box (cx,cy,w,h), then 1 = conf, then 9*3 = keypoints
    n_kpt = 9
    expected_dim = 4 + 1 + n_kpt * 3  # = 32
    if preds.shape[-1] < expected_dim:
        print(f"  [warn] pred dim {preds.shape[-1]} < expected {expected_dim}")

    cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
    conf = preds[:, 4] if preds.shape[-1] > 4 else torch.zeros(preds.shape[0])

    # Filter by confidence
    mask = conf > 0.25
    if mask.sum() == 0:
        # fallback: top 5
        topk = min(5, preds.shape[0])
        _, idx = conf.topk(topk)
        mask = torch.zeros(preds.shape[0], dtype=torch.bool)
        mask[idx] = True

    preds_f = preds[mask].cpu().numpy()
    boxes, confs_out, kpts_out = [], [], []
    for det in preds_f:
        cx_, cy_, w_, h_ = det[0], det[1], det[2], det[3]
        x1 = cx_ - w_ / 2
        y1 = cy_ - h_ / 2
        x2 = cx_ + w_ / 2
        y2 = cy_ + h_ / 2
        boxes.append([x1, y1, x2, y2])
        confs_out.append(det[4] if len(det) > 4 else 0.0)
        if len(det) >= expected_dim:
            kp = det[5:5 + n_kpt * 3].reshape(n_kpt, 3)
        else:
            kp = np.zeros((n_kpt, 3))
        kpts_out.append(kp)

    return boxes, kpts_out, confs_out


def infer_plain_yolo(model, img_pil):
    """Returns boxes(N,4), kpts(N,9,3), confs(N,) in letterboxed 640 coords"""
    # letterbox 적용 후 추론 → TD-DRP와 동일한 좌표계
    lb_img, _, _, _ = letterbox(img_pil, IMG_SIZE)
    lb_np = np.array(lb_img)
    results = model.predict(lb_np, imgsz=IMG_SIZE, conf=0.25, verbose=False)
    boxes, kpts_out, confs_out = [], [], []
    if results and results[0].boxes is not None:
        r = results[0]
        for i in range(len(r.boxes)):
            b = r.boxes.xyxy[i].cpu().numpy()
            boxes.append(b.tolist())
            confs_out.append(float(r.boxes.conf[i]))
            if r.keypoints is not None and i < len(r.keypoints):
                kp = r.keypoints[i].data[0].cpu().numpy()  # (9,3)
                kpts_out.append(kp)
            else:
                kpts_out.append(np.zeros((9, 3)))
    return boxes, kpts_out, confs_out


def run():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    tddrp_model = load_tddrp_model(device)
    yolo_model = load_plain_yolo()

    subsets = [
        ("clean", os.path.join(DATASET_ROOT, "test/images_clean")),
        ("aug", os.path.join(DATASET_ROOT, "test/images_aug")),
    ]

    for subset_name, img_dir in subsets:
        save_sub = os.path.join(SAVE_DIR, subset_name)
        os.makedirs(save_sub, exist_ok=True)

        img_files = sorted(glob.glob(os.path.join(img_dir, "*.png")))[:N_IMAGES]
        if not img_files:
            img_files = sorted(glob.glob(os.path.join(img_dir, "*.jpg")))[:N_IMAGES]
        print(f"\n[{subset_name}] {len(img_files)} images from {img_dir}")

        for img_path in img_files:
            fname = os.path.splitext(os.path.basename(img_path))[0]
            img_pil = Image.open(img_path).convert("RGB")

            # ── letterbox for display ──
            lb_img, _, _, _ = letterbox(img_pil, IMG_SIZE)
            lb_np = np.array(lb_img)

            # ── TD-DRP inference ──
            boxes_td, kpts_td, confs_td = infer_tddrp(tddrp_model, img_pil, device)
            vis_td = draw_pose(lb_np, boxes_td, kpts_td, confs_td, "TD-DRP (ExLPose adapter)")

            # ── Plain YOLO inference ──
            boxes_yl, kpts_yl, confs_yl = infer_plain_yolo(yolo_model, img_pil)
            # plain yolo results are in original image coords → need letterbox version for display
            lb_np2 = lb_np.copy()
            vis_yl = draw_pose(lb_np2, boxes_yl, kpts_yl, confs_yl, "Plain YOLO26n-pose")

            # ── side-by-side ──
            combined = np.hstack([
                cv2.cvtColor(lb_np, cv2.COLOR_RGB2BGR),      # original
                cv2.cvtColor(vis_td, cv2.COLOR_RGB2BGR),      # TD-DRP
                cv2.cvtColor(vis_yl, cv2.COLOR_RGB2BGR),      # Plain
            ])
            # labels
            h = 30
            header = np.zeros((h, combined.shape[1], 3), dtype=np.uint8)
            for ci, label in enumerate(["Original", "TD-DRP + Adapter", "Plain YOLO"]):
                cv2.putText(header, label, (ci * IMG_SIZE + 10, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            final = np.vstack([header, combined])

            out_path = os.path.join(save_sub, f"{fname}.png")
            cv2.imwrite(out_path, final)
            n_td = len(boxes_td)
            n_yl = len(boxes_yl)
            print(f"  {fname}: TD-DRP={n_td} det, YOLO={n_yl} det → {out_path}")

    print(f"\nDone. Results in {SAVE_DIR}/")


if __name__ == "__main__":
    run()
