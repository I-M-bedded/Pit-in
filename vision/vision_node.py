#!/usr/bin/env python
"""
Vision Node — ROS2 node for robust center-hole pose estimation.
AGX Orin tuned: TensorRT engine, FP16, CUDA warm-up.

Camera: Intel RealSense (pyrealsense2), color only — depth not used.
Output: /cam0 (geometry_msgs/Point) — camera-frame 3D position [m]

Pipeline per frame:
    1. RealSense → color frame
    2. HolePoseEstimator.estimate(color) → result (solvePnP tvec + confidence)
    3. WorldTracker: tvec → world-space EMA (via robot FK from /agv_joint_state)
    4. Back-project filtered world_xyz → camera frame
    5. Publish Point(x, y, z) on /cam0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs

REPO_ROOT = Path(__file__).resolve().parent.parent
_yolo_dir = str(REPO_ROOT / "vision" / "yolo")
if _yolo_dir not in sys.path:
    sys.path.insert(0, _yolo_dir)
_vision_dir = str(REPO_ROOT / "vision")
if _vision_dir not in sys.path:
    sys.path.insert(0, _vision_dir)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from hole_pose_estimator import HolePoseEstimator
from world_tracker import WorldTracker

try:
    from pitin_msgs.msg import JointState as PitinJointState
    _HAS_JOINT_MSG = True
except ImportError:
    _HAS_JOINT_MSG = False

try:
    sys.path.insert(0, str(REPO_ROOT / "src_robot"))
    from controller.ik import Topik
    _HAS_TOPIK = True
except ImportError:
    _HAS_TOPIK = False

# Camera mount rotation and offset per side (mirrored from macro_micro.py SIDE_CFG)
_CAM_MOUNT_R = {
    "left":  np.array([[ 0.0,  1.0, 0.0],   # rot_Z(-90): camera → pin frame
                        [-1.0,  0.0, 0.0],
                        [ 0.0,  0.0, 1.0]], dtype=np.float64),
    "right": np.array([[ 0.0, -1.0, 0.0],   # rot_Z(+90): camera → pin frame
                        [ 1.0,  0.0, 0.0],
                        [ 0.0,  0.0, 1.0]], dtype=np.float64),
}
_CAM_OFFSET = {           # camera optical center in pin frame [m]
    "left":  np.array([-0.057,  0.029, 0.0], dtype=np.float64),
    "right": np.array([ 0.057, -0.029, 0.0], dtype=np.float64),
}


def _export_trt_engine(weights_path: str, imgsz: int, half: bool) -> str:
    """Export .pt → TensorRT .engine and return the engine path.
    Skips export if the .engine file already exists.
    """
    from ultralytics import YOLO as _YOLO
    pt = Path(weights_path)
    engine = pt.with_suffix(".engine")
    if engine.exists():
        return str(engine)
    print(f"[TRT] Exporting {pt.name} → {engine.name} (half={half}, imgsz={imgsz}) ...")
    m = _YOLO(str(pt))
    out = m.export(format="engine", half=half, imgsz=imgsz, device=0)
    return str(out)


class VisionNode(Node):
    """RealSense (RGB only) → HolePoseEstimator → WorldTracker → /cam0 Point."""

    def __init__(
        self,
        weights_path: str,
        side: str = "left",
        device: str = "cuda:0",
        conf_threshold: float = 0.5,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        show_gui: bool = True,
        half: bool = False,
        hz: float = 15.0,
        robot_version: int = 4,
    ):
        super().__init__("vision_node")
        self.side = side
        self.show_gui = show_gui

        # -- RealSense: color only (depth removed, not used) --
        self.rs_pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        try:
            self.rs_pipeline.start(rs_config)
            self.get_logger().info("RealSense connected (color only).")
        except Exception as e:
            self.get_logger().error(f"RealSense init failed: {e}")
            self.rs_pipeline = None
            return

        # -- Camera intrinsics from color stream --
        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        intrin = color_frame.profile.as_video_stream_profile().intrinsics
        K = np.array([
            [intrin.fx, 0.0, intrin.ppx],
            [0.0, intrin.fy, intrin.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        D = np.array(intrin.coeffs[:5], dtype=np.float64)
        self.get_logger().info(
            f"Intrinsics: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
            f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}"
        )

        # -- YOLO estimator (warm-up happens inside __init__) --
        self.estimator = HolePoseEstimator(
            weights_path=weights_path,
            camera_matrix=K,
            dist_coeffs=D,
            device=device,
            conf_threshold=conf_threshold,
            half=half,
            imgsz=width,
        )

        # -- World-space EMA tracker --
        self.tracker = WorldTracker(ema_alpha=0.3, decay=0.9, min_confidence=0.1)

        # -- FK: Topik + /agv_joint_state subscriber --
        self._topik = None
        self._fk_ready = False
        if _HAS_TOPIK and _HAS_JOINT_MSG:
            self._topik = Topik(robot_version)
            self.create_subscription(
                PitinJointState, "agv_joint_state",
                self._joint_state_callback, 10,
            )
            self.get_logger().info("FK wired to /agv_joint_state.")
        else:
            self.get_logger().warn(
                "FK unavailable (pitin_msgs or ik not found) — "
                "WorldTracker runs in camera-space fallback mode."
            )

        # -- ROS2 publisher + timer --
        pub_topic = "/cam0" if side == "left" else "/cam1"
        self.pub_point = self.create_publisher(Point, pub_topic, 10)
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f"Publishing on {pub_topic} @ {hz:.0f} Hz")

    # ------------------------------------------------------------------
    #  FK subscriber
    # ------------------------------------------------------------------

    def _joint_state_callback(self, msg: PitinJointState):
        """Reconstruct q[7] from JointState counts and run FK."""
        # Motor index mapping (interface.py agv_joint_update):
        # msg.y_pos = cpos[0]=trY, msg.x_pos = cpos[1]=trX,
        # msg.yaw_pos = cpos[2]=rtZ, msg.lpin_rotate = cpos[3]=Lr,
        # msg.rpin_rotate = cpos[4]=Rr, msg.lpin_pos = cpos[5]=Lz,
        # msg.rpin_pos = cpos[6]=Rz
        q = [
            int(msg.y_pos),
            int(msg.x_pos),
            int(msg.yaw_pos),
            int(msg.lpin_rotate),
            int(msg.rpin_rotate),
            int(msg.lpin_pos),
            int(msg.rpin_pos),
        ]
        self._topik.get_q(q)
        self._topik.fk()
        self._fk_ready = True

    # ------------------------------------------------------------------
    #  FK helpers
    # ------------------------------------------------------------------

    def _get_cam_to_world(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (cam_to_world 3x3, cam_origin_world 3x1).

        cam_to_world : rotation from camera frame to robot world frame
                       = so3_pin @ cam_mount_R
        cam_origin   : camera optical center in world frame
                       = pin_world + so3_pin @ cam_offset_pin

        Falls back to (identity, zeros) when FK is unavailable,
        which makes WorldTracker operate in camera-frame space.
        """
        if self._fk_ready and self._topik is not None:
            side = self.side
            so3_pin = np.asarray(
                self._topik.so3_lcam if side == "left" else self._topik.so3_rcam,
                dtype=np.float64,
            )
            pin_world = np.array(
                self._topik.x[0] if side == "left" else self._topik.x[1],
                dtype=np.float64,
            )
            cam_to_world = so3_pin @ _CAM_MOUNT_R[side]
            cam_origin = pin_world + so3_pin @ _CAM_OFFSET[side]
            return cam_to_world, cam_origin

        return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

    def _compute_prior_xy(
        self, cam_to_world: np.ndarray, cam_origin: np.ndarray
    ) -> Optional[np.ndarray]:
        """Project the previous WorldTracker estimate into current image pixel coords.

        Used by Case-2b to resolve the symmetric line-direction ambiguity.
        Returns (u, v) pixel or None when no prior exists or is behind the camera.
        """
        prev_world = self.tracker.get_world_position()
        if prev_world is None:
            return None
        tvec_cam = cam_to_world.T @ (prev_world - cam_origin)
        if tvec_cam[2] <= 0:
            return None
        K = self.estimator.K
        u = K[0, 0] * tvec_cam[0] / tvec_cam[2] + K[0, 2]
        v = K[1, 1] * tvec_cam[1] / tvec_cam[2] + K[1, 2]
        return np.array([u, v], dtype=np.float64)

    # ------------------------------------------------------------------
    #  Main loop
    # ------------------------------------------------------------------

    def _tick(self):
        if self.rs_pipeline is None:
            return

        try:
            frames = self.rs_pipeline.wait_for_frames()
        except RuntimeError:
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        color_image = np.asanyarray(color_frame.get_data())

        # 1. FK snapshot (needed before estimate for prior_xy)
        cam_to_world, cam_origin = self._get_cam_to_world()

        # 2. Project previous world estimate → pixel prior for Case-2b disambiguation
        prior_xy = self._compute_prior_xy(cam_to_world, cam_origin)

        # 3. YOLO → solvePnP tvec in camera frame
        result = self.estimator.estimate(color_image, prior_xy=prior_xy)

        # 4. World-space EMA (ambiguity resolution + temporal filter)
        world_est = self.tracker.update(result, cam_to_world, cam_origin)

        # 5. Back-project filtered world → camera frame for /cam0
        #    world_xyz = cam_origin + cam_to_world @ tvec_cam
        #    ∴ tvec_cam = cam_to_world.T @ (world_xyz - cam_origin)  (rotation is orthogonal)
        msg = Point()
        if world_est.world_xyz is not None:
            tvec_cam = cam_to_world.T @ (world_est.world_xyz - cam_origin)
            msg.x = float(tvec_cam[0])
            msg.y = float(tvec_cam[1])
            msg.z = float(tvec_cam[2])
            self.get_logger().info(
                f"[{world_est.source}] conf={world_est.confidence:.2f}  "
                f"x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f}"
            )
        self.pub_point.publish(msg)

        # 5. GUI
        if self.show_gui:
            vis = color_image.copy()
            if result.center_xy is not None:
                cu = int(round(result.center_xy[0]))
                cv_y = int(round(result.center_xy[1]))
                color = (0, 255, 0) if result.source == "conic" else (0, 255, 255)
                cv2.circle(vis, (cu, cv_y), 6, color, -1)
                cv2.putText(vis, f"{result.source} {result.confidence:.2f}",
                            (cu + 10, cv_y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1)
            cv2.imshow("Vision Node", vis)
            cv2.waitKey(1)

    # ------------------------------------------------------------------

    def destroy_node(self):
        if self.rs_pipeline is not None:
            self.rs_pipeline.stop()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description="Vision Node — center-hole estimator (AGX Orin)")
    ap.add_argument("--weights", type=str,
                    default=str(REPO_ROOT / "vision/result/weight/yolo_baseline_n.pt"))
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--device", type=str, default="cuda:0",
                    help="Inference device: cuda:0 | cpu")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--half", action="store_true",
                    help="FP16 inference — Tensor Core acceleration on AGX Orin")
    ap.add_argument("--trt", action="store_true",
                    help="Auto-export .pt → TensorRT .engine before starting")
    ap.add_argument("--robot-version", type=int, default=4)
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    weights = args.weights
    if args.trt and weights.endswith(".pt"):
        weights = _export_trt_engine(weights, imgsz=args.width, half=args.half)

    rclpy.init()
    node = VisionNode(
        weights_path=weights,
        side=args.side,
        device=args.device,
        conf_threshold=args.conf,
        width=args.width,
        height=args.height,
        fps=args.fps,
        show_gui=not args.no_gui,
        half=args.half,
        hz=args.hz,
        robot_version=args.robot_version,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
