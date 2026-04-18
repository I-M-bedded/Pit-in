#!/usr/bin/env python
"""
Vision Node — ROS2 node for robust center-hole pose estimation.

Replaces the placeholder in test_vision.py with the full
HolePoseEstimator + WorldTracker pipeline.

Camera: Intel RealSense (pyrealsense2), direct USB — not ROS Image topics.
Output: /cam0 (geometry_msgs/Point) — camera-frame 3D position [m],
        matching the interface expected by src_robot/ros2/interface.py.

Pipeline per frame:
    1. RealSense → color + aligned depth
    2. HolePoseEstimator.estimate(color)  → center_xy, confidence (camera 2D)
    3. depth at center_xy → rs2_deproject  → camera-frame 3D point
    4. WorldTracker (EMA in camera 3D; robot FK available via /agv_joint_state)
    5. Publish Point(x, y, z) on /cam0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

# -- Ensure vision/yolo is on sys.path before local imports --
REPO_ROOT = Path(__file__).resolve().parent.parent
_yolo_dir = str(REPO_ROOT / "vision" / "yolo")
if _yolo_dir not in sys.path:
    sys.path.insert(0, _yolo_dir)
# Also add vision/ itself so sibling modules are reachable
_vision_dir = str(REPO_ROOT / "vision")
if _vision_dir not in sys.path:
    sys.path.insert(0, _vision_dir)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from hole_pose_estimator import HolePoseEstimator

# Optional: joint state for world-space EMA (better than camera-space)
try:
    from pitin_msgs.msg import JointState as PitinJointState
    _HAS_JOINT_MSG = True
except ImportError:
    _HAS_JOINT_MSG = False


class VisionNode(Node):
    """RealSense → HolePoseEstimator → /cam0 Point."""

    def __init__(
        self,
        weights_path: str,
        side: str = "left",
        device: str = "0",
        conf_threshold: float = 0.5,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        show_gui: bool = True,
    ):
        super().__init__("vision_node")
        self.side = side
        self.show_gui = show_gui

        # -- RealSense setup (mirrors test_vision.py) --
        self.rs_pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        rs_config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        try:
            profile = self.rs_pipeline.start(rs_config)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("RealSense connected.")
        except Exception as e:
            self.get_logger().error(f"RealSense init failed: {e}")
            self.rs_pipeline = None
            return

        # -- Camera intrinsics (from RealSense, not a file) --
        frames = self.rs_pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        self._depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        K = np.array([
            [self._depth_intrin.fx, 0.0, self._depth_intrin.ppx],
            [0.0, self._depth_intrin.fy, self._depth_intrin.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        D = np.array(self._depth_intrin.coeffs[:5], dtype=np.float64)

        self.get_logger().info(
            f"Intrinsics: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
            f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}"
        )

        # -- Estimator (stateless, single-frame) --
        self.estimator = HolePoseEstimator(
            weights_path=weights_path,
            camera_matrix=K,
            dist_coeffs=D,
            device=device,
            conf_threshold=conf_threshold,
        )

        # -- EMA state (camera-frame 3D) --
        self._ema_pos: np.ndarray | None = None
        self._ema_conf: float = 0.0
        self._ema_alpha: float = 0.3
        self._ema_decay: float = 0.9

        # -- ROS2 pub/sub --
        pub_topic = "/cam0" if side == "left" else "/cam1"
        self.pub_point = self.create_publisher(Point, pub_topic, 10)

        # Timer-driven at ~10 Hz (matches test_vision.py)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(f"Publishing on {pub_topic} @ 10 Hz")

    # ------------------------------------------------------------------
    #  Main loop
    # ------------------------------------------------------------------

    def _tick(self):
        if self.rs_pipeline is None:
            return

        # Grab frames
        try:
            frames = self.rs_pipeline.wait_for_frames()
        except RuntimeError:
            return
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # 1. Per-frame estimation (camera 2D)
        result = self.estimator.estimate(color_image)

        point_3d = None
        if result.center_xy is not None:
            u = int(round(result.center_xy[0]))
            v = int(round(result.center_xy[1]))
            # Clamp to image bounds
            u = max(0, min(u, color_image.shape[1] - 1))
            v = max(0, min(v, color_image.shape[0] - 1))

            dist_m = depth_frame.get_distance(u, v)
            if dist_m > 0:
                # RealSense deprojection → camera-frame 3D [m]
                point_3d = np.array(
                    rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], dist_m),
                    dtype=np.float64,
                )

        # 2. EMA in camera-frame 3D
        cam_3d = self._update_ema(point_3d, result.confidence)

        # 3. Publish
        msg = Point()
        if cam_3d is not None:
            msg.x, msg.y, msg.z = float(cam_3d[0]), float(cam_3d[1]), float(cam_3d[2])
            self.get_logger().info(
                f"[{result.source}] conf={result.confidence:.2f}  "
                f"x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f}"
            )
        self.pub_point.publish(msg)

        # 4. GUI
        if self.show_gui:
            vis = color_image.copy()
            if result.center_xy is not None:
                cu, cv = int(round(result.center_xy[0])), int(round(result.center_xy[1]))
                color = (0, 255, 0) if result.source == "conic" else (0, 255, 255)
                cv2.circle(vis, (cu, cv), 6, color, -1)
                cv2.putText(vis, f"{result.source} {result.confidence:.2f}",
                            (cu + 10, cv - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1)
            cv2.imshow("Vision Node", vis)
            cv2.waitKey(1)

    # ------------------------------------------------------------------
    #  Camera-frame EMA
    # ------------------------------------------------------------------

    def _update_ema(
        self, point_3d: np.ndarray | None, confidence: float,
    ) -> np.ndarray | None:
        """EMA filter on camera-frame 3D position.

        Note: Ideally this should be done in world space (using robot FK)
        so that robot motion doesn't corrupt the filter.  For now we filter
        in camera 3D — acceptable when the robot is stationary during
        observation (stop-observe protocol).
        """
        if point_3d is None:
            self._ema_conf *= self._ema_decay
            if self._ema_pos is not None and self._ema_conf > 0.1:
                return self._ema_pos.copy()
            return None

        alpha = self._ema_alpha * confidence
        if self._ema_pos is None:
            self._ema_pos = point_3d.copy()
            self._ema_conf = confidence
        else:
            self._ema_pos = alpha * point_3d + (1.0 - alpha) * self._ema_pos
            self._ema_conf = alpha * confidence + (1.0 - alpha) * self._ema_conf

        return self._ema_pos.copy()

    # ------------------------------------------------------------------
    #  Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        if self.rs_pipeline is not None:
            self.rs_pipeline.stop()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description="Vision Node — center-hole estimator")
    ap.add_argument("--weights", type=str,
                    default=str(REPO_ROOT / "vision/result/weight/yolo_baseline_n.pt"))
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--device", type=str, default="0", help="YOLO device (0=GPU, cpu)")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = VisionNode(
        weights_path=args.weights,
        side=args.side,
        device=args.device,
        conf_threshold=args.conf,
        width=args.width,
        height=args.height,
        fps=args.fps,
        show_gui=not args.no_gui,
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
