#!/usr/bin/env python3
"""Live FK / world-coordinate diagnostic for manual motor motion tests.

Subscribes to:
  - /agv_joint_state (pitin_msgs/JointState)
  - /cam0 or /cam1   (geometry_msgs/Point)

Then prints:
  - current joint counts
  - current pin world position from Topik FK
  - current camera origin world position (same model as vision_node.py)
  - current hole world position from /camX
  - deltas from the captured baseline
  - pin-to-hole world error

How to use
----------
1. Source ROS:
   source /opt/ros/humble/setup.bash
   source /workspaces/pitin_ros2_ws/install/setup.bash
2. Run:
   python src_robot/check_fk.py --side left
3. Keep the hole physically fixed, then move stage / arm manually.
   If world conversion is correct:
     - d_pin changes
     - d_cam changes
     - d_hole should stay near zero
     - pin_to_hole_err should change opposite to pin motion

Press Enter at any time to recapture the baseline.
Type 'q' + Enter to quit.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_ROBOT = REPO_ROOT / "src_robot"
if str(SRC_ROBOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROBOT))

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from pitin_msgs.msg import JointState as PitinJointState
from controller.ik import Topik


_CAM_MOUNT_R = {
    "left": np.array(
        [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    ),
    "right": np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    ),
}
_CAM_OFFSET = {
    "left": np.array([-0.057, 0.029, 0.0], dtype=np.float64),
    "right": np.array([0.057, -0.029, 0.0], dtype=np.float64),
}


@dataclass
class Snapshot:
    counts: np.ndarray
    pin_world: np.ndarray
    cam_world: np.ndarray
    hole_world: Optional[np.ndarray]
    hole_rel_world: Optional[np.ndarray]
    err_world_xy: Optional[np.ndarray]


class FkChecker(Node):
    def __init__(self, side: str, robot_version: int, print_hz: float):
        super().__init__("fk_checker")
        self.side = side
        self._fk_idx = 0 if side == "left" else 1
        self._topik = Topik(robot_version)

        self._counts: Optional[np.ndarray] = None
        self._hole_world: Optional[np.ndarray] = None
        self._baseline: Optional[Snapshot] = None
        self._lock = threading.Lock()
        self._quit_requested = False

        cam_topic = "/cam0" if side == "left" else "/cam1"
        self.create_subscription(PitinJointState, "/agv_joint_state", self._joint_cb, 10)
        self.create_subscription(Point, cam_topic, self._cam_cb, 10)
        self.create_timer(1.0 / print_hz, self._tick)

        self.get_logger().info(
            f"Watching /agv_joint_state and {cam_topic} for side={side}. "
            "Press Enter to reset baseline, q+Enter to quit."
        )
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

    def _joint_cb(self, msg: PitinJointState):
        counts = np.array(
            [
                int(msg.y_pos),
                int(msg.x_pos),
                int(msg.yaw_pos),
                int(msg.lpin_rotate),
                int(msg.rpin_rotate),
                int(msg.lpin_pos),
                int(msg.rpin_pos),
            ],
            dtype=np.int64,
        )
        with self._lock:
            self._counts = counts

    def _cam_cb(self, msg: Point):
        hole_world = np.array([float(msg.x), float(msg.y), float(msg.z)], dtype=np.float64)
        with self._lock:
            self._hole_world = hole_world

    def _compute_snapshot(self) -> Optional[Snapshot]:
        with self._lock:
            counts = None if self._counts is None else self._counts.copy()
            hole_world = None if self._hole_world is None else self._hole_world.copy()

        if counts is None:
            return None

        self._topik.get_q(counts)
        self._topik.fk()

        pin_world = np.array(self._topik.x[self._fk_idx], dtype=np.float64)
        so3_pin = np.asarray(
            self._topik.so3_lcam if self.side == "left" else self._topik.so3_rcam,
            dtype=np.float64,
        )
        cam_world = pin_world + so3_pin @ _CAM_OFFSET[self.side]
        hole_rel_world = None
        err_world_xy = None
        if hole_world is not None:
            hole_rel_world = hole_world - cam_world
            err_world_xy = hole_world[:2] - pin_world[:2]

        return Snapshot(
            counts=counts,
            pin_world=pin_world,
            cam_world=cam_world,
            hole_world=hole_world,
            hole_rel_world=hole_rel_world,
            err_world_xy=err_world_xy,
        )

    def _format_vec(self, vec: Optional[np.ndarray], dims: int = 3) -> str:
        if vec is None:
            return "none"
        vals = np.asarray(vec, dtype=np.float64).reshape(-1)[:dims]
        return "[" + ", ".join(f"{v:+.4f}" for v in vals) + "]"

    def _tick(self):
        snap = self._compute_snapshot()
        if snap is None:
            self.get_logger().info("Waiting for /agv_joint_state...")
            return

        if self._baseline is None:
            self._baseline = snap
            self.get_logger().info("Baseline captured from first valid sample.")

        base = self._baseline
        d_counts = snap.counts - base.counts
        d_pin = snap.pin_world - base.pin_world
        d_cam = snap.cam_world - base.cam_world
        d_hole = None if snap.hole_world is None or base.hole_world is None else snap.hole_world - base.hole_world
        d_hole_rel = (
            None
            if snap.hole_rel_world is None or base.hole_rel_world is None
            else snap.hole_rel_world - base.hole_rel_world
        )
        d_err = None if snap.err_world_xy is None or base.err_world_xy is None else snap.err_world_xy - base.err_world_xy

        lines = [
            "=" * 78,
            f"side={self.side}  counts=[trY,trX,rtZ,Lr,Rr,Lz,Rz]={snap.counts.tolist()}",
            f"d_counts={d_counts.tolist()}",
            f"pin_world      = {self._format_vec(snap.pin_world)}   d_pin={self._format_vec(d_pin)}",
            f"cam_origin     = {self._format_vec(snap.cam_world)}   d_cam={self._format_vec(d_cam)}",
            f"hole_world     = {self._format_vec(snap.hole_world)}   d_hole={self._format_vec(d_hole)}",
            f"hole_rel_world = {self._format_vec(snap.hole_rel_world)}   d_rel={self._format_vec(d_hole_rel)}",
            f"pin_to_hole_xy = {self._format_vec(snap.err_world_xy, dims=2)}   d_err={self._format_vec(d_err, dims=2)}",
        ]

        if d_hole is not None:
            lines.append(
                "expectation: fixed physical hole + moving stage => "
                "d_hole ~ 0, while d_pin / d_cam change"
            )

        print("\n".join(lines), flush=True)

        if self._quit_requested:
            rclpy.shutdown()

    def _stdin_loop(self):
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
            except EOFError:
                return

            if cmd == "q":
                self._quit_requested = True
                return

            snap = self._compute_snapshot()
            if snap is not None:
                self._baseline = snap
                print("[BASELINE] recaptured from current sample", flush=True)
            else:
                print("[BASELINE] skipped: no joint sample yet", flush=True)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Check FK / world stability while moving motors manually")
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--robot-version", type=int, default=4)
    ap.add_argument("--print-hz", type=float, default=2.0)
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = FkChecker(
        side=args.side,
        robot_version=args.robot_version,
        print_hz=args.print_hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
