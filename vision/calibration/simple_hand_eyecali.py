#!/usr/bin/env python
"""Planar hand-eye calibration for Pit-in camera mount tuning.

This keeps the original idea: with mostly planar robot motion and yaw changes,
estimate camera->pin rotation by minimizing repeated ArUco marker world-XY
spread. That is better conditioned for the current dataset than full 6-DOF
hand-eye solvers.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


CALIB_DIR = Path(__file__).resolve().parent
DEFAULT_OFFSET_CM = np.array([5.55, 3.07, -21.51], dtype=np.float64)
DEFAULT_CSV_BY_SIDE = {
    "left": CALIB_DIR / "Calibration_data_cam0.csv",
    "right": CALIB_DIR / "Calibration_data_cam1.csv",
}
DEFAULT_OUT_BY_SIDE = {
    "left": CALIB_DIR / "calibration_result_cam0.txt",
    "right": CALIB_DIR / "calibration_result_cam1.txt",
}


@dataclass
class SolveResult:
    label: str
    loss: str
    mean_xy_err_cm: float
    std_xy_err_cm: float
    max_xy_err_cm: float
    cost: float
    success: bool
    rotvec: np.ndarray
    rot_cam_to_pin: np.ndarray
    euler_xyz_deg: np.ndarray


class PlanarHandEyeCalibration:
    def __init__(self, offset_vector_cm: Iterable[float], robot_unit: str = "auto"):
        self.offset_t = np.asarray(offset_vector_cm, dtype=np.float64)
        self.robot_unit = robot_unit
        self.last_robot_scale = 1.0

    def load_data(self, csv_path: Path):
        df = pd.read_csv(csv_path)
        df = df[df["m1_id"] != "N/A"].reset_index(drop=True)
        if df.empty:
            raise ValueError(f"No valid calibration rows in {csv_path}")

        robot_xy_raw = df[["robot_x", "robot_y"]].to_numpy(dtype=np.float64)
        if self.robot_unit == "m":
            robot_scale = 100.0
        elif self.robot_unit == "cm":
            robot_scale = 1.0
        else:
            robot_scale = 1.0 if float(np.nanmax(np.abs(robot_xy_raw))) > 10.0 else 100.0
        self.last_robot_scale = robot_scale

        robot_pos = np.hstack([
            robot_xy_raw * robot_scale,
            np.zeros((len(robot_xy_raw), 1), dtype=np.float64),
        ])
        robot_rot = np.array(
            [R.from_euler("z", th, degrees=False).as_matrix()
             for th in df["robot_th"].to_numpy(dtype=np.float64)],
            dtype=np.float64,
        )

        m1_data = df[["m1_x", "m1_y", "m1_z"]].to_numpy(dtype=np.float64) * 100.0
        m1_ids = df["m1_id"].to_numpy()
        m2_data = df[["m2_x", "m2_y", "m2_z"]].to_numpy(dtype=np.float64) * 100.0
        m2_ids = df["m2_id"].to_numpy()
        return robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids

    def marker_world_xy(self, rotvec, robot_pos, robot_rot, marker_data, marker_ids):
        valid_mask = ~pd.isna(marker_ids)
        if not np.any(valid_mask):
            return None

        target_R = R.from_rotvec(rotvec).as_matrix()
        v_robot_pos = robot_pos[valid_mask]
        v_robot_rot = robot_rot[valid_mask]
        v_marker_data = marker_data[valid_mask]

        rotated_cam = (target_R @ v_marker_data.T).T
        offset_added = rotated_cam - self.offset_t
        term_transformed = np.einsum("nij,nj->ni", v_robot_rot, offset_added)
        return (v_robot_pos + term_transformed)[:, :2]

    def residuals(self, rotvec, robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids):
        all_residuals = []
        for marker_data, marker_ids in ((m1_data, m1_ids), (m2_data, m2_ids)):
            xy = self.marker_world_xy(rotvec, robot_pos, robot_rot, marker_data, marker_ids)
            if xy is None:
                continue
            center_xy = np.mean(xy, axis=0)
            all_residuals.append((xy - center_xy).reshape(-1))
        if not all_residuals:
            return np.zeros(0, dtype=np.float64)
        return np.concatenate(all_residuals)

    def error_stats(self, rotvec, robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids):
        errors = []
        for marker_data, marker_ids in ((m1_data, m1_ids), (m2_data, m2_ids)):
            xy = self.marker_world_xy(rotvec, robot_pos, robot_rot, marker_data, marker_ids)
            if xy is None:
                continue
            center_xy = np.mean(xy, axis=0)
            errors.extend(np.linalg.norm(xy - center_xy, axis=1).tolist())
        errors = np.asarray(errors, dtype=np.float64)
        if errors.size == 0:
            return float("inf"), float("inf"), float("inf")
        return float(errors.mean()), float(errors.std()), float(errors.max())

    def solve_one(self, data, initial_rot, label: str, loss: str) -> SolveResult:
        robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids = data
        initial_rotvec = R.from_matrix(initial_rot).as_rotvec()
        result = least_squares(
            self.residuals,
            initial_rotvec,
            args=(robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids),
            method="trf",
            loss=loss,
            f_scale=1.0,
            max_nfev=2000,
        )
        rot = R.from_rotvec(result.x)
        mean_err, std_err, max_err = self.error_stats(
            result.x, robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids
        )
        return SolveResult(
            label=label,
            loss=loss,
            mean_xy_err_cm=mean_err,
            std_xy_err_cm=std_err,
            max_xy_err_cm=max_err,
            cost=float(result.cost),
            success=bool(result.success),
            rotvec=result.x,
            rot_cam_to_pin=rot.as_matrix(),
            euler_xyz_deg=rot.as_euler("xyz", degrees=True),
        )

    def solve_all(self, csv_path: Path, side: str) -> list[SolveResult]:
        data = self.load_data(csv_path)
        results = []
        for label, initial_rot in initial_rotations(side):
            for loss in ("linear", "soft_l1", "huber", "cauchy"):
                results.append(self.solve_one(data, initial_rot, label, loss))
        results.sort(key=lambda r: (r.mean_xy_err_cm, r.std_xy_err_cm, r.cost))
        return results


def initial_rotations(side: str):
    seeds = [("identity", np.eye(3, dtype=np.float64))]
    expected_yaw = -90.0 if side == "left" else 90.0
    seeds.append((f"expected_yaw_{expected_yaw:+.0f}", R.from_euler("z", expected_yaw, degrees=True).as_matrix()))
    for yaw in (-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0):
        seeds.append((f"yaw_{yaw:+.0f}", R.from_euler("z", yaw, degrees=True).as_matrix()))
    for roll in (180.0, -180.0):
        for yaw in (-90.0, 90.0, 180.0):
            mat = R.from_euler("xz", [roll, yaw], degrees=True).as_matrix()
            seeds.append((f"roll_{roll:+.0f}_yaw_{yaw:+.0f}", mat))
    return seeds


def print_result(result: SolveResult, title: str):
    print(f"\n[{title}] {result.label} | loss={result.loss}")
    print(f"mean/std/max XY error: {result.mean_xy_err_cm:.4f} / {result.std_xy_err_cm:.4f} / {result.max_xy_err_cm:.4f} cm")
    print(f"cost: {result.cost:.6f}  success={result.success}")
    print(f"Euler XYZ deg: {np.round(result.euler_xyz_deg, 6)}")
    print(f"R_cam_to_pin:\n{result.rot_cam_to_pin}")


def write_result(path: Path, side: str, csv_path: Path, result: SolveResult):
    path.parent.mkdir(parents=True, exist_ok=True)
    topic = "/cam0" if side == "left" else "/cam1"
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"side: {side}\n")
        f.write(f"topic: {topic}\n")
        f.write(f"csv: {csv_path}\n")
        f.write(f"solver: {result.label}\n")
        f.write(f"loss: {result.loss}\n")
        f.write(f"mean_xy_error_cm: {result.mean_xy_err_cm:.6f}\n")
        f.write(f"std_xy_error_cm: {result.std_xy_err_cm:.6f}\n")
        f.write(f"max_xy_error_cm: {result.max_xy_err_cm:.6f}\n")
        f.write(f"cost: {result.cost:.9f}\n")
        f.write(f"euler_xyz_deg: {np.array2string(result.euler_xyz_deg, precision=6)}\n")
        f.write("R_cam_to_pin:\n")
        f.write(np.array2string(result.rot_cam_to_pin, precision=8))
        f.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Planar Pit-in hand-eye calibration")
    parser.add_argument("--side", choices=["left", "right"], default="left")
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--offset-cm", nargs=3, type=float, default=DEFAULT_OFFSET_CM.tolist())
    parser.add_argument("--robot-unit", choices=["auto", "m", "cm"], default="auto")
    parser.add_argument("--top", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = args.csv or DEFAULT_CSV_BY_SIDE[args.side]
    out_path = args.out or DEFAULT_OUT_BY_SIDE[args.side]

    calibrator = PlanarHandEyeCalibration(args.offset_cm, robot_unit=args.robot_unit)
    print(f"side: {args.side} ({'/cam0' if args.side == 'left' else '/cam1'})")
    print(f"CSV: {csv_path}")
    print(f"fixed camera offset: {np.asarray(args.offset_cm)} cm")

    results = calibrator.solve_all(csv_path, args.side)
    unit = "m" if calibrator.last_robot_scale == 100.0 else "cm"
    print(f"robot_x/y unit: {args.robot_unit} (using scale x{calibrator.last_robot_scale:g}, {unit}->cm)")

    print("\n[Candidates sorted by lowest marker world-XY spread]")
    for i, result in enumerate(results[:max(args.top, 1)], start=1):
        print(
            f"{i:02d}. mean={result.mean_xy_err_cm:.4f} cm "
            f"std={result.std_xy_err_cm:.4f} cm "
            f"loss={result.loss} seed={result.label} "
            f"Euler={np.round(result.euler_xyz_deg, 3)}"
        )

    best = results[0]
    print_result(best, "BEST")
    write_result(out_path, args.side, csv_path, best)
    print(f"\nSaved best result to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
