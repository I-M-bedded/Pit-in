#!/usr/bin/env python3
"""
OpenCV fisheye intrinsic calibration for STURDeCAM31 wide-angle camera images.

Example:
    python3 calibrate_fisheye.py \
        --images "calib_images/*.png" \
        --board-cols 9 \
        --board-rows 6 \
        --square-size 0.025 \
        --output sturdecam31_fisheye_intrinsics.yaml

Notes:
    --board-cols and --board-rows are the number of INNER chessboard corners,
    not the number of printed squares.

    --square-size can be meters, millimeters, or any unit you prefer. Intrinsic
    K/D are unitless/pixel-based, but rvec/tvec will use this same unit.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Iterable

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".bmp", ".dib", ".jpeg", ".jpg", ".jpe", ".png", ".webp", ".tif", ".tiff"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate fisheye intrinsics from chessboard images."
    )
    parser.add_argument(
        "--images",
        required=True,
        help='Image glob or directory, for example "calib_images/*.png" or "calib_images".',
    )
    parser.add_argument(
        "--board-cols",
        type=int,
        required=True,
        help="Number of inner chessboard corners along the board width.",
    )
    parser.add_argument(
        "--board-rows",
        type=int,
        required=True,
        help="Number of inner chessboard corners along the board height.",
    )
    parser.add_argument(
        "--square-size",
        type=float,
        required=True,
        help="Physical chessboard square size. Use a consistent unit, e.g. 0.025 for 25 mm.",
    )
    parser.add_argument(
        "--output",
        default="fisheye_intrinsics.yaml",
        help="Output OpenCV YAML path.",
    )
    parser.add_argument(
        "--preview-dir",
        default=None,
        help="Optional directory to save corner detection preview images.",
    )
    parser.add_argument(
        "--balance",
        type=float,
        default=0.0,
        help="Balance for estimated undistort projection. 0=crop more, 1=keep more FOV.",
    )
    parser.add_argument(
        "--min-valid",
        type=int,
        default=10,
        help="Minimum number of valid chessboard images required.",
    )
    return parser.parse_args()


def resolve_images(path_or_glob: str) -> list[str]:
    if os.path.isdir(path_or_glob):
        candidates = [
            os.path.join(path_or_glob, name)
            for name in sorted(os.listdir(path_or_glob))
            if os.path.splitext(name.lower())[1] in IMAGE_EXTENSIONS
        ]
    else:
        candidates = sorted(glob.glob(path_or_glob))

    return [path for path in candidates if os.path.isfile(path)]


def make_object_points(board_cols: int, board_rows: int, square_size: float) -> np.ndarray:
    objp = np.zeros((1, board_cols * board_rows, 3), np.float64)
    grid = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp[0, :, :2] = grid * square_size
    return objp


def find_chessboard(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6,
    )
    refined = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
    return True, refined.astype(np.float64)


def iter_valid_detections(
    image_paths: Iterable[str],
    pattern_size: tuple[int, int],
    preview_dir: str | None,
) -> tuple[list[np.ndarray], list[str], tuple[int, int]]:
    image_points: list[np.ndarray] = []
    used_paths: list[str] = []
    image_size: tuple[int, int] | None = None

    if preview_dir:
        os.makedirs(preview_dir, exist_ok=True)

    for path in image_paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            print(f"SKIP unreadable: {path}", file=sys.stderr)
            continue

        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            print(
                f"SKIP size mismatch: {path} has {current_size}, expected {image_size}",
                file=sys.stderr,
            )
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = find_chessboard(gray, pattern_size)
        print(f"{'OK  ' if found else 'MISS'} {path}")

        if not found or corners is None:
            continue

        image_points.append(corners.reshape(1, -1, 2))
        used_paths.append(path)

        if preview_dir:
            preview = image.copy()
            preview_corners = corners.reshape(-1, 1, 2).astype(np.float32)
            cv2.drawChessboardCorners(preview, pattern_size, preview_corners, found)

            preview_path = os.path.join(preview_dir, os.path.basename(path))
            cv2.imwrite(preview_path, preview)

    if image_size is None:
        raise RuntimeError("No readable images found.")

    return image_points, used_paths, image_size


def compute_reprojection_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    k: np.ndarray,
    d: np.ndarray,
) -> tuple[float, list[float]]:
    per_view_errors: list[float] = []
    total_sq_error = 0.0
    total_points = 0

    for objp, imgp, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, k, d)
        error = cv2.norm(imgp, projected, cv2.NORM_L2)
        point_count = objp.shape[1]
        per_view_errors.append(float(np.sqrt((error * error) / point_count)))
        total_sq_error += error * error
        total_points += point_count

    mean_error = float(np.sqrt(total_sq_error / total_points))
    return mean_error, per_view_errors


def write_yaml(
    output_path: str,
    image_size: tuple[int, int],
    board_size: tuple[int, int],
    square_size: float,
    k: np.ndarray,
    d: np.ndarray,
    new_k: np.ndarray,
    rms: float,
    mean_reprojection_error: float,
    per_view_errors: list[float],
    used_paths: list[str],
) -> None:
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Could not open output file: {output_path}")

    fs.write("camera_model", "opencv_fisheye")
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("board_cols", int(board_size[0]))
    fs.write("board_rows", int(board_size[1]))
    fs.write("square_size", float(square_size))
    fs.write("rms", float(rms))
    fs.write("mean_reprojection_error_px", float(mean_reprojection_error))
    fs.write("K", k)
    fs.write("D", d)
    fs.write("K_undistort", new_k)
    fs.write("per_view_errors_px", np.array(per_view_errors, dtype=np.float64))
    fs.startWriteStruct("used_images", cv2.FileNode_SEQ)
    for path in used_paths:
        fs.write("", path)
    fs.endWriteStruct()
    fs.release()


def main() -> int:
    args = parse_args()
    if args.board_cols <= 1 or args.board_rows <= 1:
        print("board-cols and board-rows must be greater than 1.", file=sys.stderr)
        return 2
    if args.square_size <= 0:
        print("square-size must be positive.", file=sys.stderr)
        return 2

    image_paths = resolve_images(args.images)
    if not image_paths:
        print(f"No images matched: {args.images}", file=sys.stderr)
        return 1

    board_size = (args.board_cols, args.board_rows)
    image_points, used_paths, image_size = iter_valid_detections(
        image_paths,
        board_size,
        args.preview_dir,
    )

    if len(image_points) < args.min_valid:
        print(
            f"Only {len(image_points)} valid images found. Need at least {args.min_valid}.",
            file=sys.stderr,
        )
        return 1

    objp = make_object_points(args.board_cols, args.board_rows, args.square_size)
    object_points = [objp.copy() for _ in image_points]

    k = np.zeros((3, 3), dtype=np.float64)
    d = np.zeros((4, 1), dtype=np.float64)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in image_points]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in image_points]

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        200,
        1e-7,
    )
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_CHECK_COND
        | cv2.fisheye.CALIB_FIX_SKEW
    )

    rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        k,
        d,
        rvecs,
        tvecs,
        flags=flags,
        criteria=criteria,
    )

    new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        k,
        d,
        image_size,
        np.eye(3),
        balance=args.balance,
        new_size=image_size,
    )
    mean_error, per_view_errors = compute_reprojection_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        k,
        d,
    )

    write_yaml(
        args.output,
        image_size,
        board_size,
        args.square_size,
        k,
        d,
        new_k,
        rms,
        mean_error,
        per_view_errors,
        used_paths,
    )

    print()
    print(f"Used images: {len(used_paths)} / {len(image_paths)}")
    print(f"Image size: {image_size[0]} x {image_size[1]}")
    print(f"RMS: {rms:.6f}")
    print(f"Mean reprojection error: {mean_error:.6f} px")
    print("K:")
    print(k)
    print("D:")
    print(d.ravel())
    print(f"Wrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
