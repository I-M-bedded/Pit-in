#!/usr/bin/env python
"""List connected RealSense cameras and print copy-ready side commands."""

from __future__ import annotations

import argparse
from pathlib import Path

import pyrealsense2 as rs


REPO_ROOT = Path(__file__).resolve().parent.parent
VISION_NODE = REPO_ROOT / "vision" / "vision_node.py"


def get_info(device, info) -> str:
    try:
        if device.supports(info):
            return device.get_info(info)
    except Exception:
        pass
    return ""


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print RealSense serial numbers for left/right camera mapping."
    )
    parser.add_argument(
        "--weights",
        default=str(REPO_ROOT / "vision/result/weight/yolo_baseline_n.pt"),
        help="Weights path to include in the copy-ready vision_node commands.",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Include --no-gui in the copy-ready vision_node commands.",
    )
    args = parser.parse_args()

    devices = list(rs.context().query_devices())
    if not devices:
        print("No RealSense devices detected.")
        print("Check USB connection and camera permissions, then run again.")
        return 1

    print(f"Detected {len(devices)} RealSense device(s):")
    print()

    serials = []
    for idx, device in enumerate(devices, start=1):
        serial = get_info(device, rs.camera_info.serial_number)
        serials.append(serial)
        name = get_info(device, rs.camera_info.name)
        product_line = get_info(device, rs.camera_info.product_line)
        usb = get_info(device, rs.camera_info.usb_type_descriptor)
        firmware = get_info(device, rs.camera_info.firmware_version)
        physical_port = get_info(device, rs.camera_info.physical_port)

        print(f"[{idx}]")
        print(f"  serial        : {serial}")
        print(f"  name          : {name}")
        print(f"  product_line  : {product_line}")
        print(f"  usb           : {usb}")
        print(f"  firmware      : {firmware}")
        print(f"  physical_port : {physical_port}")
        print()

    print("Copy one serial into LEFT_SERIAL and the other into RIGHT_SERIAL:")
    print()
    print("LEFT_SERIAL=<paste_left_camera_serial_here>")
    print("RIGHT_SERIAL=<paste_right_camera_serial_here>")
    print()

    extra = " --no-gui" if args.no_gui else ""
    print("Copy-ready commands:")
    print()
    print(
        f"uv run python {VISION_NODE} --side left "
        f"--serial <LEFT_SERIAL> --weights {args.weights}{extra}"
    )
    print(
        f"uv run python {VISION_NODE} --side right "
        f"--serial <RIGHT_SERIAL> --weights {args.weights}{extra}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
