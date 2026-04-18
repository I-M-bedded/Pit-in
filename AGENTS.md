# Repository Guidelines

## Project Overview
This repository contains control software for **Pit-in**, a battery-swapping AGV testbed. A 7-axis servo robot picks up and inserts battery pins into EVs, while the vision stack estimates hole 3D pose for visual servoing.

- Full operation target: Ubuntu 22.04 LTS, ROS2 Humble, Python 3.10.12
- Partial operation target: Windows with unavailable modules skipped automatically
- Main focus areas: robot task orchestration, macro-micro PBVS, and robust hole pose estimation under difficult lighting

## Project Structure & Module Organization
- `src_robot/robot_main/`: runtime entry points, supervisor/task orchestration, config
- `src_robot/controller/`: joystick handling, kinematics, shared robot-side logic
- `src_robot/fastech/`: legacy UDP servo drivers for 7 axes, modify narrowly
- `src_robot/leadshine/`: AGV interface, prefer `agv_dummy.py` over legacy paths
- `src_robot/ros2/`: ROS2 integration assets and message bridge helpers
- `vision/calibration/`: intrinsic and hand-eye calibration scripts
- `vision/dataset_gen/`: auto-annotation and board pose estimation utilities
- `vision/yolo/`: training, evaluation, backbone extraction, online hole pose estimation
- `vision/result/`, `vision/yolo/runs/`: generated outputs, checkpoints, and experiment artifacts

## Build, Test, and Development Commands
Use `uv` from the repository root.

- `uv sync`: install dependencies from `pyproject.toml` and `uv.lock`
- `uv run python src_robot/robot_main/main.py`: start robot control entry point
- `uv run python vision/yolo/train_yolo_baseline.py --epochs 100`: YOLO baseline training
- `uv run python vision/yolo/train_clahe_baseline.py --epochs 100`: CLAHE + YOLO baseline
- `uv run python vision/yolo/train_retinex_baseline.py --epochs 100`: Retinex + YOLO baseline
- `uv run python vision/yolo/train_coin_pose.py --phase 12 --epochs-p1 50 --epochs-p2 80`: full COIN-Pose training
- `uv run python vision/yolo/compare_all.py`: compare trained models and real inference
- `uv run python vision/yolo/yolo_backbone.py --mode eval --data_root <path>`: evaluate extracted backbone features
- `uv run python vision/test_vision.py`: local/manual ROS2 vision publisher check

Terminate robot operation with Xbox `L_JOY_BTN + R_JOY_BTN` or `Ctrl+C`.

## Runtime Architecture
The `robot` class in `src_robot/robot_main/main.py` starts daemon threads on initialization.

- `agv_planning_main`: ROS2 `rclpy.spin()` loop on Linux
- `pygame_gui` or `joystick`: platform-specific Xbox controller input
- `control_servo` x7: per-servo UDP control loop on real hardware
- `control_agv`: AGV state machine
- `supervisor.run_loop`: task dispatcher

The supervisor layer should stay thin. Keep state machines as wrapper/orchestration logic and put implementation detail in dedicated modules.

## Vision Pipeline Notes
`vision/vision_node.py` runs `HolePoseEstimator` and `WorldTracker`, then publishes **world-frame** hole positions to `/cam0` and `/cam1`.

Shared class mapping:

- `Hole` means inner ring
- `Hole_B` means outer ring

Current online estimator in `vision/yolo/hole_pose_estimator.py` is tiered:

- Case 1: center inner + outer detected, use full-template `solvePnP`
- Case 2a: center-only fallback using conic back-projection from ellipse fit
- Case 2b: geometry match over centers and guides using PCA line fitting and board-template LSQ
- Deduplication applies only to center classes; guide detections are preserved

`WorldTracker` performs EMA in world coordinates with confidence gating. Because the camera moves with the robot, image-space smoothing is not the right place to stabilize estimates.

## Robot and PBVS Notes
- `src_robot/robot_test/macro_micro.py` contains the macro-micro PBVS controller
- 4-DOF split: macro arm for angle-related motion, micro stage for XY correction
- Motor order in the Fastech stack is `trY, trX, rtZ, Lr, Rr, Lz, Rz`
- `RobotConfig` in `src_robot/robot_main/config.py` is the central place for tunable parameters

Coordinate convention: X-axis is the line from left pin to right pin, with left-to-right as positive X. Pin frames and the robot base frame share the same orientation.

## Coding Style & Naming Conventions
Follow existing Python conventions:

- 4-space indentation
- `snake_case` for functions, modules, and variables
- `PascalCase` for classes
- `UPPER_SNAKE_CASE` for meaningful constants

Code style is intentionally flexible, but keep modules script-friendly and readable. Prefer focused edits over broad cleanups.

## Rules and Safety Constraints
- Treat this repository as hardware-adjacent: favor minimal, well-scoped changes
- Do not do broad refactors in legacy driver code such as `src_robot/fastech/`
- Prefer wrapper/state-dispatch logic in supervisor/task modules, not heavy implementation there
- Before real-hardware runs, verify servo IPs in `src_robot/fastech/servolist.py`
- Before controller-based runs, verify Bluetooth controller settings in `src_robot/controller/evdev_controlthread.py`
- Review ROS2 message dependencies before assuming the ROS path is usable

## Testing Guidelines
There is no formal `pytest` suite yet. Validation is script-based and often hardware-dependent.

- For vision changes, run the closest relevant training, evaluation, or inference script
- Record dataset path, weights, and output directory when reporting results
- For robot-side changes, validate with the smallest safe entry point before launching the full main runtime
- Avoid testing assumptions on real hardware unless controller, ROS2, and motor settings are confirmed

## Commit & Pull Request Guidelines
Use short imperative commit messages scoped to one logical change, for example:

- `vision: improve center-hole fallback confidence gating`
- `robot: fix joystick shutdown handling`

PRs should include a concise summary, affected modules, manual validation steps, and logs or screenshots when behavior changes in the vision pipeline.

## Current Direction
- Refine macro-micro controller behavior, especially transition into visual servoing during Z lift
- Finalize the vision model architecture around the chosen `yolo26n-pose` backbone
- Continue training code cleanup and consolidation
- Optimize for robust hole 3D pose estimation under backlight, saturation, and low-light conditions without auxiliary lighting
