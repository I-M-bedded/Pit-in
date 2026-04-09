# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Control software for **Pit-in**, a battery-swapping AGV testbed. A 7-axis servo robot picks up and inserts battery pins into EVs. A vision system estimates hole 3D pose for visual servoing.

- **Full operation**: Ubuntu 22.04 LTS + ROS2 Humble, Python 3.10.12
- **Partial operation** (no ROS2, no evdev): Windows — auto-detects OS and skips unavailable modules

## Commands

```bash
# Run robot (from repo root)
python src_robot/robot_main/main.py

# Train vision model (TD-DRP on YOLO26n-pose backbone)
python vision/yolo/train.py --data_root ./data/LIS --phase 1 --epochs 50 --batch_size 4   # Phase 1: SSL warm-up
python vision/yolo/train.py --data_root ./data/LIS --phase 2 --epochs 30 --resume ./checkpoints/tddrp_phase1.pt  # Phase 2: Task tuning
python vision/yolo/train.py --data_root ./data/LIS --phase 12 --epochs_p1 50 --epochs_p2 30  # Both phases

# Backbone evaluation
python vision/yolo/yolo_backbone.py --mode eval --data_root ./data/LIS

# Dependencies (managed with uv)
uv sync
```

Terminate robot: **L_JOY_BTN + R_JOY_BTN** on Xbox controller, or `Ctrl+C`.

## Quick Setup (first-time)

1. Match motor IP addresses in `src_robot/fastech/servolist.py`
2. Match Bluetooth controller MAC in `src_robot/controller/evdev_controlthread.py` line 9 (`controller_address_2`)

## Architecture

### Thread Model (`src_robot/robot_main/main.py`)

The `robot` class spawns daemon threads on `__init__`:

| Thread | Platform | Description |
|---|---|---|
| `agv_planning_main` | Linux only | ROS2 `rclpy.spin()` loop |
| `pygame_gui` / `joystick` | Win / Linux | Xbox controller input |
| `control_servo` (x7) | Real robot only | Per-servo UDP control loop |
| `control_agv` | Always | AGV state machine, ~6 Hz |
| `supervisor.run_loop` | Always | Button-to-Task dispatcher, ~25 Hz |

Termination: `robot.process_command` flag (set to 1 to stop all threads).

### Supervisor & Tasks (`src_robot/robot_main/suprevisor.py`, `tasks.py`)

`TopPlateSupervisor.run_loop()` reads controller state each cycle and dispatches to task objects. All tasks extend `BaseTask` (fields: `is_active`, `step`, `timer`, `flag`).

- `HomingTask` -- multi-step origin-finding for all 7 axes
- `LoadingTask` -- approach + load battery pin (triggered by AGV server or LT+B/X)
- `ManualpinTask` -- joystick manual control of lift, pin height, pin rotation, wing gap
- `VisionTask` -- FK check and calibration data saving (L button)
- `PbvsTask` -- Position-Based Visual Servoing via `HybridController` from `robot_test/macro_micro.py` (LT+Y)

### Configuration (`src_robot/robot_main/config.py`)

- `RobotConfig` -- central dataclass for all parameters (IPs, servo heights, control gains, timeouts, vision offsets). Tune the robot here without touching logic.
- `InputManager` -- wraps raw joystick axes/buttons into a named-key dict (`get_state()`).

### Macro-Micro PBVS (`src_robot/robot_test/macro_micro.py`)

4-DOF hybrid controller: 2-DOF arm (Macro: rtZ rotation + wing) + 2-DOF XY stage (Micro: trY + trX).

State machine: `MACRO_ARM -> WAIT_MACRO -> MICRO_STAGE -> WAIT_MICRO -> VS_LIFT -> DONE`

Motor mapping: `[0]trY, [1]trX, [2]rtZ, [3]Lr, [4]Rr, [5]Lz, [6]Rz`. Left/Right side configs in `SIDE_CFG` dict select FK index, wing joint, z motor, Jacobian rows/cols, camera mount rotation, and IK sign.

`ControlConfig` dataclass holds all PBVS gains, thresholds, velocity limits, and EMA filter coefficient.

### Servo Control (`src_robot/fastech/`)

7 Fastech servo drives over UDP. Motor IDs 0-6: trY, trX, rtZ, Lr, Rr, Lz, Rz (see `servolist.py`).

`t_action` per axis: `0`=position control (tracks `t_pos`), `1`=homing, `2`=servo on, `3`=servo off.

### Kinematics (`src_robot/controller/ik.py`)

`Topik` class -- top-plate inverse kinematics. Version-dependent link parameters (`d2`, `d3`) selected by `RobotConfig.version` (currently `4` = CORA). `cnt2m`/`m2cnt` arrays convert encoder counts to meters/radians. Rotation axes 2, 3, 4 have sign inversion in `get_q()` and `_q_cmd_to_cnt()`.

### AGV Interface (`src_robot/leadshine/`)

- `agv_dummy.py` -- clean AGV controller (use this)
- `agv_con.py` -- original with legacy code

### ROS2 Interface (`src_robot/ros2/interface.py`)

Subscribes to `/cam0` for camera data. Uses `ros2-websocket-bridge` for non-ROS2 systems (see `ros2/test.js`). Custom messages in `src_robot/ros2/pitin_msgs.zip` -- build in ROS2 workspace.

### Vision Pipeline (`vision/`)

- `calibration/` -- intrinsic (`intrinsic_camera_calibration.py`) and hand-eye calibration (`simple_hand_eyecali.py`)
- `dataset_gen/` -- auto-annotation (`auto_annotator.py`) and pose estimation (`pose_estimation_Board.py`)
- `yolo/` -- TD-DRP vision model built on YOLO26n-pose backbone

#### TD-DRP Model (`vision/yolo/td_drp.py`)

Task-Driven Dynamic Routing with Feature-Consistency SSL. Designed for illumination robustness (backlight/saturation/low-light) without auxiliary lighting.

Key components: `SpatiallyVariantIlluminationEncoder` (extracts illumination embedding Z from RGB), `SpatiallyAwareSoftRouter` (expert routing weights from Z), `RSVFiLM` (feature-wise linear modulation). Multi-scale features from YOLO26n-pose: P3(layer4, 128ch, stride8), P4(layer6, 128ch, stride16), P5(layer10, 256ch, stride32).

Training: Phase 1 = SSL warm-up (encoder + router + RSV-FiLM only, YOLO frozen), Phase 2 = full task tuning (all params including YOLO backbone).

#### Backbone Extraction (`vision/yolo/yolo_backbone.py`)

Extracts intermediate features from YOLO26n-pose for TD-DRP pipeline. Layer map in `YOLO26N_LAYER_INFO`.

## Coordinate Convention

X-axis = line connecting L pin to R pin (L-to-R is +X). Right-handed coordinate system. Pin frames and robot base frame share the same orientation. Robot structure visible in `urdf_pitin.urdf`.

## Rules for Claude

- Expert in vision-based Visual Servoing, especially hole 3D pose estimation
- State machines must contain only wrapper functions; actual implementation goes in separate modules (like supervisor/task pattern)
- **DO NOT** do full refactoring or modify legacy driver code (`src_robot/fastech/` etc.)
- Code style is free/flexible
- Vision model goal: hole 3D pose estimation with robustness to backlight/saturation/low-light, no auxiliary lighting
- Backbone: yolo26n-pose (decided)

## Current Work

- Macro-Micro controller refinement: angle = Macro, x/y = Micro. Plan to switch to VS during z-pin lift phase.
- Vision model architecture finalization
- Training code rewrite
