# Pit-in: Battery-Swapping AGV Testbed

Control software and vision system for **Pit-in**, a battery-swapping AGV testbed. A 7-axis servo robot picks up and inserts battery pins into EVs, guided by a vision system that estimates hole 3D pose for visual servoing.

## System Requirements

| Mode | OS | Dependencies |
|---|---|---|
| Full operation | Ubuntu 22.04 LTS + ROS2 Humble | Python 3.10.12, all packages |
| Partial operation | Windows | Auto-detects OS, skips ROS2/evdev |

## Quick Start

```bash
# Install dependencies (managed with uv)
uv sync

# Run robot
python src_robot/robot_main/main.py
```

**First-time setup:**
1. Set motor IP addresses in `src_robot/fastech/servolist.py`
2. Set Bluetooth controller MAC in `src_robot/controller/evdev_controlthread.py` (line 9)
3. Build ROS2 custom messages from `src_robot/ros2/pitin_msgs.zip` in your ROS2 workspace (Linux only)

**Terminate:** `L_JOY_BTN + R_JOY_BTN` on Xbox controller, or `Ctrl+C`

## Repository Structure

```
src_robot/                  # Robot control software
├── robot_main/
│   ├── main.py             # Entry point, thread spawning
│   ├── config.py           # RobotConfig dataclass, InputManager
│   ├── suprevisor.py       # TopPlateSupervisor — button-to-task dispatcher
│   └── tasks.py            # Homing, Loading, ManualPin, Vision, PBVS tasks
├── robot_test/
│   └── macro_micro.py      # Macro-Micro PBVS hybrid controller
├── fastech/                # Fastech servo driver (7x UDP)
├── controller/
│   ├── ik.py               # Top-plate inverse kinematics
│   ├── evdev_controlthread.py  # Linux Xbox controller
│   └── joy.py              # Windows Xbox controller
├── leadshine/              # AGV motor controller
└── ros2/                   # ROS2 interface & websocket bridge

vision/                     # Vision system
├── vision_node.py          # ROS2 node — publishes world-frame hole pose
├── calibration/            # Intrinsic & hand-eye calibration
├── dataset_gen/            # Auto-annotation & synthetic data generation
└── yolo/                   # Detection models & online pose pipeline
    ├── hole_pose_estimator.py  # Tiered pose estimator (solvePnP / conic / geometry)
    ├── world_tracker.py        # EMA filter in world coordinates
    ├── PRIM_pose.py            # PRIM model architecture
    ├── PRIMv2.py               # PRIMv2 model architecture
    ├── coin_pose.py            # COIN-Pose model architecture
    ├── train_*.py              # Training scripts for all models
    └── yolo_backbone.py        # YOLO26n-pose feature extraction
```

## Robot Architecture

### Thread Model

The `robot` class spawns daemon threads on initialization:

| Thread | Platform | Description |
|---|---|---|
| `agv_planning_main` | Linux | ROS2 spin loop |
| `pygame_gui` / `joystick` | Win / Linux | Xbox controller input |
| `control_servo` (x7) | Real robot | Per-servo UDP control loop |
| `control_agv` | All | AGV state machine (~6 Hz) |
| `supervisor.run_loop` | All | Task dispatcher (~25 Hz) |

### Servo Configuration

7 Fastech servo drives over UDP (Motor IDs 0–6):

| ID | Axis | Description |
|----|------|-------------|
| 0 | trY | Translation Y |
| 1 | trX | Translation X |
| 2 | rtZ | Rotation Z |
| 3 | Lr | Left rotation |
| 4 | Rr | Right rotation |
| 5 | Lz | Left Z |
| 6 | Rz | Right Z |

### Macro-Micro PBVS Controller

4-DOF hybrid controller for position-based visual servoing:
- **Macro** (2-DOF arm): rtZ rotation + wing — coarse angular alignment
- **Micro** (2-DOF XY stage): trY + trX — fine positional correction

State machine: `MACRO_ARM → WAIT_MACRO → MICRO_STAGE → WAIT_MICRO → VS_LIFT → DONE`

### Coordinate Convention

X-axis = L pin → R pin direction (+X). Right-handed coordinate system. See `urdf_pitin.urdf` for robot structure.

## Vision System

### Detection Backbone

**YOLO26n-pose** — lightweight pose estimation backbone.

### Trained Models

| Model | Description | Training |
|---|---|---|
| YOLO Baseline | Vanilla YOLO26n-pose | `train_yolo_baseline.py` |
| CLAHE + YOLO | CLAHE preprocessing | `train_clahe_baseline.py` |
| MSRCR + YOLO | Retinex preprocessing | `train_retinex_baseline.py` |
| COIN-Pose | Illumination-aware modulation + cross-scale context + occlusion reconstruction | `train_coin_pose.py` (Phase 1: SSL, Phase 2: task tuning) |
| PRIM | Pose-Robust Illumination Model | `train_PRIM.py` |
| PRIMv2 | PRIM with architectural improvements | `train_PRIM.py` |

### Online Pose Pipeline

`HolePoseEstimator` performs tiered per-frame estimation:

1. **Case 1** — Both inner + outer center holes detected → solvePnP (highest confidence)
2. **Case 2a** — Single center hole → conic back-projection from fitted ellipse
3. **Case 2b** — No center hole → geometry matching with guide holes via combinatorial LSQ

### Class Mapping

| ID | Name | Description |
|----|------|-------------|
| 0 | center_hole_inner | CenterHole (inner ring) |
| 1 | center_hole_outer | CenterHole_B (outer ring, AR > 1.3) |
| 2 | guide_hole_inner | Hole |
| 3 | guide_hole_outer | Hole_B |

### World-space Tracker

EMA filter in world coordinates with confidence-gated updates. Non-linear alpha weighting ensures high-confidence detections dominate the estimate.

### Training Commands

```bash
# Baselines
python vision/yolo/train_yolo_baseline.py --epochs 100
python vision/yolo/train_clahe_baseline.py --epochs 100
python vision/yolo/train_retinex_baseline.py --epochs 100

# COIN-Pose (two-phase)
python vision/yolo/train_coin_pose.py --phase 1 --epochs-p1 50 --batch-p1 4
python vision/yolo/train_coin_pose.py --phase 2 --p1-ckpt vision/yolo/runs/coin/phase1/best.pt --epochs-p2 80

# PRIM
python vision/yolo/train_PRIM.py

# Evaluate all models
python vision/yolo/compare_all.py
```

## Xbox Controller Mapping

Manual control is available via Xbox controller (Bluetooth on Linux, USB/Bluetooth on Windows).

| Input | Action |
|---|---|
| Left stick | Manual pin control |
| LT + B/X | Trigger loading task |
| LT + Y | Start PBVS |
| L button | Vision/FK check |
| L_JOY + R_JOY | Emergency stop |
