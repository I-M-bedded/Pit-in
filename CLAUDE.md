# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the control software for **Pit-in**, a battery-swapping AGV (Automated Guided Vehicle) testbed. It controls a 7-axis servo robot that picks up and inserts battery pins into EVs.

- **Full operation**: Ubuntu 22.04 LTS + ROS2 Humble, Python 3.10.12
- **Partial operation** (no ROS2, no evdev): Windows — the code auto-detects the OS and skips unavailable modules

## Running the Robot

```bash
# From repo root:
python src_robot/robot_main/main.py
```

Terminate: press **L_JOY_BTN + R_JOY_BTN** simultaneously on the Xbox controller, or `Ctrl+C`.

## Quick Setup (first-time)

1. Match motor IP addresses in `src_robot/fastech/servolist.py`
2. Match Bluetooth controller MAC address in `src_robot/controller/evdev_controlthread.py` line 9 (`controller_address_2`)

## Required Packages

```
numpy, matplotlib, pygame, evdev (Linux), requests
```
ROS2 custom messages are in `src_robot/ros2/pitin_msgs.zip` — build this in your ROS2 workspace.

## Architecture

### Thread Model (`src_robot/robot_main/main.py`)

The `robot` class spawns daemon threads on `__init__`:
| Thread | Platform | Description |
|---|---|---|
| `agv_planning_main` | Linux only | ROS2 `rclpy.spin()` loop |
| `pygame_gui` / `joystick` | Win / Linux | Xbox controller input |
| `control_servo` (×7) | Real robot only | Per-servo UDP control loop |
| `control_agv` | Always | AGV state machine, ~6 Hz |
| `supervisor.run_loop` | Always | Button→Task dispatcher, ~25 Hz |

Termination is coordinated via `robot.process_command` flag (set to 1 to stop all threads).

### Configuration (`src_robot/robot_main/config.py`)

- `RobotConfig` — central dataclass for all parameters (IP addresses, servo heights, control gains, timeouts, vision offsets). Edit this to tune the robot without touching logic.
- `InputManager` — wraps raw joystick axes/buttons into a named-key dict (`get_state()`). All task code reads input through this interface.

### Supervisor & Tasks (`src_robot/robot_main/suprevisor.py`, `tasks.py`)

`TopPlateSupervisor.run_loop()` reads controller state each cycle and dispatches to task objects:
- `HomingTask` — multi-step origin-finding sequence for all 7 axes
- `LoadingTask` — approach + load battery pin sequence (triggered by AGV server request or LT+B/X)
- `ManualpinTask` — direct joystick manual control of lift, pin height, pin rotation, wing gap
- `VisionTask` — FK check and calibration data saving (L button)
- `PbvsTask` — Position-Based Visual Servoing using `HybridController` from `robot_test/macro_micro.py` (LT+Y)

All tasks extend `BaseTask` (fields: `is_active`, `step`, `timer`, `flag`).

### Servo Control (`src_robot/fastech/`)

7 Fastech servo drives controlled over UDP. Motor IDs 0–6 correspond to axes trY, trX, rtZ, Lr, Rr, Lz, Rz (see `servolist.py`).

`t_action` values per axis:
- `0` = position control (tracks `t_pos`)
- `1` = homing (origin finding)
- `2` = servo on
- `3` = servo off

### Kinematics (`src_robot/controller/ik.py`)

`Topik` class — top-plate inverse kinematics. Version-dependent link parameters (`d2`, `d3`) are selected by `RobotConfig.version` (currently `4` = CORA). Unit conversion: `cnt2m` / `m2cnt` arrays convert between encoder counts and meters/radians. Rotation axes 2, 3, 4 have sign inversion applied in `get_q()` and `_q_cmd_to_cnt()`.

### AGV Interface (`src_robot/leadshine/`)

- `agv_dummy.py` — clean AGV controller (use this)
- `agv_con.py` — original AGV controller with legacy code

### ROS2 Interface (`src_robot/ros2/interface.py`)

Subscribes to `/cam0` for camera data. Communicates via `ros2-websocket-bridge` for systems without native ROS2 (see `ros2/test.js` for websocket commands).

### Vision (`vision/`)

- `calibration/` — intrinsic and hand-eye calibration tools
- `dataset_gen/` — auto-annotation and pose estimation for dataset creation
- `yolo/` — YOLO segmentation training pipeline (`train.py`, `yolo_seg_backbone.py`)


너는 비전기반 Visual Servoing에 정통한 개발자이며, 특히 hole 3D pose estimation과 같은 분야에서 두각을 나타내는 개발자임.
로보틱스 코드를 관리할때는 꼭 state machine내부에는 래퍼함수를 넣어 놓고, 실제구현은 모듈화하기. (supevisor와 task의 구조 처럼)

robot의 구조는 urdf_pitin을 읽으면 조금은 확인 가능.
현재 나는 pin을 서로 잇는 직선을 x축이라고 설정하고, x축은 l에서 r방향이 +, right-handed 로 축설정해서 fk, ik 코드를 작성해놓음.
각 핀과 로봇 베이스의 좌표계는 모두 동일하게 설정하려고 의도함.

비전 모델은 목표는 hole의 3d pose를 찾는 것.
특별한 목표는 robustness. 역광/saturation/저조도 상황에서 강건하길 원함.
추가적인 조명없이도 모델과 카메라 만으로도 일반적인 성능이 나오면 좋겠기 때문임.
백본은 yolo pose를 사용하기로 결정함.

금지 : 전체 리팩토링, 레거시 코드 변경 (fastech과 같은 드라이버류를 변경하지 말 것.)

현재 구현해야 하는 것
- Macro-Micro 고도화 : angle이 Macro, x,y가 micro. 이후 z핀 상승중 VS로 변경할 생각임.
- 모델 구조 정확하게 생각해야함.
- 학습 코드 재작성




코드 스타일은 자유롭게